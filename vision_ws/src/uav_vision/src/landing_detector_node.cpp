#include <uav_vision/landing_detector_node.h>

namespace uav_vision {

LandingDetectorNode::LandingDetectorNode(const ros::NodeHandle &nh)
    : nh_(nh), it_(nh)
{
  loadParameters();

  image_sub_ = it_.subscribe(image_topic_, 1,
                             &LandingDetectorNode::imageCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &LandingDetectorNode::cameraInfoCallback, this);

  detections_pub_ = nh_.advertise<TargetDetectionArray>("/uav_vision/detections", 1);
  debug_pub_ = it_.advertise(debug_image_topic_, 1);

  ROS_INFO("[LandingDetector] ready  image=%s",
           image_topic_.c_str());
}

void LandingDetectorNode::loadParameters()
{
  nh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
  nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/camera_info");
  nh_.param("enable_debug_image", enable_debug_image_, false);
  nh_.param<std::string>("debug_image_topic", debug_image_topic_,
                         "/uav_vision/landing_debug");

  nh_.param("landing_blur_kernel_size", blur_kernel_size_, 5);
  nh_.param("landing_adaptive_block_size", adaptive_block_size_, 31);
  nh_.param("landing_adaptive_c", adaptive_c_, 10.0);
  nh_.param("landing_morphology_kernel_size", morphology_kernel_size_, 7);
  nh_.param("landing_min_contour_points", min_contour_points_, 15);
  nh_.param("landing_aspect_ratio_threshold", aspect_ratio_threshold_, 0.85);
  nh_.param("landing_radius_min", radius_min_, 15.0);
  nh_.param("landing_radius_max", radius_max_, 300.0);
  nh_.param("landing_enable_h_structure_check", enable_h_structure_check_, true);
  nh_.param("landing_h_inner_scale", h_inner_scale_, 0.78);
  nh_.param("landing_h_saturation_max", h_saturation_max_, 90);
  nh_.param("landing_h_value_max", h_value_max_, 110);
  nh_.param("landing_h_open_kernel_size", h_open_kernel_size_, 5);
  nh_.param("landing_h_min_area_ratio", h_min_area_ratio_, 0.10);
  nh_.param("landing_h_max_area_ratio", h_max_area_ratio_, 0.70);
  nh_.param("landing_h_min_aspect_ratio", h_min_aspect_ratio_, 0.55);
  nh_.param("landing_h_min_solidity", h_min_solidity_, 0.25);
  nh_.param("landing_h_max_solidity", h_max_solidity_, 0.80);
  nh_.param("landing_h_min_concave_points", h_min_concave_points_, 2);
  nh_.param("landing_h_defect_depth_ratio", h_defect_depth_ratio_, 0.04);
  nh_.param("landing_h_max_center_distance_ratio", h_max_center_distance_ratio_, 0.35);
}

void LandingDetectorNode::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &msg)
{
  camera_model_.fromCameraInfo(*msg);
}

void LandingDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (!camera_model_.initialized()) return;

  cv::Mat image;
  try {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_THROTTLE(5, "[LandingDetector] cv_bridge: %s", e.what());
    return;
  }

  cv::Point2f center;
  float radius;
  cv::Mat debug_mask;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<double> quality_metrics;
  cv::Rect best_bbox;

  bool found = detectLandingPad(image, center, radius, debug_mask, contours,
                                quality_metrics, best_bbox);

  TargetDetectionArray arr;
  arr.header.stamp = msg->header.stamp;
  arr.header.frame_id = msg->header.frame_id;
  arr.source = "landing_detector";
  arr.completed_sources.push_back(arr.source);

  if (found) {
    TargetDetection det;
    det.header = arr.header;
    det.class_name = "landing_pad";
    const float quality = quality_metrics.size() >= 4
                              ? static_cast<float>(quality_metrics[3])
                              : 0.85f;
    det.class_confidence = quality;
    det.geometry_confidence = quality;
    det.geometry_verified = true;
    det.center_refined = true;
    det.center_source = "landing_geometry";
    det.association_valid = true;
    det.reject_reason = "";
    det.transform_age_sec = -1.0f;
    det.roi.x_offset = best_bbox.x;
    det.roi.y_offset = best_bbox.y;
    det.roi.width = static_cast<uint32_t>(std::max(best_bbox.width, 0));
    det.roi.height = static_cast<uint32_t>(std::max(best_bbox.height, 0));
    det.center_px.x = center.x;
    det.center_px.y = center.y;
    det.center_px.z = radius;
    arr.detections.push_back(det);
  }

  detections_pub_.publish(arr);

  if (enable_debug_image_ && debug_pub_.getNumSubscribers() > 0) {
    cv::RotatedRect best_e;
    if (found)
      best_e = cv::RotatedRect(center, cv::Size2f(radius * 2, radius * 2), 0);
    cv::Mat dbg = drawDebug(debug_mask, contours,
                            found ? &best_e : nullptr,
                            found ? &quality_metrics : nullptr);
    sensor_msgs::ImagePtr dbg_msg =
        cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
    debug_pub_.publish(dbg_msg);
  }
}

bool LandingDetectorNode::detectLandingPad(
    const cv::Mat &image, cv::Point2f &center, float &radius,
    cv::Mat &debug_mask, std::vector<std::vector<cv::Point>> &contours,
    std::vector<double> &quality_metrics, cv::Rect &best_bbox)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  int bks = (blur_kernel_size_ % 2 == 0) ? blur_kernel_size_ + 1
                                          : blur_kernel_size_;
  cv::GaussianBlur(gray, gray, cv::Size(bks, bks), 0);

  cv::Mat binary;
  int abs = (adaptive_block_size_ % 2 == 0) ? adaptive_block_size_ + 1
                                             : adaptive_block_size_;
  cv::adaptiveThreshold(gray, binary, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY_INV, abs, adaptive_c_);

  int mks = (morphology_kernel_size_ % 2 == 0) ? morphology_kernel_size_ + 1
                                                : morphology_kernel_size_;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                             cv::Size(mks, mks));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  debug_mask = binary.clone();
  cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  cv::RotatedRect best_ellipse;
  double best_area = 0;
  double best_aspect_ratio = 0;
  double best_radius = 0;
  int best_contour_points = 0;
  std::vector<double> best_h_metrics;
  bool found = false;

  for (const auto &contour : contours) {
    if (contour.size() < static_cast<size_t>(min_contour_points_))
      continue;

    cv::RotatedRect ellipse = cv::fitEllipse(contour);
    double area = cv::contourArea(contour);
    double w = ellipse.size.width;
    double h = ellipse.size.height;
    if (w < 1e-3 || h < 1e-3) continue;

    double ar = std::min(w, h) / std::max(w, h);
    if (ar < aspect_ratio_threshold_) continue;

    double r = (w + h) / 4.0;
    if (r < radius_min_ || r > radius_max_) continue;

    std::vector<double> h_metrics;
    if (enable_h_structure_check_ &&
        !validateHStructure(image, ellipse, h_metrics)) {
      continue;
    }

    if (area > best_area) {
      best_area = area;
      best_ellipse = ellipse;
      best_aspect_ratio = ar;
      best_radius = r;
      best_contour_points = static_cast<int>(contour.size());
      best_bbox = cv::boundingRect(contour);
      best_h_metrics = h_metrics;
      found = true;
    }
  }

  if (found) {
    center = best_ellipse.center;
    radius = static_cast<float>((best_ellipse.size.width +
                                 best_ellipse.size.height) / 4.0);
    const double area_score = std::max(0.0, std::min(1.0, best_area / 5000.0));
    const double radius_score = std::max(0.0, std::min(1.0, best_radius / std::max(radius_min_, 1.0)));
    const double h_score = best_h_metrics.size() >= 5 ? best_h_metrics[4] : 1.0;
    const double geom_conf = std::max(0.0, std::min(1.0,
        0.3 * best_aspect_ratio + 0.15 * area_score +
        0.10 * radius_score + 0.45 * h_score));
    quality_metrics = {
        best_area,
        best_aspect_ratio,
        best_radius,
        std::max(0.85, geom_conf),
        static_cast<double>(best_contour_points),
        h_score,
    };
    return true;
  }
  return false;
}

bool LandingDetectorNode::validateHStructure(
    const cv::Mat &image, const cv::RotatedRect &ellipse,
    std::vector<double> &metrics) const
{
  cv::RotatedRect inner = ellipse;
  inner.size.width = static_cast<float>(inner.size.width * h_inner_scale_);
  inner.size.height = static_cast<float>(inner.size.height * h_inner_scale_);
  if (inner.size.width < 4.0f || inner.size.height < 4.0f) return false;

  cv::Mat ellipse_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  cv::ellipse(ellipse_mask, inner, cv::Scalar(255), cv::FILLED);

  cv::Mat hsv, dark_neutral;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 0, 0),
              cv::Scalar(180, h_saturation_max_, h_value_max_), dark_neutral);
  cv::bitwise_and(dark_neutral, ellipse_mask, dark_neutral);

  int kernel_size = std::max(1, h_open_kernel_size_);
  if (kernel_size % 2 == 0) ++kernel_size;
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(dark_neutral, dark_neutral, cv::MORPH_OPEN, kernel);

  std::vector<std::vector<cv::Point>> components;
  cv::findContours(dark_neutral, components, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  const double inner_area = CV_PI * inner.size.width * inner.size.height * 0.25;
  const double max_radius = 0.5 * std::max(inner.size.width, inner.size.height);

  double best_area = 0.0;
  std::vector<double> best;
  for (const auto &component : components) {
    if (component.size() < 8) continue;
    const double area = cv::contourArea(component);
    const double area_ratio = area / std::max(inner_area, 1.0);
    if (area_ratio < h_min_area_ratio_ || area_ratio > h_max_area_ratio_)
      continue;

    const cv::Moments moments = cv::moments(component);
    if (moments.m00 <= 0.0) continue;
    const cv::Point2f center(
        static_cast<float>(moments.m10 / moments.m00),
        static_cast<float>(moments.m01 / moments.m00));
    const double center_ratio = cv::norm(center - ellipse.center) /
                                std::max(max_radius, 1.0);
    if (center_ratio > h_max_center_distance_ratio_) continue;

    const cv::RotatedRect component_box = cv::minAreaRect(component);
    const double box_min = std::min(component_box.size.width, component_box.size.height);
    const double box_max = std::max(component_box.size.width, component_box.size.height);
    if (box_min < 1.0 || box_max < 1.0) continue;
    const double aspect_ratio = box_min / box_max;
    if (aspect_ratio < h_min_aspect_ratio_) continue;

    std::vector<cv::Point> hull_points;
    cv::convexHull(component, hull_points);
    const double hull_area = cv::contourArea(hull_points);
    const double solidity = hull_area > 0.0 ? area / hull_area : 0.0;
    if (solidity < h_min_solidity_ || solidity > h_max_solidity_) continue;

    std::vector<int> hull_indices;
    cv::convexHull(component, hull_indices, false);
    std::vector<cv::Vec4i> defects;
    if (hull_indices.size() >= 3)
      cv::convexityDefects(component, hull_indices, defects);
    int concave_points = 0;
    const double minimum_depth = h_defect_depth_ratio_ * box_min;
    for (const auto &defect : defects) {
      if (defect[3] / 256.0 >= minimum_depth) ++concave_points;
    }
    if (concave_points < h_min_concave_points_) continue;

    const double area_score = std::min(1.0, area_ratio / 0.35);
    const double concavity_score = std::min(1.0,
        concave_points / static_cast<double>(std::max(h_min_concave_points_ + 2, 1)));
    const double solidity_score = std::max(0.0, 1.0 - std::abs(solidity - 0.5) / 0.5);
    const double h_score = std::max(0.0, std::min(1.0,
        0.25 * area_score + 0.25 * aspect_ratio +
        0.25 * concavity_score + 0.25 * solidity_score));
    if (area > best_area) {
      best_area = area;
      best = {area_ratio, solidity, aspect_ratio,
              static_cast<double>(concave_points), h_score};
    }
  }
  metrics = best;
  return !metrics.empty();
}

cv::Mat LandingDetectorNode::drawDebug(
    const cv::Mat &mask,
    const std::vector<std::vector<cv::Point>> &contours,
    const cv::RotatedRect *best_ellipse,
    const std::vector<double> *quality_metrics) const
{
  cv::Mat out;
  cv::cvtColor(mask, out, cv::COLOR_GRAY2BGR);
  cv::drawContours(out, contours, -1, cv::Scalar(0, 0, 255), 2);

  if (best_ellipse) {
    cv::ellipse(out, *best_ellipse, cv::Scalar(0, 255, 0), 3);
    cv::circle(out, best_ellipse->center, 6, cv::Scalar(255, 0, 0), -1);
    int y = 30;
    if (quality_metrics && quality_metrics->size() >= 1) {
      cv::putText(out, cv::format("Area: %.1f", (*quality_metrics)[0]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 2);
      y += 24;
    }
    if (quality_metrics && quality_metrics->size() >= 2) {
      cv::putText(out, cv::format("AR: %.3f", (*quality_metrics)[1]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 2);
      y += 24;
    }
    if (quality_metrics && quality_metrics->size() >= 3) {
      cv::putText(out, cv::format("Radius: %.1f", (*quality_metrics)[2]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 2);
      y += 24;
    }
    if (quality_metrics && quality_metrics->size() >= 5) {
      cv::putText(out, cv::format("Pts: %.0f", (*quality_metrics)[4]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 2);
    }
  } else {
    cv::putText(out, "No Landing Pad", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }
  return out;
}

}  // namespace uav_vision

int main(int argc, char **argv)
{
  ros::init(argc, argv, "landing_detector_node");
  ros::NodeHandle nh("~");
  uav_vision::LandingDetectorNode node(nh);
  ros::spin();
  return 0;
}
