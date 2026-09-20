#include <uav_vision/circular_detector_node.h>

#include <algorithm>
#include <cmath>

namespace uav_vision {

CircularDetectorNode::CircularDetectorNode(const ros::NodeHandle &nh)
    : nh_(nh), it_(nh)
{
  loadParameters();

  image_sub_ = it_.subscribe(image_topic_, 1,
                             &CircularDetectorNode::imageCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &CircularDetectorNode::cameraInfoCallback, this);

  detections_pub_ = nh_.advertise<TargetDetectionArray>("/uav_vision/detections", 1);
  debug_pub_ = it_.advertise(debug_image_topic_, 1);

  ROS_INFO("[CircleDetector] ready  image=%s  camera_info=%s",
           image_topic_.c_str(), camera_info_topic_.c_str());
}

void CircularDetectorNode::loadParameters()
{
  nh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
  nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/camera_info");
  nh_.param("enable_debug_image", enable_debug_image_, false);
  nh_.param<std::string>("debug_image_topic", debug_image_topic_,
                         "/uav_vision/circle_debug");

  // HSV 蓝色区间
  nh_.param("circle_h_min", h_min_, 90);
  nh_.param("circle_s_min", s_min_, 80);
  nh_.param("circle_v_min", v_min_, 80);
  nh_.param("circle_h_max", h_max_, 130);
  nh_.param("circle_s_max", s_max_, 255);
  nh_.param("circle_v_max", v_max_, 255);

  // 圆形质量
  nh_.param("circle_min_contour_points", min_contour_points_, 15);
  nh_.param("circle_aspect_ratio_threshold", aspect_ratio_threshold_, 0.85);
  nh_.param("circle_radius_min", radius_min_, 10.0);
  nh_.param("circle_radius_max", radius_max_, 300.0);
  nh_.param("circle_min_quality", min_quality_, 0.70);
  nh_.param("circle_duplicate_center_ratio", duplicate_center_ratio_, 0.45);
  nh_.param("circle_max_candidates", max_candidates_, 12);
  nh_.param("circle_reject_border_clipped", reject_border_clipped_, true);

  // 预处理
  nh_.param("circle_blur_kernel_size", blur_kernel_size_, 5);
  nh_.param("circle_enable_morphology", enable_morphology_, true);
  nh_.param("circle_morphology_kernel_size", morphology_kernel_size_, 7);

  // 缩放
  nh_.param("circle_enable_resize", enable_resize_, false);
  nh_.param("circle_preserve_aspect_ratio", preserve_aspect_ratio_, true);
  nh_.param("circle_resize_width", resize_width_, 640);
  nh_.param("circle_resize_height", resize_height_, 512);
}

void CircularDetectorNode::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &msg)
{
  camera_model_.fromCameraInfo(*msg);
  ROS_INFO_ONCE("[CircleDetector] CameraInfo received %ux%u frame=%s",
               msg->width, msg->height, msg->header.frame_id.c_str());
}

// ---------------------------------------------------------------------------
void CircularDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ROS_DEBUG_THROTTLE(2.0, "[CircleDetector] image received %ux%u encoding=%s",
                     msg->width, msg->height, msg->encoding.c_str());
  if (!camera_model_.initialized()) return;

  cv::Mat image;
  try {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_THROTTLE(5, "[CircleDetector] cv_bridge: %s", e.what());
    return;
  }

  const int original_width = image.cols;
  const int original_height = image.rows;
  double scale_x = 1.0;
  double scale_y = 1.0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  if (enable_resize_ && preserve_aspect_ratio_) {
    const double scale = std::min(
        static_cast<double>(resize_width_) / original_width,
        static_cast<double>(resize_height_) / original_height);
    const int scaled_width = std::max(1, static_cast<int>(std::round(original_width * scale)));
    const int scaled_height = std::max(1, static_cast<int>(std::round(original_height * scale)));
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(scaled_width, scaled_height),
               0, 0, cv::INTER_AREA);
    // 反变换必须与下方实际使用的整数 ROI 完全一致。剩余边距为奇数时，若在这里使用
    // 浮点半边距，会引入 0.5 像素偏差。
    const int pad_x = (resize_width_ - scaled_width) / 2;
    const int pad_y = (resize_height_ - scaled_height) / 2;
    offset_x = static_cast<double>(pad_x);
    offset_y = static_cast<double>(pad_y);
    cv::Mat letterboxed = cv::Mat::zeros(resize_height_, resize_width_, image.type());
    resized.copyTo(letterboxed(cv::Rect(pad_x,
                                        pad_y,
                                        scaled_width, scaled_height)));
    image = letterboxed;
    scale_x = 1.0 / scale;
    scale_y = 1.0 / scale;
  } else if (enable_resize_) {
    cv::resize(image, image, cv::Size(resize_width_, resize_height_),
               0, 0, cv::INTER_AREA);
    scale_x = static_cast<double>(original_width) / image.cols;
    scale_y = static_cast<double>(original_height) / image.rows;
  }

  cv::Mat debug_mask;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<CircleCandidate> candidates;

  bool found = detectBlueCircles(image, candidates, debug_mask, contours);
  ROS_DEBUG_THROTTLE(2.0, "[CircleDetector] contours=%zu candidates=%zu resized=%dx%d",
                     contours.size(), candidates.size(), image.cols, image.rows);

  if (found) {
    publishDetections(candidates, scale_x, scale_y, offset_x, offset_y,
                      original_width, original_height,
                      msg->header.stamp, msg->header.frame_id);
  } else {
    TargetDetectionArray empty;
    empty.header.stamp = msg->header.stamp;
    empty.header.frame_id = msg->header.frame_id;
    empty.source = "circle_detector";
    empty.completed_sources.push_back(empty.source);
    detections_pub_.publish(empty);
  }

  if (enable_debug_image_ && debug_pub_.getNumSubscribers() > 0) {
    cv::RotatedRect best_ellipse;
    if (found)
      best_ellipse = candidates.front().ellipse;
    cv::Mat dbg = drawDebug(debug_mask, contours,
                            found ? &best_ellipse : nullptr);
    sensor_msgs::ImagePtr dbg_msg =
        cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
    debug_pub_.publish(dbg_msg);
  }
}

// ---------------------------------------------------------------------------
void CircularDetectorNode::publishDetections(
                                            const std::vector<CircleCandidate> &candidates,
                                            double scale_x,
                                            double scale_y,
                                            double offset_x,
                                            double offset_y,
                                            int original_width,
                                            int original_height,
                                            const ros::Time &stamp,
                                            const std::string &frame_id)
{
  TargetDetectionArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = frame_id;
  arr.source = "circle_detector";
  arr.completed_sources.push_back(arr.source);

  for (const CircleCandidate &candidate : candidates) {
    TargetDetection det;
    det.header.stamp = stamp;
    det.header.frame_id = frame_id;
    det.class_name = "circle";
    det.class_confidence = static_cast<float>(candidate.quality);
    det.geometry_confidence = static_cast<float>(candidate.quality);
    det.geometry_verified = candidate.quality >= min_quality_;
    det.center_refined = true;
    det.center_source = "circle_geometry";
    det.association_valid = det.geometry_verified;
    det.reject_reason = det.geometry_verified ? "" : "geometry_quality_low";
    det.transform_age_sec = -1.0f;

    const cv::Point2f center(
        static_cast<float>((candidate.ellipse.center.x - offset_x) * scale_x),
        static_cast<float>((candidate.ellipse.center.y - offset_y) * scale_y));
    const float radius = static_cast<float>(
        ((candidate.ellipse.size.width * scale_x) +
         (candidate.ellipse.size.height * scale_y)) / 4.0);
    det.center_px.x = std::max(0.0f, std::min(center.x,
                                             static_cast<float>(original_width - 1)));
    det.center_px.y = std::max(0.0f, std::min(center.y,
                                             static_cast<float>(original_height - 1)));
    // 保留历史接口语义：circle 检测的 center_px.z 为像素半径。
    det.center_px.z = radius;
    const int x = std::max(0, static_cast<int>(det.center_px.x - radius));
    const int y = std::max(0, static_cast<int>(det.center_px.y - radius));
    const int x2 = std::min(original_width, static_cast<int>(det.center_px.x + radius));
    const int y2 = std::min(original_height, static_cast<int>(det.center_px.y + radius));
    det.roi.x_offset = x;
    det.roi.y_offset = y;
    det.roi.width = std::max(0, x2 - x);
    det.roi.height = std::max(0, y2 - y);
    arr.detections.push_back(det);
  }
  detections_pub_.publish(arr);
}

// ---------------------------------------------------------------------------
bool CircularDetectorNode::detectBlueCircles(
    const cv::Mat &image,
    std::vector<CircleCandidate> &candidates,
    cv::Mat &debug_mask,
    std::vector<std::vector<cv::Point>> &contours)
{
  cv::Mat hsv, mask;
  cv::Mat filtered = image;
  if (blur_kernel_size_ > 1) {
    int bks = blur_kernel_size_ | 1;
    cv::GaussianBlur(image, filtered, cv::Size(bks, bks), 0.0);
  }
  cv::cvtColor(filtered, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
              cv::Scalar(h_max_, s_max_, v_max_), mask);

  if (enable_morphology_) {
    int mks = morphology_kernel_size_ & 1 ? morphology_kernel_size_
                                          : morphology_kernel_size_ + 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(mks, mks));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  }

  debug_mask = mask.clone();
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  for (const auto &contour : contours) {
    if (contour.size() < std::max(5, min_contour_points_))
      continue;

    cv::RotatedRect ellipse = cv::fitEllipse(contour);
    double w = ellipse.size.width;
    double h = ellipse.size.height;
    if (w < 1e-3 || h < 1e-3) continue;

    double ar = std::min(w, h) / std::max(w, h);
    if (ar < aspect_ratio_threshold_) continue;

    double r = (w + h) / 4.0;
    if (r < radius_min_ || r > radius_max_) continue;

    const bool clipped = ellipse.center.x - r < 1.0 ||
                         ellipse.center.y - r < 1.0 ||
                         ellipse.center.x + r >= image.cols - 1 ||
                         ellipse.center.y + r >= image.rows - 1;
    if (clipped && reject_border_clipped_) continue;

    const double perimeter = std::max(1.0, 2.0 * CV_PI * r);
    const double contour_density = std::min(1.0,
        static_cast<double>(contour.size()) / (perimeter * 0.8));
    const double aspect_quality = std::min(1.0, ar);
    const double border_quality = clipped ? 0.55 : 1.0;
    const double quality = std::max(0.0, std::min(1.0,
        0.45 * aspect_quality + 0.40 * contour_density +
        0.15 * border_quality));
    if (quality < min_quality_) continue;

    bool duplicate = false;
    for (const CircleCandidate &kept : candidates) {
      const double dx = kept.ellipse.center.x - ellipse.center.x;
      const double dy = kept.ellipse.center.y - ellipse.center.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      const double duplicate_radius = std::max(kept.ellipse.size.width,
                                                kept.ellipse.size.height) *
                                      duplicate_center_ratio_;
      if (distance < duplicate_radius) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      candidates.push_back(CircleCandidate{ellipse, quality});
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const CircleCandidate &a, const CircleCandidate &b) {
              return a.quality > b.quality;
            });
  if (static_cast<int>(candidates.size()) > max_candidates_)
    candidates.resize(max_candidates_);
  return !candidates.empty();
}

// ---------------------------------------------------------------------------
cv::Mat CircularDetectorNode::drawDebug(
    const cv::Mat &mask,
    const std::vector<std::vector<cv::Point>> &contours,
    const cv::RotatedRect *best_ellipse) const
{
  cv::Mat out;
  cv::cvtColor(mask, out, cv::COLOR_GRAY2BGR);
  cv::drawContours(out, contours, -1, cv::Scalar(0, 0, 255), 2);

  if (best_ellipse) {
    cv::ellipse(out, *best_ellipse, cv::Scalar(0, 255, 0), 3);
    cv::circle(out, best_ellipse->center, 6, cv::Scalar(255, 0, 0), -1);
  } else {
    cv::putText(out, "No Circle Detected", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }
  return out;
}

}  // namespace uav_vision

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_detector_node");
  ros::NodeHandle nh("~");
  uav_vision::CircularDetectorNode node(nh);
  ros::spin();
  return 0;
}
