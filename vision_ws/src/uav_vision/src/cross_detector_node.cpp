#include <uav_vision/cross_detector_node.h>

namespace uav_vision {

CrossDetectorNode::CrossDetectorNode(const ros::NodeHandle &nh)
    : nh_(nh), it_(nh), red_cross_found_(false)
{
  loadParameters();

  image_sub_ = it_.subscribe(image_topic_, 1,
                             &CrossDetectorNode::imageCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &CrossDetectorNode::cameraInfoCallback, this);

  detections_pub_ = nh_.advertise<TargetDetectionArray>("/uav_vision/detections", 1);

  debug_pub_ = it_.advertise(debug_image_topic_, 1);
  cross_debug_pub_ = it_.advertise("/uav_vision/cross_debug", 1);

  ROS_INFO("[CrossDetector] ready  image=%s  camera_info=%s",
           image_topic_.c_str(), camera_info_topic_.c_str());
}

void CrossDetectorNode::loadParameters()
{
  nh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
  nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/camera_info");
  nh_.param("enable_debug_image", enable_debug_image_, false);
  nh_.param<std::string>("debug_image_topic", debug_image_topic_,
                         "/uav_vision/debug_image");

  // 红色 HSV 区间
  nh_.param("red_s_min", red_s_min_, 50);
  nh_.param("red_v_min", red_v_min_, 50);
  nh_.param("red_s_max", red_s_max_, 255);
  nh_.param("red_v_max", red_v_max_, 255);

  // 形状验证
  nh_.param("cross_aspect_ratio_min", cross_aspect_ratio_min_, 0.6);
  nh_.param("cross_min_contour_points", cross_min_contour_points_, 20);
  nh_.param("cross_area_min", cross_area_min_, 200.0);
  nh_.param("cross_solidity_min", cross_solidity_min_, 0.6);
  nh_.param("cross_solidity_max", cross_solidity_max_, 0.85);
  nh_.param("cross_reject_border_clipped", cross_reject_border_clipped_, true);
  nh_.param("cross_border_margin_px", cross_border_margin_px_, 3);
  nh_.param("cross_enable_relaxed_scoring", cross_enable_relaxed_scoring_, true);
  nh_.param("cross_relaxed_min_score", cross_relaxed_min_score_, 3.0);
  nh_.param("cross_relaxed_depth_threshold", cross_relaxed_depth_threshold_, 10.0);
  nh_.param("cross_relaxed_min_solidity", cross_relaxed_min_solidity_, 0.4);
  nh_.param("cross_relaxed_max_solidity", cross_relaxed_max_solidity_, 0.9);
  nh_.param("cross_relaxed_min_extent", cross_relaxed_min_extent_, 0.2);
  nh_.param("cross_relaxed_max_extent", cross_relaxed_max_extent_, 0.75);
  nh_.param("cross_relaxed_max_aspect_ratio", cross_relaxed_max_aspect_ratio_, 2.0);
  nh_.param("cross_relaxed_prefer_aspect_ratio", cross_relaxed_prefer_aspect_ratio_, 1.4);
  nh_.param("cross_relaxed_min_cover", cross_relaxed_min_cover_, 0.3);
  nh_.param("cross_relaxed_good_cover", cross_relaxed_good_cover_, 0.5);
  nh_.param("cross_relaxed_min_concave_points", cross_relaxed_min_concave_points_, 2);
  nh_.param("cross_relaxed_max_concave_points", cross_relaxed_max_concave_points_, 6);
  nh_.param("cross_relaxed_prefer_concave_points", cross_relaxed_prefer_concave_points_, 4);

  // 预处理
  nh_.param("enable_gaussian_blur", enable_gaussian_blur_, true);
  nh_.param("blur_kernel_size", blur_kernel_size_, 5);

  // 黑色外圈
  nh_.param("enable_black_ring_check", enable_black_ring_check_, false);
  nh_.param("black_ring_roi_radius", black_ring_roi_radius_, 80);

  // 图像中心 (对准参考点)
  nh_.param("target_center_x", target_center_x_, 640.0);
  nh_.param("target_center_y", target_center_y_, 480.0);
  image_center_ = cv::Point2d(target_center_x_, target_center_y_);
}

void CrossDetectorNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camera_model_.fromCameraInfo(*msg);
}

// ---------------------------------------------------------------------------
// 主回调
// ---------------------------------------------------------------------------
void CrossDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (!camera_model_.initialized()) return;

  cv::Mat image;
  try {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_THROTTLE(5, "[CrossDetector] cv_bridge: %s", e.what());
    return;
  }

  // 预处理
  cv::Mat processed;
  if (enable_gaussian_blur_) {
    int ks = (blur_kernel_size_ % 2 == 0) ? blur_kernel_size_ + 1
                                          : blur_kernel_size_;
    cv::GaussianBlur(image, processed, cv::Size(ks, ks), 0);
  } else {
    processed = image;
  }

  cv::Point2f cross_center;
  double cross_area;
  std::vector<std::vector<cv::Point>> red_contours;
  cv::Mat red_mask;
  std::vector<double> quality_params;

  bool found = detectRedCross(processed, cross_center, cross_area,
                              red_contours, red_mask, quality_params);

  if (found && enable_black_ring_check_)
    found = checkBlackOuterRing(image, cross_center);

  red_cross_found_ = found;
  if (found) {
    red_cross_center_ = cross_center;
    red_cross_area_ = cross_area;
    publishDetection(cross_center, cross_area, quality_params,
                     msg->header.stamp, msg->header.frame_id);
  } else {
    TargetDetectionArray empty;
    empty.header.stamp = msg->header.stamp;
    empty.header.frame_id = msg->header.frame_id;
    empty.source = "cross_detector";
    empty.completed_sources.push_back(empty.source);
    detections_pub_.publish(empty);
  }

  if (enable_debug_image_ && debug_pub_.getNumSubscribers() > 0) {
    cv::Mat dbg = drawDebugImage(image);
    sensor_msgs::ImagePtr dbg_msg =
        cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
    debug_pub_.publish(dbg_msg);
  }

  if (enable_debug_image_ && cross_debug_pub_.getNumSubscribers() > 0) {
    cv::Mat bdbg = drawBinaryDebug(red_mask, red_contours,
                                   found ? &cross_center : nullptr,
                                   quality_params);
    sensor_msgs::ImagePtr bmsg =
        cv_bridge::CvImage(msg->header, "bgr8", bdbg).toImageMsg();
    cross_debug_pub_.publish(bmsg);
  }
}

// ---------------------------------------------------------------------------
// 发布检测结果 — 只发图像域信息，不投影世界坐标
// ---------------------------------------------------------------------------
void CrossDetectorNode::publishDetection(const cv::Point2f &cross_center,
                                         double cross_area,
                                         const std::vector<double> &quality_params,
                                         const ros::Time &stamp,
                                         const std::string &frame_id)
{
  TargetDetectionArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = frame_id;
  arr.source = "cross_detector";
  arr.completed_sources.push_back(arr.source);

  TargetDetection det;
  det.header.stamp = stamp;
  det.header.frame_id = frame_id;
  det.class_name = "red_cross";
  const float conf = quality_params.size() >= 5
                         ? static_cast<float>(quality_params[4])
                         : (quality_params.size() >= 4
                                ? static_cast<float>(quality_params[3])
                                : 0.7f);
  det.class_confidence = conf;
  det.geometry_confidence = conf;
  det.geometry_verified = true;
  det.center_refined = true;
  det.center_source = "red_cross_geometry";
  det.association_valid = true;
  det.reject_reason = "";
  det.transform_age_sec = -1.0f;
  const int roi_half = std::max(24, static_cast<int>(std::sqrt(std::max(cross_area, 1.0)) * 0.8));
  det.roi.x_offset = std::max(0, static_cast<int>(cross_center.x) - roi_half);
  det.roi.y_offset = std::max(0, static_cast<int>(cross_center.y) - roi_half);
  det.roi.width = static_cast<uint32_t>(roi_half * 2);
  det.roi.height = static_cast<uint32_t>(roi_half * 2);
  det.center_px.x = cross_center.x;
  det.center_px.y = cross_center.y;
  det.center_px.z = 0;

  arr.detections.push_back(det);
  detections_pub_.publish(arr);
}

// ---------------------------------------------------------------------------
// 核心算法：HSV 双区间 → 轮廓 → 形状约束
// ---------------------------------------------------------------------------
bool CrossDetectorNode::detectRedCross(
    const cv::Mat &image, cv::Point2f &cross_center, double &cross_area,
    std::vector<std::vector<cv::Point>> &red_contours, cv::Mat &red_mask,
    std::vector<double> &quality_params)
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask1, mask2;
  cv::inRange(hsv, cv::Scalar(0, red_s_min_, red_v_min_),
              cv::Scalar(10, red_s_max_, red_v_max_), mask1);
  cv::inRange(hsv, cv::Scalar(170, red_s_min_, red_v_min_),
              cv::Scalar(180, red_s_max_, red_v_max_), mask2);
  cv::bitwise_or(mask1, mask2, red_mask);

  // 形态学：闭运算连接断裂区域，开运算去噪
  {
    int ks = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(ks, ks));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
  }

  cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  double best_area = 0;
  cv::Point2f best_center;
  bool found = false;

  for (const auto &contour : red_contours) {
    if (contour.size() < static_cast<size_t>(cross_min_contour_points_))
      continue;

    double area = cv::contourArea(contour);
    if (area < cross_area_min_) continue;

    const cv::Rect contour_bbox = cv::boundingRect(contour);
    if (cross_reject_border_clipped_ &&
        (contour_bbox.x <= cross_border_margin_px_ ||
         contour_bbox.y <= cross_border_margin_px_ ||
         contour_bbox.x + contour_bbox.width >= image.cols - cross_border_margin_px_ ||
         contour_bbox.y + contour_bbox.height >= image.rows - cross_border_margin_px_)) {
      continue;
    }

    cv::Point2f center;
    double contour_area;
    std::vector<double> candidate_quality;
    bool valid = validateCrossShape(contour, center, contour_area);
    if (!valid && cross_enable_relaxed_scoring_) {
      valid = validateCrossShapeRelaxed(contour, center, contour_area,
                                        candidate_quality);
    }
    if (valid) {
      if (contour_area > best_area) {
        best_area = contour_area;
        best_center = center;
        found = true;

        if (!candidate_quality.empty()) {
          quality_params = candidate_quality;
        } else {
          quality_params.clear();
          quality_params.push_back(area);
          quality_params.push_back(static_cast<double>(contour.size()));

          cv::Rect br = cv::boundingRect(contour);
          double ar = std::min(br.width, br.height) /
                      static_cast<double>(std::max(br.width, br.height));
          quality_params.push_back(ar);

          double solidity = calculateSolidity(contour);
          quality_params.push_back(solidity);
          quality_params.push_back(static_cast<double>(std::max(0.0, std::min(1.0, solidity))));
        }
      }
    }
  }

  if (found) {
    cross_center = best_center;
    cross_area = best_area;
    return true;
  }
  return false;
}

bool CrossDetectorNode::validateCrossShapeRelaxed(
    const std::vector<cv::Point> &contour,
    cv::Point2f &center,
    double &area,
    std::vector<double> &quality_params)
{
  cv::Rect br = cv::boundingRect(contour);
  if (br.width <= 0 || br.height <= 0) return false;

  const double rect_area = static_cast<double>(br.width) * br.height;
  if (rect_area <= 1e-6) return false;

  const double contour_area = cv::contourArea(contour);
  const double extent = contour_area / rect_area;
  const double solidity = calculateSolidity(contour);
  const double aspect_ratio = static_cast<double>(std::max(br.width, br.height)) /
                              static_cast<double>(std::max(1, std::min(br.width, br.height)));
  const int concave_points = countConcavePoints(contour);

  cv::Moments m = cv::moments(contour);
  if (m.m00 <= 0.0) return false;
  center.x = static_cast<float>(m.m10 / m.m00);
  center.y = static_cast<float>(m.m01 / m.m00);
  area = m.m00;

  cv::Mat contour_mask = cv::Mat::zeros(br.height, br.width, CV_8UC1);
  std::vector<std::vector<cv::Point>> shifted(1);
  shifted[0].reserve(contour.size());
  for (const auto &pt : contour)
    shifted[0].push_back(cv::Point(pt.x - br.x, pt.y - br.y));
  cv::drawContours(contour_mask, shifted, -1, cv::Scalar(255), cv::FILLED);

  double h_cover = 0.0;
  double v_cover = 0.0;
  calculateCenterCoverage(contour_mask, contour, h_cover, v_cover);

  double score = 0.0;
  if (concave_points >= cross_relaxed_min_concave_points_ &&
      concave_points <= cross_relaxed_max_concave_points_)
    score += 2.0;
  if (concave_points == cross_relaxed_prefer_concave_points_)
    score += 1.0;
  if (aspect_ratio <= cross_relaxed_max_aspect_ratio_)
    score += 1.0;
  if (aspect_ratio <= cross_relaxed_prefer_aspect_ratio_)
    score += 1.0;
  if (solidity >= cross_relaxed_min_solidity_ &&
      solidity <= cross_relaxed_max_solidity_)
    score += 1.0;
  if (extent >= cross_relaxed_min_extent_ &&
      extent <= cross_relaxed_max_extent_)
    score += 1.0;
  if (h_cover > cross_relaxed_min_cover_ && v_cover > cross_relaxed_min_cover_)
    score += 1.0;
  if (h_cover > cross_relaxed_good_cover_ && v_cover > cross_relaxed_good_cover_)
    score += 0.5;
  score += std::min(contour_area / 5000.0, 2.0);

  if (score < cross_relaxed_min_score_) return false;

  const double score_norm = std::max(0.0, std::min(1.0, score / 8.5));
  quality_params.clear();
  quality_params.push_back(contour_area);
  quality_params.push_back(static_cast<double>(contour.size()));
  quality_params.push_back(1.0 / aspect_ratio);
  quality_params.push_back(solidity);
  quality_params.push_back(score_norm);
  quality_params.push_back(extent);
  quality_params.push_back(h_cover);
  quality_params.push_back(v_cover);
  quality_params.push_back(static_cast<double>(concave_points));
  return true;
}

// ---------------------------------------------------------------------------
// 形状验证
// ---------------------------------------------------------------------------
bool CrossDetectorNode::validateCrossShape(const std::vector<cv::Point> &contour,
                                           cv::Point2f &center, double &area)
{
  cv::Rect br = cv::boundingRect(contour);
  double ar = std::min(br.width, br.height) /
              static_cast<double>(std::max(br.width, br.height));
  if (ar < cross_aspect_ratio_min_) return false;

  double solidity = calculateSolidity(contour);
  if (solidity < cross_solidity_min_ || solidity > cross_solidity_max_)
    return false;

  if (!isCrossLikeShape(contour)) return false;

  cv::Moments m = cv::moments(contour);
  if (m.m00 > 0) {
    center.x = static_cast<float>(m.m10 / m.m00);
    center.y = static_cast<float>(m.m01 / m.m00);
    area = m.m00;
    return true;
  }
  return false;
}

bool CrossDetectorNode::isCrossLikeShape(const std::vector<cv::Point> &contour)
{
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull);
  if (hull.size() < 8) return false;

  cv::Rect r = cv::boundingRect(contour);
  cv::Point2f rc(r.x + r.width / 2.f, r.y + r.height / 2.f);

  bool top = false, bot = false, left = false, right = false;
  for (const auto &pt : contour) {
    if (pt.y < rc.y - r.height * 0.3f) top = true;
    if (pt.y > rc.y + r.height * 0.3f) bot = true;
    if (pt.x < rc.x - r.width * 0.3f) left = true;
    if (pt.x > rc.x + r.width * 0.3f) right = true;
  }
  return top && bot && left && right;
}

double CrossDetectorNode::calculateSolidity(const std::vector<cv::Point> &contour)
{
  double ca = cv::contourArea(contour);
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull);
  double ha = cv::contourArea(hull);
  return ha > 0 ? ca / ha : 0.0;
}

double CrossDetectorNode::calculateExtent(const std::vector<cv::Point> &contour) const
{
  const cv::Rect br = cv::boundingRect(contour);
  const double rect_area = static_cast<double>(br.width) * br.height;
  return rect_area > 0.0 ? cv::contourArea(contour) / rect_area : 0.0;
}

int CrossDetectorNode::countConcavePoints(const std::vector<cv::Point> &contour) const
{
  std::vector<int> hull_indices;
  cv::convexHull(contour, hull_indices, false);
  std::vector<cv::Vec4i> defects;
  if (hull_indices.size() >= 3) {
    cv::convexityDefects(contour, hull_indices, defects);
  }
  int concave_points = 0;
  for (const auto &d : defects) {
    const double depth = d[3] / 256.0;
    if (depth > cross_relaxed_depth_threshold_) {
      ++concave_points;
    }
  }
  return concave_points;
}

void CrossDetectorNode::calculateCenterCoverage(
    const cv::Mat &mask,
    const std::vector<cv::Point> &contour,
    double &horizontal_cover,
    double &vertical_cover) const
{
  const cv::Rect br = cv::boundingRect(contour);
  const cv::Moments m = cv::moments(contour);
  const int cxi = std::max(0, std::min(mask.cols - 1,
      static_cast<int>(std::round((m.m00 > 0 ? m.m10 / m.m00 : br.width * 0.5) - br.x))));
  const int cyi = std::max(0, std::min(mask.rows - 1,
      static_cast<int>(std::round((m.m00 > 0 ? m.m01 / m.m00 : br.height * 0.5) - br.y))));

  int h_hit = 0;
  for (int x = 0; x < mask.cols; ++x)
    if (mask.at<uchar>(cyi, x) > 0) ++h_hit;

  int v_hit = 0;
  for (int y = 0; y < mask.rows; ++y)
    if (mask.at<uchar>(y, cxi) > 0) ++v_hit;

  horizontal_cover = mask.cols > 0 ? static_cast<double>(h_hit) / mask.cols : 0.0;
  vertical_cover = mask.rows > 0 ? static_cast<double>(v_hit) / mask.rows : 0.0;
}

// ---------------------------------------------------------------------------
// 黑色外圈判断 — 新加
// ---------------------------------------------------------------------------
bool CrossDetectorNode::checkBlackOuterRing(const cv::Mat &bgr,
                                            const cv::Point2f &cross_center)
{
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  int cx = static_cast<int>(cross_center.x);
  int cy = static_cast<int>(cross_center.y);
  int r = black_ring_roi_radius_;

  int x0 = std::max(cx - r, 0), y0 = std::max(cy - r, 0);
  int x1 = std::min(cx + r, gray.cols), y1 = std::min(cy + r, gray.rows);
  if (x1 - x0 < 10 || y1 - y0 < 10) return true;  // 边界情况，不否决

  cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
  cv::Mat patch = gray(roi);

  // 统计低亮度像素占比，低于阈值则认为存在暗区（黑色外圈/中心暗区）
  int dark = cv::countNonZero(patch < 60);
  double ratio = static_cast<double>(dark) / patch.total();

  return ratio > 0.15;
}

// ---------------------------------------------------------------------------
// 调试绘制
// ---------------------------------------------------------------------------
cv::Mat CrossDetectorNode::drawDebugImage(const cv::Mat &image) const
{
  cv::Mat out = image.clone();

  if (red_cross_found_) {
    cv::circle(out, red_cross_center_, 8, cv::Scalar(255, 255, 255), -1);
    int s = 15;
    cv::line(out, cv::Point(red_cross_center_.x - s, red_cross_center_.y),
             cv::Point(red_cross_center_.x + s, red_cross_center_.y),
             cv::Scalar(0, 255, 0), 3);
    cv::line(out, cv::Point(red_cross_center_.x, red_cross_center_.y - s),
             cv::Point(red_cross_center_.x, red_cross_center_.y + s),
             cv::Scalar(0, 255, 0), 3);

    cv::putText(out, "Red Cross",
                cv::Point(red_cross_center_.x + 10, red_cross_center_.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    double ox = red_cross_center_.x - image_center_.x;
    double oy = red_cross_center_.y - image_center_.y;
    cv::putText(out, cv::format("Offset: (%.1f, %.1f)", ox, oy),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2);
  } else {
    cv::putText(out, "No Red Cross Detected",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 2);
  }

  cv::circle(out, image_center_, 5, cv::Scalar(255, 0, 0), -1);
  return out;
}

cv::Mat CrossDetectorNode::drawBinaryDebug(
    const cv::Mat &red_mask,
    const std::vector<std::vector<cv::Point>> &contours,
    const cv::Point2f *best_center,
    const std::vector<double> &quality_params) const
{
  cv::Mat out;
  cv::cvtColor(red_mask, out, cv::COLOR_GRAY2BGR);
  cv::drawContours(out, contours, -1, cv::Scalar(0, 0, 255), 2);

  if (best_center && !quality_params.empty()) {
    cv::circle(out, *best_center, 8, cv::Scalar(0, 255, 0), -1);
    int y = 30;
    if (quality_params.size() > 0)
      cv::putText(out, cv::format("Area: %.1f", quality_params[0]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1), y += 20;
    if (quality_params.size() > 3)
      cv::putText(out, cv::format("Solidity: %.3f", quality_params[3]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1), y += 20;
    if (quality_params.size() > 4)
      cv::putText(out, cv::format("Score: %.3f", quality_params[4]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1), y += 20;
    if (quality_params.size() > 8)
      cv::putText(out, cv::format("Concave: %.0f", quality_params[8]),
                  cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1);
  }

  cv::circle(out, image_center_, 5, cv::Scalar(255, 255, 255), -1);
  return out;
}

}  // namespace uav_vision

// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cross_detector_node");
  ros::NodeHandle nh("~");
  uav_vision::CrossDetectorNode node(nh);
  ros::spin();
  return 0;
}
