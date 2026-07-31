#ifndef UAV_VISION_CROSS_DETECTOR_NODE_H
#define UAV_VISION_CROSS_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <uav_vision/TargetDetection.h>
#include <uav_vision/TargetDetectionArray.h>

namespace uav_vision {

class CrossDetectorNode {
public:
  explicit CrossDetectorNode(const ros::NodeHandle &nh);
  ~CrossDetectorNode() = default;

private:
  void loadParameters();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  bool detectRedCross(const cv::Mat &image,
                      cv::Point2f &cross_center,
                      double &cross_area,
                      std::vector<std::vector<cv::Point>> &red_contours,
                      cv::Mat &red_mask,
                      std::vector<double> &quality_params);

  bool validateCrossShape(const std::vector<cv::Point> &contour,
                          cv::Point2f &center,
                          double &area);
  bool validateCrossShapeRelaxed(const std::vector<cv::Point> &contour,
                                 cv::Point2f &center,
                                 double &area,
                                 std::vector<double> &quality_params);

  bool isCrossLikeShape(const std::vector<cv::Point> &contour);
  double calculateSolidity(const std::vector<cv::Point> &contour);
  double calculateExtent(const std::vector<cv::Point> &contour) const;
  int countConcavePoints(const std::vector<cv::Point> &contour) const;
  void calculateCenterCoverage(const cv::Mat &mask,
                               const std::vector<cv::Point> &contour,
                               double &horizontal_cover,
                               double &vertical_cover) const;

  bool checkBlackOuterRing(const cv::Mat &bgr_image,
                           const cv::Point2f &cross_center);

  cv::Mat drawDebugImage(const cv::Mat &image) const;
  cv::Mat drawBinaryDebug(const cv::Mat &red_mask,
                          const std::vector<std::vector<cv::Point>> &contours,
                          const cv::Point2f *best_center,
                          const std::vector<double> &quality_params) const;

  void publishDetection(const cv::Point2f &cross_center,
                        double cross_area,
                        const std::vector<double> &quality_params,
                        const ros::Time &stamp,
                        const std::string &frame_id);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;

  image_transport::Publisher debug_pub_;
  image_transport::Publisher cross_debug_pub_;
  ros::Publisher detections_pub_;

  image_geometry::PinholeCameraModel camera_model_;

  // 参数
  std::string image_topic_;
  std::string camera_info_topic_;
  bool enable_debug_image_;
  std::string debug_image_topic_;

  // HSV 红色区间检测参数
  int red_s_min_, red_v_min_;
  int red_s_max_, red_v_max_;

  // 形状验证参数
  double cross_aspect_ratio_min_;
  int cross_min_contour_points_;
  double cross_area_min_;
  double cross_solidity_min_;
  double cross_solidity_max_;
  bool cross_reject_border_clipped_;
  int cross_border_margin_px_;
  bool cross_enable_relaxed_scoring_;
  double cross_relaxed_min_score_;
  double cross_relaxed_depth_threshold_;
  double cross_relaxed_min_solidity_;
  double cross_relaxed_max_solidity_;
  double cross_relaxed_min_extent_;
  double cross_relaxed_max_extent_;
  double cross_relaxed_max_aspect_ratio_;
  double cross_relaxed_prefer_aspect_ratio_;
  double cross_relaxed_min_cover_;
  double cross_relaxed_good_cover_;
  int cross_relaxed_min_concave_points_;
  int cross_relaxed_max_concave_points_;
  int cross_relaxed_prefer_concave_points_;

  // 预处理
  bool enable_gaussian_blur_;
  int blur_kernel_size_;

  // 黑色外圈判断
  bool enable_black_ring_check_;
  int black_ring_roi_radius_;

  // 图像中心 (对准目标)
  double target_center_x_, target_center_y_;
  cv::Point2d image_center_;

  // 最新检测结果
  bool red_cross_found_;
  cv::Point2f red_cross_center_;
  double red_cross_area_;
};

}  // namespace uav_vision

#endif  // UAV_VISION_CROSS_DETECTOR_NODE_H
