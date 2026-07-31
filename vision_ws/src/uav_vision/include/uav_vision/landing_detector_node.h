#pragma once

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

class LandingDetectorNode {
public:
  explicit LandingDetectorNode(const ros::NodeHandle &nh);

private:
  void loadParameters();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  bool detectLandingPad(const cv::Mat &image,
                        cv::Point2f &center, float &radius,
                        cv::Mat &debug_mask,
                        std::vector<std::vector<cv::Point>> &contours,
                        std::vector<double> &quality_metrics,
                        cv::Rect &best_bbox);

  bool validateHStructure(const cv::Mat &image,
                          const cv::RotatedRect &ellipse,
                          std::vector<double> &metrics) const;

  cv::Mat drawDebug(const cv::Mat &mask,
                    const std::vector<std::vector<cv::Point>> &contours,
                    const cv::RotatedRect *best_ellipse,
                    const std::vector<double> *quality_metrics) const;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher detections_pub_;
  image_transport::Publisher debug_pub_;

  image_geometry::PinholeCameraModel camera_model_;

  // 参数
  std::string image_topic_;
  std::string camera_info_topic_;
  bool enable_debug_image_;
  std::string debug_image_topic_;

  int blur_kernel_size_;
  int adaptive_block_size_;
  double adaptive_c_;
  int morphology_kernel_size_;
  int min_contour_points_;
  double aspect_ratio_threshold_;
  double radius_min_;
  double radius_max_;
  bool enable_h_structure_check_;
  double h_inner_scale_;
  int h_saturation_max_;
  int h_value_max_;
  int h_open_kernel_size_;
  double h_min_area_ratio_;
  double h_max_area_ratio_;
  double h_min_aspect_ratio_;
  double h_min_solidity_;
  double h_max_solidity_;
  int h_min_concave_points_;
  double h_defect_depth_ratio_;
  double h_max_center_distance_ratio_;
};

}  // namespace uav_vision
