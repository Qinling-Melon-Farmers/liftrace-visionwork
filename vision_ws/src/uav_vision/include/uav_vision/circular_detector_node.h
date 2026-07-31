#ifndef UAV_VISION_CIRCULAR_DETECTOR_NODE_H
#define UAV_VISION_CIRCULAR_DETECTOR_NODE_H

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

class CircularDetectorNode {
public:
  explicit CircularDetectorNode(const ros::NodeHandle &nh);
  ~CircularDetectorNode() = default;

private:
  struct CircleCandidate {
    cv::RotatedRect ellipse;
    double quality;
  };

  void loadParameters();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  bool detectBlueCircles(const cv::Mat &image,
                         std::vector<CircleCandidate> &candidates,
                         cv::Mat &debug_mask,
                         std::vector<std::vector<cv::Point>> &contours);

  void publishDetections(const std::vector<CircleCandidate> &candidates,
                        double scale_x,
                        double scale_y,
                        double offset_x,
                        double offset_y,
                        int original_width,
                        int original_height,
                        const ros::Time &stamp,
                        const std::string &frame_id);

  cv::Mat drawDebug(const cv::Mat &mask,
                    const std::vector<std::vector<cv::Point>> &contours,
                    const cv::RotatedRect *best_ellipse) const;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;

  ros::Publisher detections_pub_;
  image_transport::Publisher debug_pub_;

  image_geometry::PinholeCameraModel camera_model_;

  // 统一参数
  std::string image_topic_;
  std::string camera_info_topic_;
  bool enable_debug_image_;
  std::string debug_image_topic_;

  // HSV 蓝色区间
  int h_min_, s_min_, v_min_;
  int h_max_, s_max_, v_max_;

  // 圆形质量参数
  int min_contour_points_;
  double aspect_ratio_threshold_;
  double radius_min_, radius_max_;
  double min_quality_;
  double duplicate_center_ratio_;
  int max_candidates_;
  bool reject_border_clipped_;

  // 预处理
  int blur_kernel_size_;
  bool enable_morphology_;
  int morphology_kernel_size_;

  // 图像缩放 (降低计算量)
  bool enable_resize_;
  bool preserve_aspect_ratio_;
  int resize_width_, resize_height_;
};

}  // namespace uav_vision

#endif  // UAV_VISION_CIRCULAR_DETECTOR_NODE_H
