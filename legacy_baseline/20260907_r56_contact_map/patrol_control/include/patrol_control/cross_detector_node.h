#ifndef CROSS_DETECTOR_NODE_H
#define CROSS_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace patrol_control {

class CrossDetectorNode {
public:
    CrossDetectorNode(ros::NodeHandle nh);
    ~CrossDetectorNode();

private:
    void loadParameters();
    void initializeCameraParams();
    void detectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // 十字检测相关函数
    bool detectRedCross(const cv::Mat& image, cv::Point2f& cross_center, double& cross_area);
    bool validateCrossShape(const std::vector<cv::Point>& contour, cv::Point2f& center, double& area);
    bool isCrossLikeShape(const std::vector<cv::Point>& contour);
    double calculateSolidity(const std::vector<cv::Point>& contour);
    void drawDetectionResult(cv::Mat& image);

    // ROS相关
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber detect_control_sub_;
    ros::Publisher pixel_offset_pub_;
    ros::Publisher cross_center_pub_;
    ros::Publisher detection_status_pub_;

    // 检测控制
    bool detection_enabled_;
    bool debug_mode_;
    bool show_image_;
    double max_fps_;

    // 相机参数
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
    
    // 图像中心点（对准目标）
    cv::Point2f image_center_;
    double target_center_x_, target_center_y_;

    // 红色十字检测参数
    bool enable_red_cross_detection_;
    int red_h_min_, red_s_min_, red_v_min_;
    int red_h_max_, red_s_max_, red_v_max_;
    double cross_aspect_ratio_threshold_;
    int cross_min_contour_points_;
    double cross_area_threshold_;
    double cross_solidity_threshold_;

    // 图像预处理参数
    bool enable_gaussian_blur_;
    int blur_kernel_size_;

    // 检测结果
    bool red_cross_found_;
    cv::Point2f red_cross_center_;
    double red_cross_area_;
};

} // namespace patrol_control

#endif // CROSS_DETECTOR_NODE_H 