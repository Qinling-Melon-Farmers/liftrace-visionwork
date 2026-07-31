#ifndef LANDING_DETECTOR_NODE_H
#define LANDING_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

namespace patrol_control {

class LandingDetectorNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber img_sub_;
    ros::Subscriber detect_control_sub_;
    ros::Publisher pixel_offset_pub_;
    ros::Publisher circle_center_pub_;
    ros::Publisher detection_status_pub_;
    // ros::Subscriber land_mark_sub_;
    // 新增发布器
    ros::Publisher waypoint_mark_pub_;
    image_transport::Publisher binary_result_pub_;
    
    cv::Mat cv_image;
    bool detection_enabled_;
    bool detection_enabled_prev_;
    bool show_image_;
    bool ellipse_found_;
    cv::Point2f center;
    float radius;

    // 相机参数
    double camera_fx_;
    double camera_fy_;
    double camera_cx_;
    double camera_cy_;
    double camera_width_;
    double camera_height_;
    double target_center_x_;
    double target_center_y_;

    // 检测参数
    double area_threshold_;
    double aspect_ratio_threshold_;
    bool debug_mode_;
    // 新增像素偏差缩放参数
    double pixel_offset_scale_;
    
    // 检测状态
    bool circle_found_;
    cv::Point2f detected_center_;
    double detected_radius_;
    cv::Point2f image_center_;
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void detectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void image_process(cv::Mat &cv_img);
    void loadParameters();
    void initializeCameraParams();
    void drawDetectionResult(cv::Mat& image, const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, 
                           const cv::RotatedRect* best_ellipse);
    // void landmarkCallback
    // 新增方法
    cv::Mat drawBinaryResultWithQualityParams(const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, 
                                             const cv::RotatedRect* best_ellipse, const std::vector<double>& quality_params);
public:
    LandingDetectorNode(ros::NodeHandle nh);
    ~LandingDetectorNode();
};

} // namespace patrol_control

#endif // LANDING_DETECTOR_NODE_H 