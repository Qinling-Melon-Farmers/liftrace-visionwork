#ifndef CIRCLE_DETECTOR_NODE_H
#define CIRCLE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

namespace patrol_control {

class CircleDetectorNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    // 订阅器
    image_transport::Subscriber image_sub_;
    ros::Subscriber detect_control_sub_;
    ros::Subscriber dynamic_control_sub_;
    
    // 发布器
    ros::Publisher pixel_offset_pub_;  // 发布像素偏差
    ros::Publisher detection_status_pub_;
    ros::Publisher circle_center_pub_;
    ros::Publisher image_pub_;
    ros::Publisher waypoint_mark_pub_;
    ros::Subscriber point_class_sub_;
    image_transport::Publisher binary_result_pub_;  // 发布带质量参数标注的二值化图像
    
    // 检测参数
    double min_radius_;
    double max_radius_;
    double param1_;
    double param2_;
    int min_dist_;
    
    // 圆弧检测参数
    bool enable_arc_detection_;
    double arc_angle_threshold_;
    
    // 质量评估参数
    double circularity_threshold_;
    int min_contour_points_;
    double aspect_ratio_threshold_;
    double contour_area_ratio_;
    
    // 图像预处理参数
    bool enable_histogram_equalization_;
    bool enable_threshold_;
    bool enable_gaussian_blur_;
    int blur_kernel_size_;
    
    // 色彩分割参数 (新增)
    bool enable_color_segmentation_;
    int h_min_, s_min_, v_min_;
    int h_max_, s_max_, v_max_;
    
    // 红色十字检测参数 (新增)
    bool enable_red_cross_detection_;
    int red_h_min_, red_s_min_, red_v_min_;
    int red_h_max_, red_s_max_, red_v_max_;
    
    // 红色十字形状验证参数
    double cross_aspect_ratio_threshold_;
    int cross_min_contour_points_;
    double cross_area_threshold_;
    double cross_solidity_threshold_;
    
    // 检测控制参数
    bool detection_enabled_;
    bool dynamic_detection_enabled_;
    bool detection_enabled_prev_;
    bool debug_mode_;
    bool show_image_;
    double max_fps_;
    double scale_factor_;
    
    // 相机参数
    double camera_fx_;
    double camera_fy_;
    double camera_cx_;
    double camera_cy_;
    
    // 图像中心点
    cv::Point2f image_center_;
    
    // 自定义目标中心点参数
    double target_center_x_;
    double target_center_y_;
    
    // 检测状态
    bool circle_found_;
    cv::Point2f detected_center_;
    double detected_radius_;
    
    // 红色十字检测状态
    bool red_cross_found_;
    cv::Point2f red_cross_center_;
    double red_cross_area_;
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void detectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void dynamicControlCallback(const std_msgs::Bool::ConstPtr& msg);
    // 辅助函数
    void loadParameters();
    void initializeCameraParams();
    cv::Mat drawDetectionResult(const cv::Mat& image, const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse);
    cv::Mat drawBinaryResultWithQualityParams(const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse, const std::vector<double>& quality_params);
    
    // 质量评估函数
    bool evaluateCircleQuality(const cv::Mat& gray_image, const cv::Vec3f& circle);
    double calculateCircularity(const cv::Mat& gray_image, const cv::Vec3f& circle);
    double calculateAspectRatio(const cv::Mat& gray_image, const cv::Vec3f& circle);
    double calculateContourAreaRatio(const cv::Mat& gray_image, const cv::Vec3f& circle);
    
    // 圆弧检测函数
    bool detectArc(const cv::Mat& gray_image, cv::Point& center, double& radius);
    bool evaluateArcQuality(const cv::Mat& gray_image, const std::vector<cv::Point>& contour);
    double calculateArcAngle(const std::vector<cv::Point>& contour, const cv::Point& center);
    
    // 红色十字检测函数 (新增)
    bool detectRedCross(const cv::Mat& image, cv::Point2f& cross_center, double& cross_area);
    bool validateCrossShape(const std::vector<cv::Point>& contour, cv::Point2f& center, double& area);
    bool isCrossLikeShape(const std::vector<cv::Point>& contour);
    double calculateSolidity(const std::vector<cv::Point>& contour);

public:
    CircleDetectorNode(ros::NodeHandle nh);
    ~CircleDetectorNode();
};

} // namespace patrol_control

#endif // CIRCLE_DETECTOR_NODE_H 