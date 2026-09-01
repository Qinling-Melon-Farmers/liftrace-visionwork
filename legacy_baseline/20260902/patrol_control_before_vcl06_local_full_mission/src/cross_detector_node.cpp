/**
 * @file cross_detector_node.cpp
 * @author luli (luli.gptt@gmail.com)
 * @brief 红色十字检测节点，专门用于检测红色十字标记
 * @version 1.0
 * @date 2025-01-16
 */

//  TODO:添加调试图象以及偏差乘以系数
#include "patrol_control/cross_detector_node.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class CrossDetectorNode {
public:
    explicit CrossDetectorNode(const ros::NodeHandle &nh);

    ~CrossDetectorNode();

private:
    void loadParameters();

    void detectionControlCallback(const std_msgs::Bool::ConstPtr &msg);

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    // 十字检测相关函数
    bool detectRedCross(const cv::Mat &image, cv::Point2f &cross_center, double &cross_area,
                        std::vector<std::vector<cv::Point> > &red_contours, cv::Mat &red_mask,
                        std::vector<double> &quality_params);

    static bool validateCrossShape(const std::vector<cv::Point> &contour, cv::Point2f &center, double &area);

    static bool isCrossLikeShape(const std::vector<cv::Point> &contour);

    static double calculateSolidity(const std::vector<cv::Point> &contour);

    void camera_info_callback(const sensor_msgs::CameraInfo &camera_info);

    cv::Mat drawDetectionResult(const cv::Mat &image) const; // 修改返回类型为cv::Mat
    cv::Mat drawBinaryResultWithQualityParams(const cv::Mat &red_mask,
                                              const std::vector<std::vector<cv::Point> > &contours,
                                              const cv::Point2f *best_cross_center,
                                              const std::vector<double> &quality_params) const;

    // ROS相关
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher result_image_pub_; // 添加结果图像发布器
    ros::Subscriber detect_control_sub_;
    ros::Publisher pixel_offset_pub_;
    ros::Publisher detection_status_pub_;

    // 发布带质量参数标注的二值化图像
    image_transport::Publisher binary_result_pub_;

    // 检测控制
    bool detection_enabled_;
    bool debug_mode_;
    bool show_image_;
    double max_fps_;

    // 图像中心点（对准目标）
    cv::Point2d image_center_;
    double target_center_x_, target_center_y_;

    ros::Subscriber camera_info_sub;
    image_geometry::PinholeCameraModel camera_model_;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transform_listener;
    geometry_msgs::TransformStamped camera2map_transform;

    // 红色十字检测参数
    bool enable_red_cross_detection_;
    int red_s_min_, red_v_min_;
    int red_s_max_, red_v_max_;
    double cross_aspect_ratio_threshold_;
    int cross_min_contour_points_;
    double cross_area_threshold_;
    double cross_solidity_threshold_;

    // 图像预处理参数
    bool enable_gaussian_blur_;
    int blur_kernel_size_;

    // 像素偏差系数参数
    double pixel_offset_scale_;

    // 检测结果
    bool red_cross_found_;
    cv::Point2d red_cross_center_;
    double red_cross_area_;
};

CrossDetectorNode::CrossDetectorNode(const ros::NodeHandle &nh)
    : nh_(nh), it_(nh), detection_enabled_(false), red_cross_found_(false), transform_listener(buffer) {
    // 加载参数
    loadParameters();

    // 订阅图像话题
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &CrossDetectorNode::imageCallback, this);
    // 订阅检测控制话题
    detect_control_sub_ = nh_.subscribe("/detect/cross_control", 1, &CrossDetectorNode::detectionControlCallback,
                                        this);

    // 发布像素偏差
    pixel_offset_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detect/cross_mark_point", 1);

    // 发布检测结果图像（压缩格式）
    result_image_pub_ = it_.advertise("/detect/cross_result_image/compressed", 1);

    // 发布带质量参数标注的二值化图像
    binary_result_pub_ = it_.advertise("/detect/cross_binary_result_image/compressed", 1);

    // 发布检测状态
    detection_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/cross_status", 1);

    camera_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &CrossDetectorNode::camera_info_callback, this);

    ROS_INFO("\033[32m[CrossDetectorNode] Red Cross Detector Initialized\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Subscribing to image topic: /iris_mid360/camera/rgb/image_raw\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Subscribing to control topic: /detect/cross_control\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Publishing pixel offset to: /detect/pixel_offset\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Publishing detection status to: /detect/cross_status\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Publishing result image to: /detect/cross_result_image/compressed\033[0m")
    ;
    ROS_INFO(
        "\033[32m[CrossDetectorNode] Publishing binary result image to: /detect/cross_binary_result_image/compressed\033[0m")
    ;
}

CrossDetectorNode::~CrossDetectorNode() {
    // 确保OpenCV窗口正确关闭
    try {
        cv::destroyAllWindows();
    } catch (const cv::Exception &e) {
        ROS_WARN("\033[33m[CrossDetectorNode] Exception while closing OpenCV windows: %s\033[0m", e.what());
    }

    ROS_INFO("\033[33m[CrossDetectorNode] Cross detector node shut down.\033[0m");
}

void CrossDetectorNode::loadParameters() {
    // 检测控制参数
    nh_.param("cross_detection/enabled", detection_enabled_, true);
    nh_.param("cross_detection/debug", debug_mode_, false);
    nh_.param("cross_detection/show_image", show_image_, false);
    nh_.param("cross_detection/max_fps", max_fps_, 15.0);

    // 红色十字检测参数
    nh_.param("red_cross_detection/enable", enable_red_cross_detection_, true);
    nh_.param("red_cross_detection/s_min", red_s_min_, 120);
    nh_.param("red_cross_detection/v_min", red_v_min_, 70);
    nh_.param("red_cross_detection/s_max", red_s_max_, 255);
    nh_.param("red_cross_detection/v_max", red_v_max_, 255);

    // 红色十字形状验证参数
    nh_.param("red_cross_detection/aspect_ratio_threshold", cross_aspect_ratio_threshold_, 0.8);
    nh_.param("red_cross_detection/min_contour_points", cross_min_contour_points_, 20);
    nh_.param("red_cross_detection/area_threshold", cross_area_threshold_, 2000.0);
    nh_.param("red_cross_detection/solidity_threshold", cross_solidity_threshold_, 0.6);

    // 图像预处理参数
    nh_.param("image_preprocessing/enable_gaussian_blur", enable_gaussian_blur_, true);
    nh_.param("image_preprocessing/blur_kernel_size", blur_kernel_size_, 5);

    // 加载自定义的目标中心点
    nh_.param("detection_control/target_center_x", target_center_x_, 640.0);
    nh_.param("detection_control/target_center_y", target_center_y_, 480.0);

    // 像素偏差系数参数
    nh_.param("cross_detection/pixel_offset_scale", pixel_offset_scale_, 0.0008);

    // 使用加载的参数设置图像中心，这将是我们的"靶心"
    image_center_ = cv::Point2d(target_center_x_, target_center_y_);

    ROS_INFO("\033[34m[CrossDetectorNode] Parameters loaded:\033[0m");
    ROS_INFO("\033[34m[CrossDetectorNode] Alignment Target Center: (%.1f, %.1f)\033[0m", image_center_.x,
             image_center_.y);
    ROS_INFO("\033[34m[CrossDetectorNode] Show image: %s, Max FPS: %.1f\033[0m", show_image_ ? "ON" : "OFF",
             max_fps_);
    ROS_INFO("\033[34m[CrossDetectorNode] Pixel offset scale: %.6f\033[0m", pixel_offset_scale_);

    // 红色十字检测参数日志
    if (enable_red_cross_detection_) {
        ROS_INFO("\033[34m[CrossDetectorNode] Red Cross Detection ENABLED\033[0m");
        ROS_INFO(
            "\033[34m[CrossDetectorNode] Cross Quality: aspect_ratio=%.2f, min_points=%d, area_threshold=%.1f, solidity=%.2f\033[0m",
            cross_aspect_ratio_threshold_, cross_min_contour_points_, cross_area_threshold_,
            cross_solidity_threshold_);
    } else {
        ROS_INFO("\033[34m[CrossDetectorNode] Red Cross Detection DISABLED\033[0m");
    }
}

void CrossDetectorNode::detectionControlCallback(const std_msgs::Bool::ConstPtr &msg) {
    static bool last_state = false;
    static bool first_call = true;

    bool new_state = msg->data;

    // 只在状态变化时输出提示
    if (first_call || new_state != last_state) {
        if (new_state) {
            ROS_INFO("\033[32m[CrossDetectorNode] Cross Detection ENABLED by control topic.\033[0m");
        } else {
            ROS_INFO("\033[33m[CrossDetectorNode] Cross Detection DISABLED by control topic.\033[0m");
        }
        last_state = new_state;
        first_call = false;
    }

    detection_enabled_ = new_state;
}

void CrossDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // 如果检测被禁用，直接返回
    if (!detection_enabled_) {
        // 关闭显示窗口
        if (show_image_) {
            try {
                cv::destroyWindow("Cross Detection Result");
            } catch (const cv::Exception &e) {
                // 窗口可能已经被关闭，忽略异常
            }
        }

        // 发布false状态，表示没有检测到十字
        return;
    }

    if (!camera_model_.initialized()) {
        return;
    }
    cv::Mat image;

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CrossDetectorNode] cv_bridge exception: %s\033[0m", e.what());
    }
    // 预处理
    cv::Mat processed_image;
    if (enable_gaussian_blur_) {
        int kernel_size = (blur_kernel_size_ % 2 == 0) ? blur_kernel_size_ + 1 : blur_kernel_size_;
        cv::GaussianBlur(image, processed_image, cv::Size(kernel_size, kernel_size), 0);
    } else {
        processed_image = image.clone();
    }

    // 检测红色十字
    cv::Point2f cross_center;
    double cross_area;
    std::vector<std::vector<cv::Point> > red_contours;
    cv::Mat red_mask;
    std::vector<double> quality_params;
    bool red_cross_detected = detectRedCross(processed_image, cross_center, cross_area, red_contours, red_mask,
                                             quality_params);

    if (red_cross_detected) {
        // 4. 获取相机和map之间的变换
        try {
            camera2map_transform = buffer.lookupTransform("camera_link", "map", ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR("tf2::TransformException %s", e.what());
            return;
        }

        // 5. 根据中心像素和相机参数求取中心点坐标
        cv::Point3d center_vec = camera_model_.projectPixelTo3dRay(cross_center);
        geometry_msgs::Vector3 camera_vec_msg;
        camera_vec_msg.x = center_vec.x;
        camera_vec_msg.y = center_vec.y;
        camera_vec_msg.z = center_vec.z;
        tf2::Vector3 camera_vec;
        tf2::fromMsg(camera_vec_msg, camera_vec);

        tf2::Quaternion rot_quat;
        tf2::fromMsg(camera2map_transform.transform.rotation, rot_quat);

        tf2::Vector3 base_vec = tf2::quatRotate(rot_quat, camera_vec);

        double t = -camera2map_transform.transform.translation.z / base_vec.z();
        double x = base_vec.x() * t + camera2map_transform.transform.translation.x;
        double y = base_vec.y() * t - camera2map_transform.transform.translation.y;

        geometry_msgs::PoseStamped pose_stamped_res;
        pose_stamped_res.header.frame_id = "map";
        pose_stamped_res.header.stamp = ros::Time::now();
        pose_stamped_res.pose.position.x = x;
        pose_stamped_res.pose.position.y = y;
        pose_stamped_res.pose.position.z = 0;
        pose_stamped_res.pose.orientation.w = 1;
        pixel_offset_pub_.publish(pose_stamped_res);
        ROS_INFO("camera2map_transform x, y, z, w: %f, %f, %f, %f", camera2map_transform.transform.rotation.x,
                 camera2map_transform.transform.rotation.y, camera2map_transform.transform.rotation.z,
                 camera2map_transform.transform.rotation.w);
        ROS_INFO("base_vec x, y, z: %f %f %f", base_vec.x(), base_vec.y(), base_vec.z());


        // 发布检测状态
        std_msgs::Bool status_msg;
        status_msg.data = true;
        detection_status_pub_.publish(status_msg);

        if (debug_mode_) {
            ROS_INFO_THROTTLE(1.0, "\033[35m[CrossDetectorNode] Red cross detected!\033[0m");
        }
    } else {
        // 没有检测到十字
        red_cross_found_ = false;

        std_msgs::Bool status_msg;
        status_msg.data = false;
        detection_status_pub_.publish(status_msg);

        if (debug_mode_) {
            ROS_INFO_THROTTLE(2.0, "\033[33m[CrossDetectorNode] No red cross detected.\033[0m");
        }
    }
    // 绘制检测结果图像
    cv::Mat result_image = drawDetectionResult(image);

    // 发布检测结果图像
    if (result_image_pub_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).
                toImageMsg();
        image_msg->header.stamp = msg->header.stamp;
        image_msg->header.frame_id = msg->header.frame_id;
        result_image_pub_.publish(image_msg);
    }

    // 绘制带质量参数标注的二值化图像
    cv::Mat binary_result_image = drawBinaryResultWithQualityParams(
        red_mask, red_contours, red_cross_detected ? &cross_center : nullptr, quality_params);

    // 发布带质量参数标注的二值化图像
    if (binary_result_pub_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binary_result_image).
                toImageMsg();
        binary_msg->header.stamp = msg->header.stamp;
        binary_msg->header.frame_id = msg->header.frame_id;
        binary_result_pub_.publish(binary_msg);
    }

    // 显示检测结果图像（如果启用）
    if (show_image_) {
        cv::imshow("Cross Detection Result", result_image);
        cv::waitKey(1);
    }
}

// 红色十字检测函数
bool CrossDetectorNode::detectRedCross(const cv::Mat& image, cv::Point2f& cross_center, double& cross_area,
                                       std::vector<std::vector<cv::Point>>& red_contours, cv::Mat& red_mask,
                                       std::vector<double>& quality_params) {
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 红色在HSV中有两个范围: [0,10] 和 [170,180] 进行二值化处理
    cv::Mat red_mask1, red_mask2;
    cv::inRange(hsv_image, cv::Scalar(0, red_s_min_, red_v_min_),
                cv::Scalar(10, red_s_max_, red_v_max_), red_mask1);
    cv::inRange(hsv_image, cv::Scalar(170, red_s_min_, red_v_min_),
                cv::Scalar(180, red_s_max_, red_v_max_), red_mask2);

    // 合并两个红色范围
    cv::bitwise_or(red_mask1, red_mask2, red_mask);

    // 形态学操作，去除噪点并连接断裂的区域
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //分别进行开运算和闭运算，去除噪点和内部的小孔

    // 寻找轮廓
    cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0;
    cv::Point2f best_center;
    bool found = false;

    // 遍历轮廓寻找十字形状
    for (const auto& contour : red_contours) {
        if (contour.size() < cross_min_contour_points_) {
            continue;
        }

        double area = cv::contourArea(contour);
        if (area <  1700.0) {
            continue;
        }
        ROS_INFO("AREA %f" , area);

        cv::Point2f center;
        double contour_area;
        if (validateCrossShape(contour, center, contour_area)) {
            // 选择面积最大的合格十字
            if (contour_area > best_area) {
                best_area = contour_area;
                best_center = center;
                found = true;

                // 收集质量参数
                quality_params.clear();
                quality_params.push_back(area);  // 面积
                quality_params.push_back(contour.size());  // 轮廓点数

                // 计算长宽比
                cv::Rect bounding_rect = cv::boundingRect(contour);
                double aspect_ratio = std::min(bounding_rect.width, bounding_rect.height) /
                                     static_cast<double>(std::max(bounding_rect.width, bounding_rect.height));
                quality_params.push_back(aspect_ratio);

                // 计算实体度
                double solidity = calculateSolidity(contour);
                quality_params.push_back(solidity);
            }
        }
    }

    if (found) {
        cross_center = best_center;
        cross_area = best_area;
        red_cross_found_ = true;
        red_cross_center_ = best_center;
        red_cross_area_ = best_area;
        return true;
    }

    red_cross_found_ = false;
    return false;
}

// 验证十字形状
bool CrossDetectorNode::validateCrossShape(const std::vector<cv::Point>& contour, cv::Point2f& center, double& area) {
    // 计算轮廓的边界矩形
    cv::Rect bounding_rect = cv::boundingRect(contour);

    // 计算长宽比，十字应该接近正方形
    double aspect_ratio = std::min(bounding_rect.width, bounding_rect.height) /
                         static_cast<double>(std::max(bounding_rect.width, bounding_rect.height));

    if (aspect_ratio < 0.6) {
        return false;
    }

    // 计算实体度（solidity）：轮廓面积与其凸包面积的比值
    double solidity = calculateSolidity(contour);
    if (solidity < 0.7 || solidity > 0.73) {
        return false;
    }

    // 使用更严格的十字形状验证
    if (!isCrossLikeShape(contour)) {
        return false;
    }

    // 计算质心作为十字中心
    cv::Moments moments = cv::moments(contour);
    if (moments.m00 > 0) {
        center.x = moments.m10 / moments.m00;
        center.y = moments.m01 / moments.m00;
        area = moments.m00;
        return true;
    }

    return false;
}

// 检查是否为十字形状
bool CrossDetectorNode::isCrossLikeShape(const std::vector<cv::Point>& contour) {
    // 计算轮廓的凸包
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);

    // 如果凸包点数过少，不是十字
    if (hull.size() < 8) {
        return false;
    }

    // 计算边界矩形
    cv::Rect rect = cv::boundingRect(contour);
    cv::Point2f rect_center(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

    // 检查轮廓是否在四个主要方向上都有延伸
    bool has_top = false, has_bottom = false, has_left = false, has_right = false;

    for (const auto& point : contour) {
        if (point.y < rect_center.y - rect.height * 0.3) has_top = true;
        if (point.y > rect_center.y + rect.height * 0.3) has_bottom = true;
        if (point.x < rect_center.x - rect.width * 0.3) has_left = true;
        if (point.x > rect_center.x + rect.width * 0.3) has_right = true;
    }

    // 十字应该在四个方向上都有延伸
    return has_top && has_bottom && has_left && has_right;
}

// 计算实体度
double CrossDetectorNode::calculateSolidity(const std::vector<cv::Point>& contour) {
    double contour_area = cv::contourArea(contour);

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hull_area = cv::contourArea(hull);

    if (hull_area > 0) {
        return contour_area / hull_area;
    }

    return 0.0;
}
cv::Mat CrossDetectorNode::drawDetectionResult(const cv::Mat& image) const {
    cv::Mat result_image = image.clone();

    // 绘制红色十字检测结果
    if (red_cross_found_) {
        // 绘制十字中心点（白色）
        cv::circle(result_image, red_cross_center_, 8, cv::Scalar(255, 255, 255), -1);
        // 绘制十字标记（绿色）
        int cross_size = 15;
        cv::line(result_image,
                 cv::Point(red_cross_center_.x - cross_size, red_cross_center_.y),
                 cv::Point(red_cross_center_.x + cross_size, red_cross_center_.y),
                 cv::Scalar(0, 255, 0), 3);
        cv::line(result_image,
                 cv::Point(red_cross_center_.x, red_cross_center_.y - cross_size),
                 cv::Point(red_cross_center_.x, red_cross_center_.y + cross_size),
                 cv::Scalar(0, 255, 0), 3);

        // 添加文本标签
        cv::putText(result_image, "Red Cross",
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        // 显示面积信息
        cv::putText(result_image, cv::format("Area: %.1f", red_cross_area_),
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

        // 显示像素偏差信息
        double pixel_offset_x = red_cross_center_.x - image_center_.x;
        double pixel_offset_y = red_cross_center_.y - image_center_.y;
        cv::putText(result_image, cv::format("Offset: (%.1f, %.1f)", pixel_offset_x, pixel_offset_y),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    } else {
        // 没有检测到十字时显示状态
        cv::putText(result_image, "No Red Cross Detected",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }

    // 绘制图像中心点（蓝色）
    cv::circle(result_image, image_center_, 5, cv::Scalar(255, 0, 0), -1);

    // 绘制中心十字线（用于参考）
    int center_size = 20;
    cv::line(result_image,
             cv::Point(image_center_.x - center_size, image_center_.y),
             cv::Point(image_center_.x + center_size, image_center_.y),
             cv::Scalar(255, 0, 0), 2);
    cv::line(result_image,
             cv::Point(image_center_.x, image_center_.y - center_size),
             cv::Point(image_center_.x, image_center_.y + center_size),
             cv::Scalar(255, 0, 0), 2);

    return result_image;
}

cv::Mat CrossDetectorNode::drawBinaryResultWithQualityParams(const cv::Mat& red_mask, const std::vector<std::vector<cv::Point>>& contours, const cv::Point2f* best_cross_center, const std::vector<double>& quality_params) const {
    // 将二值化图像转换为彩色图像
    cv::Mat binary_color;
    cv::cvtColor(red_mask, binary_color, cv::COLOR_GRAY2BGR);

    // 在二值化图像上绘制所有轮廓（红色）
    cv::drawContours(binary_color, contours, -1, cv::Scalar(0, 0, 255), 2);

    if (best_cross_center && !quality_params.empty()) {
        // 绘制最佳十字中心（绿色）
        cv::circle(binary_color, *best_cross_center, 8, cv::Scalar(0, 255, 0), -1);

        // 绘制十字标记（蓝色）
        int cross_size = 15;
        cv::line(binary_color,
                 cv::Point(best_cross_center->x - cross_size, best_cross_center->y),
                 cv::Point(best_cross_center->x + cross_size, best_cross_center->y),
                 cv::Scalar(255, 0, 0), 3);
        cv::line(binary_color,
                 cv::Point(best_cross_center->x, best_cross_center->y - cross_size),
                 cv::Point(best_cross_center->x, best_cross_center->y + cross_size),
                 cv::Scalar(255, 0, 0), 3);

        // 添加质量参数文本标注
        int y_offset = 30;
        int line_height = 25;

        // 标题
        cv::putText(binary_color, "Cross Quality Parameters:",
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        y_offset += line_height;

        // 面积
        if (quality_params.size() > 0) {
            cv::putText(binary_color, cv::format("Area: %.1f px^2", quality_params[0]),
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }

        // 轮廓点数
        if (quality_params.size() > 1) {
            cv::putText(binary_color, cv::format("Contour Points: %d", static_cast<int>(quality_params[1])),
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }

        // 长宽比
        if (quality_params.size() > 2) {
            cv::putText(binary_color, cv::format("Aspect Ratio: %.3f", quality_params[2]),
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }

        // 实体度
        if (quality_params.size() > 3) {
            cv::putText(binary_color, cv::format("Solidity: %.3f", quality_params[3]),
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }

        // 十字中心坐标
        cv::putText(binary_color, cv::format("Center: (%.1f, %.1f)", best_cross_center->x, best_cross_center->y),
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    } else {
        // 没有检测到有效十字时显示状态
        cv::putText(binary_color, "No Valid Cross Detected",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(binary_color, "Check HSV parameters and quality thresholds",
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }

    // 绘制图像中心点（白色十字）
    cv::circle(binary_color, image_center_, 5, cv::Scalar(255, 255, 255), -1);
    int center_size = 15;
    cv::line(binary_color,
             cv::Point(image_center_.x - center_size, image_center_.y),
             cv::Point(image_center_.x + center_size, image_center_.y),
             cv::Scalar(255, 255, 255), 2);
    cv::line(binary_color,
             cv::Point(image_center_.x, image_center_.y - center_size),
             cv::Point(image_center_.x, image_center_.y + center_size),
             cv::Scalar(255, 255, 255), 2);

    // 添加图例
    int legend_y = binary_color.rows - 120;
    cv::putText(binary_color, "Legend:", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    legend_y += 20;
    cv::putText(binary_color, "Red: All contours", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    legend_y += 15;
    cv::putText(binary_color, "Green: Cross center", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    legend_y += 15;
    cv::putText(binary_color, "Blue: Cross marker", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
    legend_y += 15;
    cv::putText(binary_color, "White: Image center", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

    return binary_color;
}


void CrossDetectorNode::camera_info_callback(const sensor_msgs::CameraInfo &camera_info) {
    this->camera_model_.fromCameraInfo(camera_info);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cross_detector_node");
    ros::NodeHandle nh;

    CrossDetectorNode detector(nh);

    ros::spin();

    return 0;
}
