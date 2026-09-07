/**
 * @file cross_detector_node.cpp
 * @author luli (luli.gptt@gmail.com)
 * @brief 红色十字检测节点，专门用于检测红色十字标记
 * @version 1.0
 * @date 2025-01-16
 */

#include "patrol_control/cross_detector_node.h"
#include <tf/transform_datatypes.h>

namespace patrol_control {

CrossDetectorNode::CrossDetectorNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), detection_enabled_(false), red_cross_found_(false) {
    
    // 加载参数
    loadParameters();
    
    // 初始化相机参数
    initializeCameraParams();
    
    // 订阅图像话题
    image_sub_ = it_.subscribe("/iris_mid360/camera/rgb/image_raw", 1, &CrossDetectorNode::imageCallback, this);
    // 订阅检测控制话题
    detect_control_sub_ = nh_.subscribe("/detect/cross_control", 1, &CrossDetectorNode::detectionControlCallback, this);
    
    // 发布像素偏差
    pixel_offset_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/pixel_offset", 1);
    cross_center_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/circle_center", 1);

    
    // 发布检测状态
    detection_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/cross_status", 1);
    
    ROS_INFO("\033[32m[CrossDetectorNode] Red Cross Detector Initialized\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Subscribing to image topic: /iris_mid360/camera/rgb/image_raw\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Subscribing to control topic: /detect/cross_control\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Publishing pixel offset to: /detect/cross_pixel_offset\033[0m");
    ROS_INFO("\033[32m[CrossDetectorNode] Publishing detection status to: /detect/cross_status\033[0m");
}

CrossDetectorNode::~CrossDetectorNode() {
    // 确保OpenCV窗口正确关闭
    try {
        cv::destroyAllWindows();
    } catch (const cv::Exception& e) {
        ROS_WARN("\033[33m[CrossDetectorNode] Exception while closing OpenCV windows: %s\033[0m", e.what());
    }
    
    ROS_INFO("\033[33m[CrossDetectorNode] Cross detector node shut down.\033[0m");
}

void CrossDetectorNode::loadParameters() {
    // 检测控制参数
    nh_.param("cross_detection/enabled", detection_enabled_, true);
    nh_.param("cross_detection/debug", debug_mode_, false);
    nh_.param("cross_detection/show_image", show_image_, true);
    nh_.param("cross_detection/max_fps", max_fps_, 15.0);

    // 红色十字检测参数
    nh_.param("red_cross_detection/enable", enable_red_cross_detection_, true);
    nh_.param("red_cross_detection/h_min", red_h_min_, 0);
    nh_.param("red_cross_detection/s_min", red_s_min_, 120);
    nh_.param("red_cross_detection/v_min", red_v_min_, 70);
    nh_.param("red_cross_detection/h_max", red_h_max_, 10);
    nh_.param("red_cross_detection/s_max", red_s_max_, 255);
    nh_.param("red_cross_detection/v_max", red_v_max_, 255);
    
    // 红色十字形状验证参数
    nh_.param("red_cross_detection/aspect_ratio_threshold", cross_aspect_ratio_threshold_, 0.6);
    nh_.param("red_cross_detection/min_contour_points", cross_min_contour_points_, 20);
    nh_.param("red_cross_detection/area_threshold", cross_area_threshold_, 200.0);
    nh_.param("red_cross_detection/solidity_threshold", cross_solidity_threshold_, 0.6);

    // 图像预处理参数
    nh_.param("image_preprocessing/enable_gaussian_blur", enable_gaussian_blur_, true);
    nh_.param("image_preprocessing/blur_kernel_size", blur_kernel_size_, 5);

    // 加载自定义的目标中心点
    nh_.param("detection_control/target_center_x", target_center_x_, 424.0);
    nh_.param("detection_control/target_center_y", target_center_y_, 240.0);

    // 使用加载的参数设置图像中心，这将是我们的"靶心"
    image_center_ = cv::Point2f(target_center_x_, target_center_y_);
    
    ROS_INFO("\033[34m[CrossDetectorNode] Parameters loaded:\033[0m");
    ROS_INFO("\033[34m[CrossDetectorNode] Alignment Target Center: (%.1f, %.1f)\033[0m", image_center_.x, image_center_.y);
    ROS_INFO("\033[34m[CrossDetectorNode] Show image: %s, Max FPS: %.1f\033[0m", show_image_ ? "ON" : "OFF", max_fps_);
    
    // 红色十字检测参数日志
    if(enable_red_cross_detection_) {
        ROS_INFO("\033[34m[CrossDetectorNode] Red Cross Detection ENABLED\033[0m");
        ROS_INFO("\033[34m[CrossDetectorNode] Red Cross HSV: H[%d, %d], S[%d, %d], V[%d, %d]\033[0m",
                 red_h_min_, red_h_max_, red_s_min_, red_s_max_, red_v_min_, red_v_max_);
        ROS_INFO("\033[34m[CrossDetectorNode] Cross Quality: aspect_ratio=%.2f, min_points=%d, area_threshold=%.1f, solidity=%.2f\033[0m",
                 cross_aspect_ratio_threshold_, cross_min_contour_points_, cross_area_threshold_, cross_solidity_threshold_);
    } else {
        ROS_INFO("\033[34m[CrossDetectorNode] Red Cross Detection DISABLED\033[0m");
    }
}

void CrossDetectorNode::initializeCameraParams() {
    // 从配置文件加载相机内参
    nh_.param("camera/fx", camera_fx_, 479.652738);
    nh_.param("camera/fy", camera_fy_, 482.690306);
    nh_.param("camera/cx", camera_cx_, 657.45208);
    nh_.param("camera/cy", camera_cy_, 364.6207);
    
    ROS_INFO("\033[34m[CrossDetectorNode] Camera params initialized: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\033[0m", 
             camera_fx_, camera_fy_, camera_cx_, camera_cy_);
}

void CrossDetectorNode::detectionControlCallback(const std_msgs::Bool::ConstPtr& msg) {
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

void CrossDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!detection_enabled_ || !enable_red_cross_detection_) {
        if (show_image_) {
            try {
                // 关闭所有十字检测相关的窗口
                cv::destroyAllWindows();
                ROS_INFO_ONCE("\033[33m[CrossDetectorNode] Cross detection disabled, closing all display windows\033[0m");
            } catch (const cv::Exception& e) {
                // 窗口可能已经被关闭，忽略异常
            }
        }
        return;
    }
    
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
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
        bool red_cross_detected = detectRedCross(processed_image, cross_center, cross_area);
        
        if (red_cross_detected) {
            // 计算像素偏差（相对于图像中心）
            double pixel_offset_x = cross_center.x - image_center_.x;
            double pixel_offset_y = cross_center.y - image_center_.y;
            
            // 发布像素偏差
            geometry_msgs::Point pixel_offset;
            pixel_offset.x = pixel_offset_x;
            pixel_offset.y = pixel_offset_y;
            pixel_offset.z = sqrt(cross_area / M_PI) * 4.6; // 使用面积计算等效半径
            pixel_offset_pub_.publish(pixel_offset);
            
            // 发布十字中心
            geometry_msgs::Point cross_center_msg;
            cross_center_msg.x = cross_center.x;
            cross_center_msg.y = cross_center.y;
            cross_center_msg.z = cross_area;
            cross_center_pub_.publish(cross_center_msg);
            
            // 发布检测状态
            std_msgs::Bool status_msg;
            status_msg.data = true;
            detection_status_pub_.publish(status_msg);
            
            if (debug_mode_) {
                ROS_INFO_THROTTLE(1.0, "\033[35m[CrossDetectorNode] Red cross detected!\033[0m");
                ROS_INFO_THROTTLE(1.0, "\033[35m[CrossDetectorNode] Center: (%.1f, %.1f), Area: %.1f, Offset: (%.1f, %.1f)\033[0m", 
                                  cross_center.x, cross_center.y, cross_area, pixel_offset_x, pixel_offset_y);
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
        
        // 显示检测结果图像
        if (show_image_) {
            drawDetectionResult(image);
        }
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CrossDetectorNode] cv_bridge exception: %s\033[0m", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CrossDetectorNode] Image processing exception: %s\033[0m", e.what());
    }
}

// 红色十字检测函数
bool CrossDetectorNode::detectRedCross(const cv::Mat& image, cv::Point2f& cross_center, double& cross_area) {
    cv::Mat hsv_image, red_mask;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
    
    // 红色在HSV中有两个范围: [0,10] 和 [170,180] 进行二值化处理
    cv::Mat red_mask1, red_mask2;
    cv::inRange(hsv_image, cv::Scalar(red_h_min_, red_s_min_, red_v_min_), 
                cv::Scalar(red_h_max_, red_s_max_, red_v_max_), red_mask1);
    cv::inRange(hsv_image, cv::Scalar(170, red_s_min_, red_v_min_), 
                cv::Scalar(180, red_s_max_, red_v_max_), red_mask2);
    
    // 合并两个红色范围
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    
    // 形态学操作，去除噪点并连接断裂的区域
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //分别进行开运算和闭运算，去除噪点和内部的小孔
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    double best_area = 0;
    cv::Point2f best_center;
    bool found = false;
    
    // 遍历轮廓寻找十字形状
    for (const auto& contour : contours) {
        if (contour.size() < cross_min_contour_points_) {
            continue;
        }
        
        double area = cv::contourArea(contour);
        if (area < cross_area_threshold_) {
            continue;
        }
        
        cv::Point2f center;
        double contour_area;
        if (validateCrossShape(contour, center, contour_area)) {
            // 选择面积最大的合格十字
            if (contour_area > best_area) {
                best_area = contour_area;
                best_center = center;
                found = true;
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
                         (double)std::max(bounding_rect.width, bounding_rect.height);
    
    if (aspect_ratio < cross_aspect_ratio_threshold_) {
        return false;
    }
    
    // 计算实体度（solidity）：轮廓面积与其凸包面积的比值
    double solidity = calculateSolidity(contour);
    if (solidity < cross_solidity_threshold_) {
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
void CrossDetectorNode::drawDetectionResult(cv::Mat& image) {
    // 绘制红色十字检测结果
    if (red_cross_found_) {
        // 绘制十字中心点（红色）
        cv::circle(image, red_cross_center_, 8, cv::Scalar(255, 255, 255), -1);
        // 绘制十字标记
        int cross_size = 15;
        cv::line(image, 
                 cv::Point(red_cross_center_.x - cross_size, red_cross_center_.y), 
                 cv::Point(red_cross_center_.x + cross_size, red_cross_center_.y), 
                 cv::Scalar(0, 255, 0), 3);
        cv::line(image, 
                 cv::Point(red_cross_center_.x, red_cross_center_.y - cross_size), 
                 cv::Point(red_cross_center_.x, red_cross_center_.y + cross_size), 
                 cv::Scalar(0, 255, 0), 3);
        
        // 添加文本标签
        cv::putText(image, "Red Cross", 
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        
        // 显示面积信息
        cv::putText(image, cv::format("Area: %.1f", red_cross_area_), 
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y + 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }
    
    // 绘制图像中心点（蓝色）
    cv::circle(image, image_center_, 5, cv::Scalar(255, 0, 0), -1);
    
    // 显示图像
    cv::imshow("Cross Detection Result", image);
    cv::waitKey(1);
}

} // namespace patrol_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "cross_detector_node");
    ros::NodeHandle nh;

    patrol_control::CrossDetectorNode detector(nh);
    
    ros::spin();
    
    return 0;
} 