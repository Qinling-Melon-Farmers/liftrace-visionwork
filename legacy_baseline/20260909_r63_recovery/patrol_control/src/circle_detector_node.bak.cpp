
/**
 * @file circle_detector_node.cpp
 * @author luli (luli.gptt@gmail.com)
 * @brief 圆形检测节点，基于图像中心点对准控制
 * @version 2.0
 * @date 2025-01-16
 */

#include "patrol_control/circle_detector_node.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>

namespace patrol_control {

CircleDetectorNode::CircleDetectorNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), detection_enabled_(false), detection_enabled_prev_(false), circle_found_(false),
      red_cross_found_(false) {
    
    // 加载参数
    loadParameters();
    
    // 初始化相机参数
    initializeCameraParams();
    
    // 订阅图像话题
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &CircleDetectorNode::imageCallback, this);
    
    // 订阅检测控制话题
    detect_control_sub_ = nh_.subscribe("/detect/control", 1, &CircleDetectorNode::detectionControlCallback, this);
    dynamic_control_sub_ = nh_.subscribe("/dynamic/control", 1, &CircleDetectorNode::dynamicControlCallback, this);
    // 发布像素偏差
    pixel_offset_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/pixel_offset", 1);
    circle_center_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/circle_center", 1);
    waypoint_mark_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detect/waypoint_mark_point", 1);
    
    // 发布带质量参数标注的二值化图像
    binary_result_pub_ = it_.advertise("/detect/binary_result_image/compressed", 1);
    
    // 发布检测状态
    detection_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/status", 1);
    
    
    ROS_INFO("\033[32m[CircleDetectorNode] Color-based Circle Detector Initialized\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Subscribing to image topic: /iris_mid360/camera/rgb/image_raw\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Subscribing to control topic: /detect/control\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Publishing pixel offset to: /detect/pixel_offset\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Publishing detection status to: /detect/status\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Publishing binary result image to: /detect/binary_result_image/compressed\033[0m");
}

CircleDetectorNode::~CircleDetectorNode() {
    // 确保OpenCV窗口正确关闭
    try {
        cv::destroyAllWindows();
    } catch (const cv::Exception& e) {
        ROS_WARN("\033[33m[CircleDetectorNode] Exception while closing OpenCV windows: %s\033[0m", e.what());
    }
    
    ROS_INFO("\033[33m[CircleDetectorNode] Circle detector node shut down.\033[0m");
}

void CircleDetectorNode::loadParameters() {
    // 半径范围参数 (用于过滤)
    nh_.param("circle_detection/min_radius", min_radius_, 10.0);
    nh_.param("circle_detection/max_radius", max_radius_, 300.0);

    // 色彩分割参数 (*** 已根据实际图像进行调整 ***)
    nh_.param("color_segmentation/enable", enable_color_segmentation_, true);
    nh_.param("color_segmentation/h_min", h_min_, 90);
    nh_.param("color_segmentation/s_min", s_min_, 80);
    nh_.param("color_segmentation/v_min", v_min_, 80);
    nh_.param("color_segmentation/h_max", h_max_, 130);
    nh_.param("color_segmentation/s_max", s_max_, 255);
    nh_.param("color_segmentation/v_max", v_max_, 255);
    
    // 质量评估参数 (基于轮廓和椭圆拟合)
    nh_.param("quality_assessment/min_contour_points", min_contour_points_, 15);
    nh_.param("quality_assessment/aspect_ratio_threshold", aspect_ratio_threshold_, 0.85); // 更严格，因为我们期望是圆
    
    // 图像预处理参数
    nh_.param("image_preprocessing/enable_gaussian_blur", enable_gaussian_blur_, true);
    nh_.param("image_preprocessing/blur_kernel_size", blur_kernel_size_, 5);
    
    // 检测控制参数
    nh_.param("detection/enabled", detection_enabled_, true);
    nh_.param("detection/dynamic_enabled", dynamic_detection_enabled_, false);
    nh_.param("detection/debug", debug_mode_, false);
    nh_.param("detection_control/show_image", show_image_, true);
    nh_.param("detection_control/max_fps", max_fps_, 15.0);
    nh_.param("detection_control/scale_factor", scale_factor_, 0.01);

    // 新增: 加载自定义的目标中心点
    nh_.param("detection_control/target_center_x", target_center_x_, 320.0);
    nh_.param("detection_control/target_center_y", target_center_y_, 240.0);

    // 使用加载的参数设置图像中心，这将是我们的"靶心"
    image_center_ = cv::Point2f(target_center_x_, target_center_y_);
    
    ROS_INFO("\033[34m[CircleDetectorNode] Parameters loaded:\033[0m");
    ROS_INFO("\033[34m[CircleDetectorNode] Alignment Target Center: (%.1f, %.1f)\033[0m", image_center_.x, image_center_.y);
    ROS_INFO("\033[34m[CircleDetectorNode] Radius range: %.1f - %.1f pixels\033[0m", min_radius_, max_radius_);
    if(enable_color_segmentation_) {
        ROS_INFO("\033[34m[CircleDetectorNode] Color Segmentation (HSV): H[%d, %d], S[%d, %d], V[%d, %d]\033[0m",
                 h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
    }
    ROS_INFO("\033[34m[CircleDetectorNode] Quality: min_contour_points=%d, aspect_ratio_threshold=%.2f\033[0m",
             min_contour_points_, aspect_ratio_threshold_);
    ROS_INFO("\033[34m[CircleDetectorNode] Preprocessing: GaussianBlur=%s (kernel: %d)\033[0m", 
             enable_gaussian_blur_ ? "ON" : "OFF", blur_kernel_size_);
    ROS_INFO("\033[34m[CircleDetectorNode] Show image: %s, Max FPS: %.1f\033[0m", show_image_ ? "ON" : "OFF", max_fps_);
}

void CircleDetectorNode::initializeCameraParams() {
    // 从配置文件加载相机内参
    nh_.param("camera/fx", camera_fx_, 479.652738);
    nh_.param("camera/fy", camera_fy_, 482.690306);
    nh_.param("camera/cx", camera_cx_, 657.45208);
    nh_.param("camera/cy", camera_cy_, 364.6207);
    
    ROS_INFO("\033[34m[CircleDetectorNode] Camera params initialized: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\033[0m", 
             camera_fx_, camera_fy_, camera_cx_, camera_cy_);
}

void CircleDetectorNode::detectionControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    static bool last_state = false;
    static bool first_call = true;
    
    bool new_state = msg->data;
    
    // 只在状态变化时输出提示
    if (first_call || new_state != last_state) {
        if (new_state) {
            ROS_INFO("\033[32m[CircleDetectorNode] Detection ENABLED by control topic.\033[0m");
        } else {
            ROS_INFO("\033[33m[CircleDetectorNode] Detection DISABLED by control topic.\033[0m");
            // 当检测被禁用时，重置检测状态和数据
            circle_found_ = false;
            detected_center_ = cv::Point2f(0, 0);
            detected_radius_ = 0.0;
            ROS_INFO("\033[33m[CircleDetectorNode] Detection data reset due to disable.\033[0m");
        }
        last_state = new_state;
        first_call = false;
    }
    
    detection_enabled_ = new_state;
}

void CircleDetectorNode::dynamicControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    static bool last_state = false;
    static bool first_call = true;
    
    bool new_state = msg->data;
    if (first_call || new_state != last_state) {
        if (new_state) {
            ROS_INFO("\033[32m[CircleDetectorNode] Dynamic Detection ENABLED by control topic.\033[0m");
        } else {
            ROS_INFO("\033[33m[CircleDetectorNode] Dynamic Detection DISABLED by control topic.\033[0m");
        }
        last_state = new_state;
        first_call = false;
    }
    
    dynamic_detection_enabled_ = new_state;
}

void CircleDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!detection_enabled_) {
        if (show_image_) {
            try {
                // 如果禁用了检测，但窗口仍然打开，则关闭它
                cv::destroyWindow("Detection Result (Left) & Color Mask (Right)");
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

        // --- 蓝色圆形检测逻辑 ---
        
        // 1. 色彩分割
        cv::Mat hsv_image, mask;
        cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);

        // 形态学操作，去除噪点，连接断裂区域
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 2. 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool valid_circle_found = false;
        cv::RotatedRect best_ellipse;
        double best_area = 0;
        std::vector<double> best_quality_params;  // 存储最佳圆形的质量参数

        // 3. 遍历轮廓并进行椭圆拟合
        
        for (const auto& contour : contours) {
            // 轮廓点数太少，无法稳定拟合
            if (contour.size() < min_contour_points_) {
                continue;
            }

            cv::RotatedRect ellipse = cv::fitEllipse(contour);
            double area = cv::contourArea(contour);

            // 4. 质量评估
            double width = ellipse.size.width;
            double height = ellipse.size.height;
            // 避免除以零
            if (width < 1e-3 || height < 1e-3) continue;

            // a. 长宽比判断，越接近1越像圆
            double aspect_ratio = std::min(width, height) / std::max(width, height);
            if (aspect_ratio < aspect_ratio_threshold_) {
                continue;
            }

            // b. 半径范围判断
            double radius = (width + height) / 4.0; // 平均半径
            if (radius < min_radius_ || radius > max_radius_) {
                continue;
            }

            // 选择面积最大的合格轮廓
            if (area > best_area) {
                best_area = area;
                best_ellipse = ellipse;
                valid_circle_found = true;
                
                // 收集质量参数
                best_quality_params.clear();
                best_quality_params.push_back(radius);  // 半径
                best_quality_params.push_back(aspect_ratio);  // 宽高比
                best_quality_params.push_back(area);  // 面积
                best_quality_params.push_back(contour.size());  // 轮廓点数
            }
        }
        
        // --- 检测结果处理 ---
        if (valid_circle_found) {
            // 使用质量最好的圆形
            double center_x = best_ellipse.center.x;
            double center_y = best_ellipse.center.y;
            double radius = (best_ellipse.size.width + best_ellipse.size.height) / 4.0;
            
            // 计算像素偏差（相对于图像中心）
            double pixel_offset_x = center_x - image_center_.x;
            double pixel_offset_y = center_y - image_center_.y;
            ROS_INFO_THROTTLE(0.5, "\033[36m[CircleDetectorNode] Center: (%.1f, %.1f), Radius: %.1f, image_center: (%.1f, %.1f)\033[0m", 
                              center_x, center_y, radius, image_center_.x, image_center_.y);
            
            // 更新检测状态
            circle_found_ = true;
            detected_center_ = best_ellipse.center;
            detected_radius_ = radius;
            
            // 发布像素偏差
            geometry_msgs::Point pixel_offset;
            pixel_offset.x = pixel_offset_x;
            pixel_offset.y = pixel_offset_y;
            pixel_offset.z = radius;
            geometry_msgs::Point circle_center;
            circle_center.x = center_x;
            circle_center.y = center_y;
            circle_center.z = radius;

            pixel_offset_pub_.publish(pixel_offset);
            circle_center_pub_.publish(circle_center);

            // 计算waypoint_mark的详细信息
            double waypoint_x = -pixel_offset_y * scale_factor_;
            double waypoint_y = -pixel_offset_x * scale_factor_;
            double waypoint_z = radius * 0.01;
            
            geometry_msgs::PoseStamped waypoint_mark;
            waypoint_mark.pose.position.x = waypoint_x;
            waypoint_mark.pose.position.y = waypoint_y;
            waypoint_mark.pose.position.z = waypoint_z;
            waypoint_mark.pose.orientation.x = 0.0;
            waypoint_mark.pose.orientation.y = 0.0;
            waypoint_mark.pose.orientation.z = 0.0;
            waypoint_mark.pose.orientation.w = 1.0;
            waypoint_mark.header.stamp = msg->header.stamp;
            waypoint_mark.header.frame_id = msg->header.frame_id;

            // 打印waypoint_mark计算的详细信息
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode] ===== Waypoint Mark Calculation =====\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode] Input Parameters:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Circle Center: (%.1f, %.1f)\033[0m", center_x, center_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Image Center: (%.1f, %.1f)\033[0m", image_center_.x, image_center_.y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Pixel Offset: (%.1f, %.1f)\033[0m", pixel_offset_x, pixel_offset_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Scale Factor: %.6f\033[0m", scale_factor_);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Circle Radius: %.1f\033[0m", radius);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode] Calculation Process:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - waypoint_x = -pixel_offset_y * scale_factor_\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - waypoint_x = -%.1f * %.6f = %.6f\033[0m", pixel_offset_y, scale_factor_, waypoint_x);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - waypoint_y = -pixel_offset_x * scale_factor_\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - waypoint_y = -%.1f * %.6f = %.6f\033[0m", pixel_offset_x, scale_factor_, waypoint_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - waypoint_z = radius = %.1f\033[0m", waypoint_z);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode] Output Result:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Waypoint Position: (%.6f, %.6f, %.1f)\033[0m", waypoint_x, waypoint_y, waypoint_z);
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode]   - Frame ID: %s\033[0m", msg->header.frame_id.c_str());
            ROS_INFO_THROTTLE(0.5, "\033[35m[CircleDetectorNode] ======================================\033[0m");

            waypoint_mark_pub_.publish(waypoint_mark);
            
            // 发布检测状态
            std_msgs::Bool status_msg;
            status_msg.data = true;
            detection_status_pub_.publish(status_msg);
            
            if (debug_mode_) {
                ROS_INFO_THROTTLE(1.0, "\033[36m[CircleDetectorNode] High-quality circle detected!\033[0m");
                ROS_INFO_THROTTLE(1.0, "\033[36m[CircleDetectorNode] Center: (%.1f, %.1f), Radius: %.1f, Offset: (%.1f, %.1f)\033[0m", 
                                  center_x, center_y, radius, pixel_offset_x, pixel_offset_y);
            }
        } else {
            // 没有检测到有效圆形
            circle_found_ = false;
            
            std_msgs::Bool status_msg;
            status_msg.data = false;
            detection_status_pub_.publish(status_msg);
            
            if (debug_mode_) {
                ROS_INFO_THROTTLE(2.0, "\033[33m[CircleDetectorNode] No valid circle detected.\033[0m");
            }
        }

        // 绘制带质量参数标注的二值化图像
        cv::Mat binary_result_image = drawBinaryResultWithQualityParams(mask, contours, valid_circle_found ? &best_ellipse : nullptr, best_quality_params);
        
        // 发布带质量参数标注的二值化图像
        if (binary_result_pub_.getNumSubscribers() > 0) {
            sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binary_result_image).toImageMsg();
            binary_msg->header.stamp = msg->header.stamp;
            binary_msg->header.frame_id = msg->header.frame_id;
            binary_result_pub_.publish(binary_msg);
        }
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CircleDetectorNode] cv_bridge exception: %s\033[0m", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CircleDetectorNode] Image processing exception: %s\033[0m", e.what());
    }
}

cv::Mat CircleDetectorNode::drawDetectionResult(const cv::Mat& image, const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse) {
    cv::Mat result_image = image.clone();
    
    // 在主图像上绘制所有找到的蓝色轮廓
    cv::drawContours(result_image, contours, -1, cv::Scalar(255, 0, 255), 1);

    if (best_ellipse) {
        // 在图像上绘制拟合的椭圆（绿色）
        cv::ellipse(result_image, *best_ellipse, cv::Scalar(0, 255, 0), 2);
        // 绘制圆心
        cv::circle(result_image, best_ellipse->center, 5, cv::Scalar(0, 0, 255), -1);
        
        // 添加检测信息文本
        cv::putText(result_image, cv::format("Circle: (%.1f, %.1f)", best_ellipse->center.x, best_ellipse->center.y), 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(result_image, cv::format("Radius: %.1f", (best_ellipse->size.width + best_ellipse->size.height) / 4.0), 
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        
        // 显示像素偏差信息
        double pixel_offset_x = best_ellipse->center.x - image_center_.x;
        double pixel_offset_y = best_ellipse->center.y - image_center_.y;
        cv::putText(result_image, cv::format("Offset: (%.1f, %.1f)", pixel_offset_x, pixel_offset_y), 
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    } else {
        // 没有检测到圆形时显示状态
        cv::putText(result_image, "No Circle Detected", 
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
    
    // 将mask转换为彩色图像以便与主图像拼接
    cv::Mat mask_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);

    // 拼接图像
    cv::Mat combined_image;
    cv::hconcat(result_image, mask_bgr, combined_image);

    return combined_image;
}

cv::Mat CircleDetectorNode::drawBinaryResultWithQualityParams(const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse, const std::vector<double>& quality_params) {
    // 将二值化图像转换为彩色图像
    cv::Mat binary_color;
    cv::cvtColor(mask, binary_color, cv::COLOR_GRAY2BGR);
    
    // 在二值化图像上绘制所有轮廓（红色）
    cv::drawContours(binary_color, contours, -1, cv::Scalar(0, 0, 255), 2);
    
    if (best_ellipse && !quality_params.empty()) {
        // 绘制最佳拟合椭圆（绿色）
        cv::ellipse(binary_color, *best_ellipse, cv::Scalar(0, 255, 0), 5);
        
        // 绘制圆心（蓝色）
        cv::circle(binary_color, best_ellipse->center, 8, cv::Scalar(255, 0, 0), -1);
        
        // 绘制半径线（黄色）
        double radius = (best_ellipse->size.width + best_ellipse->size.height) / 4.0;
        cv::line(binary_color, best_ellipse->center, 
                 cv::Point(best_ellipse->center.x + radius, best_ellipse->center.y), 
                 cv::Scalar(0, 255, 255), 2);
        
        // 添加质量参数文本标注
        int y_offset = 30;
        int line_height = 25;
        
        // 标题
        cv::putText(binary_color, "Quality Parameters:", 
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        y_offset += line_height;
        
        // 半径
        if (quality_params.size() > 0) {
            cv::putText(binary_color, cv::format("Radius: %.1f px", quality_params[0]), 
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }
        
        // 宽高比
        if (quality_params.size() > 1) {
            cv::putText(binary_color, cv::format("Aspect Ratio: %.3f", quality_params[1]), 
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }
        
        // 面积
        if (quality_params.size() > 2) {
            cv::putText(binary_color, cv::format("Area: %.1f px^2", quality_params[2]), 
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }
        
        // 轮廓点数
        if (quality_params.size() > 3) {
            cv::putText(binary_color, cv::format("Contour Points: %d", (int)quality_params[3]), 
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }
        
        // 椭圆参数
        cv::putText(binary_color, cv::format("Ellipse: %.1fx%.1f", best_ellipse->size.width, best_ellipse->size.height), 
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        y_offset += line_height;
        
        // 圆心坐标
        cv::putText(binary_color, cv::format("Center: (%.1f, %.1f)", best_ellipse->center.x, best_ellipse->center.y), 
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        
    } else {
        // 没有检测到有效圆形时显示状态
        cv::putText(binary_color, "No Valid Circle Detected", 
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
    int legend_y = binary_color.rows - 100;
    cv::putText(binary_color, "Legend:", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    legend_y += 20;
    cv::putText(binary_color, "Red: All contours", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    legend_y += 15;
    cv::putText(binary_color, "Green: Best ellipse", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    legend_y += 15;
    cv::putText(binary_color, "Blue: Circle center", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
    legend_y += 15;
    cv::putText(binary_color, "Yellow: Radius line", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
    legend_y += 15;
    cv::putText(binary_color, "White: Image center", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    return binary_color;
}



} // namespace patrol_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_detector_node");
    ros::NodeHandle nh;

    patrol_control::CircleDetectorNode detector(nh);
    
    ros::spin();
    
    return 0;
} 