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

namespace patrol_control {

CircleDetectorNode::CircleDetectorNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), detection_enabled_(false), detection_enabled_prev_(false), circle_found_(false),
      red_cross_found_(false) {
    
    // 加载参数
    loadParameters();
    
    // 初始化相机参数
    initializeCameraParams();
    
    // 订阅图像话题
    image_sub_ = it_.subscribe("/iris_mid360/camera/rgb/image_raw", 1, &CircleDetectorNode::imageCallback, this);
    
    // 订阅检测控制话题
    detect_control_sub_ = nh_.subscribe("/detect/control", 1, &CircleDetectorNode::detectionControlCallback, this);
    
    // 发布像素偏差
    pixel_offset_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/pixel_offset", 1);
    
    // 发布检测状态
    detection_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/status", 1);
    
    
    ROS_INFO("\033[32m[CircleDetectorNode] Color-based Circle Detector Initialized\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Subscribing to image topic: /iris_mid360/camera/rgb/image_raw\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Subscribing to control topic: /detect/control\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Publishing pixel offset to: /detect/pixel_offset\033[0m");
    ROS_INFO("\033[32m[CircleDetectorNode] Publishing detection status to: /detect/status\033[0m");
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
    nh_.param("color_segmentation/s_min", s_min_, 40);
    nh_.param("color_segmentation/v_min", v_min_, 40);
    nh_.param("color_segmentation/h_max", h_max_, 130);
    nh_.param("color_segmentation/s_max", s_max_, 255);
    nh_.param("color_segmentation/v_max", v_max_, 255);
    
    // 红色十字检测参数 (新增)
    nh_.param("red_cross_detection/enable", enable_red_cross_detection_, false);
    nh_.param("red_cross_detection/h_min", red_h_min_, 0);
    nh_.param("red_cross_detection/s_min", red_s_min_, 120);
    nh_.param("red_cross_detection/v_min", red_v_min_, 70);
    nh_.param("red_cross_detection/h_max", red_h_max_, 10);
    nh_.param("red_cross_detection/s_max", red_s_max_, 255);
    nh_.param("red_cross_detection/v_max", red_v_max_, 255);
    
    // 红色十字形状验证参数
    nh_.param("cross_validation/aspect_ratio_threshold", cross_aspect_ratio_threshold_, 0.4);
    nh_.param("cross_validation/min_contour_points", cross_min_contour_points_, 30);
    nh_.param("cross_validation/area_threshold", cross_area_threshold_, 200.0);
    nh_.param("cross_validation/solidity_threshold", cross_solidity_threshold_, 0.6);
    
    // 质量评估参数 (基于轮廓和椭圆拟合)
    nh_.param("quality_assessment/min_contour_points", min_contour_points_, 15);
    nh_.param("quality_assessment/aspect_ratio_threshold", aspect_ratio_threshold_, 0.7); // 更严格，因为我们期望是圆
    
    // 图像预处理参数
    nh_.param("image_preprocessing/enable_gaussian_blur", enable_gaussian_blur_, true);
    nh_.param("image_preprocessing/blur_kernel_size", blur_kernel_size_, 5);
    
    // 检测控制参数
    nh_.param("detection/enabled", detection_enabled_, true);
    nh_.param("detection/debug", debug_mode_, false);
    nh_.param("detection_control/show_image", show_image_, true);
    nh_.param("detection_control/max_fps", max_fps_, 15.0);

    // 新增: 加载自定义的目标中心点
    nh_.param("detection_control/target_center_x", target_center_x_, 424.0);
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
    if(enable_red_cross_detection_) {
        ROS_INFO("\033[34m[CircleDetectorNode] Red Cross Detection (HSV): H[%d, %d], S[%d, %d], V[%d, %d]\033[0m",
                 red_h_min_, red_h_max_, red_s_min_, red_s_max_, red_v_min_, red_v_max_);
        ROS_INFO("\033[34m[CircleDetectorNode] Cross Validation: aspect_ratio=%.2f, min_points=%d, area=%.1f, solidity=%.2f\033[0m",
                 cross_aspect_ratio_threshold_, cross_min_contour_points_, cross_area_threshold_, cross_solidity_threshold_);
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
        }
        last_state = new_state;
        first_call = false;
    }
    
    detection_enabled_ = new_state;
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

        // --- 核心检测逻辑 ---
        
        // 1. 色彩分割
        cv::Mat hsv_image, mask;
        cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);

        // 形态学操作，去除噪点，连接断裂区域
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 2. 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool valid_circle_found = false;
        cv::RotatedRect best_ellipse;
        double best_area = 0;

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
            
            // 更新检测状态
            circle_found_ = true;
            detected_center_ = best_ellipse.center;
            detected_radius_ = radius;
            
            // 发布像素偏差
            geometry_msgs::Point pixel_offset;
            pixel_offset.x = pixel_offset_x;
            pixel_offset.y = pixel_offset_y;
            pixel_offset.z = radius;
            pixel_offset_pub_.publish(pixel_offset);
            
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
        
        // --- 红色十字检测 (新增) ---
        
        bool red_cross_detected = false;
        cv::Point2f cross_center;
        double cross_area;
        
        if (enable_red_cross_detection_) {
            red_cross_detected = detectRedCross(processed_image, cross_center, cross_area);
            
            if (red_cross_detected) {
                // 计算像素偏差（相对于图像中心）
                double pixel_offset_x = cross_center.x - image_center_.x;
                double pixel_offset_y = cross_center.y - image_center_.y;
                
                // 发布像素偏差 (优先级高于蓝色圆形检测)
                geometry_msgs::Point pixel_offset;
                pixel_offset.x = pixel_offset_x;
                pixel_offset.y = pixel_offset_y;
                pixel_offset.z = sqrt(cross_area / M_PI); // 使用面积计算等效半径
                pixel_offset_pub_.publish(pixel_offset);
                
                // 发布检测状态
                std_msgs::Bool status_msg;
                status_msg.data = true;
                detection_status_pub_.publish(status_msg);
                
                if (debug_mode_) {
                    ROS_INFO_THROTTLE(1.0, "\033[35m[CircleDetectorNode] Red cross detected!\033[0m");
                    ROS_INFO_THROTTLE(1.0, "\033[35m[CircleDetectorNode] Center: (%.1f, %.1f), Area: %.1f, Offset: (%.1f, %.1f)\033[0m", 
                                      cross_center.x, cross_center.y, cross_area, pixel_offset_x, pixel_offset_y);
                }
            }
        }
        
        // 如果没有检测到任何目标，发布false状态
        if (!valid_circle_found && !red_cross_detected) {
            std_msgs::Bool status_msg;
            status_msg.data = false;
            detection_status_pub_.publish(status_msg);
        }
        
        // 显示检测结果图像
        if (show_image_) {
            drawDetectionResult(image, mask, contours, valid_circle_found ? &best_ellipse : nullptr);
        }
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CircleDetectorNode] cv_bridge exception: %s\033[0m", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[CircleDetectorNode] Image processing exception: %s\033[0m", e.what());
    }
}

void CircleDetectorNode::drawDetectionResult(cv::Mat& image, const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse) {
    
    // 在主图像上绘制所有找到的蓝色轮廓
    cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 255), 1);

    if (best_ellipse) {
        // 在图像上绘制拟合的椭圆（绿色）
        cv::ellipse(image, *best_ellipse, cv::Scalar(0, 255, 0), 2);
        // 绘制圆心
        cv::circle(image, best_ellipse->center, 5, cv::Scalar(0, 0, 255), -1);
    }
    
    // 绘制红色十字检测结果
    if (red_cross_found_) {
        // 绘制十字中心点（红色）
        cv::circle(image, red_cross_center_, 8, cv::Scalar(0, 0, 255), -1);
        // 绘制十字标记
        int cross_size = 15;
        cv::line(image, 
                 cv::Point(red_cross_center_.x - cross_size, red_cross_center_.y), 
                 cv::Point(red_cross_center_.x + cross_size, red_cross_center_.y), 
                 cv::Scalar(0, 0, 255), 3);
        cv::line(image, 
                 cv::Point(red_cross_center_.x, red_cross_center_.y - cross_size), 
                 cv::Point(red_cross_center_.x, red_cross_center_.y + cross_size), 
                 cv::Scalar(0, 0, 255), 3);
        
        // 添加文本标签
        cv::putText(image, "Red Cross", 
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        
        // 显示面积信息
        cv::putText(image, cv::format("Area: %.1f", red_cross_area_), 
                    cv::Point(red_cross_center_.x + 10, red_cross_center_.y + 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
    
    // 绘制图像中心点（蓝色）
    cv::circle(image, image_center_, 5, cv::Scalar(255, 0, 0), -1);
    
    // 将mask转换为彩色图像以便与主图像拼接
    cv::Mat mask_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);

    // 拼接图像
    cv::Mat combined_image;
    cv::hconcat(image, mask_bgr, combined_image);

    // 显示图像
    cv::imshow("Detection Result (Left) & Color Mask (Right)", combined_image);
    cv::waitKey(1);
}

// 红色十字检测函数
bool CircleDetectorNode::detectRedCross(const cv::Mat& image, cv::Point2f& cross_center, double& cross_area) {
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
bool CircleDetectorNode::validateCrossShape(const std::vector<cv::Point>& contour, cv::Point2f& center, double& area) {
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
bool CircleDetectorNode::isCrossLikeShape(const std::vector<cv::Point>& contour) {
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
double CircleDetectorNode::calculateSolidity(const std::vector<cv::Point>& contour) {
    double contour_area = cv::contourArea(contour);
    
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hull_area = cv::contourArea(hull);
    
    if (hull_area > 0) {
        return contour_area / hull_area;
    }
    
    return 0.0;
}

} // namespace patrol_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_detector_node");
    ros::NodeHandle nh;

    patrol_control::CircleDetectorNode detector(nh);
    
    ros::spin();
    
    return 0;
} 