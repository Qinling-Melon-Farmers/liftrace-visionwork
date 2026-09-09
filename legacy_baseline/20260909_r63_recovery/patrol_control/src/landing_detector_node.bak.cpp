#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include "patrol_control/landing_detector_node.h"

namespace patrol_control {

LandingDetectorNode::LandingDetectorNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), detection_enabled_(false), show_image_(true), area_threshold_(1000), 
      aspect_ratio_threshold_(0.7), debug_mode_(true), circle_found_(false), detected_radius_(0.0){
    
    // 加载参数
    loadParameters();
    
    // 初始化相机参数
    initializeCameraParams();
    
    // 订阅
    image_sub_ = it_.subscribe("/iris_mid360/camera/rgb/image_raw", 1, &LandingDetectorNode::imageCallback, this);
    detect_control_sub_ = nh_.subscribe("/detect/landing_control", 1, &LandingDetectorNode::detectionControlCallback, this);
    
    // 发布
    pixel_offset_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/pixel_offset", 1);
    circle_center_pub_ = nh_.advertise<geometry_msgs::Point>("/detect/circle_center", 1);
    detection_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/status", 1);
    // 新增发布器
    waypoint_mark_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detect/land_mark_point", 1);
    binary_result_pub_ = it_.advertise("/detect/landing_binary_result_image/compressed", 1);
    
    ROS_INFO("[LandingDetectorNode] Landing detector node initialized");
}

LandingDetectorNode::~LandingDetectorNode()
{
    image_sub_.shutdown();
    detect_control_sub_.shutdown();
}

void LandingDetectorNode::loadParameters()
{
    nh_.param("area_threshold", area_threshold_, 1000.0);
    nh_.param("show_image", show_image_, true);
    nh_.param("detection_enabled", detection_enabled_, true);
    nh_.param("aspect_ratio_threshold", aspect_ratio_threshold_, 0.7);
    nh_.param("debug_mode", debug_mode_, true);
    nh_.param("detection_control/target_center_x", target_center_x_,640.0);
    nh_.param("detection_control/target_center_y", target_center_y_, 480.0);
    // 新增像素偏差缩放参数
    nh_.param("landing_detection/pixel_offset_scale", pixel_offset_scale_, 0.0008);

    // 使用加载的参数设置图像中心，这将是我们的"靶心"
    image_center_ = cv::Point2f(target_center_x_, target_center_y_);
}

void LandingDetectorNode::initializeCameraParams()
{
    nh_.getParam("camera/fx", camera_fx_);
    nh_.getParam("camera/fy", camera_fy_);
    nh_.getParam("camera/cx", camera_cx_);
    nh_.getParam("camera/cy", camera_cy_);
    nh_.getParam("camera/width", camera_width_);
    nh_.getParam("camera/height", camera_height_);
}

void LandingDetectorNode::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{   
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
    try
    {
        sensor_msgs::Image image = *msg;
        cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv_image = cvimg->image;
        cv::RotatedRect best_ellipse;
        bool ellipse_found = false;
        cv::Point2f center;
        float radius;
        // 1. 彩色图像转灰度图像
        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
 
        // 1.5 灰度范围归一化到0-255
        gray_image.convertTo(gray_image, CV_32F);
        cv::Mat log_image;
        cv::log(gray_image + 1, log_image); // 避免log(0)
        double minVal, maxVal;
        cv::minMaxLoc(log_image, &minVal, &maxVal);
        log_image = 255 * (log_image - minVal) / (maxVal - minVal);
        log_image.convertTo(gray_image, CV_8U);
 
        // 1.6 高斯滤波去噪
        cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
    
        // 2. 使用自适应阈值处理阴影
        cv::Mat adaptive_bin;
        cv::adaptiveThreshold(gray_image, adaptive_bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 31, 10);
        // 3. 形态学操作去除小阴影和噪声
        cv::Mat morph = adaptive_bin;
        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);
        // cv::morphologyEx(adaptive_bin, morph, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);

    
        // 4. 直接输出形态学处理后的结果
        //cv::imshow("H Segment Result", morph);
        //cv::waitKey(1);
    
        // 5. 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<double> quality_params; // 存储质量参数
    
        if (!contours.empty())
        {
            // 找到最大轮廓
            size_t max_idx = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); ++i)
            {
                double area = cv::contourArea(contours[i]);
                
                // 椭圆拟合需要至少5个点
                if (contours[i].size() < 5) continue;
                
                cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);
                double width = ellipse.size.width;
                double height = ellipse.size.height;
                // 避免除以零
                if (width < 1e-3 || height < 1e-3) continue;
                // a. 长宽比判断，越接近1越像圆
                double aspect_ratio = std::min(width, height) / std::max(width, height);
                if (aspect_ratio < aspect_ratio_threshold_) {
                    continue;
                }
                if (area > max_area && area > area_threshold_)
                {
                    max_area = area;
                    max_idx = i;
                    best_ellipse = ellipse;
                    ellipse_found = true;
                    
                    // 收集质量参数
                    quality_params.clear();
                    quality_params.push_back((width + height) / 4.0);  // 半径
                    quality_params.push_back(aspect_ratio);  // 长宽比
                    quality_params.push_back(area);  // 面积
                    quality_params.push_back(contours[i].size());  // 轮廓点数
                }
            }
            
            if (max_area > area_threshold_ && ellipse_found) {
                // 计算最小外接圆
                
                cv::minEnclosingCircle(contours[max_idx], center, radius);
            }
        }
        if (ellipse_found) {
            // 使用质量最好的圆形
            double center_x = best_ellipse.center.x;
            double center_y = best_ellipse.center.y;
            double radius = (best_ellipse.size.width + best_ellipse.size.height) / 4.0;
            
            // 计算像素偏差（相对于图像中心）
            double pixel_offset_x = center_x - image_center_.x;
            double pixel_offset_y = center_y - image_center_.y;
            ROS_INFO_THROTTLE(1.0, "\033[36m[CircleDetectorNode] Center: (%.1f, %.1f), Radius: %.1f, image_center: (%.1f, %.1f)\033[0m", 
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

            // 计算waypoint_mark（参考cross_detector_node的逻辑）
            double waypoint_x = -pixel_offset_y * pixel_offset_scale_;
            double waypoint_y = -pixel_offset_x * pixel_offset_scale_;
            double waypoint_z = sqrt(radius * radius / M_PI) * 4.34; // 使用面积计算等效半径

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
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode] ===== Waypoint Mark Calculation =====\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode] Input Parameters:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Circle Center: (%.1f, %.1f)\033[0m", center_x, center_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Image Center: (%.1f, %.1f)\033[0m", image_center_.x, image_center_.y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Pixel Offset: (%.1f, %.1f)\033[0m", pixel_offset_x, pixel_offset_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Scale Factor: %.6f\033[0m", pixel_offset_scale_);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Circle Radius: %.1f\033[0m", radius);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode] Calculation Process:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - waypoint_x = -pixel_offset_y * scale_factor_\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - waypoint_x = -%.1f * %.6f = %.6f\033[0m", pixel_offset_y, pixel_offset_scale_, waypoint_x);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - waypoint_y = -pixel_offset_x * scale_factor_\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - waypoint_y = -%.1f * %.6f = %.6f\033[0m", pixel_offset_x, pixel_offset_scale_, waypoint_y);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - waypoint_z = radius = %.1f\033[0m", waypoint_z);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode] Output Result:\033[0m");
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Waypoint Position: (%.6f, %.6f, %.1f)\033[0m", waypoint_x, waypoint_y, waypoint_z);
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode]   - Frame ID: %s\033[0m", msg->header.frame_id.c_str());
            ROS_INFO_THROTTLE(0.5, "\033[35m[LandingDetectorNode] ======================================\033[0m");

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
        cv::Mat binary_result_image = drawBinaryResultWithQualityParams(morph, contours, ellipse_found ? &best_ellipse : nullptr, quality_params);
        
        // 发布带质量参数标注的二值化图像
        if (binary_result_pub_.getNumSubscribers() > 0) {
            sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binary_result_image).toImageMsg();
            binary_msg->header.stamp = msg->header.stamp;
            binary_msg->header.frame_id = msg->header.frame_id;
            binary_result_pub_.publish(binary_msg);
        }
        
        show_image_ = false;
        // 显示检测结果图像
        if (show_image_) {
            drawDetectionResult(cv_image, morph, contours, ellipse_found ? &best_ellipse : nullptr);
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[LandingDetectorNode] cv_bridge exception: %s\033[0m", e.what());
    } 
    catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(5, "\033[31m[LandingDetectorNode] Image processing exception: %s\033[0m", e.what());
    }
}

void LandingDetectorNode::drawDetectionResult(cv::Mat& image, const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse)
{  
    // 在主图像上绘制所有找到的黑色轮廓（紫色）
    cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 255), 1);

    if (best_ellipse) {
        // 在图像上绘制拟合的椭圆（绿色）
        cv::ellipse(image, *best_ellipse, cv::Scalar(0, 255, 0), 2);
        // 绘制圆心（红色）
        cv::circle(image, best_ellipse->center, 5, cv::Scalar(0, 0, 255), -1);
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
    cv::imshow("Landing Detection Result (Left) & Landing Color Mask (Right)", combined_image);
    cv::waitKey(1);
}
void LandingDetectorNode::detectionControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    static bool last_state = false;
    static bool first_call = true;
    
    bool new_state = msg->data;
    
    // 只在状态变化时输出提示
    if (first_call || new_state != last_state) {
        if (new_state) {
            ROS_INFO("\033[32m[LandingDetectorNode] Landing Detection ENABLED by control topic.\033[0m");
        } else {
            ROS_INFO("\033[33m[LandingDetectorNode] Landing Detection DISABLED by control topic.\033[0m");
        }
        last_state = new_state;
        first_call = false;
    }
    
    detection_enabled_ = new_state;
}

cv::Mat LandingDetectorNode::drawBinaryResultWithQualityParams(const cv::Mat& mask, const std::vector<std::vector<cv::Point>>& contours, const cv::RotatedRect* best_ellipse, const std::vector<double>& quality_params) {
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
        cv::putText(binary_color, "Landing Circle Quality Parameters:", 
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        y_offset += line_height;
        
        // 半径
        if (quality_params.size() > 0) {
            cv::putText(binary_color, cv::format("Radius: %.1f px", quality_params[0]), 
                        cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
        }
        
        // 长宽比
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
        cv::putText(binary_color, "No Valid Landing Circle Detected", 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(binary_color, "Check area and aspect ratio thresholds", 
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_detector_node");
    ros::NodeHandle nh;
    patrol_control::LandingDetectorNode detector(nh);
    ros::spin();
    return 0;
}
