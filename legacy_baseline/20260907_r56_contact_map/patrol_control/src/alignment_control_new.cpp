#include "patrol_control/alignment_control_converter.h"
#include <cmath>
#include <tf/transform_datatypes.h>

namespace patrol_control {

AlignmentControlConverter::AlignmentControlConverter(ros::NodeHandle& nh) : nh_(nh) {
    // 加载参数
    nh_.param("pixel_to_meter_ratio", pixel_to_m_ratio_, 0.0015);
    nh_.param("max_movement_distance", max_movement_distance_, 0.5);
    
    // 加载渐进降高参数
    nh_.param("progressive_descent/enable", progressive_descent_enable_, true);
    nh_.param("progressive_descent/target_height", progressive_descent_target_height_, 0.8);
    nh_.param("progressive_descent/descent_duration", progressive_descent_duration_, 5.0);
    nh_.param("progressive_descent/min_detection_count", progressive_descent_min_detection_count_, 5);
    
    // 初始化渐进降高状态变量
    descent_started_ = false;
    detection_count_ = 0;
    first_detection_ = true;
    initial_height_ = 0.0;
    
    // 初始化订阅者和发布者
    pixel_offset_sub_ = nh_.subscribe("/detect/pixel_offset", 1, &AlignmentControlConverter::pixelOffsetCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &AlignmentControlConverter::odomCallback, this);
    detection_control_sub_ = nh_.subscribe("/detect/control", 1, &AlignmentControlConverter::detectionControlCallback, this);
    target_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detect/waypoint_mark", 1);

    ROS_INFO("\033[1;32m[AlignmentConverter] Node initialized successfully\033[0m");
    ROS_INFO("[AlignmentConverter] pixel_to_m_ratio: %f", pixel_to_m_ratio_);
    ROS_INFO("[AlignmentConverter] max_movement_distance: %f", max_movement_distance_);
    ROS_INFO("[AlignmentConverter] Progressive descent enabled: %s", progressive_descent_enable_ ? "true" : "false");
    if (progressive_descent_enable_) {
        ROS_INFO("[AlignmentConverter] Progressive descent target height: %.2f m", progressive_descent_target_height_);
        ROS_INFO("[AlignmentConverter] Progressive descent duration: %.1f s", progressive_descent_duration_);
        ROS_INFO("[AlignmentConverter] Min detection count: %d", progressive_descent_min_detection_count_);
    }
    ROS_INFO("[AlignmentConverter] Waiting for data input...");
}

void AlignmentControlConverter::detectionControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    bool prev_state = is_detection_active_;
    is_detection_active_ = msg->data;
    
    // 只在状态变化时输出
    if (prev_state != is_detection_active_) {
        if (is_detection_active_) {
            ROS_INFO("\033[1;32m[AlignmentConverter] Detection ENABLED - Listening for pixel offsets\033[0m");
            // 重置渐进降高状态
            descent_started_ = false;
            detection_count_ = 0;
            first_detection_ = true;
        } else {
            ROS_INFO("\033[1;33m[AlignmentConverter] Detection DISABLED\033[0m");
        }
    }
}

void AlignmentControlConverter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_uav_pose_.header = msg->header;
    current_uav_pose_.pose = msg->pose.pose;
    
    if (!uav_pose_received_) {
        uav_pose_received_ = true;
        ROS_INFO("\033[1;36m[AlignmentConverter] First UAV position received\033[0m");
        ROS_INFO("[AlignmentConverter] Current position: (%.3f, %.3f, %.3f)", 
                 msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
    
    // 定期输出无人机位置（每3秒一次，避免刷屏）
    ROS_INFO_THROTTLE(3.0, "[AlignmentConverter] UAV Position: (%.3f, %.3f, %.3f)", 
                      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void AlignmentControlConverter::pixelOffsetCallback(const geometry_msgs::Point::ConstPtr& msg) {
    // 显示接收到的像素偏差（每次都显示，这是关键信息）
    ROS_INFO("[AlignmentConverter] Received pixel offset: (%.1f, %.1f, radius=%.1f)", msg->x, msg->y, msg->z);
    
    if (!is_detection_active_) {
        ROS_WARN_THROTTLE(2.0, "[AlignmentConverter] Detection not active, skipping processing");
        return;
    }
    
    if (!uav_pose_received_) {
        ROS_WARN_THROTTLE(2.0, "[AlignmentConverter] No UAV position received, skipping processing");
        return;
    }

    // 增加检测计数
    detection_count_++;
    
    // 获取像素误差和圆环半径
    double pixel_error_x = msg->x;  // 图像左右误差
    double pixel_error_y = msg->y;  // 图像上下误差
    double circle_radius_pixels = msg->z;  // 检测到的圆环半径（像素）

    // 动态计算像素到米的转换比例
    // 假设真实圆环直径为1米，则半径为0.5米
    double real_circle_radius = 0.5;  // 米
    double dynamic_pixel_to_meter_ratio;
    
    if (circle_radius_pixels > 10.0) {  // 确保半径有效
        dynamic_pixel_to_meter_ratio = real_circle_radius / circle_radius_pixels;
    } else {
        // 如果半径无效，使用默认比例
        dynamic_pixel_to_meter_ratio = pixel_to_m_ratio_;
        ROS_WARN_THROTTLE(1.0, "[AlignmentConverter] Invalid circle radius, using default ratio");
    }

    // 转换为米制误差
    double meter_error_x = pixel_error_x * dynamic_pixel_to_meter_ratio;
    double meter_error_y = pixel_error_y * dynamic_pixel_to_meter_ratio;

    // 获取无人机当前位置和姿态
    double uav_x = current_uav_pose_.pose.position.x;
    double uav_y = current_uav_pose_.pose.position.y;
    double uav_z = current_uav_pose_.pose.position.z;
    double uav_yaw = tf::getYaw(current_uav_pose_.pose.orientation);

    // 渐进降高逻辑
    double target_z = uav_z;  // 默认保持当前高度
    
    if (progressive_descent_enable_) {
        // 第一次检测到圆环，记录初始高度
        if (first_detection_) {
            initial_height_ = uav_z;
            first_detection_ = false;
            ROS_INFO("\033[1;34m[AlignmentConverter] First circle detection! Initial height: %.2f m\033[0m", initial_height_);
        }
        
        // 检测次数达到阈值后开始渐进降高
        if (!descent_started_ && detection_count_ >= progressive_descent_min_detection_count_) {
            descent_started_ = true;
            descent_start_time_ = ros::Time::now();
            ROS_INFO("\033[1;32m[AlignmentConverter] Progressive descent started! Target: %.2f m, Duration: %.1f s\033[0m", 
                     progressive_descent_target_height_, progressive_descent_duration_);
        }
        
        // 如果已开始渐进降高，计算当前目标高度
        if (descent_started_) {
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - descent_start_time_).toSec();
            
            if (elapsed_time <= progressive_descent_duration_) {
                // 线性插值计算当前目标高度
                double progress = elapsed_time / progressive_descent_duration_;
                target_z = initial_height_ + (progressive_descent_target_height_ - initial_height_) * progress;
                
                ROS_INFO_THROTTLE(0.5, "\033[1;36m[ProgressiveDescent] Progress: %.1f%%, Target Height: %.2f m\033[0m", 
                                  progress * 100.0, target_z);
            } else {
                // 降高完成，保持目标高度
                target_z = progressive_descent_target_height_;
            }
        }
    }

    // 简化的坐标系转换：直接从像素误差转换到世界坐标偏移
    // 假设无人机姿态不变，直接进行坐标映射
    // 
    // 对准逻辑（基于实际测试反馈调整）：
    // - 当目标在图像右侧(pixel_error_x > 0)时，无人机需要向世界坐标Y+方向移动
    // - 当目标在图像下方(pixel_error_y > 0)时，无人机需要向世界坐标X+方向移动
    
    // 直接映射到世界坐标系偏移（修正版本）
    double world_offset_x = -meter_error_y;   // 图像Y+ -> 世界X-
    double world_offset_y = -meter_error_x;   // 图像X+ -> 世界Y-

    // 计算目标点（使用渐进降高的目标高度）
    double target_x = uav_x + world_offset_x;
    double target_y = uav_y + world_offset_y;

    // 安全限制：限制单次移动距离
    double move_distance = sqrt(world_offset_x * world_offset_x + world_offset_y * world_offset_y);
    if (move_distance > max_movement_distance_) {
        double scale = max_movement_distance_ / move_distance;
        target_x = uav_x + world_offset_x * scale;
        target_y = uav_y + world_offset_y * scale;
        ROS_WARN("[AlignmentConverter] Movement distance limited to %.2f meters", max_movement_distance_);
    }

    // 构造并发布目标点消息
    geometry_msgs::PoseStamped target_msg;
    target_msg.header.stamp = ros::Time::now();
    target_msg.header.frame_id = current_uav_pose_.header.frame_id;
    
    target_msg.pose.position.x = target_x;
    target_msg.pose.position.y = target_y;
    target_msg.pose.position.z = target_z;  // 使用渐进降高的目标高度
    target_msg.pose.orientation = current_uav_pose_.pose.orientation;

    target_point_pub_.publish(target_msg);

    // 关键的对准信息输出（每次都显示）
    ROS_INFO("\033[1;35m[ALIGNMENT] UAV(%.3f,%.3f,%.3f) | PixelErr(%.1f,%.1f) | Target(%.3f,%.3f,%.3f) | DetectCount:%d\033[0m",
        uav_x, uav_y, uav_z, pixel_error_x, pixel_error_y, target_x, target_y, target_z, detection_count_);
    
    // 详细计算过程（降低频率避免刷屏）
    ROS_INFO_THROTTLE(0.5, "[DETAIL] Radius=%.1fpx | DynamicRatio=%.6f | MeterErr(%.4f,%.4f) | WorldOffset(%.4f,%.4f)",
        circle_radius_pixels, dynamic_pixel_to_meter_ratio, meter_error_x, meter_error_y, world_offset_x, world_offset_y);
}

} // namespace patrol_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "alignment_control_converter");
    ros::NodeHandle nh;
    
    patrol_control::AlignmentControlConverter converter(nh);
    
    ros::spin();
    
    return 0;
} 