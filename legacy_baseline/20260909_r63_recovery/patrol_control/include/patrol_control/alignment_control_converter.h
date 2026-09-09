#ifndef ALIGNMENT_CONTROL_CONVERTER_H
#define ALIGNMENT_CONTROL_CONVERTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <cmath>

namespace patrol_control {

class AlignmentControlConverter {
public:
    AlignmentControlConverter(ros::NodeHandle& nh);

private:
    enum class DetectionType {
        CIRCLE = 0,
        CROSS = 1,
        LANDING = 2
    };

    DetectionType current_detection_type_;
    void pixelOffsetCallback(const geometry_msgs::Point::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void detectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void CircleDetectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void CrossDetectionControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void circleCenterCallback(const geometry_msgs::Point::ConstPtr& msg);
    void servoCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // 十字检测相关回调函数
    void crossPixelOffsetCallback(const geometry_msgs::Point::ConstPtr& msg);
    void crossCenterCallback(const geometry_msgs::Point::ConstPtr& msg);
    void crossStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    void servoStatusCallback(const std_msgs::Bool::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber pixel_offset_sub_;
    ros::Subscriber circle_center_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber circle_detection_control_sub_;
    ros::Subscriber cross_detection_control_sub_;
    ros::Subscriber landing_detection_control_sub_;
    ros::Subscriber servo_sub_;
    ros::Subscriber servo_status_sub_;
    
    // 十字检测相关订阅者和发布者
    ros::Subscriber cross_pixel_offset_sub_;
    ros::Subscriber cross_center_sub_;
    ros::Subscriber cross_status_sub_;
    ros::Publisher cross_control_pub_;
    ros::Publisher servo_marky_pub_;
    
    ros::Publisher target_point_pub_;

    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::Point circle_center_;
    ros::Time circle_center_timestamp_;  // 圆环中心数据的时间戳
    std_msgs::Bool servo_marky;
    std_msgs::Bool servo_status;
    bool uav_pose_received_ = false;
    bool circle_is_detection_active_ = false;
    bool cross_is_detection_active_ = false;
    bool land_is_detection_active_ = false;
    bool servo_mark = false;
    
    // 十字检测相关状态变量
    geometry_msgs::Point cross_center_;
    geometry_msgs::Point cross_pixel_offset_;
    bool cross_detection_active_ = false;
    bool cross_found_ = false;

    double pixel_to_m_ratio_;
    double max_movement_distance_;
    double temp_x = 0.0;
    double temp_y = 0.0;
    ros::Time last_check_time_;
    ros::Time current_time_;
    // 渐进降高相关参数
    bool progressive_descent_enable_;
    double progressive_descent_target_height_;
    double progressive_descent_duration_;
    int progressive_descent_min_detection_count_;
    double land_height_[3] = {0.7, 0.2, 0.3};

    
    // 状态变量
    bool dynamic_state;
    bool temp_mark = false;
    int count_servo_mark;
    int count1 = 1;

    
    // 渐进降高状态变量
    bool descent_started_;
    ros::Time descent_start_time_;
    double initial_height_;
    int detection_count_;
    bool first_detection_;
};

} // namespace patrol_control

#endif // ALIGNMENT_CONTROL_CONVERTER_H 