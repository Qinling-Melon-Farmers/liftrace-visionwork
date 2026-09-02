/**
 *  @file patrol_control.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief 本程序为无人机巡检流程控制程序，可以修改指定yaml，修改路径点
 *  @version 0.2
 *  @date 5-16-2025
 */
#include "patrol_control/patrol_control.h"
#include "patrol_control/Servo.h"
#include <tf/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>
#include <vector>
 
int times_detect = 0;  
bool flag_takeoff_done = 0;

// for detect
bool have_planner_cmd = false, flag_land = false, have_waypoint_mark = false, have_land_mark = false, have_cross_mark = false;
ros::Time start_time;

geometry_msgs::PoseStamped uav_pose;
geometry_msgs::PoseStamped last_mavros_point_cmd;
//for take off
Eigen::Vector3f takeoff_point;

//last and now pub point
Eigen::Vector3f last_pub_point={0,0,0};
Eigen::Vector3f now_pub_point;

/** newest position **/
Eigen::Vector3f uav_newest_position;
/** used to save the adjustment target position **/
Eigen::Vector4f adjust_target_position;


namespace patrol_control {

namespace {

bool readNumber(const XmlRpc::XmlRpcValue& value, double* output) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        *output = static_cast<double>(value);
        return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        *output = static_cast<int>(value);
        return true;
    }
    return false;
}

bool loadSlotOffsets(ros::NodeHandle& nh, const std::string& param_name,
                     std::array<std::array<double, 2>, 3>* offsets) {
    XmlRpc::XmlRpcValue value;
    if (!nh.getParam(param_name, value)) {
        return true;
    }
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) {
        ROS_ERROR("[DropSystem] %s must contain three [x, y] pairs",
                  param_name.c_str());
        return false;
    }
    for (int slot = 0; slot < 3; ++slot) {
        if (value[slot].getType() != XmlRpc::XmlRpcValue::TypeArray ||
            value[slot].size() != 2 ||
            !readNumber(value[slot][0], &(*offsets)[slot][0]) ||
            !readNumber(value[slot][1], &(*offsets)[slot][1])) {
            ROS_ERROR("[DropSystem] %s slot %d must be numeric [x, y]",
                      param_name.c_str(), slot + 1);
            return false;
        }
    }
    return true;
}

}  // namespace

LLController::LLController(ros::NodeHandle nh):nh_(nh) {   
    initializeNode();
    
    // 默认禁用圆形检测
    std_msgs::Bool detect_disable_msg;
    detect_disable_msg.data = false;
    detect_control_pub_.publish(detect_disable_msg);
    
    std::cout << "\033[47;30m ---------------------------------- Start mission ---------------------------------- \033[0m" << std::endl;
}
LLController::~LLController(){}


// 读取航路点，订阅无人机位置、圆环中心，发布目标点、投递装置动作指令
void LLController::initializeNode() {

    load_params();

    waypoint_now = -1;
    waypoint_next = 0;
    uav_pose = geometry_msgs::PoseStamped();
    uav_pose.pose.position.x = 0.0;
    uav_pose.pose.position.y = 0.0;
    uav_pose.pose.position.z = 0.0;
    uav_pose.pose.orientation.x = 0.0;
    uav_pose.pose.orientation.y = 0.0;
    uav_pose.pose.orientation.z = 0.0;
    uav_pose.pose.orientation.w = 1.0;

    control_ready_pub_ =
        nh_.advertise<std_msgs::Bool>(control_ready_topic_, 1, true);
    publishControlReady(false);

    // 订阅无人机当前位置
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,&LLController::positionCallback, this);
    fastplanner_cmd_sub_ = nh_.subscribe("/fastplanner/setpoint_position/local", 1,&LLController::plannercmdCallback, this);
    mavros_point_cmd_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);//px4 直接接收
    detect_sub_ = nh_.subscribe("/detect/waypoint_mark_point", 1,&LLController::waypointMarkCallback, this);
    cross_mark_sub_ = nh_.subscribe("/detect/cross_mark_point", 1,&LLController::crossMarkCallback, this);
    class_sub_ = nh_.subscribe("/yolo_detect", 1,&LLController::ClassCallback, this);
    land_client = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    servo_marky_sub_ = nh_.subscribe("/detect/servo_complete", 1,&LLController::servoMarkyCallback, this);
    servo_status_pub_ = nh_.advertise<std_msgs::Bool>("/detect/servo_status", 1);
    land_mark_sub_ = nh_.subscribe("/detect/land_mark_point", 1,&LLController::landMarkCallback, this);
    //send goal to planner
    //setplanner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planner_planner/goal_position", 1);
    if (!external_mission_mode_) {
        setplanner_goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/fastplanner/goal", 1);
    } else {
        ROS_INFO("[PatrolControl] External mission mode active; planner goal publisher disabled");
    }
    servo_complete_sub_ = nh_.subscribe("/servo/complete", 1,&LLController::servoCompleteCallback, this);
    class_control_pub_ = nh_.advertise<std_msgs::Bool>("/detect/class_control", 1);
    tank_control_pub_ = nh_.advertise<std_msgs::Bool>("/detect/tank_control",1);
    tank_status_sub_ = nh_.subscribe("/detect/tank_status", 1,&LLController::TankStatusCallback,this);
    // detection_status_sub_ = nh_.subscribe<const std_msgs::Bool&>("/detect/cross_status", 1,&LLController::CrossStatusCallback, this);
    // 设置px4工作模式 land 
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    cmd_timer = nh_.createTimer(ros::Duration(0.05), &LLController::cmdCallback, this);
    
    // 发布检测控制话题
    detect_control_pub_ = nh_.advertise<std_msgs::Bool>("/detect/control", 1);
    point_class_pub_ = nh_.advertise<std_msgs::Int8>("/detect/point_class",1);
    
    // 发布降落检测控制话题
    landing_detect_control_pub_ = nh_.advertise<std_msgs::Bool>("/detect/landing_control", 1);
    align_mode_pub_ = nh_.advertise<std_msgs::String>("/uav_vision/align_mode", 1);
    
    // 初始化舵机控制发布器
    servo1_pub_ = nh_.advertise<std_msgs::Bool>("/control1", 1);
    servo2_pub_ = nh_.advertise<std_msgs::Bool>("/control2", 1);
    servo3_pub_ = nh_.advertise<std_msgs::Bool>("/control3", 1);

    servo_client = nh_.serviceClient<patrol_control::Servo>("Servo");
    
    // 订阅对准反馈话题（从 alignment_control_converter 获取像素偏差）
    // alignment_feedback_sub_ = nh_.subscribe("/detect/pixel_offset", 1, &LLController::alignmentFeedbackCallback, this);
    
    // 十字检测相关订阅者和发布者
    // cross_pixel_offset_sub_ = nh_.subscribe("/detect/cross_pixel_offset", 1, &LLController::crossPixelOffsetCallback, this);
    // cross_center_sub_ = nh_.subscribe("/detect/cross_center", 1, &LLController::crossCenterCallback, this);
    cross_status_sub_ = nh_.subscribe("/detect/cross_status", 1, &LLController::crossStatusCallback, this);
    cross_control_pub_ = nh_.advertise<std_msgs::Bool>("/cross/control", 1);
    selected_target_sub_ = nh_.subscribe("/uav_vision/selected_target", 1,
                                         &LLController::selectedTargetCallback, this);
    drop_offset_sub_ = nh_.subscribe("/uav_vision/drop_offset", 1,
                                     &LLController::dropOffsetCallback, this);
    drop_ready_sub_ = nh_.subscribe("/uav_vision/drop_ready", 1,
                                    &LLController::dropReadyCallback, this);
    mission_release_permission_sub_ = nh_.subscribe(
        mission_release_permission_topic_, 1,
        &LLController::missionReleasePermissionCallback, this);
    mission_command_sub_ = nh_.subscribe(
        mission_command_topic_, 4, &LLController::missionCommandCallback, this);
    
    // 初始化投递相关变量
    detect_point_counter = 0;
    drop_condition_met = false;
    current_pixel_error = 1000.0;
    descent_completed = false;
    final_target_height = 0.0;
    last_target_height_ = 0.0;
    last_check_time_ = ros::Time::now();
    
    ROS_INFO("\033[32m[DropSystem] Drop system initialized with detect_point_counter: %d\033[0m", detect_point_counter);
    
    // 确保drop_enabled已经从参数中加载
    if (!drop_enabled) {
        ROS_WARN("\033[33m[DropSystem] Drop system is disabled in configuration\033[0m");
    }
    
    // 根据配置的航路点数量初始化投递完成状态
    int detect_point_count = 0;
    for (const auto& wp : waypoint_list) {
        if (wp.pointmode == "Detect_point") {
            detect_point_count++;
        }
    }
    drop_completed.resize(detect_point_count, false);
    detect_enable_msg_temp.data = false;
    ROS_INFO("\033[32m[DropSystem] Initialized drop system for %d detect points\033[0m", detect_point_count);
    ROS_INFO("\033[36m[PatrolControl] Cross detection interface initialized\033[0m");
}

void LLController::positionCallback(const geometry_msgs::PoseStamped& msg) {
    uav_pose = msg;
    double qn = std::sqrt(msg.pose.orientation.x * msg.pose.orientation.x +
                          msg.pose.orientation.y * msg.pose.orientation.y +
                          msg.pose.orientation.z * msg.pose.orientation.z +
                          msg.pose.orientation.w * msg.pose.orientation.w);
    if (qn > 1e-9) {
        uav_pose.pose.orientation.x /= qn;
        uav_pose.pose.orientation.y /= qn;
        uav_pose.pose.orientation.z /= qn;
        uav_pose.pose.orientation.w /= qn;
    } else {
        uav_pose.pose.orientation.x = 0.0;
        uav_pose.pose.orientation.y = 0.0;
        uav_pose.pose.orientation.z = 0.0;
        uav_pose.pose.orientation.w = 1.0;
    }
    uav_newest_position = LLController::toEigen(msg.pose.position);
    // 主函数
    if(!flag_takeoff_done){
        if(debug) ROS_INFO_THROTTLE(1, "\033[34mDistance with Next Waypoint =  %3lf m \033[0m ", (uav_newest_position - takeoff_point).norm());

        if((uav_newest_position - takeoff_point).norm() < takeoff_threshould){
            flag_takeoff_done = 1;
            if (!external_mission_mode_) {
                NextPoint();
            } else {
                ROS_INFO("[PatrolControl] Takeoff complete; waiting for external mission commands");
            }
            Drone_mode = Run_point;
            if (external_mission_mode_) {
                publishControlReady(true);
            }
        }
    }
    else {
        if (external_mission_mode_) {
            externalMissionTick();
        } else {
            patrol();
        }
    }
}

void LLController::publishControlReady(bool ready) {
    if (control_ready_latched_ && !ready) {
        ROS_WARN_THROTTLE(
            5.0,
            "[PatrolControl] Ignoring attempt to clear latched control readiness");
        return;
    }
    control_ready_latched_ = control_ready_latched_ || ready;
    std_msgs::Bool message;
    message.data = control_ready_latched_;
    control_ready_pub_.publish(message);
}

void LLController::externalMissionTick() {
    if (Drone_mode == Land) {
        externalLandingTick();
        return;
    }
    if (Drone_mode != Aligning) {
        return;
    }

    std_msgs::Bool detect_enable_msg;
    detect_enable_msg.data = true;
    detect_control_pub_.publish(detect_enable_msg);
    align_ok = true;
    patrol_cmd.pose.position.x = adjust_target_position[0];
    patrol_cmd.pose.position.y = adjust_target_position[1];
    patrol_cmd.pose.position.z = align_height;
    patrol_cmd.pose.orientation = waypoint_mark_point.pose.orientation;

    const bool align_done =
        (current_task_type == CROSS_MISSION) ? CrossDetectionDone()
                                             : WayPointDetectDone();
    if (align_done) {
        detect_enable_msg.data = false;
        detect_control_pub_.publish(detect_enable_msg);
        Drone_mode = Run_point;
        ROS_INFO("[PatrolControl] External ALIGN completed; waiting for RESUME command");
    }
}

bool LLController::externalLandingMarkFresh(const ros::Time& now) const {
    if (!have_land_mark || external_landing_last_mark_stamp_.isZero() ||
        external_landing_last_mark_receipt_.isZero()) {
        return false;
    }
    const double source_age =
        (now - external_landing_last_mark_stamp_).toSec();
    const double receipt_age =
        (now - external_landing_last_mark_receipt_).toSec();
    return source_age >= 0.0 && receipt_age >= 0.0 &&
           source_age <= external_landing_mark_max_age_sec_ &&
           receipt_age <= external_landing_mark_max_age_sec_;
}

void LLController::clearExternalLandingState(bool disable_detector) {
    external_landing_active_ = false;
    external_landing_new_mark_ = false;
    external_landing_alignment_complete_ = false;
    external_landing_auto_land_requested_ = false;
    external_landing_stable_count_ = 0;
    external_landing_started_at_ = ros::Time(0);
    external_landing_command_stamp_ = ros::Time(0);
    external_landing_last_mark_stamp_ = ros::Time(0);
    external_landing_last_mark_receipt_ = ros::Time(0);
    external_landing_last_auto_land_attempt_ = ros::Time(0);
    have_land_mark = false;
    flag_land = false;

    if (disable_detector) {
        std_msgs::Bool landing_enable;
        landing_enable.data = false;
        landing_detect_control_pub_.publish(landing_enable);
    }
}

void LLController::failExternalLanding(const std::string& reason) {
    clearExternalLandingState(true);

    patrol_cmd = uav_pose;
    patrol_cmd.header.frame_id = external_landing_frame_;
    mavros_point_cmd = patrol_cmd;
    last_mavros_point_cmd = patrol_cmd;
    have_planner_cmd = false;
    Point_mode = Nothing_point;
    Drone_mode = Run_point;
    ROS_ERROR("[ExternalLanding] failed closed and holding position: %s",
              reason.c_str());
}

void LLController::externalLandingTick() {
    if (!external_landing_active_) {
        failExternalLanding("landing_state_not_initialized");
        return;
    }

    std_msgs::Bool landing_enable;
    landing_enable.data = true;
    landing_detect_control_pub_.publish(landing_enable);

    const ros::Time now = ros::Time::now();
    if (flag_land) {
        patrol_cmd.pose.position.x = external_landing_aligned_goal_.pose.position.x;
        patrol_cmd.pose.position.y = external_landing_aligned_goal_.pose.position.y;
        patrol_cmd.pose.position.z = land_height;
        patrol_cmd.pose.orientation = external_landing_goal_.pose.orientation;
        return;
    }
    if ((now - external_landing_started_at_).toSec() >
        external_landing_timeout_sec_) {
        failExternalLanding("h_alignment_or_auto_land_timeout");
        return;
    }

    const bool mark_fresh = externalLandingMarkFresh(now);
    if (!mark_fresh) {
        external_landing_stable_count_ = 0;
        external_landing_alignment_complete_ = false;
    }

    if (mark_fresh && external_landing_new_mark_) {
        const double horizontal_error = std::hypot(
            uav_pose.pose.position.x - land_mark_point.pose.position.x,
            uav_pose.pose.position.y - land_mark_point.pose.position.y);
        if (horizontal_error <= external_landing_alignment_tolerance_) {
            ++external_landing_stable_count_;
        } else {
            external_landing_stable_count_ = 0;
        }
        external_landing_new_mark_ = false;
        if (external_landing_stable_count_ >=
            external_landing_stable_frames_) {
            external_landing_alignment_complete_ = true;
            external_landing_aligned_goal_ = land_mark_point;
            external_landing_aligned_goal_.pose.position.z = land_height;
            external_landing_aligned_goal_.pose.orientation =
                external_landing_goal_.pose.orientation;
            ROS_INFO("[ExternalLanding] fresh H alignment completed with %d frames",
                     external_landing_stable_count_);
        }
    }

    const geometry_msgs::PoseStamped& target =
        external_landing_alignment_complete_
            ? external_landing_aligned_goal_
            : (mark_fresh ? land_mark_point : external_landing_goal_);
    adjust_target_position[0] = target.pose.position.x;
    adjust_target_position[1] = target.pose.position.y;
    adjust_target_position[2] = external_landing_alignment_complete_
        ? land_height : external_landing_capture_height_;
    adjust_target_position[3] =
        tf::getYaw(external_landing_goal_.pose.orientation);
    patrol_cmd.header.frame_id = external_landing_frame_;
    patrol_cmd.pose.position.x = adjust_target_position[0];
    patrol_cmd.pose.position.y = adjust_target_position[1];
    patrol_cmd.pose.position.z = adjust_target_position[2];
    patrol_cmd.pose.orientation = external_landing_goal_.pose.orientation;

    if (!external_landing_alignment_complete_) {
        ROS_INFO_THROTTLE(
            1.0,
            "[ExternalLanding] waiting for fresh H alignment %d/%d",
            external_landing_stable_count_, external_landing_stable_frames_);
        return;
    }

    const double horizontal_error = std::hypot(
        uav_pose.pose.position.x - external_landing_aligned_goal_.pose.position.x,
        uav_pose.pose.position.y - external_landing_aligned_goal_.pose.position.y);
    if (uav_pose.pose.position.z <= external_landing_auto_land_height_ &&
        horizontal_error <= external_landing_alignment_tolerance_) {
        if (!(simulation_auto_land && auto_land)) {
            ROS_ERROR_THROTTLE(
                2.0,
                "[ExternalLanding] AUTO.LAND blocked: explicit simulation gate is not enabled");
            return;
        }
        if (external_landing_last_auto_land_attempt_.isZero() ||
            (now - external_landing_last_auto_land_attempt_).toSec() >=
                external_landing_auto_land_retry_sec_) {
            external_landing_last_auto_land_attempt_ = now;
            CallLand();
            external_landing_auto_land_requested_ = flag_land;
        }
    }
}
void LLController::patrol(){
    geometry_msgs::PoseStamped next_position_msg;
    float dis_to_next_position = 0;// diostance with next goal
    double yaw;
    //this flag is to adjust: run waypoint or circle adjust or landing 
    //to select next position
    std_msgs::Int8 point_class_msg;
    point_class_msg.data = Drone_mode;
    point_class_pub_.publish(point_class_msg);
    switch(Drone_mode) {
        case Run_point:  // 前往waypoint_list_中的下一个航路点
            // 计算与目标点的距离\detect_enable_msg_temp.data = false;
            detect_enable_msg_temp.data = false;
            detect_control_pub_.publish(detect_enable_msg_temp);
           // landing_detect_control_pub_.publish(detect_enable_msg_temp);
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2], 
            waypoint_list[waypoint_next].x ,waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

            next_position_msg.pose.position.x = waypoint_list[waypoint_next].x;
            next_position_msg.pose.position.y = waypoint_list[waypoint_next].y;
            next_position_msg.pose.position.z = waypoint_list[waypoint_next].z;
            yaw = waypoint_list[waypoint_next].yaw;
            next_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            arrive_goal_threshould = waypoint_threshould;//走点模式阈值为waypoint_threshould
            pub_goal(next_position_msg); //发送目标点给fastplanner或者直接给mavros
            break;

        case Hover:   // 悬停状态
            // 保持当前位置和姿态
            next_position_msg.pose.position.x = waypoint_list[waypoint_next].x;
            next_position_msg.pose.position.y = waypoint_list[waypoint_next].y;
            next_position_msg.pose.position.z = waypoint_list[waypoint_next].z;
            yaw = waypoint_list[waypoint_next].yaw;
            next_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            
            pub_goal(next_position_msg); //发送目标点给fastplanner或者直接给mavros
            break;

        case Aligning:   // 位置调整 
            // 计算与目标点的距离
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2],
            adjust_target_position[0],adjust_target_position[1], align_height);

            patrol_cmd.pose.position.x = adjust_target_position[0];
            patrol_cmd.pose.position.y = adjust_target_position[1];
            patrol_cmd.pose.position.z = align_height;
            ROS_INFO_THROTTLE(1, "\033[34mAligning position: %.2f, %.2f, %.2f\033[0m", adjust_target_position[0], adjust_target_position[1], adjust_target_position[2]);

            if (std::isnan(adjust_target_position[3])) yaw = waypoint_list[waypoint_next].yaw;
            else yaw = adjust_target_position[3];
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            arrive_goal_threshould = aligning_threshould;//走点模式阈值为waypoint_threshould
            break;

        case Land:   // 降落
            // 如果启用了检测，持续调用LandDetectDone来更新检测结果
            // 计算与目标点的距离
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], 0,
            adjust_target_position[0], adjust_target_position[1], 0);

            patrol_cmd.pose.position.x = adjust_target_position[0];
            patrol_cmd.pose.position.y = adjust_target_position[1];
            patrol_cmd.pose.position.z = align_height;
            // 判断检测得到的yaw是否有效
            if (std::isnan(adjust_target_position[3])) yaw = waypoint_list[waypoint_next].yaw;
            else yaw = adjust_target_position[3];
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            if(flag_landing_detect)arrive_goal_threshould = landing_threshould;//降落模式如果要求对准，阈值为landing_threshould
            else arrive_goal_threshould = waypoint_threshould;//降落模式如果不要求对准，阈值为waypoint_threshould
            break;

        default:
            ROS_INFO_THROTTLE(1, "\033[31m[Error]: unknow  Drone_mode \033[0m ");
            break;
    }
    if(debug&&!flag_land) ROS_INFO_THROTTLE(1, "\033[34mDistance with Next Waypoint =  %3lf m\033[0m", dis_to_next_position);
    if(debug&&flag_land)  ROS_INFO_THROTTLE(1, "\033[34mHave Land!\033[0m ");
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    //arrive goal position
    if (dis_to_next_position <= arrive_goal_threshould&& abs(current_yaw - yaw) <= arrive_yaw_threshould){
        switch (Point_mode)
        {
            case Detect_point:{  // Detect

                if (current_task_type == MAIN_MISSION) {
                    // 无论是否完成十字投递，都可以进行圆环检测
                    ROS_INFO_THROTTLE(5, "\033[34marrive waypoint, and detect... \033[0m ");
                    
                    // 启用圆形检测
                    std_msgs::Bool detect_enable_msg;
                    detect_enable_msg.data = true;
                    cross_control_pub_.publish(detect_enable_msg_temp);
                    detect_control_pub_.publish(detect_enable_msg);
                    class_control_pub_.publish(detect_enable_msg);
                    tank_control_pub_.publish(detect_enable_msg_temp);
                    
                    // 如果已经完成十字投递，确保十字检测系统保持关闭
                    // if (cross_drop_completed) {
                    //     std_msgs::Bool cross_disable_msg;
                    //     cross_disable_msg.data = false;
                    //     cross_control_pub_.publish(cross_disable_msg);
                    //     ROS_INFO("\033[36m[PatrolControl] Circle detection enabled, cross detection remains disabled\033[0m");
                    // }
                    if (align_ok)//允许对准
                    {
                        std_msgs::Bool detect_enable_msg;
                        detect_enable_msg.data = true;
                        detect_control_pub_.publish(detect_enable_msg);
                        class_control_pub_.publish(detect_enable_msg_temp);
                        //class_control_pub_.publish(detect_enable_msg); 
                        
                    }
                    else{
                        ROS_INFO("\033[32m[PatrolControl] Aligning not ok, wait for next cycle\033[0m");
                        waypoint_mark_point.pose.position.x = waypoint_list[waypoint_next].x;
                        waypoint_mark_point.pose.position.y = waypoint_list[waypoint_next].y;
                        waypoint_mark_point.pose.position.z = waypoint_list[waypoint_next].z;
                        waypoint_mark_point.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);
                    }

                    Drone_mode= Aligning;
                        // 如果该点调整结束，切到下一个路点
                        if (WayPointDetectDone()){   
                            // 禁用圆形检测
                            std_msgs::Bool detect_disable_msg;
                            detect_disable_msg.data = false;
                            detect_control_pub_.publish(detect_disable_msg);
                            class_control_pub_.publish(detect_disable_msg);
        
                            NextPoint(); 

                            // adjust_target_position[0] = waypoint_list[waypoint_next].x;
                            // adjust_target_position[1] = waypoint_list[waypoint_next].y;
                            // adjust_target_position[2] = align_height;
                            // adjust_target_position[3] = waypoint_list[waypoint_next].yaw;

                            // 更新last_waypoint_mark,防止下次到达该点时，直接去下一个点
                            last_waypoint_mark.pose.position.x = waypoint_mark_point.pose.position.x;
                            last_waypoint_mark.pose.position.y = waypoint_mark_point.pose.position.y;
                            last_waypoint_mark.pose.position.z = waypoint_mark_point.pose.position.z;
                            last_waypoint_mark.pose.orientation = waypoint_mark_point.pose.orientation;
                            servo_complete.data = false;
                            times_detect = 0;
                            have_waypoint_mark = false;
                            Drone_mode= Run_point;
                        }                   
                }
                // 移除十字任务处理逻辑，因为它不应该在Detect_point中处理
            break;}

            case Nothing_point: {   // 跑点 nothing to do
                ROS_INFO_THROTTLE(2, "\033[34marrive goal position,and Pure run point.\033[0m ");
                
                // 确保禁用圆形检测
                std_msgs::Bool detect_disable_msg;
                detect_disable_msg.data = false;
                detect_control_pub_.publish(detect_disable_msg);
                
                // 检查是否需要悬停
                if (waypoint_list[waypoint_next].hover_time > 0.0) {
                    // 开始悬停
                    if (!flag_hover_started) {
                        flag_hover_started = true;
                        hover_start_time = ros::Time::now();
                        current_hover_time = waypoint_list[waypoint_next].hover_time;
                        Drone_mode = Hover;
                        ROS_INFO("\033[33m开始悬停, 悬停时间: %.1f 秒\033[0m", current_hover_time);
                    }
                } else {
                    // 不需要悬停，直接前往下一个点
                    NextPoint();
                    have_waypoint_mark = false;
                    Drone_mode= Run_point;
                }
                break;}

            case Land_point: {   // 降落  landing_position
                landig_mark = 1;
                if(debug&&!flag_land){
                    if(flag_landing_detect) {ROS_INFO_THROTTLE(5, "\033[34mArrive landing position, start detect land mark ...\033[0m  ");}
                    else{ROS_INFO_THROTTLE(5, "\033[34mArrive landing position, and not detect. \033[0m");}
                }
                
                // 确保禁用圆形检测
                std_msgs::Bool detect_disable_msg;
                detect_disable_msg.data = false;
                detect_control_pub_.publish(detect_disable_msg);
                
                // 启用降落检测（黑色圆环+H）
                std_msgs::Bool landing_detect_enable_msg;
                landing_detect_enable_msg.data = true;
                landing_detect_control_pub_.publish(landing_detect_enable_msg);
                ROS_INFO("\033[36m[PatrolControl] Landing detection (Black Circle + H) enabled\033[0m");

                current_task_type = MAIN_MISSION;
                // 切换到Land模式，开始检测和降落流程
                Drone_mode = Land;
                if (LandDetectDone()) {
                    // 检测完成，开始降落
                    CallLand();
                    have_land_mark = false;
                    times_detect = 0;
                }
                // 如果需要检测，让Land模式来处理检测过程，不要在这里调用LandDetectDone()
                
                break;}
            case Dynamic_point:{
                    // 无论是否完成十字投递，都可以进行圆环检测
                    ROS_INFO_THROTTLE(5, "\033[34marrive waypoint, and detect... \033[0m ");
                    
                    // 启用圆形检测
                    std_msgs::Bool detect_enable_msg;
                    detect_enable_msg.data = true;
                    cross_control_pub_.publish(detect_enable_msg_temp);
                    detect_control_pub_.publish(detect_enable_msg);
                    class_control_pub_.publish(detect_enable_msg_temp);
                    
                    // 如果已经完成十字投递，确保十字检测系统保持关闭
                    // if (cross_drop_completed) {
                    //     std_msgs::Bool cross_disable_msg;
                    //     cross_disable_msg.data = false;
                    //     cross_control_pub_.publish(cross_disable_msg);
                    //     ROS_INFO("\033[36m[PatrolControl] Circle detection enabled, cross detection remains disabled\033[0m");
                    // }
                    detect_control_pub_.publish(detect_enable_msg);
                    Drone_mode= Aligning;
                        // 如果该点调整结束，切到下一个路点
                        if (DynamicProcess()){   
                            // 禁用圆形检测
                            std_msgs::Bool detect_disable_msg;
                            detect_disable_msg.data = false;
                            detect_control_pub_.publish(detect_disable_msg);
                            class_control_pub_.publish(detect_disable_msg);
        
                            NextPoint(); 

                            // adjust_target_position[0] = waypoint_list[waypoint_next].x;
                            // adjust_target_position[1] = waypoint_list[waypoint_next].y;
                            // adjust_target_position[2] = align_height;
                            // adjust_target_position[3] = waypoint_list[waypoint_next].yaw;

                            // 更新last_waypoint_mark,防止下次到达该点时，直接去下一个点
                            last_waypoint_mark.pose.position.x = waypoint_mark_point.pose.position.x;
                            last_waypoint_mark.pose.position.y = waypoint_mark_point.pose.position.y;
                            last_waypoint_mark.pose.position.z = waypoint_mark_point.pose.position.z;
                            last_waypoint_mark.pose.orientation = waypoint_mark_point.pose.orientation;
                            servo_complete.data = false;
                            times_detect = 0;
                            have_waypoint_mark = false;
                            Drone_mode= Run_point;
                        }                   
                // 移除十字任务处理逻辑，因为它不应该在Detect_point中处理
                break;
            }
            default:
                ROS_INFO_THROTTLE(1, "\033[31m[Error]: unknow  Point_mode \033[0m ");
                break;
        }
    }
    
    // 悬停时间检查
    if (Drone_mode == Hover && flag_hover_started) {
        ros::Time current_time = ros::Time::now();
        double elapsed_hover_time = (current_time - hover_start_time).toSec();
        
        // 显示悬停进度
        if (debug) {
            ROS_INFO_THROTTLE(1, "\033[33m悬停进度: %.1f/%.1f 秒\033[0m", elapsed_hover_time, current_hover_time);
        }
        
        // 悬停时间到，前往下一个点
        if (elapsed_hover_time >= current_hover_time) {
            ROS_INFO("\033[32m悬停完成，前往下一个点\033[0m");
            flag_hover_started = false;
            NextPoint();
            have_waypoint_mark = false;
            Drone_mode = Run_point;
        }
    }
}

void LLController::plannercmdCallback(const geometry_msgs::PoseStamped& msg) {
    have_planner_cmd = true;
    planner_cmd = msg;
    latest_planner_cmd_time_ = ros::Time::now();
}

void LLController::TankStatusCallback(const geometry_msgs::PoseStamped& msg){
    tank_found_ = true;
    tank_mark_point.pose.position.x = msg.pose.position.x;
    tank_mark_point.pose.position.y = msg.pose.position.y;
    tank_mark_point.pose.position.z = msg.pose.position.z;
    ROS_INFO("[Tankcallback]tank mark point %.2f %.2f",tank_mark_point.pose.position.x,tank_mark_point.pose.position.y);
}

void LLController::waypointMarkCallback(const geometry_msgs::PoseStamped& msg) {
    have_waypoint_mark = true;
    // waypoint_mark_point = msg;
    waypoint_mark_point.pose.position.x = msg.pose.position.x;
    waypoint_mark_point.pose.position.y = msg.pose.position.y;
    waypoint_mark_point.pose.position.z = align_height;
    ROS_INFO("[waypointMarkCallback] target_position: %.2f, %.2f, %.2f", waypoint_mark_point.pose.position.x, waypoint_mark_point.pose.position.y, waypoint_mark_point.pose.position.z);
}

void LLController::crossMarkCallback(const geometry_msgs::PoseStamped& msg) {
    have_cross_mark = true;
    cross_mark_point.pose.position.x = msg.pose.position.x;
    cross_mark_point.pose.position.y = msg.pose.position.y;
    cross_mark_point.pose.position.z = align_height;
    ROS_INFO("[crossMarkCallback] cross_mark_point: %.2f, %.2f, %.2f", cross_mark_point.pose.position.x, cross_mark_point.pose.position.y, cross_mark_point.pose.position.z);
}

void LLController::landMarkCallback(const geometry_msgs::PoseStamped& msg)
{
    if (external_mission_mode_) {
        if (!external_landing_active_ || Drone_mode != Land) {
            return;
        }
        const ros::Time now = ros::Time::now();
        const double source_age = (now - msg.header.stamp).toSec();
        const bool coordinates_valid =
            std::isfinite(msg.pose.position.x) &&
            std::isfinite(msg.pose.position.y) &&
            std::isfinite(msg.pose.position.z);
        const double anchor_error = std::hypot(
            msg.pose.position.x - external_landing_goal_.pose.position.x,
            msg.pose.position.y - external_landing_goal_.pose.position.y);
        if (msg.header.frame_id != external_landing_frame_ ||
            msg.header.stamp.isZero() || source_age < 0.0 ||
            source_age > external_landing_mark_max_age_sec_ ||
            msg.header.stamp <= external_landing_command_stamp_ ||
            (!external_landing_last_mark_stamp_.isZero() &&
             msg.header.stamp <= external_landing_last_mark_stamp_) ||
            !coordinates_valid ||
            anchor_error > external_landing_max_mark_offset_) {
            ROS_WARN_THROTTLE(
                1.0,
                "[ExternalLanding] rejected H mark frame=%s age=%.3f anchor_error=%.3f",
                msg.header.frame_id.c_str(), source_age, anchor_error);
            return;
        }
        land_mark_point = msg;
        have_land_mark = true;
        external_landing_new_mark_ = true;
        external_landing_last_mark_stamp_ = msg.header.stamp;
        external_landing_last_mark_receipt_ = now;
        ROS_INFO_THROTTLE(
            1.0, "[ExternalLanding] accepted fresh H mark at %.3f %.3f",
            msg.pose.position.x, msg.pose.position.y);
        return;
    }
    have_land_mark = true;
    land_mark_point = msg;
    ROS_INFO("[landmarkCallback] cross_mark_point: %.2f, %.2f, %.2f", land_mark_point.pose.position.x, land_mark_point.pose.position.y, land_mark_point.pose.position.z);    
}
bool isQuaternionNormalized(const geometry_msgs::Quaternion& q, double tolerance = 1e-6)
{
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return std::abs(norm - 1.0) < tolerance;
}

bool LLController::hasValidExternalPlannerCommand() const {
    if (!have_planner_cmd) {
        ROS_WARN_THROTTLE(1.0, "[ExternalPlanner] no planner command received");
        return false;
    }
    const double age =
        (ros::Time::now() - latest_planner_cmd_time_).toSec();
    if (age > external_planner_cmd_timeout_) {
        ROS_WARN_THROTTLE(
            1.0, "[ExternalPlanner] stale command age=%.3f limit=%.3f",
            age, external_planner_cmd_timeout_);
        return false;
    }
    const geometry_msgs::Point& position = planner_cmd.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || position.z <= 0.05) {
        ROS_WARN_THROTTLE(
            1.0, "[ExternalPlanner] invalid command position=(%.3f, %.3f, %.3f)",
            position.x, position.y, position.z);
        return false;
    }
    if (!isQuaternionNormalized(planner_cmd.pose.orientation, 1e-3)) {
        ROS_WARN_THROTTLE(1.0, "[ExternalPlanner] invalid command quaternion");
        return false;
    }
    const double dx = position.x - uav_pose.pose.position.x;
    const double dy = position.y - uav_pose.pose.position.y;
    const double dz = position.z - uav_pose.pose.position.z;
    const double distance_sq = dx * dx + dy * dy + dz * dz;
    const double max_distance_sq = external_planner_start_max_distance_ * external_planner_start_max_distance_;
    if (distance_sq > max_distance_sq) {
        ROS_WARN_THROTTLE(
            1.0,
            "[ExternalPlanner] command too far distance=%.3f limit=%.3f "
            "current=(%.3f, %.3f, %.3f) command=(%.3f, %.3f, %.3f)",
            std::sqrt(distance_sq), external_planner_start_max_distance_,
            uav_pose.pose.position.x, uav_pose.pose.position.y,
            uav_pose.pose.position.z, position.x, position.y, position.z);
        return false;
    }
    ROS_INFO_THROTTLE(
        1.0,
        "[ExternalPlanner] accepted current=(%.3f, %.3f, %.3f) "
        "command=(%.3f, %.3f, %.3f) distance=%.3f",
        uav_pose.pose.position.x, uav_pose.pose.position.y,
        uav_pose.pose.position.z, position.x, position.y, position.z,
        std::sqrt(distance_sq));
    return true;
}
void LLController::servoMarkyCallback(const std_msgs::Bool& msg) {
    servo_marky = msg;
    ROS_INFO("\033[1;35m[servoMarkyCallback] Received servo_marky: %s\033[0m", servo_marky.data ? "true" : "false");
}
void LLController::cmdCallback(const ros::TimerEvent& event) {
    if(!isQuaternionNormalized(uav_pose.pose.orientation)){
        std::cout<<"\033[33m[WARN]: The quaternion of the drone position has not been unitized. Please check whether the position information is correct!\033[0m"<<std::endl;
        return;
    }
    if (have_selected_target_ && !hasFreshSelectedTarget()) {
        have_selected_target_ = false;
    }
    if (have_drop_offset_ && !hasFreshDropOffset()) {
        have_drop_offset_ = false;
        uav_drop_ready_ = false;
    }
    publishAlignMode(desiredAlignMode());
    std_msgs::Int8 point_class_msg;
    point_class_msg.data = Drone_mode;
    point_class_pub_.publish(point_class_msg);
    switch(Drone_mode) {
        case Takeoff:  // Takeoff
            have_planner_cmd = false;
            detect_enable_msg_temp.data = false;
            detect_control_pub_.publish(detect_enable_msg_temp);
            landing_detect_control_pub_.publish(detect_enable_msg_temp);
            mavros_point_cmd.pose.position.x = takeoff_point[0];
            mavros_point_cmd.pose.position.y = takeoff_point[1];
            mavros_point_cmd.pose.position.z = takeoff_point[2];
            waypoint_mark_point.pose.position.x = takeoff_point[0];
            waypoint_mark_point.pose.position.y = takeoff_point[1];
            waypoint_mark_point.pose.position.z = takeoff_point[2];
            mavros_point_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[0].yaw);
            ROS_INFO_THROTTLE(5, "Send point to take off ");
            break;

        case Run_point: { // 前往waypoint_list_中的下一个航路点
            // 1. 首先处理正常的航路点导航
            adjust_target_position[0] = uav_pose.pose.position.x;
            adjust_target_position[1] = uav_pose.pose.position.y;
            adjust_target_position[2] = uav_pose.pose.position.z;
            detect_control_pub_.publish(detect_enable_msg_temp);
            if (external_mission_mode_) {
                if (!flag_planner_px4) {
                    if (hasValidExternalPlannerCommand()) {
                        mavros_point_cmd = planner_cmd;
                        if (mavros_point_cmd.pose.position.z >
                                external_planner_max_command_z_) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[ExternalPlanner] capping command height=%.3f "
                                "to %.3f while preserving horizontal progress",
                                mavros_point_cmd.pose.position.z,
                                external_planner_max_command_z_);
                            mavros_point_cmd.pose.position.z =
                                external_planner_max_command_z_;
                        }
                    } else {
                        mavros_point_cmd = last_mavros_point_cmd;
                    }
                } else {
                    mavros_point_cmd = patrol_cmd;
                }
                ROS_INFO_THROTTLE(5, "[PatrolControl] Forwarding external planner trajectory");
                break;
            }
            if (current_task_type == MAIN_MISSION) {
                if (hasFreshSelectedTarget()) {
                    if (latest_selected_target_.class_name == "red_cross" &&
                        cross_mark && !cross_drop_completed) {
                        ROS_INFO("\033[32m[UavVision] selected red_cross confirmed, requesting CROSS mission interrupt\033[0m");
                        cross_mark = false;
                        current_task_type = CROSS_MISSION;
                        Point_temp = Point_mode;
                        Point_mode = Detect_point;
                        Drone_mode = Aligning;
                        have_selected_target_ = false;
                        break;
                    }
                    if (latest_selected_target_.class_name == "tank" &&
                        enable_selected_tank_interrupt_ &&
                        tank_mark && !tank_drop_completed) {
                        ROS_INFO("\033[32m[UavVision] selected tank confirmed, requesting TANK mission interrupt\033[0m");
                        tank_mark = false;
                        current_task_type = TANK_MISSION;
                        Point_temp = Point_mode;
                        Point_mode = Detect_point;
                        Drone_mode = Aligning;
                        have_selected_target_ = false;
                        break;
                    }
                }

                // 正常的航路点导航逻辑
                // 启用途中十字检测（仅在未完成十字投递时）
                if (cross_mark && !cross_drop_completed) {
                    std_msgs::Bool detect_enable_msg;
                    detect_enable_msg.data = false;
                    detect_control_pub_.publish(detect_enable_msg);
                    std_msgs::Bool cross_enable_msg;
                    cross_enable_msg.data = true;
                    cross_control_pub_.publish(cross_enable_msg);
                } else if (cross_drop_completed || (waypoint_next >= (waypoint_list.size() - waypoint_skipping_index))) {
                    std_msgs::Bool detect_enable_msg;
                    detect_enable_msg.data = false;
                    detect_control_pub_.publish(detect_enable_msg);
                    // 确保十字检测保持关闭状态
                    std_msgs::Bool cross_disable_msg;
                    cross_disable_msg.data = false;
                    cross_control_pub_.publish(cross_disable_msg);
                }

                if (tank_mark && !tank_drop_completed) {
                    std_msgs::Bool tank_enable_msg;
                    tank_enable_msg.data = true;
                    tank_control_pub_.publish(tank_enable_msg);
                    std_msgs::Bool cross_enable_msg;
                    cross_enable_msg.data = false;
                    cross_control_pub_.publish(cross_enable_msg);
                    std_msgs::Bool tank_enable_msg_temp;
                    tank_enable_msg.data = true;
                    detect_control_pub_.publish(tank_enable_msg_temp);
                } else if (tank_drop_completed || (waypoint_next >= (waypoint_list.size() - waypoint_skipping_index))) {
                    std_msgs::Bool detect_enable_msg;
                    detect_enable_msg.data = false;
                    tank_control_pub_.publish(detect_enable_msg);
                    std_msgs::Bool cross_enable_msg;
                    cross_enable_msg.data = true;
                    cross_control_pub_.publish(cross_enable_msg);

                    // 确保十字检测保持关闭状态
                    std_msgs::Bool detect_disable_msg;
                    detect_disable_msg.data = false;
                    detect_control_pub_.publish(detect_disable_msg);
                }
                
                // 设置目标为当前航路点
                if(!flag_planner_px4){
                    patrol_cmd.pose.position.x = uav_pose.pose.position.x;
                    patrol_cmd.pose.position.y = uav_pose.pose.position.y;
                    patrol_cmd.pose.position.z = uav_pose.pose.position.z;
                    if(have_planner_cmd) mavros_point_cmd = planner_cmd;
                    else mavros_point_cmd = last_mavros_point_cmd;
                }else{
                    mavros_point_cmd = patrol_cmd;
                }
            }            
            // 2. 检查是否检测到十字（中断主任务）
            if (tank_found_ && current_task_type == MAIN_MISSION && tank_mark && !tank_drop_completed) {
                ROS_INFO("\033[32m[PatrolControl] TANK detected! Starting TANK mission interrupt sequence\033[0m");
                
                // 立即禁用十字检测，防止重复触发
                tank_mark = false;
                
                // 切换到十字任务
                current_task_type = TANK_MISSION;
                Point_temp = Point_mode;
                Point_mode = Detect_point;
                
                Drone_mode = Aligning;
                ROS_INFO("\033[32m[PatrolControl] Cross mission started\033[0m");
            }
            if (cross_found_ && current_task_type == MAIN_MISSION && cross_mark && !cross_drop_completed) {
                ROS_INFO("\033[32m[PatrolControl] Cross detected! Starting cross mission interrupt sequence\033[0m");
                
                // 立即禁用十字检测，防止重复触发
                cross_mark = false;
                
                // 切换到十字任务
                current_task_type = CROSS_MISSION;
                Point_temp = Point_mode;
                Point_mode = Detect_point;
                
                Drone_mode = Aligning;
                ROS_INFO("\033[32m[PatrolControl] Cross mission started\033[0m");
            }

            
            // 3. 如果是从十字任务返回，恢复主任务
            if (current_task_type == CROSS_MISSION && cross_mission_completed) {
                ROS_INFO("\033[33m[PatrolControl] Cross mission completed! Cleaning up and resuming main mission\033[0m");
                // if()
                // 使用专门的清理函数
                cleanupAfterCrossDrop();
                
                // 彻底禁用所有检测系统，避免冲突
                std_msgs::Bool detect_disable_msg;
                detect_disable_msg.data = false;
                detect_control_pub_.publish(detect_disable_msg);
                
                // 永久禁用十字检测，关闭画面显示
                cross_drop_completed = true;
                cross_mark = false;
                std_msgs::Bool cross_disable_msg;
                cross_disable_msg.data = false;
                cross_control_pub_.publish(cross_disable_msg);
                
                ROS_INFO("\033[36m[DEBUG] Resuming mission: waypoint_now=%d, waypoint_next=%d\033[0m", waypoint_now, waypoint_next);
                ROS_INFO("\033[36m[DEBUG] Will continue towards waypoint %d: (%.2f, %.2f, %.2f)\033[0m", 
                         waypoint_next, waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);
                ROS_INFO("\033[36m[PatrolControl] All detection systems disabled, cross detection permanently disabled\033[0m");
                Point_mode = Point_temp;
                // if(detect_point_counter >= 3) Point_mode = Nothing_point;
                // 添加一个短暂的延迟，确保所有状态都已清理
                ros::Duration(0.1).sleep();
            }
            if(current_task_type == TANK_MISSION && tank_mission_completed){
                ROS_INFO("\033[33m[PatrolControl] TANK mission completed! Cleaning up and resuming main mission\033[0m");
                // if()
                // 使用专门的清理函数
                cleanupAfterCrossDrop();
                // tank_mark = false;
                
                std_msgs::Bool detect_disable_msg;
                detect_disable_msg.data = false;
                detect_control_pub_.publish(detect_disable_msg);
                
                // 永久禁用tank检测，关闭画面显示
                tank_drop_completed = true;
                tank_mark = false;
                std_msgs::Bool tank_disable_msg;
                tank_disable_msg.data = false;
                tank_control_pub_.publish(tank_disable_msg);
                
                ROS_INFO("\033[36m[DEBUG] Resuming mission: waypoint_now=%d, waypoint_next=%d\033[0m", waypoint_now, waypoint_next);
                ROS_INFO("\033[36m[DEBUG] Will continue towards waypoint %d: (%.2f, %.2f, %.2f)\033[0m", 
                         waypoint_next, waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);
                ROS_INFO("\033[36m[PatrolControl] TANKMISSION completed\033[0m");
                Point_mode = Point_temp;
                // if(detect_point_counter >= 3) Point_mode = Nothing_point;
                // 添加一个短暂的延迟，确保所有状态都已清理
                ros::Duration(0.1).sleep();
            }
            
            ROS_INFO_THROTTLE(5, "Send point to Run_point ");
            break;
        }

        case Hover: {   // 悬停状态
            if(!flag_planner_px4){
                if(have_planner_cmd) mavros_point_cmd = planner_cmd;
                else mavros_point_cmd = last_mavros_point_cmd;
            }else{
                mavros_point_cmd = patrol_cmd;
            }
            cross_control_pub_.publish(detect_enable_msg_temp);
            ROS_INFO_THROTTLE(5, "Send point to Hover ");
            break;
        }

        case Aligning: { // 位置调整 
            have_planner_cmd = false;
            mavros_point_cmd = patrol_cmd;
            // std::cout<<"mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z = "<<mavros_point_cmd.pose.position.x<<", "<<mavros_point_cmd.pose.position.y<<", "<<mavros_point_cmd.pose.position.z<<std::endl;
            
            if (current_task_type == MAIN_MISSION) {
                // 正常航路点的检测对准（原有逻辑）
                ROS_INFO_THROTTLE(5, "Send point to Aligning (Main Mission)");
            }
            else if (current_task_type == CROSS_MISSION) {
                // 十字检测的对准投递
                ROS_INFO_THROTTLE(5, "Send point to Aligning (Cross Mission)");
                
                // 检查十字检测任务是否完成
                if (CrossDetectionDone()) {
                    // 十字任务完成，准备返回主任务
                    if(detect_point_counter >= 3){
                        NextPoint();
                        Drone_mode = Run_point;
                        Point_mode = Nothing_point;
                    }
                    else{
                        cross_mission_completed = true;
                        std_msgs::Bool cross_disable_msg;
                        cross_disable_msg.data = false;
                        cross_control_pub_.publish(cross_disable_msg);
                        Drone_mode = Run_point;  // 返回Run_point，会被上面的逻辑处理
                        Point_mode = Point_temp;
                        // if(detect_point_counter >= 3) Point_mode = Nothing_point;
                    }                    
                    ROS_INFO("\033[32m[PatrolControl] Cross detection and drop completed!\033[0m");
                }
            }
            else if(current_task_type == TANK_MISSION){
                ROS_INFO_THROTTLE(5, "Send point to Aligning (TANK Mission)");
                align_ok = true;
                std_msgs::Bool tank_activate_msg;
                tank_activate_msg.data = true;
                detect_control_pub_.publish(tank_activate_msg);
                // 检查十字检测任务是否完成
                if (DynamicProcess()) {
                    // 十字任务完成，准备返回主任务
                    if(detect_point_counter >= 3){
                        NextPoint();
                        align_ok = false;
                        Drone_mode = Run_point;
                        Point_mode = Nothing_point;
                    }
                    else{
                        tank_mission_completed = true;
                        std_msgs::Bool tank_disable_msg;
                        tank_disable_msg.data = false;
                        tank_control_pub_.publish(tank_disable_msg);
                        Drone_mode = Run_point;  // 返回Run_point，会被上面的逻辑处理
                        Point_mode = Point_temp;
                        // if(detect_point_counter >= 3) Point_mode = Nothing_point;
                    }                    
                    ROS_INFO("\033[32m[PatrolControl] tank detection and drop completed!\033[0m");
                }
            }
            break;
        }

        case Land: {       // 降落
            have_planner_cmd = false;
            mavros_point_cmd = patrol_cmd;
            if(!flag_land)ROS_INFO_THROTTLE(5, "Send point to Land ");
            break;
        }
    }

    // 计算当前 UAV 位置到目标点的距离
    Eigen::Vector3d current_pos(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z);
    double distance_to_target = (target_pos - current_pos).norm();
    // 如果距离超过 px4_max_distance，则进行插值
    if (distance_to_target > px4_max_distance) {
        Eigen::Vector3d direction = (target_pos - current_pos).normalized();
        Eigen::Vector3d new_pos = current_pos + direction * px4_max_distance;
        // 更新目标点为插值后的点位
        mavros_point_cmd.pose.position.x = new_pos.x();
        mavros_point_cmd.pose.position.y = new_pos.y();
        mavros_point_cmd.pose.position.z = new_pos.z();
        ROS_INFO_THROTTLE(2, "Target point adjusted to max distance limit.");
    }

    // 提取当前 yaw 和目标 yaw
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    double target_yaw = tf::getYaw(mavros_point_cmd.pose.orientation);

    // 判断 yaw 角插值
    double yaw_diff = target_yaw - current_yaw;
    yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));  // 确保 yaw_diff 在 [-π, π] 内

    // 限制 yaw 改变量
    if (fabs(yaw_diff) > max_yaw_change) {
        yaw_diff = (yaw_diff > 0) ? max_yaw_change : -max_yaw_change;
        ROS_INFO_THROTTLE(2, "Target point adjusted to max yaw limit.");
    }

    // 插值后的 yaw
    double interpolated_yaw = current_yaw + yaw_diff;
    interpolated_yaw = atan2(sin(interpolated_yaw), cos(interpolated_yaw));  // 归一化到 [-π, π]

    // 将插值后的 yaw 转为四元数
    mavros_point_cmd.pose.orientation.x = 0.0;
    mavros_point_cmd.pose.orientation.y = 0.0;
    mavros_point_cmd.pose.orientation.z = sin(interpolated_yaw / 2.0);
    mavros_point_cmd.pose.orientation.w = cos(interpolated_yaw / 2.0);

    mavros_point_cmd.header.stamp = ros::Time::now();
    mavros_point_cmd.header.frame_id = "camera_init";
    mavros_point_cmd_pub.publish(mavros_point_cmd);
    // std::cout<<"Drone_mode = "<<Drone_mode<<std::endl;
    // std::cout<<"mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z = "<<mavros_point_cmd.pose.position.x<<", "<<mavros_point_cmd.pose.position.y<<", "<<mavros_point_cmd.pose.position.z<<std::endl;
    last_mavros_point_cmd = mavros_point_cmd;
    // 判断是否已经降落，降落成功就锁桨
    // External SITL landing delegates disarm to PX4 AUTO.LAND and verifies it
    // through MAVROS.  The legacy height-only force-disarm path is unsafe for
    // that contract because a bad local-z sample could stop motors in flight.
    if(!external_mission_mode_ && Drone_mode == Land &&
       uav_pose.pose.position.z <= 0.02){
        // 禁用降落检测
        std_msgs::Bool landing_detect_disable_msg;
        landing_detect_disable_msg.data = false;
        landing_detect_control_pub_.publish(landing_detect_disable_msg);
        ROS_INFO("\033[33m[PatrolControl] Landing detection disabled - landing completed\033[0m");
        Lock();
    }
}

void LLController::Lock() {
    // 准备服务请求
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = false;
    cmd.request.command = 400;          // 命令ID
    cmd.request.confirmation = 0;       // 确认
    cmd.request.param1 = 0.0;           // 参数1
    cmd.request.param2 = 21196.0;       // 参数2
    cmd.request.param3 = 0.0;           // 参数3
    cmd.request.param4 = 0.0;           // 参数4
    cmd.request.param5 = 0.0;           // 参数5
    cmd.request.param6 = 0.0;           // 参数6
    cmd.request.param7 = 0.0;           // 参数7
    if(land_client.call(cmd) && cmd.response.success){ROS_INFO_THROTTLE(2, "Vehicle DisArmed!");}
}

void LLController::CallLand() {
    if (simulation_auto_land && auto_land) {
        mavros_msgs::SetMode auto_land_mode;
        auto_land_mode.request.custom_mode = "AUTO.LAND";
        const bool mode_accepted =
            set_mode_client.call(auto_land_mode) &&
            auto_land_mode.response.mode_sent;
        if (mode_accepted) {
            ROS_INFO("[PatrolControl] Simulation AUTO.LAND mode enabled");
        } else {
            ROS_WARN("[PatrolControl] Simulation AUTO.LAND request failed; keeping safe landing setpoint");
        }
        // Do not publish the historical -1 m setpoint in this simulation path.
        align_height = land_height;
        if (external_mission_mode_ && !mode_accepted) {
            // External mission completion is observed through MAVROS landed
            // state.  A rejected mode request must remain retryable instead
            // of pretending that the landing handoff succeeded.
            flag_land = false;
            return;
        }
    } else {
        if(!flag_landing_detect){
            adjust_target_position[0] = waypoint_temp.pose.position.x;
            adjust_target_position[1] = waypoint_temp.pose.position.y;
            adjust_target_position[3] = waypoint_list[waypoint_next].yaw;
        }
        align_height = -1.0;
    }
    flag_land = true;
}

// 读取launch文件设置的若干个航路点参数
void LLController::load_params() {
    // 开关
    flag_planner_px4 = nh_.param("switch/flag_planner_px4", true);
    flag_landing_detect = nh_.param("switch/flag_landing_detect", 1);
    auto_land = nh_.param("switch/auto_land", false);
    simulation_auto_land = nh_.param("simulation/enable_auto_land", false);
    if (simulation_auto_land) {
        ROS_WARN("[PatrolControl] simulation/enable_auto_land is enabled; this must not be used on hardware");
    }
    // 阈值
    takeoff_threshould = nh_.param("threshould/takeoff_threshould", 0.3);
    planner_min_pub_threshould = nh_.param("threshould/planner_min_pub_threshould", 0.02);
    waypoint_threshould = nh_.param("threshould/waypoint_threshould", 0.3);
    aligning_threshould = nh_.param("threshould/aligning_threshould", 0.15);
    landing_threshould = nh_.param("threshould/landing_threshould", 0.15);
    arrive_yaw_threshould = nh_.param("threshould/arrive_yaw_threshould", 0.3);
    times_detect_threshould = nh_.param("threshould/times_detect_threshould", 30);
    waypoint_adjust_max_second_threshould = nh_.param("threshould/waypoint_adjust_max_second_threshould", 10);
    land_adjust_max_second_threshould = nh_.param("threshould/land_adjust_max_second_threshould", 10);
    waypoint_skipping_index = nh_.param("waypoint_skipping_index", 3);
    detect_skip_enable_ = nh_.param("detect_skip_enable", true);
    update_goal_from_selected_target_ =
        nh_.param("uav_vision/update_goal_from_selected_target", true);
    require_vision_release_permission_ =
        nh_.param("uav_vision/require_release_permission", false);
    external_mission_mode_ = nh_.param("external_mission_mode", false);
    control_ready_topic_ = nh_.param<std::string>(
        "control_ready_topic", "/mission/control_ready");
    mission_command_topic_ = nh_.param<std::string>(
        "mission_command_topic", "/mission/command");
    external_planner_cmd_timeout_ =
        nh_.param("external_planner_cmd_timeout", 0.5);
    external_alignment_timeout_sec_ =
        nh_.param("external_alignment_timeout", 75.0);

    // 目标类别列表：~goal_list 参数（XmlRpc 数组）可选，缺省保持旧行为 {"panzer"}
    external_planner_start_max_distance_ =
        nh_.param("external_planner_start_max_distance", 0.6);
    external_planner_max_command_z_ =
        nh_.param("external_planner_max_command_z", 3.5);
    if (!std::isfinite(external_planner_max_command_z_) ||
        external_planner_max_command_z_ <= 0.05 ||
        external_planner_max_command_z_ > 4.0) {
        ROS_ERROR(
            "[ExternalPlanner] invalid max command z %.3f; using safe "
            "fallback 3.5 m",
            external_planner_max_command_z_);
        external_planner_max_command_z_ = 3.5;
    }
    {
        XmlRpc::XmlRpcValue goal_list;
        if (nh_.getParam("goal_list", goal_list) &&
            goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < goal_list.size(); ++i) {
                goal.push_back(static_cast<std::string>(goal_list[i]));
            }
            ROS_INFO("[PatrolControl] goal_list loaded: %zu targets", goal.size());
        } else {
            goal = {"panzer"};
        }
    }

    // 参数
    land_height = nh_.param("land_height", 0.3);//降落时调整的固定高度
    px4_max_distance = nh_.param("px4_max_distance", 1.2);
    max_yaw_change = nh_.param("max_yaw_change", 0.3);
    align_height = nh_.param("align_height", 1.0);
    external_landing_frame_ = nh_.param<std::string>(
        "external_landing/frame", "camera_init");
    external_landing_capture_height_ = nh_.param(
        "external_landing/capture_height", 0.75);
    external_landing_timeout_sec_ = nh_.param(
        "external_landing/timeout_sec", 75.0);
    external_landing_mark_max_age_sec_ = nh_.param(
        "external_landing/mark_max_age_sec", 0.5);
    external_landing_alignment_tolerance_ = nh_.param(
        "external_landing/alignment_tolerance", 0.08);
    external_landing_max_mark_offset_ = nh_.param(
        "external_landing/max_mark_offset", 0.60);
    external_landing_auto_land_height_ = nh_.param(
        "external_landing/auto_land_height", 0.40);
    external_landing_auto_land_retry_sec_ = nh_.param(
        "external_landing/auto_land_retry_sec", 1.0);
    external_landing_stable_frames_ = nh_.param(
        "external_landing/stable_frames", 10);
    if (external_landing_frame_.empty() || land_height <= 0.0 ||
        external_landing_capture_height_ <= external_landing_auto_land_height_ ||
        external_landing_auto_land_height_ < land_height ||
        external_landing_timeout_sec_ <= 0.0 ||
        external_landing_mark_max_age_sec_ <= 0.0 ||
        external_landing_alignment_tolerance_ <= 0.0 ||
        external_landing_max_mark_offset_ <
            external_landing_alignment_tolerance_ ||
        external_landing_auto_land_retry_sec_ <= 0.0 ||
        external_landing_stable_frames_ <= 0) {
        ROS_FATAL("[ExternalLanding] invalid fail-closed landing parameters");
        throw std::invalid_argument("invalid external_landing parameters");
    }
    
    // 投递系统参数
    drop_precision_threshold = nh_.param("drop_system/precision_threshold", 20.0);
    dynamic_height = nh_.param("dynamic/ranger_height", 0.20);
    drop_height_threshold = nh_.param("drop_system/height_threshold", 0.2);
    drop_enabled = nh_.param("drop_system/enable_drop", true);
    descent_stable_duration = nh_.param("drop_system/descent_stable_duration", 2.0);
    loadSlotOffsets(nh_, "drop_system/slot_offsets", &drop_slot_offsets_);
    loadSlotOffsets(nh_, "drop_system/dynamic_slot_offsets",
                    &dynamic_drop_slot_offsets_);
    selected_target_timeout_ = nh_.param("uav_vision/selected_target_timeout", 1.0);
    drop_offset_timeout_ = nh_.param("uav_vision/drop_offset_timeout", 1.0);
    mission_release_permission_timeout_ = nh_.param(
        "uav_vision/release_permission_timeout", 0.25);
    mission_release_permission_topic_ = nh_.param<std::string>(
        "uav_vision/release_permission_state_topic",
        "/mission/release_permission_active");
    pixel_to_meter_ratio_ = nh_.param("uav_vision/pixel_to_meter_ratio", 0.0015);
    {
        XmlRpc::XmlRpcValue pixel_to_body_matrix;
        if (nh_.getParam("uav_vision/pixel_to_body_matrix", pixel_to_body_matrix) &&
            pixel_to_body_matrix.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            pixel_to_body_matrix.size() == 4) {
            for (int i = 0; i < 4; ++i) {
                pixel_to_body_matrix_[i] =
                    static_cast<double>(pixel_to_body_matrix[i]);
            }
        } else {
            ROS_WARN("[UavVision] pixel_to_body_matrix missing/invalid; using legacy mapping");
        }
    }
    max_alignment_move_distance_ = nh_.param("uav_vision/max_movement_distance", 0.5);
    drop_circle_radius_m_ = nh_.param("uav_vision/drop_circle_radius_m", 0.5);
    drop_cross_radius_m_ = nh_.param("uav_vision/drop_cross_radius_m", 0.5);
    landing_pad_radius_m_ = nh_.param("uav_vision/landing_pad_radius_m", 0.3);
    enable_selected_tank_interrupt_ = nh_.param("uav_vision/enable_tank_interrupt", false);
    
    ROS_INFO("\033[32m[DropSystem] Drop system enabled: %s\033[0m", drop_enabled ? "true" : "false");
    ROS_INFO("\033[32m[DropSystem] Precision threshold: %.1f px\033[0m", drop_precision_threshold);
    ROS_INFO("\033[32m[DropSystem] Height threshold: %.3f m (not used)\033[0m", drop_height_threshold);
    ROS_INFO("\033[32m[DropSystem] Descent stable duration: %.1f s\033[0m", descent_stable_duration);
    for (int slot = 0; slot < 3; ++slot) {
        ROS_INFO("[DropSystem] slot %d offsets standard=(%.3f, %.3f) dynamic=(%.3f, %.3f)",
                 slot + 1, drop_slot_offsets_[slot][0], drop_slot_offsets_[slot][1],
                 dynamic_drop_slot_offsets_[slot][0], dynamic_drop_slot_offsets_[slot][1]);
    }
    ROS_INFO("\033[36m[UavVision] selected_target_timeout: %.2f s, drop_offset_timeout: %.2f s\033[0m",
             selected_target_timeout_, drop_offset_timeout_);
    ROS_INFO("\033[36m[UavVision] release_permission_timeout: %.2f s topic=%s\033[0m",
             mission_release_permission_timeout_,
             mission_release_permission_topic_.c_str());
    ROS_INFO("\033[36m[UavVision] pixel_to_meter_ratio: %.4f, max_alignment_move_distance: %.2f\033[0m",
             pixel_to_meter_ratio_, max_alignment_move_distance_);
    ROS_INFO("[UavVision] pixel_to_body_matrix: [%.2f %.2f; %.2f %.2f]",
             pixel_to_body_matrix_[0], pixel_to_body_matrix_[1],
             pixel_to_body_matrix_[2], pixel_to_body_matrix_[3]);
    ROS_INFO("\033[36m[UavVision] target radii(circle/cross/landing): %.2f / %.2f / %.2f m, tank interrupt: %s\033[0m",
             drop_circle_radius_m_, drop_cross_radius_m_, landing_pad_radius_m_,
             enable_selected_tank_interrupt_ ? "true" : "false");
    ROS_INFO("[UavVision] update_goal_from_selected_target: %s",
             update_goal_from_selected_target_ ? "true" : "false");
    ROS_INFO("[UavVision] require_release_permission: %s",
             require_vision_release_permission_ ? "true" : "false");
    ROS_INFO("[PatrolControl] external_mission_mode: %s command_topic=%s",
             external_mission_mode_ ? "true" : "false",
             mission_command_topic_.c_str());
    ROS_INFO("[PatrolControl] control_ready_topic: %s",
             control_ready_topic_.c_str());
    ROS_INFO("[PatrolControl] external_alignment_timeout: %.1f s",
             external_alignment_timeout_sec_);
    ROS_INFO(
        "[ExternalLanding] frame=%s capture=%.2f handoff=%.2f land=%.2f "
        "tol=%.2f mark_age=%.2f stable=%d timeout=%.1f",
        external_landing_frame_.c_str(), external_landing_capture_height_,
        external_landing_auto_land_height_, land_height,
        external_landing_alignment_tolerance_,
        external_landing_mark_max_age_sec_, external_landing_stable_frames_,
        external_landing_timeout_sec_);
    
    nh_.getParam("/debug", debug);

    // 从 ROS 参数服务器加载 "waypoints" 参数
    XmlRpc::XmlRpcValue waypoint_list_temp;
    if (nh_.getParam("waypoints", waypoint_list_temp))
    {
        // 确保参数是一个数组类型
        if (waypoint_list_temp.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            // 遍历每个路径点
            for (int i = 0; i < waypoint_list_temp.size(); ++i)
            {
                Waypoint wp;
                // 提取路径点的参数
                wp.x = static_cast<double>(waypoint_list_temp[i]["x"]);
                wp.y = static_cast<double>(waypoint_list_temp[i]["y"]);
                wp.z = static_cast<double>(waypoint_list_temp[i]["z"]);
                wp.yaw = static_cast<double>(waypoint_list_temp[i]["yaw"])*3.1415/180;
                wp.pointmode = static_cast<std::string>(waypoint_list_temp[i]["pointmode"]);
                
                // 读取悬停时间，如果没有设置则默认为0
                if (waypoint_list_temp[i].hasMember("hover_time")) {
                    wp.hover_time = static_cast<double>(waypoint_list_temp[i]["hover_time"]);
                } else {
                    wp.hover_time = 0.0;
                }

                // 添加到航路点列表
                waypoint_list.push_back(wp);

                if(debug)ROS_INFO("Loaded waypoint: x=%f, y=%f, z=%f, yaw=%f, pointmode=%s, hover_time=%f",
                            wp.x, wp.y, wp.z, wp.yaw, wp.pointmode.c_str(), wp.hover_time);
            }
        }
    }

    // takeoff point
    takeoff_point[0] = waypoint_list[0].x;
    takeoff_point[1] = waypoint_list[0].y;
    takeoff_point[2] = waypoint_list[0].z;
    detect_enable_msg_temp.data = false;
    landing_detect_control_pub_.publish(detect_enable_msg_temp);
}

void LLController::pub_goal(geometry_msgs::PoseStamped goal_msg){
    if (external_mission_mode_) {
        ROS_WARN_THROTTLE(5.0,
            "[PatrolControl] Ignoring legacy planner goal in external mission mode");
        return;
    }
    goal_msg.header.frame_id="camera_init";
    goal_msg.header.stamp = ros::Time::now();

    if(!flag_planner_px4){
        // 发送目标点，planner_planner将会控制飞机过去     
        now_pub_point[0] = goal_msg.pose.position.x;
        now_pub_point[1] = goal_msg.pose.position.y;
        now_pub_point[2] = goal_msg.pose.position.z;

        //for send only once
        if ((last_pub_point - now_pub_point).norm() >= planner_min_pub_threshould){
            if(debug)std::cout << "Planner goal_msg:" <<"["<<goal_msg.pose.position.x<<","<<goal_msg.pose.position.y<<","<<goal_msg.pose.position.z<<"]"<<std::endl;
            goal_msg.header.seq +=1;

            setplanner_goal_pub_.publish(goal_msg);
            last_pub_point[0]=goal_msg.pose.position.x;
            last_pub_point[1]=goal_msg.pose.position.y;
            last_pub_point[2]=goal_msg.pose.position.z;
        }
    }else{
        goal_msg.header.seq +=1;
        patrol_cmd = goal_msg;
    }
}

void LLController::NextPoint() {
    if (external_mission_mode_) {
        ROS_WARN_THROTTLE(5.0,
            "[PatrolControl] NextPoint disabled in external mission mode");
        return;
    }
    // 只在主任务中调用
    if (current_task_type != MAIN_MISSION) {
        ROS_WARN("\033[33m[NextPoint] Called during cross mission, ignoring!\033[0m");
        return;
    }
    
    // 路点指数加一
    waypoint_now = waypoint_next;
    // 3 投后是否直接跳过中间航点到降落段（旧设计行为）；detect_skip_enable=false
    // 时正常顺序推进（走廊等中间航点才会被执行）
    if(detect_skip_enable_ && detect_point_counter >= 3 && waypoint_now < (waypoint_list.size() - waypoint_skipping_index)){
        waypoint_next = waypoint_list.size() - waypoint_skipping_index;
        std_msgs::Bool stop_detection;
        stop_detection.data = false;
        cross_control_pub_.publish(stop_detection);
        detect_control_pub_.publish(stop_detection);
        // Point_mode = Nothing_point;
    }
    else{
        waypoint_next++ ;
    }
    
    // 确保不超出航路点列表范围
    if(waypoint_next >= waypoint_list.size()){  
        waypoint_next = waypoint_list.size() - 1; 
        ROS_WARN("\033[33m[NextPoint] Reached final waypoint, staying at waypoint %d\033[0m", waypoint_next);
    }
    
    Point_mode = stringToPointmode(waypoint_list[waypoint_next].pointmode);
    // if(detect_point_counter >= 3) Point_mode = Nothing_point;

    // 确保在切换到下一个航路点时禁用圆形检测
    std_msgs::Bool detect_disable_msg;
    detect_disable_msg.data = false;
    detect_control_pub_.publish(detect_disable_msg);
    adjust_target_position[0] = waypoint_list[waypoint_next].x;
    adjust_target_position[1] = waypoint_list[waypoint_next].y;
    adjust_target_position[2] = waypoint_list[waypoint_next].z;
    adjust_target_position[3] = waypoint_list[waypoint_next].yaw;

    waypoint_mark_point.pose.position.x = waypoint_list[waypoint_next].x;
    waypoint_mark_point.pose.position.y = waypoint_list[waypoint_next].y;
    waypoint_mark_point.pose.position.z = waypoint_list[waypoint_next].z;
    waypoint_mark_point.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);

    align_ok = false;
    
    // 重置检测相关状态
    resetDetectionState();  // 重置所有检测状态
    resetDropState();  // 重置投递状态
    //resetCrossDetectionState();  // 重置十字检测状态
    
    // 到达新航路点后重新启用十字检测（仅在未完成十字投递时）
    if (!cross_drop_completed) {
        cross_mark = true;
        ROS_INFO("\033[36m[NextPoint] Cross detection enabled for new waypoint\033[0m");
    } else {
        cross_mark = false;
        ROS_INFO("\033[36m[NextPoint] Cross detection remains disabled - already completed\033[0m");
    }

    std::cout<<"\033[32mHave arrive Point number : "<<waypoint_now<<" \nDrone pose now [x ,y, z] : "<<uav_pose.pose.position.x<<", " \
    <<uav_pose.pose.position.y<<", "<<uav_pose.pose.position.z<<std::endl;
    std::cout<<"\033[42;37mNext Point number : "<<waypoint_next<<" ;\n"<<"Next Point Pose: x = "<<waypoint_list[waypoint_next].x<<
    ", y = "<<waypoint_list[waypoint_next].y<<", z = "<<waypoint_list[waypoint_next].z <<" \033[0m"<<std::endl;
    
    // 输出下一个点的模式信息
    std::cout<<"\033[36mNext Point Mode: "<<waypoint_list[waypoint_next].pointmode<<"\033[0m"<<std::endl;
}
bool LLController::DynamicProcess()
{
    static ros::Time last_output_time = ros::Time::now();
    double output_interval = 0.2; // 每0.2秒输出一次
    bool should_drop = false;
    double dis_to_next_position = 0;
    
    
    // 初始化检测状态
    if(first_call)
    {   
        detection_start_time = ros::Time::now();
        have_waypoint_mark = false;
        first_call = false;
        times_detect = 0;
        count_aligning = 0;
        drop_complete = false;
        down_flag = true;
        dynamic_first_flag = true;
        adjust_target_position[0] = uav_pose.pose.position.x;
        adjust_target_position[1] = uav_pose.pose.position.y;
        waypoint_temp.pose.position.x = uav_pose.pose.position.x;
        waypoint_temp.pose.position.y = uav_pose.pose.position.y;
        tank_mark_point.pose.position.x = uav_pose.pose.position.x;
        tank_mark_point.pose.position.y = uav_pose.pose.position.y;
        ROS_INFO("\033[34m[DynamicProcess] Starting detection alignment for waypoint %d, resetting all flags\033[0m", waypoint_next);
    }
    std::cout<<"have_waypoint_mark = "<<have_waypoint_mark<<std::endl;
    // if(have_waypoint_mark){
    //     ROS_INFO("\033[32m[WayPointDetectDone] have_waypoint_mark: true");
    //     adjust_target_position[0] = waypoint_mark_point.pose.position.x;  
    //     adjust_target_position[1] = waypoint_mark_point.pose.position.y;
    //     adjust_target_position[2] = align_height;
    //     adjust_target_position[3] = tf::getYaw(waypoint_mark_point.pose.orientation);
    //     // 更新检测次数
    //     times_detect++;
    //     ROS_INFO("\033[32m[WayPointDetectDone] Received valid target point, detection count: %d\033[0m", times_detect);
    //     ROS_INFO("\033[32m[WayPointDetectDone] target_x: %.2f, target_y: %.2f", waypoint_mark_point.pose.position.x, waypoint_mark_point.pose.position.y);
    // }else{
    //     // 没有目标就发送原始航路点
    //     adjust_target_position[0] = waypoint_list[waypoint_next].x;
    //     adjust_target_position[1] = waypoint_list[waypoint_next].y;
    //     adjust_target_position[2] = waypoint_list[waypoint_next].z;
    //     adjust_target_position[3] = waypoint_list[waypoint_next].yaw;

    //     ROS_DEBUG_THROTTLE(2, "\033[33m[WayPointDetectDone] Waiting for valid target point...\033[0m");
    //     ROS_INFO("\033[32m[WayPointDetectDone] adjust_target_position: %.2f, %.2f", adjust_target_position[0], adjust_target_position[1]);
    // }
    
    // 计算时间进度
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - detection_start_time).toSec();

    // 防止除零错误
    int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
    double clamped_time_threshould = std::max(0.1f, waypoint_adjust_max_second_threshould);

    // 计算进度比例 [0, 1]
    double detect_progress = std::min(1.0, static_cast<double>(times_detect) / clamped_detect_threshould);
    double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);

    // 控制输出频率
    if ((current_time - last_output_time).toSec() >= output_interval) 
    {
        last_output_time = current_time;

        // 构建进度条
        int bar_width = 20;
        auto buildProgressBar = [&](double progress) {
            std::string bar = "";
            for (int i = 0; i < bar_width; i++) {
                bar += (i < static_cast<int>(progress * bar_width)) ? "#" : "-";
            }
            return bar;
        };

        std::string detect_bar = buildProgressBar(detect_progress);
        std::string time_bar = buildProgressBar(time_progress);

        std::cout << "\033[36m[WayPointDetectDone] Alignment Progress: [" << detect_bar << "] " << times_detect << "/" << clamped_detect_threshould 
                  << " | Time Progress: [" << time_bar << "] " << elapsed_time << "/" << clamped_time_threshould << " sec\033[0m" 
                  << std::endl;
    }
    // ROS_INFO("\033[32m[WayPointDetectDone] align_ok: %d\033[0m", align_ok);
    // 完成条件：检测次数或时间达到阈值
    // waypoint_temp.pose.position.x = waypoint_mark_point.pose.position.x;
    // waypoint_temp.pose.position.y = waypoint_mark_point.pose.position.y;
    
    dis_to_next_position = distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z, 
                                      waypoint_mark_point.pose.position.x,waypoint_mark_point.pose.position.y,waypoint_mark_point.pose.position.z);
    double time_threshould = 45;
    int servo_id = detect_point_counter + 1;
    if (elapsed_time <= time_threshould) 
    {
        // drop_complete 已经在 first_call 中重置，这里不需要额外处理
        //int servo_id = detect_point_counter + 1;
        if(tank_found_ && dynamic_first_flag)
        {
            ROS_INFO("\033[32m[DynamicProcess] dis_to_next_position: %.2f\033[0m", dis_to_next_position);
            down_flag = true;
            should_drop = false;
            adjust_target_position[0] = tank_mark_point.pose.position.x;
            adjust_target_position[1] = tank_mark_point.pose.position.y;
            waypoint_temp.pose.position.x = tank_mark_point.pose.position.x;
            waypoint_temp.pose.position.y = tank_mark_point.pose.position.y;
            align_height = 1.2;
            dynamic_start_time = ros::Time::now();
            // ROS_INFO("dynamic_start_time %.2f",dynamic_start_time);
            waypoint_temp.pose.orientation = waypoint_mark_point.pose.orientation;
            ROS_INFO("[DynamicProcess]Init waypoint_temp x:%.2f y:%.2f",waypoint_temp.pose.position.x,waypoint_temp.pose.position.y);
            dynamic_first_flag = false;
        }
        dynamic_current_time = ros::Time::now();
        float dynamic_time = (dynamic_current_time - dynamic_start_time).toSec();
        ROS_INFO("[DynamicProcess]distance111 = %.2f",distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, 0.0, 
                                      waypoint_temp.pose.position.x,waypoint_temp.pose.position.y,0.0));
        ROS_INFO("[DynamicProcess]dynamic_time = %.2f",dynamic_time);
        ROS_INFO("\033[32m[DynamicProcess]waypoint_temp: %.2f, %.2f\033[0m", waypoint_temp.pose.position.x, waypoint_temp.pose.position.y);
        if(distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, 0.0, 
                                      waypoint_temp.pose.position.x,waypoint_temp.pose.position.y,0.0) <= 0.1){
                                        dynamic_record_activate = true;
                                      }
        else{
            adjust_target_position[0] = waypoint_temp.pose.position.x;
            adjust_target_position[1] = waypoint_temp.pose.position.y;
        }
        if(have_waypoint_mark && dynamic_record_activate && !dynamic_first_flag)
        {
            adjust_target_position[0]=waypoint_mark_point.pose.position.x;
            adjust_target_position[1]=waypoint_mark_point.pose.position.y;
            if(dis_to_next_position <= 0.07 && count_aligning < 50){
                ROS_INFO("\033[32m[CrossDetectionDone] dis_to_next_position: %.2f\033[0m", dis_to_next_position);
                count_aligning++;
                down_flag = true;
                should_drop = false;
                waypoint_temp.pose.position.x = waypoint_mark_point.pose.position.x;
                waypoint_temp.pose.position.y = waypoint_mark_point.pose.position.y;
                waypoint_temp.pose.orientation = waypoint_mark_point.pose.orientation;
            }
            if(count_aligning >= 50){
                ROS_INFO("\033[32m[CrossDetectionDone]waypoint_temp: %.2f, %.2f\033[0m", waypoint_temp.pose.position.x, waypoint_temp.pose.position.y);
                adjust_target_position[0] = waypoint_temp.pose.position.x;
                adjust_target_position[1] = waypoint_temp.pose.position.y;
                align_height = 0.10;
                adjust_target_position[3] = tf::getYaw(waypoint_temp.pose.orientation);
                applyDropSlotOffset(servo_id, true);
                double ttt = distance3d(uav_pose.pose.position.x,uav_pose.pose.position.y,uav_pose.pose.position.z,waypoint_temp.pose.position.x,waypoint_temp.pose.position.y,0.1);
                ROS_INFO("\033[32m[CrossDetectionDone] should_drop: %d, drop_complete: %d, uav_pose.pose.position.z: %.2f\033[0m", should_drop, drop_complete, uav_pose.pose.position.z);
                if(uav_pose.pose.position.z <= 0.17 && ttt <= 0.15 && !drop_complete)
                {
                    should_drop = true;
                    ROS_INFO("\033[32m[CrossDetectionDone] height_reach should_drop: true\033[0m");
                }
                
                const DropReleaseGate release_gate = currentDropReleaseGate();
                const bool release_authorized = canRequestDrop(
                    require_vision_release_permission_, release_gate);
                if (should_drop && !drop_complete && !release_authorized) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[DynamicProcess] Waiting for mission release permission "
                        "(active=%s fresh=%s)",
                        release_gate.mission_permission_active ? "true" : "false",
                        release_gate.mission_permission_fresh ? "true" : "false");
                    return false;
                }
                if (should_drop && !drop_complete) {
                    // 执行投递动作 - 按顺序使用舵机
                    ignore_servo_complete = false;  //FF 开始接受舵机完F成信号
                    const DropActionResult result = executeDropAction(servo_id);
                    if (dropActionSucceeded(result)) {
                        drop_complete = true;
                        if (detect_point_counter >= 0 &&
                            detect_point_counter < static_cast<int>(drop_completed.size())) {
                            drop_completed[detect_point_counter] = true;
                        }
                    }
                    should_drop = false;
                    if (!drop_complete) {
                        ROS_WARN_THROTTLE(1.0,
                            "[DynamicProcess] Drop slot %d not acknowledged; retrying while conditions remain valid",
                            servo_id);
                        return false;
                    }
                } else if (should_drop && drop_complete) {
                    ROS_INFO_THROTTLE(1.0, "\033[33m[CrossDetectionDone] Drop already completed for detect point %d, waiting for next cycle\033[0m", detect_point_counter);
                } else {
                    ROS_WARN_THROTTLE(1.0, "\033[33m[CrossDetectionDone] Height not met or already completed for detect point %d (counter: %d)\033[0m", 
                            detect_point_counter + 1, detect_point_counter);
                }            
                // 防止重复投递：如果已经投递过，直接跳过
                if(servo_complete.data){
                    ROS_INFO("\033[33m[CrossDetectionDone] servo_complete.data: %d\033[0m", servo_complete.data);
                    down_flag = false;
                    align_height = 1.15;
                    if(uav_pose.pose.position.z >= 0.95){
                        // stopDropAction(servo_id-1);
                        resetDropState();   // 重置投递状态
                        cleanupAfterCrossDrop();  // 彻底清理十字投递状态
                        // tank_mark = 
                        // detect_point_counter++;
                        tank_drop_completed = true;
                        tank_mission_completed = true;  // 标记十字任务完成
                        detect_point_counter++;
                        ROS_INFO("\033[32m[CrossDetectionDone] Cross drop completed successfully, returning to main mission\033[0m");
                        ROS_INFO("\033[32m[CrossDetectDone] Secend detect_point_counter incremented to: %d\033[0m", detect_point_counter);
                        return true;
                    }
                    ROS_INFO("\033[33m[CrossDetectionDone] Height Not ready,heignt not reached and not get to 1.2m current align_heignt: %.2f  uav_z : %.2f\033[0m", align_height, uav_pose.pose.position.z);
                    return false;
                }
                else{
                    ROS_WARN_THROTTLE(1.0,
                        "[CrossDetectionDone] Awaiting positive Servo ACK; release remains blocked");
                    return false;  // 未能够完成舵机任务
                }
            }

            // return false;
            // else{
            //     ROS_INFO("Not Suitable");
            //     double temp_x = waypoint_mark_point.pose.position.x;
            //     double temp_y = waypoint_mark_point.pose.position.y;
            //     if(have_waypoint_mark){
            //         switch(detect_point_counter){
            //         case 0:{
            //             waypoint_mark_point.pose.position.x = temp_x - 0.07;
            //             waypoint_mark_point.pose.position.y = temp_y;
            //             break;
            //         }
            //         case 1:{
            //             waypoint_mark_point.pose.position.x = temp_x;
            //             waypoint_mark_point.pose.position.y = temp_y - 0.07;
            //             break;
            //         }
            //         case 2:{
            //             waypoint_mark_point.pose.position.x = temp_x;
            //             waypoint_mark_point.pose.position.y = temp_y + 0.07;
            //             break;
            //         }
            //         default:
            //             waypoint_mark_point.pose.position.x = temp_x;
            //             waypoint_mark_point.pose.position.y = temp_y;
            //             break;
            //         }
            //         have_waypoint_mark = false;
            //     }
                
            //     adjust_target_position[0] = (waypoint_mark_point.pose.position.x - uav_pose.pose.position.x) / 4.0 + uav_pose.pose.position.x;
            //     adjust_target_position[1] = (waypoint_mark_point.pose.position.y - uav_pose.pose.position.y) / 4.0 + uav_pose.pose.position.y;
            // }
            // double distance = distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z,waypoint_mark_point.pose.position.x,waypoint_mark_point.pose.position.y,uav_pose.pose.position.z);
            //     ROS_INFO("\033[32m[DynamicProcess]dis_to_next_position: %.2f\033[0m", dis_to_next_position);
            //     ROS_INFO("[DynamicProcess]distance_E(s) : %.2f",distance - Edistance);
            //     // ROS_INFO("[DynamicProcess]vector record : %d %.2f %.2f %.2f",dynamic_point_index,dynamic_point_list[dynamic_point_index][0],dynamic_point_list[dynamic_point_index][1],dynamic_point_list[dynamic_point_index][2]);
            //     if(!dynamic_record_activate && distance <= min_distance && Edistance != 0.0){
                   
            //         // ROS_INFO("[DynamicProcess]distance_E(s) : %.2f",dynamic_point_list[dynamic_point_index][0] - dynamic_point_list[dynamic_point_index-1][0]);
            //         if(distance > Edistance - 0.005)
            //         {
            //             adjust_target_position[0] = waypoint_mark_point.pose.position.x;
            //             adjust_target_position[1] = waypoint_mark_point.pose.position.y;
            //             ROS_INFO("\033[32m[DynamicProcess]adjust back: %.2f, %.2f\033[0m",adjust_target_position[0],adjust_target_position[1]);
            //             min_distance = distance;
            //             dynamic_record_activate = true;
            //         }
            //     }
            //     dynamic_point_index++;
            //     Edistance = distance;
            // switch(detect_point_counter){
            //     case 0:{
            //         adjust_target_position[0] = waypoint_temp.pose.position.x - 0.07;
            //         break;
            //     }
            //     case 1:{
            //         adjust_target_position[1] = waypoint_temp.pose.position.y - 0.07;
            //         break;
            //     }
            //     case 2:{
            //         adjust_target_position[1] = waypoint_temp.pose.position.y + 0.07;
            //         break;
            //     }
            //     default:
            //         break;
            // }
            // if(servo_complete.data){
            //     ROS_INFO("\033[33m[WayPointDetectDone] servo_complete.data: %d\033[0m", servo_complete.data);
            //     // down_flag = false;
            //     // align_height = 1.2;
            //     first_call = true;  // 重置标志
            //     times_detect = 0;
            //     should_drop = false;
            //     align_ok = false;
            //     count_aligning = 0;
            //     stopDropAction(servo_id-1);
            //     resetDropState();   // 重置投递状态
            //     drop_complete = false;
            //     servo_complete.data = false;
            //     align_height = 1.2;
            //     time_temp = 0;
            //     drop_time_flag = false;
            //     detect_point_counter++;
            //     ROS_INFO("\033[33m[WayPointDetectDone] servo_complete.data: %d\033[0m", servo_complete.data);
            //     ROS_INFO("\033[33m[WayPointDetectDone] Landing completed, servo status true mission over. ");
            //     return true;
            // }
            // have_waypoint_mark = false;
            ROS_INFO("\033[33m[WayPointDetectDone] Height Not ready,heignt not reached and not get to 1.2m current align_heignt: %.2f  uav_z : %.2f\033[0m", align_height, uav_pose.pose.position.z);
            return false;
        }
        ROS_INFO("[DynammicProcess]flag not ready ,check logic");
        return false;
    }
    else{
        // 检测完成，重置所有状态为下一个航路点做准备
        count_aligning = 0;
        first_call = true;
        times_detect = 0;
        drop_complete = false;
        drop_time_flag = false;
        down_flag = true;
        align_ok = false;  // 重置对准状态
        servo_complete.data = false;
        // stopDropAction(servo_id-1);
        align_height = 1.2;
        detect_point_counter++;
        ROS_INFO("\033[33m[TankDetectDone] Time up, servo_complete.data: %d\033[0m", servo_complete.data);
        ROS_INFO("\033[32m[TankDetectDone] Detection completed for waypoint %d, all flags reset\033[0m", waypoint_next);
        ROS_INFO("\033[32m[TankDetectDone] Secend detect_point_counter incremented to: %d\033[0m", detect_point_counter);
        return true;
    }
    return false; // 默认返回值，防止控制流到达函数末尾
}
// TODO:加入降落投递加上升的逻辑
bool LLController::WayPointDetectDone()
{
    static ros::Time last_output_time = ros::Time::now();
    double output_interval = 0.2; // 每0.2秒输出一次
    bool should_drop = false;
    double dis_to_next_position = 0;
    
    // 初始化检测状态
    if(first_call)
    {   
        detection_start_time = ros::Time::now();
        first_call = false;
        times_detect = 0;
        count_aligning = 0;
        drop_complete = false;
        align_ok = false;
        // 外部 Mission Manager 已在 MissionCommand::ALIGN 中写入语义候选地图点。
        // 这里若仍用旧 waypoint_list 覆盖，第三个及后续候选会飞回固定检测点。
        // 旧固定航线模式保持原行为不变。
        if (external_mission_mode_) {
            waypoint_temp.pose = waypoint_mark_point.pose;
            align_ok = have_waypoint_mark;
        } else {
            adjust_target_position[0] = waypoint_list[waypoint_next].x;
            adjust_target_position[1] = waypoint_list[waypoint_next].y;
            waypoint_temp.pose.position.x = waypoint_list[waypoint_next].x;
            waypoint_temp.pose.position.y = waypoint_list[waypoint_next].y;
            waypoint_mark_point.pose.position.x = waypoint_list[waypoint_next].x;
            waypoint_mark_point.pose.position.y = waypoint_list[waypoint_next].y;
        }
        ROS_INFO("\033[34m[WayPointDetectDone] Starting detection alignment for waypoint %d, resetting all flags\033[0m", waypoint_next);
    }

    // ROS_INFO("dis_tooooo %f",dis_to_next_position);
    // std::cout<<"have_waypoint_mark = "<<have_waypoint_mark<<std::endl;
    if(have_waypoint_mark){
        ROS_INFO("\033[32m[WayPointDetectDone] have_waypoint_mark: true");
        adjust_target_position[0] = waypoint_mark_point.pose.position.x;  
        adjust_target_position[1] = waypoint_mark_point.pose.position.y;
        adjust_target_position[2] = align_height;
        adjust_target_position[3] = tf::getYaw(waypoint_mark_point.pose.orientation);
        // 更新检测次数
        times_detect++;
        ROS_INFO("\033[32m[WayPointDetectDone] Received valid target point, detection count: %d\033[0m", times_detect);
        ROS_INFO("\033[32m[WayPointDetectDone] have_waypoint_mark adjust_target_position: %.2f %.2f", waypoint_mark_point.pose.position.x, waypoint_mark_point.pose.position.y);
    }else{
        // 没有目标就发送原始航路点
        adjust_target_position[0] = waypoint_list[waypoint_next].x;
        adjust_target_position[1] = waypoint_list[waypoint_next].y;
        adjust_target_position[2] = waypoint_list[waypoint_next].z;
        adjust_target_position[3] = waypoint_list[waypoint_next].yaw;

        ROS_DEBUG_THROTTLE(2, "\033[33m[WayPointDetectDone] Waiting for valid target point...\033[0m");
        ROS_INFO("\033[32m[WayPointDetectDone] no valid circle : adjust_target_position: %.2f, %.2f", adjust_target_position[0], adjust_target_position[1]);
    }
    
    // 计算时间进度
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - detection_start_time).toSec();

    // 防止除零错误
    int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
    double clamped_time_threshould = std::max(0.1f, waypoint_adjust_max_second_threshould);

    // 计算进度比例 [0, 1]
    double detect_progress = std::min(1.0, static_cast<double>(times_detect) / clamped_detect_threshould);
    double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);

    // 控制输出频率
    if ((current_time - last_output_time).toSec() >= output_interval) {
        last_output_time = current_time;

        // 构建进度条
        int bar_width = 20;
        auto buildProgressBar = [&](double progress) {
            std::string bar = "";
            for (int i = 0; i < bar_width; i++) {
                bar += (i < static_cast<int>(progress * bar_width)) ? "#" : "-";
            }
            return bar;
        };

        std::string detect_bar = buildProgressBar(detect_progress);
        std::string time_bar = buildProgressBar(time_progress);

        std::cout << "\033[36m[WayPointDetectDone] Alignment Progress: [" << detect_bar << "] " << times_detect << "/" << clamped_detect_threshould 
                  << " | Time Progress: [" << time_bar << "] " << elapsed_time << "/" << clamped_time_threshould << " sec\033[0m" 
                  << std::endl;
    }
    ROS_INFO("\033[32m[WayPointDetectDone] align_ok: %d\033[0m", align_ok);
    // 完成条件：检测次数或时间达到阈值
    double time_threshould = 0;

    dis_to_next_position = distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z,
                                      adjust_target_position[0],adjust_target_position[1],uav_pose.pose.position.z);
    ROS_INFO("error point %f",dis_to_next_position);
    if(!align_ok){
        time_threshould = 4;
    }
    else{
        time_threshould = external_mission_mode_
            ? external_alignment_timeout_sec_ : 45.0;
    }
    int servo_id = detect_point_counter + 1;
    if (elapsed_time <= time_threshould) {
        // drop_complete 已经在 first_call 中重置，这里不需要额外处理
        //int servo_id = detect_point_counter + 1;
        ROS_INFO("time is effective,dis_to_next: %f",dis_to_next_position);
        if(have_waypoint_mark &&
           count_aligning < 70 &&
           align_ok == 1 &&
           (dis_to_next_position <= 0.1 || (current_align_mode_ == "drop_circle" && uav_drop_ready_))){
            ROS_INFO("\033[32m[CrossDetectionDone] dis_to_next_position: %.2f\033[0m", dis_to_next_position);
            count_aligning++;
            // down_flag = true;
            should_drop = false;
            waypoint_temp.pose.position.x = waypoint_mark_point.pose.position.x;
            waypoint_temp.pose.position.y = waypoint_mark_point.pose.position.y;
            waypoint_temp.pose.orientation = waypoint_mark_point.pose.orientation;
        }
        if(count_aligning >= 70 && align_ok == 1){
            ROS_INFO("\033[32m[CrossDetectionDone]waypoint_temp: %.2f, %.2f\033[0m", waypoint_temp.pose.position.x, waypoint_temp.pose.position.y);
            adjust_target_position[0] = waypoint_temp.pose.position.x;
            adjust_target_position[1] = waypoint_temp.pose.position.y;
            align_height = 0.10;
            adjust_target_position[3] = tf::getYaw(waypoint_temp.pose.orientation);
            applyDropSlotOffset(servo_id, false);
            ROS_INFO("\033[32m[CrossDetectionDone] should_drop: %d, drop_complete: %d, uav_pose.pose.position.z: %.2f\033[0m", should_drop, drop_complete, uav_pose.pose.position.z);
            ROS_INFO("dis %f",dis_to_next_position);
            ROS_INFO("waypoint temp %f %f",waypoint_temp.pose.position.x,waypoint_temp.pose.position.y);
            ROS_INFO("\033[32m[WayPointDetectDone] time_temp : %f  ,should_drop: %d, drop_complete: %d, uav_pose.pose.position.z: %.2f\033[0m",time_temp, should_drop, drop_complete, uav_pose.pose.position.z);
            double ttt = distance3d(uav_pose.pose.position.x,uav_pose.pose.position.y,uav_pose.pose.position.z,waypoint_temp.pose.position.x,waypoint_temp.pose.position.y,0.1);
            ROS_INFO("\033[32m[CrossDetectionDone] should_drop: %d, drop_complete: %d, uav_pose.pose.position.z: %.2f\033[0m", should_drop, drop_complete, uav_pose.pose.position.z);
            if(uav_pose.pose.position.z <= 0.17 && ttt <= 0.15 && !drop_complete)
            {
                should_drop = true;
                // drop_time_flag = false;
                ROS_INFO("\033[32m[WayPointDetectDone] height_reach should_drop: true\033[0m");
            }
            
            const DropReleaseGate release_gate = currentDropReleaseGate();
            const bool release_authorized = canRequestDrop(
                require_vision_release_permission_, release_gate);
            if (should_drop && !drop_complete && !release_authorized) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[WayPointDetectDone] Waiting for mission release permission "
                    "(active=%s fresh=%s)",
                    release_gate.mission_permission_active ? "true" : "false",
                    release_gate.mission_permission_fresh ? "true" : "false");
                return false;
            }
            if (should_drop && !drop_complete) {
                // 执行投递动作 - 按顺序使用舵机
                // drop_time_flag = false;
                ignore_servo_complete = false;  // 开始接受舵机完成信号
                const DropActionResult result = executeDropAction(servo_id);
                if (dropActionSucceeded(result)) {
                    drop_complete = true;
                    if (detect_point_counter >= 0 &&
                        detect_point_counter < static_cast<int>(drop_completed.size())) {
                        drop_completed[detect_point_counter] = true;
                    }
                }
                should_drop = false;
                if (!drop_complete) {
                    ROS_WARN_THROTTLE(1.0,
                        "[WayPointDetectDone] Drop slot %d not acknowledged; retrying while conditions remain valid",
                        servo_id);
                    return false;
                }
            } else if (should_drop && drop_complete) {
                ROS_INFO_THROTTLE(1.0, "\033[33m[WayPointDetectDone] Drop already completed for detect point %d, waiting for next cycle\033[0m", detect_point_counter);
            } else {
                ROS_WARN_THROTTLE(1.0, "\033[33m[WayPointDetectDone] Height not met or already completed for detect point %d (counter: %d)\033[0m", 
                        detect_point_counter + 1, detect_point_counter);
            }
            ROS_INFO("time_temp : %f" , time_temp);
            // 防止重复投递：如果已经投递过，直接跳过 / 时间到达直接起飞
            if(servo_complete.data){
                ROS_INFO("\033[33m[WayPointDetectDone] servo_complete.data: %d\033[0m", servo_complete.data);
                // down_flag = false;
                align_height = 1.2;
                if(uav_pose.pose.position.z >= 1.0){
                    first_call = true;  // 重置标志
                    times_detect = 0;
                    should_drop = false;
                    align_ok = false;
                    count_aligning = 0;
                    // stopDropAction(servo_id-1);
                    resetDropState();   // 重置投递状态
                    drop_complete = false;
                    servo_complete.data = false;
                    align_height = 1.2;
                    time_temp = 0;
                    detect_point_counter++;
                    // drop_time_flag = false;
                    temp_storage = false;
                    ROS_INFO("\033[33m[WayPointDetectDone] servo_complete.data: %d\033[0m", servo_complete.data);
                    ROS_INFO("\033[33m[WayPointDetectDone] Landing completed, servo status true mission over. ");
                    ROS_INFO("\033[32m[WayPointDetectDone] Secend detect_point_counter incremented to: %d\033[0m", detect_point_counter);
                    return true;
                }
                ROS_INFO("\033[33m[WayPointDetectDone] Height Not ready,heignt not reached and not get to 1.2m current align_heignt: %.2f  uav_z : %.2f\033[0m", align_height, uav_pose.pose.position.z);
                return false;
            }
            else{
                ROS_WARN_THROTTLE(1.0,
                    "[WayPointDetectDone] Awaiting positive Servo ACK; release remains blocked");
                return false;  // 未能够完成舵机任务
            }
        }

        return false;
    }
    else{
        // 检测完成，重置所有状态为下一个航路点做准备
        count_aligning = 0;
        first_call = true;
        times_detect = 0;
        drop_complete = false;
        drop_time_flag = false;
        // down_flag = true;
        align_ok = false;  // 重置对准状态
        servo_complete.data = false;
        stopDropAction(servo_id);
        align_height = 1.2;
        ROS_INFO("\033[33m[WayPointDetectDone] Time up, servo_complete.data: %d\033[0m", servo_complete.data);
        ROS_INFO("\033[32m[WayPointDetectDone] Detection completed for waypoint %d, all flags reset\033[0m", waypoint_next);
        return true;
    }
}


bool LLController::LandDetectDone()
{
    ROS_INFO("Landing Point setting successful");
    static ros::Time last_output_time = ros::Time::now();
    double output_interval = 0.2; // 每0.5秒输出一次
    double dis_to_next_position;
    dis_to_next_position = distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z, 
                                      adjust_target_position[0], adjust_target_position[1], align_height);

    if (first_call)
    {   
        start_time = ros::Time::now();
        first_call = false;
        times_detect = 0;
        align_height = 0.75;
    }

    if(have_land_mark){
        // 更新检测目标点
        adjust_target_position[0] = land_mark_point.pose.position.x;
        adjust_target_position[1] = land_mark_point.pose.position.y;
        adjust_target_position[2] = align_height;
        adjust_target_position[3] = tf::getYaw(land_mark_point.pose.orientation);
        // 更新检测次数
        times_detect++;
        //have_waypoint_mark = false;
    }else{
        // 没有目标就发送最后一个着陆点
        adjust_target_position[0] = waypoint_list[waypoint_next].x;
        adjust_target_position[1] = waypoint_list[waypoint_next].y;
        adjust_target_position[2] = align_height;
        adjust_target_position[3] = waypoint_list[waypoint_next].yaw;
    }
    ROS_INFO("adjust %f , %f , %f " , adjust_target_position[0] , adjust_target_position[1],adjust_target_position[2]);

    // 计算时间进度
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - start_time).toSec();

    // 防止除零错误
    int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
    double clamped_time_threshould = std::max(0.1f, land_adjust_max_second_threshould);

    // 计算进度比例 [0, 1]
    double detect_progress = std::min(1.0, static_cast<double>(times_detect) / clamped_detect_threshould);
    double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);

    // 控制输出频率
    if ((current_time - last_output_time).toSec() >= output_interval) {
        last_output_time = current_time;

        // 构建进度条
        int bar_width = 20;
        auto buildProgressBar = [&](double progress) {
            std::string bar = "";
            for (int i = 0; i < bar_width; i++) {
                bar += (i < static_cast<int>(progress * bar_width)) ? "#" : "-";
            }
            return bar;
        };

        std::string detect_bar = buildProgressBar(detect_progress);
        std::string time_bar = buildProgressBar(time_progress);

        if(!flag_land)std::cout << "Landing Progress: [" << detect_bar << "] " << times_detect << "/" << clamped_detect_threshould 
                  << " | Time Progress: [" << time_bar << "] " << elapsed_time << "/" << clamped_time_threshould << " sec" 
                  << std::endl;
    }
    // 完成条件：检测次数或时间达到阈值
    if (elapsed_time <= 15) {
        if(have_land_mark &&
           count_aligning < 70 &&
           (dis_to_next_position <= 0.07 || (current_align_mode_ == "landing" && uav_drop_ready_))){
            ROS_INFO("\033[32m[CrossDetectionDone] dis_to_next_position: %.2f\033[0m", dis_to_next_position);
            count_aligning++;
            down_flag = true;
            //should_drop = false;
            waypoint_temp.pose.position.x = land_mark_point.pose.position.x;
            waypoint_temp.pose.position.y = land_mark_point.pose.position.y;
            waypoint_temp.pose.orientation = land_mark_point.pose.orientation;
        }
        if(count_aligning >= 70){
            ROS_INFO("\033[32m[LandDetectDone]waypoint_temp: %.2f, %.2f\033[0m", waypoint_temp.pose.position.x, waypoint_temp.pose.position.y);
            adjust_target_position[0] = waypoint_temp.pose.position.x;
            adjust_target_position[1] = waypoint_temp.pose.position.y;
            align_height = -1.0;
            adjust_target_position[3] = tf::getYaw(waypoint_temp.pose.orientation);
        }
        if(uav_pose.pose.position.z <= land_height){
            // align_height = land_height;
            return true;
        }
        return false;
    }
    else{
        ROS_INFO("\033[33m[LandDetectDone] timeout, Auto landing processing\033[0m");
        return true;
    }
    return false;
}

void LLController::ClassCallback(const std_msgs::String& msg)
{
    class_ = msg;
    if (classMatchesGoal(class_.data)) {
        align_ok = true;
    } else if (update_goal_from_selected_target_ &&
               (class_.data == "Nothing" || !class_.data.empty())) {
        align_ok = false;
    }
    if(class_.data != "")
    {
        ROS_INFO("Class:%s", class_.data.c_str());
    }
}

bool LLController::classMatchesGoal(const std::string& class_name) const
{
    for (const auto& target_name : goal) {
        if (class_name == target_name) {
            return true;
        }
    }
    return false;
}

bool LLController::hasFreshSelectedTarget() const
{
    if (!have_selected_target_) {
        return false;
    }
    return (ros::Time::now() - latest_selected_target_time_).toSec() <= selected_target_timeout_;
}

bool LLController::hasFreshDropOffset() const
{
    if (!have_drop_offset_) {
        return false;
    }
    return (ros::Time::now() - latest_drop_offset_time_).toSec() <= drop_offset_timeout_;
}

bool LLController::hasFreshMissionReleasePermission() const
{
    if (!mission_release_permission_active_) {
        return false;
    }
    return (ros::Time::now() - latest_mission_release_permission_time_).toSec() <=
           mission_release_permission_timeout_;
}

DropReleaseGate LLController::currentDropReleaseGate() const
{
    DropReleaseGate gate;
    gate.mission_permission_active = mission_release_permission_active_;
    gate.mission_permission_fresh = hasFreshMissionReleasePermission();
    return gate;
}

void LLController::clearUavVisionAlignmentState()
{
    have_drop_offset_ = false;
    uav_drop_ready_ = false;
    mission_release_permission_active_ = false;
    latest_drop_ready_reason_.clear();
    have_waypoint_mark = false;
    have_cross_mark = false;
    have_land_mark = false;
}

void LLController::updateGoalFromSelectedTarget(const std::string& class_name)
{
    static const std::vector<std::string> kStandardTargets = {
        "bridge", "panzer", "pillbox", "tent", "tank"
    };
    if (std::find(kStandardTargets.begin(), kStandardTargets.end(), class_name) == kStandardTargets.end()) {
        return;
    }
    if (goal.size() == 1 && goal[0] == class_name) {
        return;
    }
    goal = {class_name};
    ROS_INFO("[UavVision] active standard target -> %s", class_name.c_str());
}

void LLController::projectDropOffsetToTarget(const uav_vision::DropOffset& msg)
{
    if (current_align_mode_ == "disabled") {
        return;
    }
    if (external_mission_mode_ && external_landing_active_) {
        // External landing accepts only map-frame landing_pad observations
        // that passed landMarkCallback's timestamp/freshness/anchor checks.
        // The pixel-offset compatibility path must never overwrite that
        // validated snapshot.
        return;
    }

    const double pixel_error_x = msg.dx_px;
    const double pixel_error_y = msg.dy_px;
    const double radius_px = msg.radius_px;
    current_pixel_error = std::sqrt(pixel_error_x * pixel_error_x + pixel_error_y * pixel_error_y);

    double real_target_radius = drop_circle_radius_m_;
    if (current_align_mode_ == "drop_cross") {
        real_target_radius = drop_cross_radius_m_;
    } else if (current_align_mode_ == "landing") {
        real_target_radius = landing_pad_radius_m_;
    }

    double dynamic_pixel_to_meter_ratio = pixel_to_meter_ratio_;
    if (radius_px > 10.0) {
        dynamic_pixel_to_meter_ratio = real_target_radius / radius_px;
    }

    const std::array<double, 2> body_offset =
        projectPixelOffsetToBody(pixel_error_x, pixel_error_y,
                                 dynamic_pixel_to_meter_ratio,
                                 pixel_to_body_matrix_);
    const double yaw = tf::getYaw(uav_pose.pose.orientation);
    double world_offset_x = std::cos(yaw) * body_offset[0] -
                            std::sin(yaw) * body_offset[1];
    double world_offset_y = std::sin(yaw) * body_offset[0] +
                            std::cos(yaw) * body_offset[1];

    const double move_distance = std::sqrt(world_offset_x * world_offset_x + world_offset_y * world_offset_y);
    if (move_distance > max_alignment_move_distance_ && move_distance > 1e-6) {
        const double scale = max_alignment_move_distance_ / move_distance;
        world_offset_x *= scale;
        world_offset_y *= scale;
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = msg.header.stamp;
    target_pose.header.frame_id = uav_pose.header.frame_id.empty() ? "camera_init" : uav_pose.header.frame_id;
    target_pose.pose.position.x = uav_pose.pose.position.x + world_offset_x;
    target_pose.pose.position.y = uav_pose.pose.position.y + world_offset_y;
    target_pose.pose.position.z = align_height;
    target_pose.pose.orientation = uav_pose.pose.orientation;

    if (current_align_mode_ == "drop_cross") {
        cross_mark_point = target_pose;
        have_cross_mark = true;
    } else if (current_align_mode_ == "landing") {
        land_mark_point = target_pose;
        have_land_mark = true;
    } else {
        waypoint_mark_point = target_pose;
        have_waypoint_mark = true;
    }
}

void LLController::selectedTargetCallback(const uav_vision::TargetCandidate::ConstPtr& msg)
{
    latest_selected_target_ = *msg;
    latest_selected_target_time_ = ros::Time::now();
    have_selected_target_ = true;

    if (update_goal_from_selected_target_) {
        updateGoalFromSelectedTarget(msg->class_name);
    }
    ROS_INFO_THROTTLE(1.0, "[UavVision] selected_target: %s (obs=%u conf=%.2f geom=%.2f)",
                      msg->class_name.c_str(), msg->observe_count,
                      msg->class_confidence, msg->geometry_confidence);
}

void LLController::dropOffsetCallback(const uav_vision::DropOffset::ConstPtr& msg)
{
    latest_drop_offset_ = *msg;
    latest_drop_offset_time_ = ros::Time::now();
    have_drop_offset_ = true;
    projectDropOffsetToTarget(*msg);
}

void LLController::dropReadyCallback(const uav_vision::DropReady::ConstPtr& msg)
{
    latest_drop_ready_time_ = ros::Time::now();
    latest_drop_ready_reason_ = msg->reason;
    uav_drop_ready_ = msg->ready;
    drop_condition_met = msg->ready;
    ROS_INFO_THROTTLE(1.0, "[UavVision] drop_ready=%s reason=%s",
                      uav_drop_ready_ ? "true" : "false",
                      latest_drop_ready_reason_.c_str());
}

void LLController::missionReleasePermissionCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    latest_mission_release_permission_time_ = ros::Time::now();
    mission_release_permission_active_ = msg->data;
    ROS_INFO_THROTTLE(
        1.0, "[UavVision] mission release permission=%s",
        mission_release_permission_active_ ? "true" : "false");
}

void LLController::missionCommandCallback(
    const patrol_control::MissionCommand::ConstPtr& msg)
{
    if (!external_mission_mode_) {
        ROS_WARN_THROTTLE(
            5.0, "[PatrolControl] Ignoring mission command while legacy mode is active");
        return;
    }
    if (!flag_takeoff_done) {
        ROS_WARN_THROTTLE(
            2.0, "[PatrolControl] Ignoring mission command before takeoff completes");
        return;
    }

    switch (msg->command) {
        case patrol_control::MissionCommand::SEARCH:
        case patrol_control::MissionCommand::APPROACH:
        case patrol_control::MissionCommand::RESUME:
        case patrol_control::MissionCommand::RETURN_HOME:
            if (external_landing_auto_land_requested_) {
                ROS_ERROR(
                    "[ExternalLanding] refusing navigation command after AUTO.LAND handoff");
                return;
            }
            clearExternalLandingState(true);
            if (msg->command == patrol_control::MissionCommand::RESUME) {
                resetDetectionState();
            }
            current_task_type = MAIN_MISSION;
            Point_mode = Nothing_point;
            Drone_mode = Run_point;
            align_ok = false;
            ROS_INFO("[PatrolControl] External command=%u target=%u class=%s",
                     msg->command, msg->target_id, msg->target_class.c_str());
            break;

        case patrol_control::MissionCommand::ALIGN:
            if (external_landing_auto_land_requested_) {
                ROS_ERROR(
                    "[ExternalLanding] refusing ALIGN after AUTO.LAND handoff");
                return;
            }
            clearExternalLandingState(true);
            resetDetectionState();
            // 随机投放区红十字与标准靶共用同一使命层队列，仅按目标类别选择
            // 对齐状态机分支（十字走 CrossDetectionDone，其余走圆环流程）。
            current_task_type = (msg->target_class == "red_cross")
                ? CROSS_MISSION : MAIN_MISSION;
            Point_mode = Detect_point;
            Drone_mode = Aligning;
            goal.clear();
            if (!msg->target_class.empty()) {
                goal.push_back(msg->target_class);
            }
            waypoint_mark_point = msg->goal;
            waypoint_mark_point.header.frame_id = "camera_init";
            waypoint_mark_point.pose.position.z = align_height;
            if (!isQuaternionNormalized(waypoint_mark_point.pose.orientation)) {
                waypoint_mark_point.pose.orientation =
                    tf::createQuaternionMsgFromYaw(0.0);
            }
            adjust_target_position[0] = waypoint_mark_point.pose.position.x;
            adjust_target_position[1] = waypoint_mark_point.pose.position.y;
            adjust_target_position[2] = align_height;
            adjust_target_position[3] =
                tf::getYaw(waypoint_mark_point.pose.orientation);
            have_waypoint_mark = true;
            align_ok = true;
            ROS_INFO("[PatrolControl] External ALIGN target=%u class=%s at (%.2f, %.2f)",
                     msg->target_id, msg->target_class.c_str(),
                     adjust_target_position[0], adjust_target_position[1]);
            break;

        case patrol_control::MissionCommand::LAND: {
            if (external_landing_active_) {
                ROS_WARN_THROTTLE(
                    2.0, "[ExternalLanding] duplicate LAND command ignored");
                return;
            }
            resetDetectionState();
            if (msg->goal.header.frame_id != external_landing_frame_ ||
                !std::isfinite(msg->goal.pose.position.x) ||
                !std::isfinite(msg->goal.pose.position.y) ||
                std::hypot(
                    msg->goal.pose.position.x - uav_pose.pose.position.x,
                    msg->goal.pose.position.y - uav_pose.pose.position.y) >
                    external_planner_start_max_distance_) {
                ROS_ERROR(
                    "[ExternalLanding] rejected LAND goal frame=%s at (%.3f, %.3f)",
                    msg->goal.header.frame_id.c_str(),
                    msg->goal.pose.position.x,
                    msg->goal.pose.position.y);
                break;
            }
            current_task_type = MAIN_MISSION;
            Point_mode = Land_point;
            external_landing_goal_ = msg->goal;
            external_landing_goal_.header.frame_id = external_landing_frame_;
            external_landing_goal_.pose.position.z =
                external_landing_capture_height_;
            if (!isQuaternionNormalized(
                    external_landing_goal_.pose.orientation)) {
                external_landing_goal_.pose.orientation =
                    tf::createQuaternionMsgFromYaw(0.0);
            }
            external_landing_aligned_goal_ = external_landing_goal_;
            external_landing_active_ = true;
            external_landing_new_mark_ = false;
            external_landing_alignment_complete_ = false;
            external_landing_auto_land_requested_ = false;
            external_landing_stable_count_ = 0;
            external_landing_started_at_ = ros::Time::now();
            external_landing_command_stamp_ = external_landing_started_at_;
            external_landing_last_mark_stamp_ = ros::Time(0);
            external_landing_last_mark_receipt_ = ros::Time(0);
            external_landing_last_auto_land_attempt_ = ros::Time(0);
            have_land_mark = false;
            flag_land = false;
            align_height = external_landing_capture_height_;
            adjust_target_position[0] =
                external_landing_goal_.pose.position.x;
            adjust_target_position[1] =
                external_landing_goal_.pose.position.y;
            adjust_target_position[2] = external_landing_capture_height_;
            adjust_target_position[3] =
                tf::getYaw(external_landing_goal_.pose.orientation);
            patrol_cmd = external_landing_goal_;
            std_msgs::Bool landing_enable;
            landing_enable.data = true;
            landing_detect_control_pub_.publish(landing_enable);
            Drone_mode = Land;
            ROS_INFO(
                "[PatrolControl] External LAND command accepted; awaiting fresh H evidence");
            break;
        }

        default:
            ROS_ERROR("[PatrolControl] Unknown external mission command: %u",
                      msg->command);
            break;
    }
}

std::string LLController::desiredAlignMode() const
{
    if (Drone_mode == Aligning) {
        if (current_task_type == CROSS_MISSION) {
            return "drop_cross";
        }
        if (current_task_type == MAIN_MISSION && Point_mode == Detect_point) {
            return "drop_circle";
        }
    }

    if (Drone_mode == Land && flag_landing_detect) {
        return "landing";
    }

    return "disabled";
}

void LLController::publishAlignMode(const std::string& mode)
{
    std_msgs::String msg;
    msg.data = mode;
    align_mode_pub_.publish(msg);

    if (mode != current_align_mode_) {
        current_align_mode_ = mode;
        clearUavVisionAlignmentState();
        ROS_INFO("[AlignMode] -> %s", current_align_mode_.c_str());
    }
}
void LLController::applyDropSlotOffset(int servo_id, bool dynamic_target) {
    if (servo_id < 1 || servo_id > 3) {
        ROS_ERROR("[DropSystem] Cannot apply offset for invalid servo ID: %d",
                  servo_id);
        return;
    }
    const auto& offsets = dynamic_target ? dynamic_drop_slot_offsets_
                                         : drop_slot_offsets_;
    adjust_target_position[0] += offsets[servo_id - 1][0];
    adjust_target_position[1] += offsets[servo_id - 1][1];
}

DropActionResult LLController::executeDropAction(int servo_id) {
    servo_complete.data = false;
    patrol_control::Servo srv;
    srv.request.req = servo_id;

    const bool service_call_ok = servo_id >= 1 && servo_id <= 3 &&
                                 servo_client.call(srv);
    const DropActionResult result = classifyDropAction(
        servo_id, service_call_ok, service_call_ok && srv.response.res);
    servo_complete.data = dropActionSucceeded(result);

    switch (result) {
        case DropActionResult::kSuccess:
            ROS_INFO("\033[32m[DropSystem] Drop action %d received positive Servo ACK\033[0m",
                     servo_id);
            break;
        case DropActionResult::kInvalidServoId:
            ROS_ERROR("\033[31m[DropSystem] Invalid servo ID: %d\033[0m", servo_id);
            break;
        case DropActionResult::kServiceCallFailed:
            ROS_ERROR("\033[31m[DropSystem] Servo service call failed for slot %d\033[0m",
                      servo_id);
            break;
        case DropActionResult::kRejected:
            ROS_WARN("\033[33m[DropSystem] Servo request rejected for slot %d\033[0m",
                     servo_id);
            break;
    }
    // // 创建投递控制消息
    // std_msgs::Bool drop_msg;
    // drop_msg.data = true;
    
    // // 选择对应的舵机发布器
    // ros::Publisher* servo_pub = nullptr;
    // std::string topic_name;
    
    // switch (servo_id) {
    //     case 1:
    //         servo_pub = &servo1_pub_;
    //         topic_name = "/control1";
    //         break;
    //     case 2:
    //         servo_pub = &servo2_pub_;
    //         topic_name = "/control2";
    //         break;
    //     case 3:
    //         servo_pub = &servo3_pub_;
    //         topic_name = "/control3";
    //         break;
    //     default:
    //         ROS_ERROR("\033[31m[DropSystem] Invalid servo ID: %d\033[0m", servo_id);
    //         return;
    // }
    
    // ROS_INFO("\033[32m[DropSystem] Executing drop action for servo %d\033[0m", servo_id);
    // ROS_INFO("\033[32m[DropSystem] Publishing to topic: %s\033[0m", topic_name.c_str());
    
    // // 发布投递命令，重复发布几次确保接收
    // for (int i = 0; i < 5; i++) {
    //     servo_pub->publish(drop_msg);
    //     ros::Duration(0.1).sleep();  // 间隔100ms
    // }
    return result;
}
void LLController::stopDropAction(int servo_id) {
   
    // 创建投递控制消息
    std_msgs::Bool drop_msg;
    drop_msg.data = false;
    
    // 选择对应的舵机发布器
    ros::Publisher* servo_pub = nullptr;
    std::string topic_name;
    
    switch (servo_id) {
        case 1:
            servo_pub = &servo1_pub_;
            topic_name = "/control1";
            break;
        case 2:
            servo_pub = &servo2_pub_;
            topic_name = "/control2";
            break;
        case 3:
            servo_pub = &servo3_pub_;
            topic_name = "/control3";
            break;
        default:
            ROS_ERROR("\033[31m[DropSystem] Invalid servo ID: %d\033[0m", servo_id);
            return;
    }

    
    // 发布投递命令，重复发布几次确保接收
    for (int i = 0; i < 5; i++) {
        servo_pub->publish(drop_msg);
        ros::Duration(0.1).sleep();  // 间隔100ms
    }
    ROS_INFO("stopped correctly");
}

void LLController::resetDropState() {
    drop_condition_met = false;
    current_pixel_error = 1000.0;
    descent_completed = false;
    final_target_height = 0.0;
    last_target_height_ = 0.0;
    last_check_time_ = ros::Time::now();
    ROS_DEBUG("\033[36m[DropSystem] Drop state reset for next detection point\033[0m");
}

// 十字标志位
// void LLController::crossPixelOffsetCallback(const geometry_msgs::Point::ConstPtr& msg) {
//     cross_found_ = true;
//     ROS_INFO_THROTTLE(1.0, "\033[35m[PatrolControl] Cross pixel offset: (%.1f, %.1f, radius=%.1f)\033[0m", 
//                       msg->x, msg->y, msg->z);
// }
void LLController::servoCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (ignore_servo_complete && msg->data == true) {
        ROS_INFO("\033[33m[ServoComplete] Ignoring servo complete signal during waypoint transition\033[0m");
        return;
    }
    servo_complete = *msg;
    ROS_INFO("\033[36m[ServoComplete] Received servo complete: %d\033[0m", servo_complete.data);
}
void LLController::crossStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
    static bool last_state = false;
    static bool first_call = true;
    
    bool new_state = msg->data;
    
    // 只在状态变化时输出提示
    if (first_call || new_state != last_state) {
        if (new_state) {
            ROS_INFO("\033[32m[PatrolControl] Cross detection: RED CROSS FOUND!\033[0m");
        } else {
            ROS_INFO("\033[33m[PatrolControl] Cross detection: RED CROSS LOST\033[0m");
        }
        last_state = new_state;
        first_call = false;
    }
    if(new_state){
        count_cross_detect++;
    }
    cross_detection_active_ = new_state;
    if(count_cross_detect >= 10){
        cross_found_ = true;
        count_cross_detect = 0;
    }
}

// 新增任务管理函数实现
bool LLController::CrossDetectionDone() {
    // 使用与圆环检测相同的逻辑，通过alignment_control_converter进行精确对准
    static ros::Time cross_start_time = ros::Time::now();
    bool should_drop = false;
    static geometry_msgs::PoseStamped waypoint_temp;
    double dis_to_next_position = 0;
    static bool cross_first_call = true;
    static int cross_detect_times = 0;
    static ros::Time last_output_time = ros::Time::now();
    double output_interval = 0.2; // 每0.2秒输出一次
    
    if (cross_first_call) {
        cross_start_time = ros::Time::now();
        cross_first_call = false;
        cross_detect_times = 0;
        count_aligning = 0;
        drop_complete = false;
        down_flag = true;
        align_height = 1.2;
        ROS_INFO("\033[34m[CrossDetection] Starting cross alignment using alignment_control_converter\033[0m");
    }
    
    if (have_cross_mark) {
        //使用与圆环检测相同的逻辑：接收alignment_control_converter的精确对准结果
        if(cross_mark_point.pose.position.x == last_waypoint_mark.pose.position.x && cross_mark_point.pose.position.y == last_waypoint_mark.pose.position.y || cross_mark_point.pose.position.x == 0 && cross_mark_point.pose.position.y == 0){
            cross_mark_point.pose.position.x = uav_pose.pose.position.x;
            cross_mark_point.pose.position.y = uav_pose.pose.position.y;
            cross_mark_point.pose.position.z = align_height;
            cross_mark_point.pose.orientation = tf::createQuaternionMsgFromYaw(uav_pose.pose.orientation.z);
        }
        adjust_target_position[0] = cross_mark_point.pose.position.x;
        adjust_target_position[1] = cross_mark_point.pose.position.y;
        adjust_target_position[2] = align_height;  // 使用alignment_control_converter计算的高度
        adjust_target_position[3] = tf::getYaw(cross_mark_point.pose.orientation);
        
        cross_detect_times++;
        ROS_INFO("\033[32m[CrossDetection] Received alignment target, detection count: %d\033[0m", cross_detect_times);
    } else {
        // 没有对准目标时，保持在十字检测点
        // 注意：这里不设置原始航路点，因为我们是在进行十字检测
        ROS_DEBUG_THROTTLE(2, "\033[33m[CrossDetection] Waiting for alignment target from alignment_control_converter...\033[0m");
    }
    
    // 计算时间进度
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - cross_start_time).toSec();
    
    // 使用与圆环检测相同的阈值参数
    int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
    double clamped_time_threshould = std::max(0.1f, waypoint_adjust_max_second_threshould);
    
    // 计算进度比例 [0, 1]
    double detect_progress = std::min(1.0, static_cast<double>(cross_detect_times) / clamped_detect_threshould);
    double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);
    
    // 控制输出频率（与圆环检测相同的进度条显示）
    if ((current_time - last_output_time).toSec() >= output_interval) {
        last_output_time = current_time;

        // 构建进度条
        int bar_width = 20;
        auto buildProgressBar = [&](double progress) {
            std::string bar = "";
            for (int i = 0; i < bar_width; i++) {
                bar += (i < static_cast<int>(progress * bar_width)) ? "#" : "-";
            }
            return bar;
        };

        std::string detect_bar = buildProgressBar(detect_progress);
        std::string time_bar = buildProgressBar(time_progress);

        std::cout << "\033[35m[CrossDetection] Alignment Progress: [" << detect_bar << "] " << cross_detect_times << "/" << clamped_detect_threshould 
                  << " | Time Progress: [" << time_bar << "] " << elapsed_time << "/" << clamped_time_threshould << " sec\033[0m" 
                  << std::endl;
    }
    int servo_id = detect_point_counter + 1;
    dis_to_next_position = distance3d(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z, 
        adjust_target_position[0], adjust_target_position[1],uav_pose.pose.position.z);
    ROS_INFO("[crossdetectiondone]dis_to yes or no %.2f",dis_to_next_position);
    if (elapsed_time <= 30) {
        if(have_cross_mark &&
           count_aligning < 50 &&
           (dis_to_next_position <= 0.07 || (current_align_mode_ == "drop_cross" && uav_drop_ready_))){
            ROS_INFO("\033[32m[CrossDetectionDone] dis_to_next_position: %.2f\033[0m", dis_to_next_position);
            count_aligning++;
            down_flag = true;
            should_drop = false;
            waypoint_temp.pose.position.x = cross_mark_point.pose.position.x;
            waypoint_temp.pose.position.y = cross_mark_point.pose.position.y;
            waypoint_temp.pose.orientation = cross_mark_point.pose.orientation;
        }
        if(count_aligning >= 50){
            ROS_INFO("\033[32m[CrossDetectionDone]waypoint_temp: %.2f, %.2f\033[0m", waypoint_temp.pose.position.x, waypoint_temp.pose.position.y);
            adjust_target_position[0] = waypoint_temp.pose.position.x;
            adjust_target_position[1] = waypoint_temp.pose.position.y;
            align_height = 0.10;
            adjust_target_position[3] = tf::getYaw(waypoint_temp.pose.orientation);
            applyDropSlotOffset(servo_id, true);
            double ttt = distance3d(uav_pose.pose.position.x,uav_pose.pose.position.y,uav_pose.pose.position.z,waypoint_temp.pose.position.x,waypoint_temp.pose.position.y,0.1);
            ROS_INFO("\033[32m[CrossDetectionDone] should_drop: %d, drop_complete: %d, uav_pose.pose.position.z: %.2f\033[0m", should_drop, drop_complete, uav_pose.pose.position.z);
            if(uav_pose.pose.position.z <= 0.17 && ttt <= 0.15 && !drop_complete)
            {
                should_drop = true;
                ROS_INFO("\033[32m[CrossDetectionDone] height_reach should_drop: true\033[0m");
            }
            
            const DropReleaseGate release_gate = currentDropReleaseGate();
            const bool release_authorized = canRequestDrop(
                require_vision_release_permission_, release_gate);
            if (should_drop && !drop_complete && !release_authorized) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[CrossDetectionDone] Waiting for mission release permission "
                    "(active=%s fresh=%s)",
                    release_gate.mission_permission_active ? "true" : "false",
                    release_gate.mission_permission_fresh ? "true" : "false");
                return false;
            }
            if (should_drop && !drop_complete) {
                // 执行投递动作 - 按顺序使用舵机
                ignore_servo_complete = false;  //FF 开始接受舵机完F成信号
                const DropActionResult result = executeDropAction(servo_id);
                if (dropActionSucceeded(result)) {
                    drop_complete = true;
                    if (detect_point_counter >= 0 &&
                        detect_point_counter < static_cast<int>(drop_completed.size())) {
                        drop_completed[detect_point_counter] = true;
                    }
                }
                should_drop = false;
                if (!drop_complete) {
                    ROS_WARN_THROTTLE(1.0,
                        "[CrossDetectionDone] Drop slot %d not acknowledged; retrying while conditions remain valid",
                        servo_id);
                    return false;
                }
            } else if (should_drop && drop_complete) {
                ROS_INFO_THROTTLE(1.0, "\033[33m[CrossDetectionDone] Drop already completed for detect point %d, waiting for next cycle\033[0m", detect_point_counter);
            } else {
                ROS_WARN_THROTTLE(1.0, "\033[33m[CrossDetectionDone] Height not met or already completed for detect point %d (counter: %d)\033[0m", 
                        detect_point_counter + 1, detect_point_counter);
            }            
            // 防止重复投递：如果已经投递过，直接跳过
            if(servo_complete.data){
                ROS_INFO("\033[33m[CrossDetectionDone] servo_complete.data: %d\033[0m", servo_complete.data);
                down_flag = false;
                align_height = 1.15;
                if(uav_pose.pose.position.z >= 0.95){
                    // stopDropAction(servo_id-1);
                    resetDropState();   // 重置投递状态
                    cleanupAfterCrossDrop();  // 彻底清理十字投递状态
                    cross_drop_completed = true;
                    cross_mission_completed = true;  // 标记十字任务完成
                    detect_point_counter++;
                    ROS_INFO("\033[32m[CrossDetectionDone] Cross drop completed successfully, returning to main mission\033[0m");
                    return true;
                }
                ROS_INFO("\033[33m[CrossDetectionDone] Height Not ready,heignt not reached and not get to 1.2m current align_heignt: %.2f  uav_z : %.2f\033[0m", align_height, uav_pose.pose.position.z);
                return false;
            }
            else{
                ROS_WARN_THROTTLE(1.0,
                    "[CrossDetectionDone] Awaiting positive Servo ACK; release remains blocked");
                return false;  // 未能够完成舵机任务
            }
        }

        return false;
    }
    else{
        ROS_WARN_THROTTLE(1.0,
            "[CrossDetectionDone] Alignment timed out after %.2f s; release remains blocked without a positive Servo ACK",
            elapsed_time);
        return false;
    }
    return false;
}
void LLController::resetCrossDetectionState() {
    cross_found_ = false;           // 重置检测状态
    cross_mission_completed = false;
    cross_detection_active_ = false;
    // cross_mark保持状态，由NextPoint控制
    
    ROS_DEBUG("\033[36m[CrossDetection] Cross detection state reset\033[0m");
}

void LLController::resetDetectionState() {
    // 重置所有检测相关状态
    first_call = true;
    times_detect = 0;
    count_aligning = 0;
    drop_complete = false;
    down_flag = true;
    align_ok = false;
    have_waypoint_mark = false;
    servo_complete.data = false;  // 重置舵机完成状态
    ignore_servo_complete = true;  // 开始忽略舵机完成信号
    clearUavVisionAlignmentState();
    
    ROS_INFO("\033[32m[DetectionState] All detection flags reset for new waypoint, servo_complete.data: %d\033[0m", servo_complete.data);
}

void LLController::cleanupAfterCrossDrop() {
    // 彻底清理十字投递后的所有状态，确保后续投递正常
    
    // 重置十字检测相关状态
    cross_found_ = false;
    cross_detection_active_ = false;
    // cross_mission_completed = false;
    have_cross_mark = false;
    tank_mission_completed = false;
    // tank_mark = false;
    tank_found_ = false;
    
    // 重置投递相关状态
    drop_complete = false;
    servo_complete.data = false;
    ignore_servo_complete = true;  // 开始忽略舵机完成信号
    
    // 重置对准相关状态
    align_ok = false;
    count_aligning = 0;
    have_waypoint_mark = false;
    clearUavVisionAlignmentState();
    
    // 重置检测状态
    first_call = true;
    times_detect = 0;
    down_flag = true;
    
    // 重置任务状态
    current_task_type = MAIN_MISSION;
    mission_interrupted = false;
    
    // 重置高度
    align_height = 1.2;
    
    ROS_INFO("\033[32m[CleanupAfterCrossDrop] All states cleaned up after cross drop, ready for next mission\033[0m");
}

}
