#ifndef _LL_CONTROLLER_NEW_H_
#define _LL_CONTROLLER_NEW_H_

#include <map>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <uav_vision/DropOffset.h>
#include <uav_vision/DropReady.h>
#include <uav_vision/TargetCandidate.h>
#include <Eigen/Core>
#include <cmath>  // 引入 math 库，使用 sqrt 函数
#include <vector>

namespace patrol_control {

// 起飞，跑点，对准，降落，悬停
enum Dronemode {Takeoff, Run_point , Aligning , Land, Hover};

enum Pointmode {Takeoff_point, Detect_point, Nothing_point , Land_point, Dynamic_point};

// 任务类型枚举
enum TaskType {
    MAIN_MISSION,      // 主任务：正常航路点
    CROSS_MISSION,      // 临时任务：十字检测投递
    TANK_MISSION
};

class LLController{
public:
    /** constructor **/
    LLController(ros::NodeHandle nh);

    /** destructor **/
    ~LLController();
    void initializeNode();
    void load_params();
    
    inline Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
        Eigen::Vector3f ev3(p.x, p.y, p.z);
        return ev3;
    }

    float distance3d(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2) {
        // 计算两点之间的三维距离
        float distance = std::sqrt(std::pow(x_2 - x_1, 2) + std::pow(y_2 - y_1, 2) + std::pow(z_2 - z_1, 2));
        return distance;
    }

    // 转换函数
    Pointmode stringToPointmode(const std::string& mode_str) {
        if (mode_str == "Takeoff_point") return Takeoff_point;
        else if (mode_str == "Nothing_point") return Nothing_point;
        else if (mode_str == "Land_point") return Land_point;
        else if (mode_str == "Detect_point") return Detect_point;
        else if (mode_str == "Dynamic_point") return Dynamic_point;
        else {return Nothing_point; std::cout<<"ERROR Pointmode"<<std::endl;}  // 默认返回 Nothing_point
    }

    
private:
    struct Waypoint
    {
        double x, y, z, yaw;
        std::string pointmode;
        double hover_time;  // 悬停时间(秒)
    };

    ros::NodeHandle nh_;
    /** publish the goal waypoint to the avoidance **/
    ros::Publisher setplanner_goal_pub_;
    /** publish the flag that arouse the circle detection with task C**/
    ros::Publisher mavros_point_cmd_pub;
    /** publish detection control signal **/
    ros::Publisher detect_control_pub_;

    ros::Publisher tank_control_pub_;
    /** publish landing detection control signal **/
    ros::Publisher landing_detect_control_pub_;
    ros::Publisher align_mode_pub_;
    /** publish servo control signal **/
    ros::Publisher servo_control_pub_;
    /** publish servo status signal **/
    ros::Publisher servo_status_pub_;
    /** get the newest position **/
    ros::Subscriber land_mark_sub_;
    ros::Subscriber tank_status_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber fastplanner_cmd_sub_, detect_sub_;
    ros::Subscriber mavros_local_position_sub_;
    ros::ServiceClient land_client;
    ros::Timer cmd_timer;
    ros::Subscriber class_sub_;
    // for auto.land
    ros::ServiceClient set_mode_client;
    ros::Subscriber servo_marky_sub_;
    ros::Subscriber cross_mark_sub_;
    ros::ServiceClient servo_client;
    ros::Subscriber selected_target_sub_;
    ros::Subscriber drop_offset_sub_;
    ros::Subscriber drop_ready_sub_;
    // 舵机控制发布器
    ros::Publisher servo1_pub_;
    ros::Publisher servo2_pub_;
    ros::Publisher servo3_pub_;
    
    // 十字检测相关订阅者和发布者
    ros::Subscriber cross_pixel_offset_sub_;
    ros::Subscriber cross_center_sub_;
    ros::Subscriber cross_status_sub_;
    ros::Publisher cross_control_pub_;
    ros::Publisher class_control_pub_;

    std::vector<std::vector<double>> dynamic_point_list;
    geometry_msgs::PoseStamped mavros_point_cmd, planner_cmd, patrol_cmd, waypoint_mark_point, land_mark;
    bool flag_planner_px4 = 1 ;
    bool landig_mark = 0;
    bool debug = 0 ;
    bool flag_landing_detect = 1;
    bool auto_land = 0;
    // Simulation-only fallback for the legacy landing path.  Keep disabled
    // by default so ordinary/hardware launches retain their historical
    // behavior until they explicitly opt in.
    bool simulation_auto_land = false;
    bool servo_detect_done = false;
    bool temp_storage = false;
    bool dynamic_first_flag = true;
    bool dynamic_record_activate = false;
    bool tank_mission_completed = false;
    bool tank_found_ = false;

    float arrive_goal_threshould = 0.3;
    float takeoff_threshould = 0.3;
    float waypoint_threshould = 0.3;
    float aligning_threshould = 0.15;
    float landing_threshould = 0.15;
    float arrive_yaw_threshould = 0.3;
    ros::Time dynamic_current_time;
    // float dynamic_start_time = 0.0;

    double Edistance = 0.0;

    int count_aligning = 0;
    std_msgs::Bool servo_complete;
    ros::Subscriber servo_complete_sub_;

    ros::Time dynamic_start_time;

    float align_height = 1.0;

    float times_detect_threshould = 30;
    float planner_min_pub_threshould = 0.01;

    float waypoint_adjust_max_second_threshould = 15;
    float land_adjust_max_second_threshould = 10;

    float land_height = 0.3;
    float px4_max_distance = 1.2;
    float max_yaw_change = 0.2;

    int waypoint_next = 0;
    int waypoint_now = 0;
    int dynamic_point_index = 0;

    Dronemode Drone_mode = Takeoff;

    //  enum Pointmode {Detect_point, Nothing_point , Land_point};
    //                       0              1              2
    Pointmode Point_mode = Nothing_point;
    Pointmode Point_temp = Detect_point;
    std::vector<Waypoint> waypoint_list;
    geometry_msgs::PoseStamped tank_mark_point;

    std::vector<std::string> goal;  // 默认由构造函数按 ~goal_list 参数填充，缺省 {"panzer"}
    bool detect_skip_enable_ = true;  // 3 投后是否跳降落段；false 时顺序推进走廊航点
    // 悬停相关变量
    bool flag_hover_started = false;
    bool overtime_drop_flag = false;
    bool align_ok = false;
    bool flagg = false;
    ros::Time hover_start_time;
    double current_hover_time = 0.0;
    geometry_msgs::PoseStamped last_waypoint_mark;

    // 投递相关变量
    int detect_point_counter = 0;          // 当前是第几个检测点
    std::vector<bool> drop_completed;      // 记录每个检测点是否已完成投递
    bool drop_condition_met = false;       // 投递条件是否满足
    double drop_precision_threshold = 20.0; // 投递精度阈值（像素）
    double drop_height_threshold = 0.2;     // 投递高度阈值（米）
    bool drop_enabled = true;               // 投递功能是否启用
    bool cross_mark = true;
    bool tank_mark = true;
    bool min_distance_flag = false;
    
    // 对准精度监控
    double current_pixel_error = 1000.0;   // 当前像素误差
    ros::Subscriber alignment_feedback_sub_; // 订阅对准反馈信息
    std_msgs::Bool detect_enable_msg_temp;
    std_msgs::String class_;

    double min_distance = 100.0;
    
    // 渐进降高状态监控
    bool descent_completed = false;         // 渐进降高是否完成
    double final_target_height = 0.0;      // 最终目标高度
    ros::Time descent_stable_start_time;   // 高度稳定开始时间
    double descent_stable_duration;        // 需要稳定的时间（秒）
    std_msgs::Bool servo_marky;
    // 避免static变量污染，使用成员变量
    double last_target_height_ = 0.0;      // 上次的目标高度
    ros::Time last_check_time_;             // 上次检查时间
    geometry_msgs::PoseStamped uav_pose;    // 无人机位置
    // 十字检测相关状态变量
    geometry_msgs::Point cross_pixel_offset_; // 十字像素偏差
    geometry_msgs::Point cross_center_;       // 十字中心点

    bool cross_detection_active_ = false;    // 十字检测是否激活
    bool cross_found_ = false;               // 是否检测到十字
    bool cross_triggered_alignment_ = false; // 是否是由十字检测触发的对准模式
    
    // 任务状态管理变量
    TaskType current_task_type = MAIN_MISSION;           // 当前任务类型
    Dronemode main_mission_mode = Run_point;             // 保存主任务状态
    bool mission_interrupted = false;                    // 任务中断标志
    bool cross_mission_completed = false;               // 十字任务完成标志
    double cross_detection_height = 1.0;                // 十字检测下降高度
    double cross_descent_height = 0.5;                  // 十字检测下降距离
    int cross_servo_id = 0;                             // 十字检测使用的舵机ID
    bool tank_drop_completed = false;
    bool cross_drop_completed = false;                  // 十字投递是否已完成（防止重复投递）
    geometry_msgs::PoseStamped waypoint_temp;
    geometry_msgs::PoseStamped land_mark_point;
    int count_cross_found = 0;
    // 新增缺失的成员变量
    bool have_cross_mark = false;                       // 是否有十字标记
    Eigen::Vector4f adjust_target_position;             // 调整目标位置
    geometry_msgs::PoseStamped cross_mark_point;        // 十字标记点
    int waypoint_skipping_index = 3;                    // 路点跳跃，为Nothing_point和Land_point总和，三个快递全部投递完成，直接跳跃到穿走廊的点
    double time_temp;
    uav_vision::TargetCandidate latest_selected_target_;
    bool have_selected_target_ = false;
    ros::Time latest_selected_target_time_;
    uav_vision::DropOffset latest_drop_offset_;
    bool have_drop_offset_ = false;
    ros::Time latest_drop_offset_time_;
    bool uav_drop_ready_ = false;
    ros::Time latest_drop_ready_time_;
    std::string latest_drop_ready_reason_;
    double selected_target_timeout_ = 1.0;
    double drop_offset_timeout_ = 1.0;
    double pixel_to_meter_ratio_ = 0.0015;
    double max_alignment_move_distance_ = 0.5;
    double drop_circle_radius_m_ = 0.5;
    double drop_cross_radius_m_ = 0.5;
    double landing_pad_radius_m_ = 0.3;
    bool enable_selected_tank_interrupt_ = false;
    
    // 检测状态管理变量
    bool first_call = true;                             // 首次调用标志
    bool drop_complete = false;                         // 投递完成标志
    bool down_flag = true;                              // 下降标志
    ros::Time detection_start_time;                     // 检测开始时间
    int times_detect = 0;                               // 检测次数
    bool ignore_servo_complete = false;                 // 忽略舵机完成信号标志  
    int count_cross_detect = 0;                         // 十字检测次数
    double dynamic_height = 0.2;

    //设置投递时间决定标志位
    bool drop_time_flag = false;
    ros::Time drop_current_time;                        //降高至投递高度时间
    ros::Time drop_start_time;                          //降高开始时间
    ros::Publisher point_class_pub_;
    std::string current_align_mode_ = "disabled";
    void publishAlignMode(const std::string& mode);
    std::string desiredAlignMode() const;
    bool classMatchesGoal(const std::string& class_name) const;
    bool hasFreshSelectedTarget() const;
    bool hasFreshDropOffset() const;
    void clearUavVisionAlignmentState();
    void updateGoalFromSelectedTarget(const std::string& class_name);
    void projectDropOffsetToTarget(const uav_vision::DropOffset& msg);
    void patrol();
    void pub_goal(geometry_msgs::PoseStamped goal_msg);
    
    void Lock();
    void CallLand();
    void NextPoint();
    
    bool WayPointDetectDone();
    bool LandDetectDone();
    bool DynamicProcess();
    
    // 投递相关函数
    void executeDropAction(int servo_id);      // 执行投递动作
    bool checkDropCondition();                 // 检查投递条件
    void alignmentFeedbackCallback(const geometry_msgs::Point::ConstPtr& msg); // 对准反馈回调
    void resetDropState();                     // 重置投递状态
    
    // 十字检测相关函数
    void crossPixelOffsetCallback(const geometry_msgs::Point::ConstPtr& msg);  // 十字像素偏差回调
    void crossCenterCallback(const geometry_msgs::Point::ConstPtr& msg);       // 十字中心回调
    void crossStatusCallback(const std_msgs::Bool::ConstPtr& msg);             // 十字检测状态回调
    void enableCrossDetection(bool enable);                                    // 启用/禁用十字检测
    
    // 新增任务管理函数
    bool CrossDetectionDone();                             // 十字检测任务完成判断
    void setupCrossDetectionPoint();                       // 设置十字检测点
    void resetCrossDetectionState();                       // 重置十字检测状态
    void resetDetectionState();                           // 重置检测状态
    void cleanupAfterCrossDrop();                         // 十字投递后清理状态
    void stopDropAction(int servo_id);
    void ClassCallback(const std_msgs::String& msg);
    void TankStatusCallback(const geometry_msgs::PoseStamped& msg);

    void positionCallback(const geometry_msgs::PoseStamped& msg);
    void plannercmdCallback(const geometry_msgs::PoseStamped& msg);
    void cmdCallback(const ros::TimerEvent& event);
    void waypointMarkCallback(const geometry_msgs::PoseStamped& msg);
    void landMarkCallback(const geometry_msgs::PoseStamped& msg);
    void crossStateCallback(const std_msgs::Bool::ConstPtr& msg);
    void mavrosLocalPositionCallback(const geometry_msgs::PoseStamped& msg);
    void servoMarkyCallback(const std_msgs::Bool& msg);
    void crossMarkCallback(const geometry_msgs::PoseStamped& msg);
    void servoCompleteCallback(const std_msgs::Bool::ConstPtr& msg);
    void selectedTargetCallback(const uav_vision::TargetCandidate::ConstPtr& msg);
    void dropOffsetCallback(const uav_vision::DropOffset::ConstPtr& msg);
    void dropReadyCallback(const uav_vision::DropReady::ConstPtr& msg);
    // void landMarkCallback(const geometry_msgs::PoseStamped& msg);
};  
}
#endif
