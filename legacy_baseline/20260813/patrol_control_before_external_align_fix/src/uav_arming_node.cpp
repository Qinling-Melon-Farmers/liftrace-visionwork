#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

class SimpleArmingNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    mavros_msgs::State current_state_;
    bool offboard_set_ = false;
    bool armed_ = false;
    
public:
    SimpleArmingNode() {
        // 订阅飞机状态
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &SimpleArmingNode::stateCallback, this);
        
        // 创建服务客户端
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        ROS_INFO("\033[32m[简单解锁] 节点已初始化，等待MAVROS连接...\033[0m");
    }
    
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
        
        // 检查MAVROS连接
        if (!current_state_.connected) {
            return;
        }
        
        // 设置OFFBOARD模式
        if (!offboard_set_ && current_state_.mode != "OFFBOARD") {
            mavros_msgs::SetMode offboard_set_mode;
            offboard_set_mode.request.custom_mode = "OFFBOARD";
            
            if (set_mode_client_.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                ROS_INFO("\033[32m[简单解锁] OFFBOARD模式已启用\033[0m");
                offboard_set_ = true;
            } else {
                ROS_WARN("\033[33m[简单解锁] 设置OFFBOARD模式失败\033[0m");
            }
        }
        
        // 解锁飞机
        if (!armed_ && !current_state_.armed && current_state_.mode == "OFFBOARD") {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            
            if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("\033[32m[简单解锁] 飞机解锁成功！\033[0m");
                ROS_INFO("\033[32m[简单解锁] 飞机已准备就绪\033[0m");
                armed_ = true;
            } else {
                ROS_WARN("\033[33m[简单解锁] 飞机解锁失败\033[0m");
            }
        }
        
        // 状态监控
        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
            ROS_INFO_THROTTLE(10.0, "\033[32m[简单解锁] 飞机状态：已解锁，模式：%s\033[0m", 
                             current_state_.mode.c_str());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_arming_node");
    
    ROS_INFO("\033[1;32m[简单解锁] 启动飞机解锁节点...\033[0m");
    
    SimpleArmingNode arming_node;
    
    ros::spin();
    
    return 0;
} 