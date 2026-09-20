/**
 *  @file patrol_control.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief 本程序为无人机巡检流程控制程序节点启动程序
 *  @version 0.2
 *  @date 5-16-2025
 */
#include "patrol_control/patrol_control.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "patrol_control");

    ros::NodeHandle NodeHandle;

    patrol_control::LLController controller(NodeHandle);   
    
    ros::spin();

    return 0;
}