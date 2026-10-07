#include "actuator_pwm/PWMController.h"
#include "patrol_control/Servo.h"
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <atomic>
std::atomic<bool> command_detected(false);
PWMController pwm_front(2, 0);
PWMController pwm_left(3, 0);
PWMController pwm_right(4, 0);

bool servocallback(patrol_control::Servo::Request &req,patrol_control::Servo::Response &res){
    switch(req.req){
        case 1:{
            pwm_front.setDutyCycle(2300000);
            ROS_INFO("!");
            break;
        }
        case 2:{
            pwm_left.setDutyCycle(2300000);
            ROS_INFO("!");
            break;
        }
        case 3:{
            pwm_right.setDutyCycle(2300000);
            ROS_INFO("!");
            break;
        }
        default:break;
    }
    ros::Duration(1.0).sleep();
    res.res = true;
    return true;
}
// void controlCallBack(const std_msgs::Bool::ConstPtr& msg)
// {
//     command_detected = msg->data;
//     ROS_INFO("Received command: %s", command_detected ? "true" : "false");
// }
int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_controller");
    ros::NodeHandle nh;
    ros::ServiceServer service=nh.advertiseService("Servo",servocallback);
	// ros::Subscriber sub = nh.subscribe("control1",100,controlCallBack);
    // ros::Publisher servo_complete_pub_ = nh.advertise<std_msgs::Bool>("/servo/complete",1);
    // pwmchip2, channel 0 -> pin16

    // 配置舵机参数
    pwm_front.setPeriod(20000000);     // 20ms周期(50Hz)
    pwm_front.setDutyCycle(500000);   // 1.5ms脉宽(中位)
    pwm_front.setPolarity("normal");   // 极性设置
    pwm_front.enable();

    pwm_left.setPeriod(20000000);     // 20ms周期(50Hz)
    pwm_left.setDutyCycle(500000);   // 1.5ms脉宽(中位)
    pwm_left.setPolarity("normal");   // 极性设置
    pwm_left.enable();

    pwm_right.setPeriod(20000000);     // 20ms周期(50Hz)
    pwm_right.setDutyCycle(500000);   // 1.5ms脉宽(中位)
    pwm_right.setPolarity("normal");   // 极性设置
    pwm_right.enable();
	// std_msgs::Bool ok;
    // ok.data = 1;
    // ros::Rate loop_rate(10);
    // while (ros::ok()) {
	// 	if(command_detected==0)
	// 	{
	// 		ROS_INFO("Command undetected");
	// 		pwm.setDutyCycle(500000);	
	// 	}
	// 	else
	// 	{
	// 		ROS_INFO("Command detected");
    //         pwm.setDutyCycle(2300000); // 2.3ms	
    //         servo_complete_pub_.publish(ok);
	// 	}
	// 	ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // pwm.setDutyCycle(1500000); // Return to neutral before disabling
    // pwm.disable();
    ros::spin();
    return 0;
}
