#include "actuator_pwm/PWMController.h"
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <atomic>
std::atomic<bool> command_detected(false);

void controlCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    command_detected = msg->data;
    ROS_INFO("Received command: %s", command_detected ? "true" : "false");
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_controller");
    ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("control3",100,controlCallBack);
    ros::Publisher servo_complete_pub_ = nh.advertise<std_msgs::Bool>("/servo/complete",1);
    PWMController pwm(4, 0); // pwmchip4, channel 0 -> pin27

    // 配置舵机参数
    pwm.setPeriod(20000000);     // 20ms周期(50Hz)
    pwm.setDutyCycle(2500000);   // 1.5ms脉宽(中位)
    pwm.setPolarity("normal");   // 极性设置
    pwm.enable();
	std_msgs::Bool ok;
    ok.data = 1;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
		if(command_detected==0)
		{
			ROS_INFO("Command undetected");
			pwm.setDutyCycle(500000);	
		}
		else
		{
			ROS_INFO("Command detected");
            pwm.setDutyCycle(2300000); // 2.3ms	
            servo_complete_pub_.publish(ok);
		}
		ros::spinOnce();
        loop_rate.sleep();
    }
    pwm.setDutyCycle(1500000); // Return to neutral before disabling
    pwm.disable();
    return 0;
}
