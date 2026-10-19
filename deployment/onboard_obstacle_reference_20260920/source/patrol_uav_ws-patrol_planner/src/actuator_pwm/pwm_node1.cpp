/*
 * 舵机投放节点（实机标定最终版，2026-09）
 *
 * Orange Pi 5 Plus 当前接线：
 *   req=1 后仓 Pin16 GPIO3_B5 pwmchip2，初始700000ns，投放1700000ns
 *   req=2 右仓 Pin18 GPIO3_B6 pwmchip3，初始1000000ns，投放2100000ns
 *   req=3 左仓 Pin7 GPIO1_D6 pwmchip4，初始1100000ns，投放2100000ns
 *
 * 行为：
 *   1. 启动时三只舵机【依次】复位到各自初始位（间隔1s，避免同时动作造成电流冲击），
 *      到位后停止PWM输出（舱门由机械机构自保持，无需舵机保持力）；
 *   2. 收到 /Servo 服务请求时：设置投放脉宽 -> enable -> 等待1s到位 -> 停止PWM输出。
 *
 * 前置条件：开机后执行过一次 init_pwm.sh（导出PWM通道并放开权限），
 *           本节点还会核对硬件地址；初始化脚本不会驱动舵机。
 * 服务接口：/Servo (patrol_control/Servo)，req=1/2/3 -> res=true
 */
#include "actuator_pwm/PWMController.h"
#include "patrol_control/Servo.h"
#include <ros/ros.h>

PWMController pwm_front(2, 0, "febf0000.pwm"); // 后仓 Pin16
PWMController pwm_left(3, 0, "febf0010.pwm"); // 右仓 Pin18
PWMController pwm_right(4, 0, "febf0020.pwm"); // 左仓 Pin7

#include "actuator_pwm/CheckedPulse.h"

bool servocallback(patrol_control::Servo::Request &req, patrol_control::Servo::Response &res) {
    PWMController* channels[]={&pwm_front,&pwm_left,&pwm_right};
    const unsigned duties[]={1700000,2100000,2100000};
    res.res=false;
    if(req.req<1 || req.req>3) {
        ROS_WARN("Invalid servo slot %d",req.req);
        return true;
    }
    res.res=checkedPulse(*channels[req.req-1],duties[req.req-1],
                        [](){ros::WallDuration(1.0).sleep();});
    if(res.res) ROS_INFO("Servo slot=%d PWM acknowledged; physical release unverified",req.req);
    else ROS_ERROR("Servo slot=%d PWM failed; do not commit release",req.req);
    return true;
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"pwm_controller");
    ros::NodeHandle nh;
    PWMController* channels[]={&pwm_front,&pwm_left,&pwm_right};
    const unsigned initial[]={700000,1000000,1100000};
    for(int i=0;i<3;++i) {
        if(!checkedInitialize(*channels[i],initial[i],[](){ros::WallDuration(1.0).sleep();})) {
            ROS_FATAL("Servo initialization failed at slot %d; raw service not advertised",i+1);
            return 1;
        }
    }
    ros::ServiceServer service=nh.advertiseService("Servo",servocallback);
    ROS_INFO("Servo ready: rear=1 right=2 left=3; initialization verified");
    ros::spin();
    return 0;
}
