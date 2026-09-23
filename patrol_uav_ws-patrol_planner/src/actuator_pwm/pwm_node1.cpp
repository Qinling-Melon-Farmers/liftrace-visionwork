/*
 * 舵机投放节点（实机标定最终版，2026-09）
 *
 * Orange Pi 5 当前接线：
 *   req=1 后仓 Pin11 GPIO4_B2 pwmchip4（接线待确认），初始700000ns，投放1700000ns
 *   req=2 右仓 Pin7 GPIO1_C6 pwmchip5，初始1000000ns，投放2100000ns
 *   req=3 左仓 Pin16 GPIO1_D3 pwmchip0，初始1100000ns，投放2100000ns
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

PWMController pwm_front(4, 0, "febf0020.pwm"); // 后仓 Pin11，接线待确认
PWMController pwm_left(5, 0, "febf0030.pwm"); // 右仓 Pin7
PWMController pwm_right(0, 0, "fd8b0010.pwm"); // 左仓 Pin16

// 投放动作：使能 -> 转到投放位 -> 等待到位 -> 停止PWM输出
bool servocallback(patrol_control::Servo::Request &req, patrol_control::Servo::Response &res){
    switch(req.req){
        case 1:{   // 后仓
            pwm_front.setDutyCycle(1700000);
            pwm_front.enable();
            ros::Duration(1.0).sleep();
            pwm_front.disable();
            ROS_INFO("Servo req=1 (rear) released");
            break;
        }
        case 2:{   // 右仓
            pwm_left.setDutyCycle(2100000);
            pwm_left.enable();
            ros::Duration(1.0).sleep();
            pwm_left.disable();
            ROS_INFO("Servo req=2 (right) released");
            break;
        }
        case 3:{   // 左仓
            pwm_right.setDutyCycle(2100000);
            pwm_right.enable();
            ros::Duration(1.0).sleep();
            pwm_right.disable();
            ROS_INFO("Servo req=3 (left) released");
            break;
        }
        default:
            ROS_WARN("Servo req=%d invalid (expect 1/2/3)", req.req);
            res.res = false;
            return true;
    }
    res.res = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_controller");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("Servo", servocallback);

    // 依次复位 + 到位断电：避免三舵机同时动作造成电流冲击
    pwm_front.setPeriod(20000000);       // 20ms周期(50Hz)
    pwm_front.setDutyCycle(700000);      // 后仓初始位 0.7ms
    pwm_front.setPolarity("normal");     // 内核默认inverse，必须显式置normal
    pwm_front.enable();
    ros::Duration(1.0).sleep();          // 等后仓到位
    pwm_front.disable();                 // 停止PWM输出

    pwm_left.setPeriod(20000000);
    pwm_left.setDutyCycle(1000000);      // 右仓初始位 1.0ms
    pwm_left.setPolarity("normal");
    pwm_left.enable();
    ros::Duration(1.0).sleep();          // 等右仓到位
    pwm_left.disable();

    pwm_right.setPeriod(20000000);
    pwm_right.setDutyCycle(1100000);     // 左仓初始位 1.1ms
    pwm_right.setPolarity("normal");
    pwm_right.enable();
    ros::Duration(1.0).sleep();          // 等左仓到位
    pwm_right.disable();

    ROS_INFO("Servo node ready. /Servo req: 1=rear 2=right 3=left");
    ros::spin();
    return 0;
}
