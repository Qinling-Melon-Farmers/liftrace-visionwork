/*
 * 舵机投放节点（实机标定；2026-10-07 左仓改到 PWM3_M0）
 *
 * Orange Pi 5 当前接线（pwmchip 编号按硬件地址在运行时解析，不再写死）：
 *   req=1 后仓 Pin11 GPIO4_B2 febf0020.pwm，初始 700000ns，投放 1700000ns
 *   req=2 右仓 Pin7  GPIO1_C6 febf0030.pwm，初始 1000000ns，投放 2100000ns
 *   req=3 左仓 Pin15 GPIO0_D4 fd8b0030.pwm（PWM3_M0），初始 1100000ns，投放 2100000ns
 *
 * 为什么不再写死 pwmchip 号：
 *   左仓原先用 fd8b0010.pwm（PWM1 / Pin16），但现场左仓舵机实际接在
 *   Pin15 = GPIO0_D4 = PWM3_M0。PWM3 默认在设备树中未启用，因此左仓初始化
 *   一直没有输出。启用 pwm3-m0 叠加层后内核枚举顺序变化
 *   （后仓=chip5、右仓=chip6、左仓=chip2），原写死的 4/5/0 全部失效。
 *   本节点按 /sys/class/pwm 的实际 device 地址解析编号，上述两种启动配置下都正确。
 *
 * 行为：
 *   1. 启动时三只舵机【依次】复位到各自初始位（间隔1s，避免同时动作造成电流冲击），
 *      到位后停止PWM输出（舱门由机械机构自保持，无需舵机保持力）；
 *   2. 收到 /Servo 服务请求时：设置投放脉宽 -> enable -> 等待1s到位 -> 停止PWM输出；
 *   3. 三槽初始化全部成功才开放服务；任一步失败则 ROS_FATAL 退出，不注册 raw 服务。
 *
 * 前置条件：开机后执行过一次 init_pwm.sh（解析地址、导出通道、设周期/极性、放开权限）。
 *           本节点不导出通道、不 unexport，退出只关闭输出。
 * 服务接口：/Servo (patrol_control/Servo)。launch_all.launch 将其重映射到
 *           /legacy/Servo_raw，对外的 /Servo 由 guarded_servo_proxy 独占。
 *           req=1/2/3 -> res=true 仅代表PWM操作被内核确认；不是机械投放证明。
 */
#include "actuator_pwm/PWMController.h"
#include "patrol_control/Servo.h"
#include <ros/ros.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {

// Resolve the pwmchip number from the hardware address instead of hardcoding it.
int findPWMChip(const std::string& device) {
    for (int chip = 0; chip < 64; ++chip) {
        const std::string path = "/sys/class/pwm/pwmchip" + std::to_string(chip);
        char* resolved = realpath(path.c_str(), nullptr);
        if (!resolved) continue;
        const std::string actual(resolved);
        free(resolved);
        if (actual.find("/" + device + "/") != std::string::npos) return chip;
    }
    throw std::runtime_error("PWM device unavailable: " + device +
                             "; enable its device-tree overlay and reboot, then rerun init_pwm.sh");
}

}  // namespace

PWMController pwm_front(findPWMChip("febf0020.pwm"), 0, "febf0020.pwm"); // 后仓 Pin11，接线待确认
PWMController pwm_left(findPWMChip("febf0030.pwm"), 0, "febf0030.pwm"); // 右仓 Pin7
PWMController pwm_right(findPWMChip("fd8b0030.pwm"), 0, "fd8b0030.pwm"); // 左仓 Pin15 PWM3_M0

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
