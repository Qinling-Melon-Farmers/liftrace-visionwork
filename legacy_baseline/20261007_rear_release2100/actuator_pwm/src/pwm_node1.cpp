/*
 * 舵机投放节点（实机标定；2026-10-07 左仓改到 PWM3_M0）
 *
 * Orange Pi 5 当前接线（pwmchip 编号按硬件地址在运行时解析，不再写死）：
 *   req=1 后仓 Pin11 GPIO4_B2 febf0020.pwm，锁止 1700000ns，投放 700000ns（2026-10-07现场确认）
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
 *   1. 默认 initialize_on_startup=false：只读验证三槽周期、极性、关闭状态与写权限；
 *      不输出脉冲、不改 duty；已装载并机械锁止时必须使用此模式。
 *   2. 仅显式 initialize_on_startup=true 才按旧事务依次复位（仅非装载人工复位）。
 *   3. 收到许可代理请求仍执行受检查的1s投放事务；三槽启动检查通过才注册服务。
 *
 * 前置条件：开机后执行过一次 init_pwm.sh（解析地址、导出通道、设周期/极性、放开权限）。
 *           本节点不导出通道、不 unexport；有写事务时退出关闭输出，被动退出不写。
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

// Controllers are constructed inside main so failures are caught before advertising.
PWMController* channels[3] = {nullptr, nullptr, nullptr};

#include "actuator_pwm/CheckedPulse.h"
#include "actuator_pwm/SlotCalibration.h"

bool servocallback(patrol_control::Servo::Request &req, patrol_control::Servo::Response &res) {
    res.res=false;
    if(req.req<1 || req.req>3) {
        ROS_WARN("Invalid servo slot %d",req.req);
        return true;
    }
    res.res=checkedPulse(*channels[req.req-1],actuator_pwm::kReleaseDutyNs[req.req-1],
                        [](){ros::WallDuration(1.0).sleep();});
    if(res.res) ROS_INFO("Servo slot=%d PWM acknowledged; physical release unverified",req.req);
    else ROS_ERROR("Servo slot=%d PWM failed; do not commit release",req.req);
    return true;
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"pwm_controller");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    bool initializeOnStartup = false;
    // Invalid parameter types must not silently fall back to a startup action.
    if (privateNh.hasParam("initialize_on_startup") &&
        !privateNh.getParam("initialize_on_startup", initializeOnStartup)) {
        ROS_FATAL("initialize_on_startup must be bool; raw service not advertised");
        return 1;
    }
    try {
        PWMController pwm_front(findPWMChip("febf0020.pwm"), 0, "febf0020.pwm"); // rear Pin11
        PWMController pwm_left(findPWMChip("febf0030.pwm"), 0, "febf0030.pwm"); // right Pin7
        PWMController pwm_right(findPWMChip("fd8b0030.pwm"), 0, "fd8b0030.pwm"); // left Pin15 PWM3_M0
        channels[0]=&pwm_front; channels[1]=&pwm_left; channels[2]=&pwm_right;
        if (initializeOnStartup)
            ROS_WARN("Explicit startup RESET selected: unloaded manual preparation only");
        for(int i=0;i<3;++i) {
            if(!checkedStartup(*channels[i],actuator_pwm::kInitialDutyNs[i],
                               [](){ros::WallDuration(1.0).sleep();}, initializeOnStartup)) {
                ROS_FATAL("Servo startup check failed at slot %d; raw service not advertised",i+1);
                return 1;
            }
        }
        ros::ServiceServer service=nh.advertiseService("Servo",servocallback);
        ROS_INFO("Servo ready: rear=1 right=2 left=3; startup=%s",
                 initializeOnStartup ? "explicit checked reset" : "passive read checks, no PWM writes");
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("Servo startup failed: %s; raw service not advertised",error.what());
        return 1;
    }
    return 0;
}
