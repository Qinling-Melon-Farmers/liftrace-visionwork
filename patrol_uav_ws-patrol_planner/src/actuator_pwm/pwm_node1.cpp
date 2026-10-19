/*
 * Orange Pi 5 Plus servo backend, 2026-10-11.
 * req=1 rear: physical Pin16 GPIO3_B5 PWM12_M0 febf0000.pwm
 * req=2 right: physical Pin7 GPIO1_D6 PWM14_M2 febf0020.pwm
 * req=3 left: physical Pin18 GPIO3_B6 PWM13_M0 febf0010.pwm
 * See BoardWiring.h and SlotCalibration.h; pulse values require mechanism validation.
 * Default startup is read-only; /legacy/Servo_raw is advertised only after all checks.
 */
#include "actuator_pwm/PWMController.h"
#include "patrol_control/Servo.h"
#include <ros/ros.h>

#include <cstdlib>
#include <iterator>
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
#include "actuator_pwm/BoardWiring.h"

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
    std::ifstream boardModel("/proc/device-tree/model");
    std::string model((std::istreambuf_iterator<char>(boardModel)), std::istreambuf_iterator<char>());
    if (model.find("OPi 5 Plus") == std::string::npos && model.find("Orange Pi 5 Plus") == std::string::npos) {
        ROS_FATAL("This actuator configuration requires Orange Pi 5 Plus; raw service not advertised");
        return 1;
    }
    try {
        PWMController rear(findPWMChip(actuator_pwm::kPwmDevices[0]), 0, actuator_pwm::kPwmDevices[0]);
        PWMController right(findPWMChip(actuator_pwm::kPwmDevices[1]), 0, actuator_pwm::kPwmDevices[1]);
        PWMController left(findPWMChip(actuator_pwm::kPwmDevices[2]), 0, actuator_pwm::kPwmDevices[2]);
        channels[0]=&rear; channels[1]=&right; channels[2]=&left;
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
