#include "actuator_pwm/PWMController.h"
#include <unistd.h>

PWMController::PWMController(int chip, int channel) :
    basePath_("/sys/class/pwm/pwmchip" + std::to_string(chip)),
    pwmPath_(basePath_ + "/pwm" + std::to_string(channel)) {

    // 导出PWM通道
    writeSysfs(basePath_ + "/export", std::to_string(channel));
    usleep(500000); // 等待设备创建
}

PWMController::~PWMController() {
    disable();
    writeSysfs(basePath_ + "/unexport", std::to_string(0));
}

bool PWMController::setPeriod(unsigned int period_ns) {
    return writeSysfs(pwmPath_ + "/period", std::to_string(period_ns));
}

bool PWMController::setDutyCycle(unsigned int duty_cycle_ns) {
    return writeSysfs(pwmPath_ + "/duty_cycle", std::to_string(duty_cycle_ns));
}

bool PWMController::setPolarity(const std::string& polarity) {
    return writeSysfs(pwmPath_ + "/polarity", polarity);
}

bool PWMController::enable() {
    return writeSysfs(pwmPath_ + "/enable", "1");
}

bool PWMController::disable() {
    return writeSysfs(pwmPath_ + "/enable", "0");
}

bool PWMController::writeSysfs(const std::string& file, const std::string& value) {
    std::ofstream fs(file);
    if (!fs.is_open()) return false;
    fs << value;
    return true;
}