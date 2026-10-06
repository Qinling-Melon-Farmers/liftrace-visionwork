#ifndef ORANGE_PWM_PWMCONTROLLER_H
#define ORANGE_PWM_PWMCONTROLLER_H

#include <string>
#include <fstream>

class PWMController {
public:
    // chip=-1 resolves the unique controller by its platform address.
    PWMController(int chip = 2, int channel = 0, const std::string& expectedDevice = "",
                  const std::string& sysfsRoot = "/sys/class/pwm");
    ~PWMController();

    bool setPeriod(unsigned int period_ns);
    bool setDutyCycle(unsigned int duty_cycle_ns);
    bool setPolarity(const std::string& polarity);
    bool enable();
    bool disable();
    const std::string& path() const { return pwmPath_; }

private:
    std::string basePath_;
    std::string pwmPath_;
    bool writeSysfs(const std::string& file, const std::string& value);
};

#endif
