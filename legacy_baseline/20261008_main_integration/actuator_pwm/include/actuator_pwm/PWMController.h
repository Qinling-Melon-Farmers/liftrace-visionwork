#ifndef ORANGE_PWM_PWMCONTROLLER_H
#define ORANGE_PWM_PWMCONTROLLER_H

#include <string>
#include <fstream>

class PWMController {
public:
    PWMController(int chip = 2, int channel = 0);
    ~PWMController();

    bool setPeriod(unsigned int period_ns);
    bool setDutyCycle(unsigned int duty_cycle_ns);
    bool setPolarity(const std::string& polarity);
    bool enable();
    bool disable();

private:
    std::string basePath_;
    std::string pwmPath_;
    bool writeSysfs(const std::string& file, const std::string& value);
};

#endif