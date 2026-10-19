#include "actuator_pwm/PWMController.h"
#include <unistd.h>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <dirent.h>
#include <vector>

namespace {
bool matchesDevice(const std::string& path, const std::string& device) {
    char* resolved = realpath(path.c_str(), nullptr);
    const std::string actual = resolved ? resolved : "";
    free(resolved);
    return actual.find("/" + device + "/") != std::string::npos;
}
std::string controllerPath(int chip, const std::string& device, const std::string& root) {
    if (chip >= 0) return root + "/pwmchip" + std::to_string(chip);
    if (chip != -1 || device.empty() || device.find('/') != std::string::npos)
        throw std::runtime_error("Automatic PWM selection requires one platform device address");
    DIR* directory = opendir(root.c_str());
    if (!directory) throw std::runtime_error("Cannot read PWM controllers: " + root);
    std::vector<std::string> matches;
    while (dirent* entry = readdir(directory)) {
        const std::string name = entry->d_name;
        if (name.compare(0, 7, "pwmchip") == 0 && matchesDevice(root + "/" + name, device))
            matches.push_back(root + "/" + name);
    }
    closedir(directory);
    if (matches.size() != 1)
        throw std::runtime_error("Expected exactly one PWM controller for " + device + ", found " + std::to_string(matches.size()));
    return matches.front();
}
}

PWMController::PWMController(int chip, int channel, const std::string& expectedDevice,
                             const std::string& sysfsRoot) :
    basePath_(controllerPath(chip, expectedDevice, sysfsRoot)),
    pwmPath_(basePath_ + "/pwm" + std::to_string(channel)) {

    // Fail before touching sysfs if enumeration differs from the verified board.
    if (!expectedDevice.empty()) {
        if (!matchesDevice(basePath_, expectedDevice))
            throw std::runtime_error("PWM address mismatch: " + basePath_ + " expected " + expectedDevice);
    }
    // Channels and permissions belong to init_pwm.sh, not this process.
    if (access((pwmPath_ + "/enable").c_str(), W_OK) != 0)
        throw std::runtime_error("PWM not initialized/writable: " + pwmPath_ + "; run init_pwm.sh");
}

PWMController::~PWMController() {
    disable();  // Keep exported channels and permissions across service restarts.
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
    if (!fs.is_open()) {
        std::cerr << "PWM open failed: " << file << std::endl;
        return false;
    }
    fs << value;
    fs.flush();
    if (!fs.good()) {
        std::cerr << "PWM write failed: " << file << std::endl;
        return false;
    }
    fs.close();
    std::ifstream input(file);
    std::string actual;
    if (!(input >> actual) || actual != value) {
        std::cerr << "PWM readback mismatch: " << file << std::endl;
        return false;
    }
    return true;
}
