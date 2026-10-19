#include "actuator_pwm/PWMController.h"
#include "actuator_pwm/CheckedPulse.h"
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 4) return 64;
    try {
        PWMController pwm(std::stoi(argv[3]), 0, "fd8b0000.pwm", argv[1]);
        if (!checkedPulse(pwm, 2100000, [](){})) return 2;
        std::cout << pwm.path() << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
