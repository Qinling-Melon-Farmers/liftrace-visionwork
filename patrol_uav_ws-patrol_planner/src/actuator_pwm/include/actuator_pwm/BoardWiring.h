#pragma once
namespace actuator_pwm {
// Orange Pi 5 Plus 40-pin physical header; req order rear/right/left.
// GPIO3_B5 PWM12_M0 Pin16; GPIO1_D6 PWM14_M2 Pin7; GPIO3_B6 PWM13_M0 Pin18.
const char* const kPwmDevices[] = {"febf0000.pwm", "febf0020.pwm", "febf0010.pwm"};
}
