#pragma once
// Return electrical command acknowledgement, never physical-release proof.
template<class PWM, class Wait>
bool checkedPulse(PWM& pwm, unsigned duty, Wait wait) {
    if (!pwm.disable()) return false;
    if (!pwm.setDutyCycle(duty)) return false;
    if (!pwm.enable()) { pwm.disable(); return false; }
    wait();
    return pwm.disable();
}
template<class PWM, class Wait>
bool checkedInitialize(PWM& pwm, unsigned duty, Wait wait) {
    if (!pwm.disable() || !pwm.setDutyCycle(0) ||
        !pwm.setPeriod(20000000) || !pwm.setPolarity("normal")) return false;
    return checkedPulse(pwm,duty,wait);
}

// Default startup is passive. Explicit reset retains the checked transaction.
template<class PWM, class Wait>
bool checkedStartup(PWM& pwm, unsigned initialDuty, Wait wait,
                    bool initializeOnStartup = false) {
    if (!initializeOnStartup) return pwm.validatePassiveStartup();
    return checkedInitialize(pwm, initialDuty, wait);
}
