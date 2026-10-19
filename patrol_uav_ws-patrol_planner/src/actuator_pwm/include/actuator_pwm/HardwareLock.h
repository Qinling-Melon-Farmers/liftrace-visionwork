#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

namespace actuator_pwm {
class HardwareLock {
public:
    explicit HardwareLock(const std::string& path = "/run/lock/liftrace-servo.lock") {
        fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0) throw std::runtime_error("PWM lock unavailable; run init_pwm.sh");
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            ::close(fd_); fd_ = -1;
            throw std::runtime_error("PWM occupied by initialization/raw/debug service");
        }
    }
    ~HardwareLock() { if (fd_ >= 0) ::close(fd_); }
    HardwareLock(const HardwareLock&) = delete;
    HardwareLock& operator=(const HardwareLock&) = delete;
private:
    int fd_ = -1;
};
}
