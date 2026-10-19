#pragma once

#include <algorithm>
#include <cmath>

namespace patrol_control {

struct DropDescentConfig {
    int mode = 1;  // 1: XYZ; 2: frozen XY/Z ramp; 3: zero XY velocity; 4: frozen XY/VZ braking.
    double speed_mps = 0.20;
    double max_step_dt_sec = 0.20;
    double velocity_speed_mps = 0.60;
    double acceleration_mps2 = 0.60;
    double braking_mps2 = 0.60;
    double hold_tolerance_m = 0.015;

    bool valid() const {
        const bool velocity_valid = mode != 4 ||
            (std::isfinite(velocity_speed_mps) && velocity_speed_mps > 0.0 && velocity_speed_mps <= 1.0 &&
             std::isfinite(acceleration_mps2) && acceleration_mps2 > 0.0 && acceleration_mps2 <= 2.0 &&
             std::isfinite(braking_mps2) && braking_mps2 > 0.0 && braking_mps2 <= 2.0 &&
             std::isfinite(hold_tolerance_m) && hold_tolerance_m > 0.0 && hold_tolerance_m <= 0.03);
        return velocity_valid && mode >= 1 && mode <= 4 && std::isfinite(speed_mps) &&
            speed_mps > 0.0 && speed_mps <= (mode == 2 ? 3.0 : 1.0) && std::isfinite(max_step_dt_sec) &&
            max_step_dt_sec > 0.0 && max_step_dt_sec <= 0.5;
    }
};

// A commanded Z ramp, independent of XY, measured drift and navigation lead.
// A missed/stale control interval holds Z instead of catching up in one jump.
class DropDescentProfile {
public:
    explicit DropDescentProfile(const DropDescentConfig& config = DropDescentConfig())
        : config_(config) {}

    void reset() { active_ = holding_ = false; z_ = target_ = last_time_ = down_speed_ = 0.0; }

    bool start(double initial_z, double target_z, double now) {
        reset();
        if (!config_.valid() || !std::isfinite(initial_z) || !std::isfinite(target_z) ||
            !std::isfinite(now) || now < 0.0) return false;
        z_ = std::max(initial_z, target_z);
        target_ = target_z;
        last_time_ = now;
        active_ = true;
        return true;
    }

    void pause(double now) {
        if (std::isfinite(now)) last_time_ = now;
        down_speed_ = 0.0;
    }
    bool active() const { return active_; }
    double height() const { return z_; }
    bool holding() const { return holding_; }

    // Mode 4 uses fresh measured height, not a time-integrated open-loop Z.
    // The envelope leaves one sample of travel before sqrt(2*a*distance)
    // braking. Terminal position hold is latched; bounce cannot restart VZ.
    double updateVelocity(double now, double measured_z, double measured_vz) {
        if (!active_ || config_.mode != 4 || !std::isfinite(now) ||
            !std::isfinite(measured_z) || !std::isfinite(measured_vz)) {
            pause(now); return 0.0;
        }
        if (holding_) { z_ = target_; pause(now); return 0.0; }
        z_ = measured_z;
        const double dt = now-last_time_;
        last_time_ = now;
        const double remaining = measured_z-target_;
        if (remaining <= config_.hold_tolerance_m) {
            holding_ = true; z_ = target_; down_speed_ = 0.0; return 0.0;
        }
        if (dt <= 0.0 || dt > config_.max_step_dt_sec) {
            down_speed_ = 0.0; return 0.0;
        }
        const double travel = std::max(down_speed_, std::max(0.0, -measured_vz))*dt;
        const double available = std::max(0.0, remaining-config_.hold_tolerance_m-travel);
        const double envelope = std::sqrt(2.0*config_.braking_mps2*available);
        down_speed_ = std::min(config_.velocity_speed_mps,
            std::min(down_speed_+config_.acceleration_mps2*dt, envelope));
        return -down_speed_;  // Mission ENU; MAVROS converts to NED.
    }

    double update(double now) {
        if (!active_ || !std::isfinite(now)) return z_;
        const double dt = now-last_time_;
        last_time_ = now;
        if (dt > 0.0 && dt <= config_.max_step_dt_sec)
            z_ = std::max(target_, z_-config_.speed_mps*dt);
        return z_;
    }

private:
    DropDescentConfig config_;
    bool active_ = false;
    bool holding_ = false;
    double z_ = 0.0, target_ = 0.0, last_time_ = 0.0;
    double down_speed_ = 0.0;
};

}  // namespace patrol_control
