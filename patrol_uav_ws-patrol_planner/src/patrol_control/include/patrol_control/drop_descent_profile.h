#pragma once

#include <algorithm>
#include <cmath>

namespace patrol_control {

struct DropDescentConfig {
    int mode = 1;  // 1: legacy XYZ; 2: frozen XY + Z ramp; 3: zero XY velocity + Z ramp.
    double speed_mps = 0.20;
    double max_step_dt_sec = 0.20;

    bool valid() const {
        return mode >= 1 && mode <= 3 && std::isfinite(speed_mps) &&
            speed_mps > 0.0 && speed_mps <= 1.0 && std::isfinite(max_step_dt_sec) &&
            max_step_dt_sec > 0.0 && max_step_dt_sec <= 0.5;
    }
};

// A commanded Z ramp, independent of XY, measured drift and navigation lead.
// A missed/stale control interval holds Z instead of catching up in one jump.
class DropDescentProfile {
public:
    explicit DropDescentProfile(const DropDescentConfig& config = DropDescentConfig())
        : config_(config) {}

    void reset() { active_ = false; z_ = target_ = last_time_ = 0.0; }

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

    void pause(double now) { if (std::isfinite(now)) last_time_ = now; }
    bool active() const { return active_; }
    double height() const { return z_; }

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
    double z_ = 0.0, target_ = 0.0, last_time_ = 0.0;
};

}  // namespace patrol_control
