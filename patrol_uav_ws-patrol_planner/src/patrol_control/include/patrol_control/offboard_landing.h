#pragma once

#include "patrol_control/drop_descent_profile.h"
#include <string>

namespace patrol_control {

struct OffboardLandingConfig {
    double ground_z = -0.22, fc_clearance = 0.22, descent_speed = 0.30;
    double touchdown_overtravel = 0.04, ground_confirm_duration = 0.3;
    double state_max_age = 2.5, disarm_retry = 1.0;
    double endpoint() const { return ground_z + fc_clearance - touchdown_overtravel; }
    bool valid() const {
        return std::isfinite(ground_z) && std::isfinite(fc_clearance) && fc_clearance > 0.0 &&
            std::isfinite(touchdown_overtravel) && touchdown_overtravel >= 0.0 &&
            touchdown_overtravel < fc_clearance && std::isfinite(endpoint()) &&
            std::isfinite(descent_speed) && descent_speed > 0.0 && descent_speed <= 1.0 &&
            std::isfinite(ground_confirm_duration) && ground_confirm_duration > 0.0 &&
            std::isfinite(state_max_age) && state_max_age > 0.0 &&
            std::isfinite(disarm_retry) && disarm_retry > 0.0;
    }
};

// Mode 4 is H-only. Keep DropDescentConfig/Profile's drop modes 1/2/3 intact.
inline bool validLandingDescent(const DropDescentConfig& descent, const std::string& handoff) {
    auto ramp = descent;
    if (descent.mode == 4) ramp.mode = 2;
    return ramp.valid() && ((descent.mode == 4) == (handoff == "OFFBOARD"));
}

inline bool freshLandingStamp(double now, double source, double receipt, double max_age) {
    return std::isfinite(now) && std::isfinite(source) && std::isfinite(receipt) &&
        source > 0.0 && receipt > 0.0 && now >= source && now >= receipt &&
        now-source <= max_age && now-receipt <= max_age;
}

// Source stamps, not timer ticks, prove sustained contact. A pre-LAND ground
// report cannot satisfy this transaction; a fresh IN_AIR after LAND must precede it.
class OffboardGroundConfirmation {
public:
    void reset(double land_stamp = 0.0) {
        land_stamp_ = land_stamp;
        in_air_ = false;
        in_air_stamp_ = last_stamp_ = last_receipt_ = first_ground_ = 0.0;
        on_ground_ = feedback_valid_ = false;
        ground_samples_ = 0;
    }
    void observe(double now, double stamp, double receipt, bool in_air, bool on_ground,
                 const OffboardLandingConfig& config) {
        if (!freshLandingStamp(now, stamp, receipt, config.state_max_age) || stamp <= land_stamp_) {
            clearGround();
            feedback_valid_ = false;
            return;
        }
        if (stamp < last_stamp_) { clearGround(); feedback_valid_ = false; return; }
        if (stamp == last_stamp_) return; // A replay does not refresh the receipt/window.
        if (last_stamp_ > 0.0 && stamp-last_stamp_ > config.state_max_age) clearGround();
        last_stamp_ = stamp;
        last_receipt_ = receipt;
        feedback_valid_ = true;
        on_ground_ = on_ground;
        if (in_air) { in_air_ = true; in_air_stamp_ = stamp; }
        if (!on_ground || !in_air_ || stamp <= in_air_stamp_) { clearGround(); return; }
        if (ground_samples_ == 0) first_ground_ = stamp;
        ++ground_samples_;
    }
    bool groundPending(double now, const OffboardLandingConfig& config) const {
        return in_air_ && on_ground_ && ground_samples_ > 0 &&
            fresh(now, config);
    }
    bool fresh(double now, const OffboardLandingConfig& config) const {
        return feedback_valid_ && freshLandingStamp(now, last_stamp_, last_receipt_, config.state_max_age);
    }
    bool confirmed(double now, const OffboardLandingConfig& config) const {
        return groundPending(now, config) && ground_samples_ >= 2 &&
            last_stamp_-first_ground_ >= config.ground_confirm_duration;
    }
private:
    void clearGround() { first_ground_ = 0.0; ground_samples_ = 0; on_ground_ = false; }
    double land_stamp_ = 0.0, in_air_stamp_ = 0.0, last_stamp_ = 0.0;
    double last_receipt_ = 0.0, first_ground_ = 0.0;
    bool in_air_ = false, on_ground_ = false, feedback_valid_ = false;
    unsigned int ground_samples_ = 0;
};

} // namespace patrol_control
