#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace patrol_control {

// Dedicated horizontal gate for compensated delivery descent. No displacement,
// height or vertical-speed gate; independent of final release and H landing.
struct DropPreDescentStabilityConfig {
    double xy_tolerance_m = 0.06;
    double max_horizontal_speed_mps = 0.10;
    double stable_duration_sec = 1.0;
    double max_odom_age_sec = 0.20;
    double max_sample_gap_sec = 0.20;
    std::size_t min_samples = 3;

    bool valid(bool require_stability = true) const {
        const double values[] = {xy_tolerance_m, max_odom_age_sec};
        for (double value : values)
            if (!std::isfinite(value) || value <= 0.0) return false;
        if (!require_stability) return true;
        const double stability[] = {max_horizontal_speed_mps, stable_duration_sec, max_sample_gap_sec};
        for (double value : stability)
            if (!std::isfinite(value) || value <= 0.0) return false;
        return min_samples >= 3;
    }
};

struct DropPreDescentStabilityResult {
    bool ready = false;
    const char* reason = "settling";
    double stable_for_sec = 0.0;
    std::size_t sample_count = 0;
};

class DropPreDescentStabilityWindow {
public:
    explicit DropPreDescentStabilityWindow(
        const DropPreDescentStabilityConfig& config = DropPreDescentStabilityConfig())
        : config_(config) {}

    void reset() {
        active_ = false;
        first_source_ = first_receipt_ = first_now_ = 0.0;
        last_source_ = last_receipt_ = last_now_ = 0.0;
        sample_count_ = 0;
    }

    DropPreDescentStabilityResult update(double now, double source, double receipt,
                                        double xy_error, double horizontal_speed,
                                        bool control_ready) {
        if (!config_.valid()) return reject("invalid_config");
        const double values[] = {now, source, receipt, xy_error, horizontal_speed};
        for (double value : values)
            if (!std::isfinite(value)) return reject("nonfinite_feedback");
        if (!control_ready) return reject("control_not_ready");
        if (source <= 0.0 || receipt <= 0.0 || source > now || receipt > now ||
            now-source > config_.max_odom_age_sec || now-receipt > config_.max_odom_age_sec)
            return reject("feedback_not_fresh");
        if (xy_error < 0.0 || horizontal_speed < 0.0) return reject("invalid_feedback");
        if (xy_error > config_.xy_tolerance_m) return reject("xy_error");
        if (horizontal_speed > config_.max_horizontal_speed_mps) return reject("horizontal_speed");
        if (active_ && (now < last_now_ || source < last_source_ || receipt < last_receipt_))
            return reject("time_reversed");
        if (active_ && (now-last_now_ > config_.max_sample_gap_sec ||
                        source-last_source_ > config_.max_sample_gap_sec ||
                        receipt-last_receipt_ > config_.max_sample_gap_sec)) reset();
        if (!active_) {
            active_ = true;
            first_source_ = last_source_ = source;
            first_receipt_ = receipt;
            first_now_ = now;
            sample_count_ = 1;
        } else if (source > last_source_) {
            last_source_ = source;
            ++sample_count_;
        }
        last_receipt_ = receipt;
        last_now_ = now;
        DropPreDescentStabilityResult result;
        result.stable_for_sec = std::min(last_source_-first_source_,
            std::min(receipt-first_receipt_, now-first_now_));
        result.sample_count = sample_count_;
        result.ready = sample_count_ >= config_.min_samples &&
            result.stable_for_sec + 1e-9 >= config_.stable_duration_sec;
        result.reason = result.ready ? "ready" : "settling";
        return result;
    }

private:
    DropPreDescentStabilityResult reject(const char* reason) {
        reset();
        DropPreDescentStabilityResult result;
        result.reason = reason;
        return result;
    }
    DropPreDescentStabilityConfig config_;
    bool active_ = false;
    double first_source_ = 0.0, first_receipt_ = 0.0, first_now_ = 0.0;
    double last_source_ = 0.0, last_receipt_ = 0.0, last_now_ = 0.0;
    std::size_t sample_count_ = 0;
};

}  // namespace patrol_control
