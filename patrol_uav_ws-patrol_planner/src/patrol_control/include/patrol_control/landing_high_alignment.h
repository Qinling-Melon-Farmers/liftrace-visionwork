#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace patrol_control {

// H-only acquisition: <=6 cm, strictly <0.1 m/s, continuous 1 second.
// Both image and motion clocks must progress; repeated images cannot latch H.
class LandingHighAlignmentWindow {
public:
    void reset() {
        active_ = false;
        count_ = 0;
    }

    bool update(double now, double odom, double receipt, double mark,
                double mark_receipt, double xy, double speed, bool valid,
                double max_mark_age_sec = 0.2) {
        const double values[] = {now, odom, receipt, mark, mark_receipt, xy, speed};
        for (double value : values) if (!std::isfinite(value)) return reject();
        if (!valid || xy < 0 || xy > 0.06 || speed < 0 || speed >= 0.1)
            return reject();
        // Odometry retains its strict 0.2 s freshness requirement. H evidence
        // uses the same configured freshness limit as the controller receiver.
        if (!std::isfinite(max_mark_age_sec) || max_mark_age_sec <= 0.0)
            return reject();
        for (double stamp : {odom, receipt})
            if (stamp <= 0 || stamp > now || now-stamp > 0.2) return reject();
        for (double stamp : {mark, mark_receipt})
            if (stamp <= 0 || stamp > now || now-stamp > max_mark_age_sec)
                return reject();
        if (active_ && (now < last_now_ || odom < last_odom_ ||
                       receipt < last_receipt_ || mark < last_mark_ ||
                       mark_receipt < last_mark_receipt_)) return reject();
        if (active_ && (now-last_now_ > 0.2 || odom-last_odom_ > 0.2 ||
                       receipt-last_receipt_ > 0.2 || mark-last_mark_ > max_mark_age_sec ||
                       mark_receipt-last_mark_receipt_ > max_mark_age_sec)) reset();
        if (!active_) {
            active_ = true;
            first_now_ = now; first_odom_ = odom; first_mark_ = mark;
            first_receipt_ = receipt; first_mark_receipt_ = mark_receipt;
            count_ = 1;
        } else if (odom > last_odom_) ++count_;
        last_now_ = now; last_odom_ = odom; last_receipt_ = receipt;
        last_mark_ = mark; last_mark_receipt_ = mark_receipt;
        return count_ >= 3 && std::min({now-first_now_, odom-first_odom_,
            receipt-first_receipt_, mark-first_mark_, mark_receipt-first_mark_receipt_})
            + 1e-9 >= 1.0;
    }

private:
    bool reject() { reset(); return false; }
    bool active_ = false;
    std::size_t count_ = 0;
    double first_now_ = 0, first_odom_ = 0, first_mark_ = 0;
    double first_receipt_ = 0, first_mark_receipt_ = 0;
    double last_now_ = 0, last_odom_ = 0, last_receipt_ = 0;
    double last_mark_ = 0, last_mark_receipt_ = 0;
};

}  // namespace patrol_control
