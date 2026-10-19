#pragma once
#include <cmath>
#include <algorithm>

namespace patrol_control {
// 只管理已知H点的等待/到位；不制造视觉观测，不请求模式或上锁。
struct LandingCoordinateFallbackConfig {
    bool enabled = false;
    double wait_sec = 15.0;
    double arrival_xy_tolerance_m = 0.20;
    double arrival_z_tolerance_m = 0.15;
    double descend_xy_tolerance_m = 0.08;
    double max_horizontal_speed_mps = 0.10;
    double stable_duration_sec = 0.30;
    bool valid() const {
        return std::isfinite(wait_sec) && wait_sec > 0 && wait_sec <= 60 &&
            std::isfinite(arrival_xy_tolerance_m) && arrival_xy_tolerance_m > 0 &&
            std::isfinite(arrival_z_tolerance_m) && arrival_z_tolerance_m > 0 &&
            std::isfinite(descend_xy_tolerance_m) && descend_xy_tolerance_m > 0 &&
            descend_xy_tolerance_m <= arrival_xy_tolerance_m &&
            std::isfinite(max_horizontal_speed_mps) && max_horizontal_speed_mps > 0 &&
            std::isfinite(stable_duration_sec) && stable_duration_sec > 0;
    }
};
class LandingCoordinateFallback {
public:
    void reset() { arrived_at_ = stable_at_ = last_now_ = last_source_ = -1; active_ = false; }
    bool active() const { return active_; }
    double waited(double now) const { return arrived_at_ < 0 ? 0 : std::max(0.0, now-arrived_at_); }
    bool update(const LandingCoordinateFallbackConfig& c, double now, double source,
                bool feedback_valid, double xy, double z_error, double speed, double max_gap) {
        if (!c.enabled || !c.valid()) { reset(); return false; }
        if (!std::isfinite(now) || (last_now_ >= 0 && now < last_now_)) { reset(); return false; }
        last_now_ = now;
        if (!feedback_valid || !std::isfinite(source) || source <= 0 || source > now ||
            !std::isfinite(max_gap) || max_gap <= 0 || now-source > max_gap ||
            !std::isfinite(xy) || xy < 0 || !std::isfinite(z_error) ||
            !std::isfinite(speed) || speed < 0) { stable_at_ = -1; return false; }
        if (source <= last_source_) return false; // 同一反馈不能累计停稳时间。
        if (last_source_ >= 0 && source-last_source_ > max_gap) stable_at_ = -1;
        last_source_ = source;
        if (arrived_at_ < 0 && xy <= c.arrival_xy_tolerance_m &&
            std::abs(z_error) <= c.arrival_z_tolerance_m) arrived_at_ = now;
        if (arrived_at_ < 0 || now-arrived_at_ < c.wait_sec) return false;
        active_ = true;
        if (xy > c.descend_xy_tolerance_m || std::abs(z_error) > c.arrival_z_tolerance_m ||
            speed > c.max_horizontal_speed_mps) { stable_at_ = -1; return false; }
        if (stable_at_ < 0) stable_at_ = source;
        return source-stable_at_ >= c.stable_duration_sec;
    }
private:
    double arrived_at_ = -1, stable_at_ = -1, last_now_ = -1, last_source_ = -1;
    bool active_ = false;
};
} // namespace patrol_control
