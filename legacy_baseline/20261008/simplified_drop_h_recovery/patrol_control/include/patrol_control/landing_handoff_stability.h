#pragma once

#include <cmath>
#include <cstddef>

namespace patrol_control {

struct LandingHandoffStabilityConfig {
    double xy_tolerance_m = 0.05;
    double height_tolerance_m = 0.02;
    double max_horizontal_speed_mps = 0.03;
    double max_vertical_speed_mps = 0.05;
    double stable_duration_sec = 0.5;
    double max_odom_age_sec = 0.2;
    double max_sample_gap_sec = 0.2;
    std::size_t min_samples = 3;

    bool valid() const {
        const double values[] = {xy_tolerance_m, height_tolerance_m,
            max_horizontal_speed_mps, max_vertical_speed_mps,
            stable_duration_sec, max_odom_age_sec, max_sample_gap_sec};
        for (double value : values)
            if (!std::isfinite(value) || value <= 0.0) return false;
        return min_samples >= 3;
    }
};

struct LandingHandoffSample {
    double source_stamp_sec = 0.0;
    double receipt_stamp_sec = 0.0;
    // 当前高度与交接上限必须共用坐标系和基准，可同时使用本地 Z 或离地高度。
    // z_error 相对停稳目标高度计算，不是相对交接上限；XY 相对选定 H 点计算。
    double height_m = 0.0;
    double max_handoff_height_m = 0.0;
    double xy_error_m = 0.0;
    double z_error_m = 0.0;
    // 先将 odom 子坐标系中的完整速度旋转到位姿坐标系，再计算水平模长及有符号 vz。
    // 不能直接以机体系 XY 速度代替地图系水平速度。
    double horizontal_speed_mps = 0.0;
    double vertical_speed_mps = 0.0;
    bool alignment_latched = false;
    bool control_ready = false;
    bool feedback_valid = false;
};

struct LandingHandoffStabilityResult {
    bool ready = false;
    const char* reason = "waiting";
    double stable_for_sec = 0.0;
    std::size_t sample_count = 0;
};

// 独立停稳门槛，不启动 ROS、不发布命令、不重定位 H，也不判断图像新鲜度。
// H 的调用方应在短暂图像丢帧时保留锁点，并在新 LAND、取消和完成交接时重置。
// 投口可用独立对象和配置复用，不能混用 H 的误差与速度阈值。
class LandingHandoffStabilityWindow {
public:
    explicit LandingHandoffStabilityWindow(
        const LandingHandoffStabilityConfig& config = LandingHandoffStabilityConfig())
        : config_(config) {}

    void reset() {
        active_ = false;
        first_source_ = last_source_ = last_receipt_ = last_now_ = 0.0;
        sample_count_ = 0;
    }

    LandingHandoffStabilityResult update(double now_sec,
                                         const LandingHandoffSample& sample) {
        if (!config_.valid()) return reject("invalid_config");
        const double values[] = {now_sec, sample.source_stamp_sec,
            sample.receipt_stamp_sec, sample.height_m, sample.max_handoff_height_m,
            sample.xy_error_m, sample.z_error_m, sample.horizontal_speed_mps,
            sample.vertical_speed_mps};
        for (double value : values)
            if (!std::isfinite(value)) return reject("nonfinite_feedback");
        if (!sample.feedback_valid) return reject("invalid_feedback");
        if (!sample.alignment_latched) return reject("alignment_not_latched");
        if (!sample.control_ready) return reject("control_not_ready");
        if (sample.source_stamp_sec <= 0.0 || sample.receipt_stamp_sec <= 0.0 ||
            sample.source_stamp_sec > now_sec || sample.receipt_stamp_sec > now_sec ||
            now_sec - sample.source_stamp_sec > config_.max_odom_age_sec ||
            now_sec - sample.receipt_stamp_sec > config_.max_odom_age_sec)
            return reject("feedback_not_fresh");
        if (sample.xy_error_m < 0.0 || sample.horizontal_speed_mps < 0.0)
            return reject("invalid_feedback");
        if (sample.height_m > sample.max_handoff_height_m)
            return reject("above_handoff_height");
        if (sample.xy_error_m > config_.xy_tolerance_m)
            return reject("xy_error");
        if (std::abs(sample.z_error_m) > config_.height_tolerance_m)
            return reject("height_error");
        if (sample.horizontal_speed_mps > config_.max_horizontal_speed_mps)
            return reject("horizontal_speed");
        if (std::abs(sample.vertical_speed_mps) > config_.max_vertical_speed_mps)
            return reject("vertical_speed");
        if (active_ && (now_sec < last_now_ ||
                        sample.source_stamp_sec < last_source_ ||
                        sample.receipt_stamp_sec < last_receipt_))
            return reject("time_reversed");
        if (active_ && (sample.source_stamp_sec - last_source_ > config_.max_sample_gap_sec ||
                        sample.receipt_stamp_sec - last_receipt_ > config_.max_sample_gap_sec ||
                        now_sec - last_now_ > config_.max_sample_gap_sec))
            reset();
        if (!active_) {
            active_ = true;
            first_source_ = sample.source_stamp_sec;
            last_source_ = sample.source_stamp_sec;
            sample_count_ = 1;
        } else if (sample.source_stamp_sec > last_source_) {
            last_source_ = sample.source_stamp_sec;
            ++sample_count_;
        }
        last_receipt_ = sample.receipt_stamp_sec;
        last_now_ = now_sec;
        LandingHandoffStabilityResult result;
        result.stable_for_sec = last_source_ - first_source_;
        result.sample_count = sample_count_;
        result.ready = result.sample_count >= config_.min_samples &&
                       result.stable_for_sec >= config_.stable_duration_sec;
        result.reason = result.ready ? "ready" : "settling";
        return result;
    }

private:
    LandingHandoffStabilityResult reject(const char* reason) {
        reset();
        LandingHandoffStabilityResult result;
        result.reason = reason;
        return result;
    }
    LandingHandoffStabilityConfig config_;
    bool active_ = false;
    double first_source_ = 0.0, last_source_ = 0.0;
    double last_receipt_ = 0.0, last_now_ = 0.0;
    std::size_t sample_count_ = 0;
};

}  // namespace patrol_control
