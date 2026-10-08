#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace patrol_control {

// Local final-drop admission over the same sensed map used by the mission.
// A populated, fresh map is a coverage prerequisite, not an unknown-space
// observation proof. A conservative prism covers every XY/Z interpolation
// between visual capture and the compensated release reference.
struct DropDescentClearance {
    std::vector<Eigen::Vector3d> points;
    std::string frame;
    double source_stamp = 0.0, receipt_stamp = 0.0;
    Eigen::Vector2d cache_center = Eigen::Vector2d::Zero();
    double cache_radius_m = 2.0, max_age_sec = 2.0, voxel_size_m = 0.10;
    double body_up_m = 0.20, body_down_m = 0.22, body_xy_radius_m = 0.39;
    bool ready = false;

    bool configured() const {
        return std::isfinite(cache_radius_m) && cache_radius_m > 0.0 &&
            std::isfinite(max_age_sec) && max_age_sec > 0.0 &&
            std::isfinite(voxel_size_m) && voxel_size_m > 0.0 &&
            std::isfinite(body_up_m) && body_up_m > 0.0 &&
            std::isfinite(body_down_m) && body_down_m > 0.0 &&
            std::isfinite(body_xy_radius_m) && body_xy_radius_m > 0.0;
    }

    const char* rejection(double now, const std::string& required_frame,
            const Eigen::Vector3d& current, const Eigen::Vector2d& target,
            const Eigen::Vector2d& fc, double release_z, double xy_body_margin) const {
        if (!configured() || !ready || points.size() > 250000 ||
            source_stamp <= 0.0 || receipt_stamp <= 0.0)
            return "drop_map_not_ready";
        if (frame.empty() || frame != required_frame) return "drop_map_frame_mismatch";
        const double source_age = now - source_stamp, receipt_age = now - receipt_stamp;
        if (source_age < 0.0 || receipt_age < 0.0) return "drop_map_future";
        if (source_age > max_age_sec || receipt_age > max_age_sec) return "drop_map_stale";
        if (!current.allFinite() || !target.allFinite() || !fc.allFinite() ||
            !std::isfinite(release_z) || !std::isfinite(xy_body_margin) ||
            xy_body_margin <= 0.0) return "drop_path_invalid";
        const double xy_pad = xy_body_margin + voxel_size_m * 0.5;
        const Eigen::Vector2d lo = current.head<2>().cwiseMin(target).cwiseMin(fc)
                                      - Eigen::Vector2d::Constant(xy_pad);
        const Eigen::Vector2d hi = current.head<2>().cwiseMax(target).cwiseMax(fc)
                                      + Eigen::Vector2d::Constant(xy_pad);
        if ((lo.array() < (cache_center.array() - cache_radius_m)).any() ||
            (hi.array() > (cache_center.array() + cache_radius_m)).any())
            return "drop_map_local_coverage_missing";
        const double zlo = std::min(current.z(), release_z) - body_down_m - voxel_size_m * 0.5;
        const double zhi = std::max(current.z(), release_z) + body_up_m + voxel_size_m * 0.5;
        for (const auto& p : points) {
            if (p.x() >= lo.x() && p.x() <= hi.x() &&
                p.y() >= lo.y() && p.y() <= hi.y() &&
                p.z() >= zlo && p.z() <= zhi)
                return "drop_path_occupied";
        }
        return nullptr;
    }
};

}  // namespace patrol_control
