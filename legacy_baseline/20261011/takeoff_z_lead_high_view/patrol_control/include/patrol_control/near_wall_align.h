#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace patrol_control {

struct NearWallAlignFence {
    bool enabled = false;
    bool valid = true;
    std::array<double, 4> bounds{{-4.8, 4.8, -0.5, 7.4}};
    double side_m = 0.55;
    double tracking_reserve_m = 0.03;
    double yaw_budget_deg = 10.0;

    double margin(double yaw) const {
        const double budget = yaw_budget_deg * 3.14159265358979323846 / 180.0;
        const double budget_projection = std::cos(budget) + std::sin(budget);
        const double actual_projection = std::abs(std::cos(yaw)) +
                                         std::abs(std::sin(yaw));
        return 0.5 * side_m * std::max(budget_projection, actual_projection) +
               tracking_reserve_m;
    }

    bool wellFormed() const {
        if (!valid || !std::isfinite(side_m) ||
            !std::isfinite(tracking_reserve_m) || side_m <= 0.0 ||
            side_m > 1.0 || tracking_reserve_m < 0.0 ||
            tracking_reserve_m > 0.1 || !std::isfinite(yaw_budget_deg) ||
            yaw_budget_deg < 0.0 || yaw_budget_deg > 45.0) return false;
        for (double value : bounds) if (!std::isfinite(value)) return false;
        const double worst = 0.5 * side_m * std::sqrt(2.0) + tracking_reserve_m;
        return bounds[1] - bounds[0] > 2.0 * worst &&
               bounds[3] - bounds[2] > 2.0 * worst;
    }

    bool contains(double x, double y, double yaw) const {
        if (!enabled) return true;
        if (!wellFormed() || !std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(yaw)) return false;
        const double m = margin(yaw);
        return x >= bounds[0] + m && x <= bounds[1] - m &&
               y >= bounds[2] + m && y <= bounds[3] - m;
    }

    std::pair<double, double> clamp(double x, double y,
                                    double current_yaw,
                                    double command_yaw) const {
        const double m = std::max(margin(current_yaw), margin(command_yaw));
        return {std::max(bounds[0] + m, std::min(bounds[1] - m, x)),
                std::max(bounds[2] + m, std::min(bounds[3] - m, y))};
    }
};

}  // namespace patrol_control
