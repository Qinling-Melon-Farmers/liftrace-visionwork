#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace fast_planner {

// Sample the existing local goal neighborhood around the REQUESTED goal.
// Repeated map updates cannot walk the target farther away. Height is fixed.
template <typename Clearance>
bool nearbyFreeGoal(const Eigen::Vector3d& requested, double radius,
                    double step, double minimum_clearance,
                    Clearance clearance, Eigen::Vector3d& result) {
  if (!(radius > 0.0 && step > 0.0)) return false;
  const int cells = static_cast<int>(std::ceil(radius / step));
  double best_distance = std::numeric_limits<double>::infinity();
  double best_clearance = -std::numeric_limits<double>::infinity();
  bool found = false;
  for (int x = -cells; x <= cells; ++x) {
    for (int y = -cells; y <= cells; ++y) {
      const double distance = step * std::hypot(x, y);
      if (distance > radius + 1e-9 || distance > best_distance + 1e-9) continue;
      const Eigen::Vector3d candidate = requested + Eigen::Vector3d(x * step, y * step, 0.0);
      const double available = clearance(candidate);
      if (!std::isfinite(available) || available < minimum_clearance) continue;
      if (distance < best_distance - 1e-9 || available > best_clearance) {
        best_distance = distance;
        best_clearance = available;
        result = candidate;
        found = true;
      }
    }
  }
  return found;
}

}  // namespace fast_planner
