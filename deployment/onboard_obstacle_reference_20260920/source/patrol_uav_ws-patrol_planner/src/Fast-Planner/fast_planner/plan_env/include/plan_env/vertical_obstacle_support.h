#ifndef PLAN_ENV_VERTICAL_OBSTACLE_SUPPORT_H
#define PLAN_ENV_VERTICAL_OBSTACLE_SUPPORT_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {

// Evidence for extending a measured obstacle through the full flight height.
// Ordinary 3-D occupancy is independent of this additional classification.
class VerticalObstacleSupport {
 public:
  VerticalObstacleSupport(int nx, int ny, int radius_cells,
                          int min_points, double min_height_span)
      : nx_(nx), ny_(ny), radius_(radius_cells), min_points_(min_points),
        min_span_(min_height_span), counts_(nx * ny, 0),
        lows_(nx * ny, std::numeric_limits<double>::infinity()),
        highs_(nx * ny, -std::numeric_limits<double>::infinity()),
        cache_(nx * ny, -1) {}

  void observe(int x, int y, double z) {
    if (!inside(x, y) || !std::isfinite(z)) return;
    const int i = x * ny_ + y;
    ++counts_[i];
    lows_[i] = std::min(lows_[i], z);
    highs_[i] = std::max(highs_[i], z);
  }

  // Call after all observations have been supplied for this cloud.
  bool supported(int x, int y) {
    if (!inside(x, y)) return false;
    const int i = x * ny_ + y;
    if (cache_[i] >= 0) return cache_[i] == 1;
    int count = 0;
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (int dx = std::max(0, x - radius_); dx <= std::min(nx_ - 1, x + radius_); ++dx)
      for (int dy = std::max(0, y - radius_); dy <= std::min(ny_ - 1, y + radius_); ++dy) {
        const int j = dx * ny_ + dy;
        count += counts_[j];
        low = std::min(low, lows_[j]);
        high = std::max(high, highs_[j]);
      }
    const bool result = count >= min_points_ && high - low + 1e-9 >= min_span_;
    cache_[i] = result ? 1 : 0;
    return result;
  }

 private:
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < nx_ && y < ny_; }
  int nx_, ny_, radius_, min_points_;
  double min_span_;
  std::vector<int> counts_;
  std::vector<double> lows_, highs_;
  std::vector<signed char> cache_;
};
}  // namespace fast_planner
#endif
