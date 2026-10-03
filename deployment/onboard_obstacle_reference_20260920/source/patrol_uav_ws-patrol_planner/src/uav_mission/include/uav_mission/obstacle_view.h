#pragma once
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>

namespace obstacle_view {
using Cell = std::array<int64_t, 3>;
struct Surface {
  double resolution;
  std::vector<geometry_msgs::Point> points;
};
inline bool finite(const geometry_msgs::Point& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
// Conservative aggregation: any occupied fine cell occupies its coarse cell.
// Rebuild from the complete input at every integer-multiple resolution.
inline Surface surface(const std::vector<geometry_msgs::Point>& input,
                       double resolution, size_t limit) {
  if (!std::isfinite(resolution) || resolution <= 0 || limit < 8)
    throw std::invalid_argument("invalid obstacle display limits");
  for (int iteration = 0; iteration < 60; ++iteration, resolution *= 2) {
    std::set<Cell> cells;
    for (const auto& p : input) {
      if (!finite(p)) continue;
      // Reject malformed coordinates before conversion to integer cell indices.
      if (std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) / resolution > 1e12)
        throw std::invalid_argument("map coordinate outside supported range");
      cells.insert({{int64_t(std::floor(p.x / resolution)),
                     int64_t(std::floor(p.y / resolution)),
                     int64_t(std::floor(p.z / resolution))}});
    }
    Surface result{resolution, {}};
    for (const auto& c : cells) {
      bool exposed = false;
      for (int axis = 0; axis < 3 && !exposed; ++axis)
        for (int d : {-1, 1}) {
          Cell adjacent = c;
          adjacent[axis] += d;
          if (!cells.count(adjacent)) { exposed = true; break; }
        }
      if (!exposed) continue;
      geometry_msgs::Point p;
      p.x = (c[0] + 0.5) * resolution;
      p.y = (c[1] + 0.5) * resolution;
      p.z = (c[2] + 0.5) * resolution;
      result.points.push_back(p);
      if (result.points.size() > limit) break;
    }
    if (result.points.size() <= limit) return result;
  }
  throw std::runtime_error("unable to bound obstacle display");
}
inline std::vector<geometry_msgs::Point> path(
    const std::vector<geometry_msgs::Point>& input, size_t limit) {
  if (limit < 2) throw std::invalid_argument("path limit must be >= 2");
  // Do not bridge across corrupt samples and invent a visually valid segment.
  for (const auto& p : input)
    if (!finite(p)) throw std::invalid_argument("nonfinite trajectory");
  if (input.size() <= limit) return input;
  std::vector<geometry_msgs::Point> output;
  output.reserve(limit);
  for (size_t i = 0; i < limit; ++i)
    output.push_back(input[i * (input.size() - 1) / (limit - 1)]);
  return output;
}
}  // namespace obstacle_view
