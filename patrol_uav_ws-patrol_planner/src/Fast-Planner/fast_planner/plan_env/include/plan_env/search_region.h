#pragma once
#include <Eigen/Core>
#include <algorithm>
namespace fast_planner {
// Signed clearance inside a fixed XY rectangle, independent of altitude.
inline double searchRegionDistance(const Eigen::Vector3d& p, const Eigen::Vector4d& b) {
  return std::min(std::min(p.x()-b[0], b[1]-p.x()), std::min(p.y()-b[2], b[3]-p.y()));
}
}
