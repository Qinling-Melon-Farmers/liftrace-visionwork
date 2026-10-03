#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace fast_planner {
// Signed clearance inside a fixed XY rectangle, independent of altitude.
inline double searchRegionDistance(const Eigen::Vector3d& p, const Eigen::Vector4d& b) {
  return std::min(std::min(p.x()-b[0], b[1]-p.x()), std::min(p.y()-b[2], b[3]-p.y()));
}
// Same square-envelope geometry as BoundaryRevisit. Its tracking reserve is
// additional command clearance, not another obstacle added around the body.
inline Eigen::Vector4d envelopeSearchBounds(const Eigen::Vector4d& field,
                                           double side, double yaw_degrees) {
  if (!field.allFinite() || !std::isfinite(side) || !std::isfinite(yaw_degrees) ||
      side <= 0.0 || side > 1.0 || yaw_degrees < 0.0 || yaw_degrees > 45.0)
    throw std::invalid_argument("invalid search envelope");
  const double yaw = yaw_degrees * std::acos(-1.0) / 180.0;
  const double half = 0.5 * side * (std::cos(yaw) + std::sin(yaw));
  Eigen::Vector4d bounds(field[0]+half, field[1]-half,
                         field[2]+half, field[3]-half);
  if (bounds[0] >= bounds[1] || bounds[2] >= bounds[3])
    throw std::invalid_argument("search field smaller than envelope");
  return bounds;
}
}
