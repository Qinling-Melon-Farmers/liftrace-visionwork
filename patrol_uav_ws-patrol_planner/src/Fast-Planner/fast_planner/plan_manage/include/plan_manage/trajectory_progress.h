#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace fast_planner {
// Project onto a bounded contiguous ARC, not a fixed time interval. Retimed
// slow initial curves may need >1s to cover even 15cm (seed36). The projection
// window must include that already commandable geometry, but never a distant
// later loop branch. No wall-clock advancement or endpoint jump is used.
template <class Evaluate>
double projectProgress(const Evaluate& position, const Eigen::Vector3d& measured,
                       double previous, double duration, double arc_window = 0.4) {
  previous = std::max(0.0, std::min(previous, duration));
  double best = previous;
  double distance = (position(previous) - measured).squaredNorm();
  double arc = 0.0;
  Eigen::Vector3d prior = position(previous);
  if (!measured.allFinite() || !prior.allFinite() || !std::isfinite(arc_window) || arc_window <= 0) return best;
  unsigned samples = 0;
  for (double t = previous + 0.02; t <= duration + 0.02 && ++samples <= 4096; t += 0.02) {
    const double sample = std::min(t, duration);
    const Eigen::Vector3d point = position(sample);
    if (!point.allFinite()) break;
    arc += (point-prior).norm();
    if (arc > arc_window + 1e-9) break;
    const double candidate = (point - measured).squaredNorm();
    if (candidate < distance) { distance = candidate; best = sample; }
    prior = point;
    if (sample == duration) break;
  }
  return best;
}

template <class Evaluate>
double boundedLookahead(const Evaluate& position, const Eigen::Vector3d& measured,
                        double progress, double duration, double distance) {
  double result = progress;
  double arc = 0.0;
  Eigen::Vector3d previous = position(progress);
  for (double t = progress + 0.02; t <= duration + 0.02; t += 0.02) {
    const double sample = std::min(t, duration);
    const Eigen::Vector3d point = position(sample);
    arc += (point - previous).norm();
    // A tight turn may fit completely inside the tracking sphere. Bound the
    // travelled arc too, so tiny pose noise cannot switch between its sides.
    if (arc > distance || (point - measured).norm() > distance) break;
    result = sample;
    previous = point;
    if (sample == duration) break;
  }
  return result;
}
}  // namespace fast_planner
