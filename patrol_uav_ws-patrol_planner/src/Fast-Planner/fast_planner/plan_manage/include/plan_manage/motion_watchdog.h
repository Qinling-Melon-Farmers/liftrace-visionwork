#pragma once
#include <Eigen/Core>
#include <cmath>

namespace fast_planner {
// Goal-scoped: no trajectory-id input, so replanning cannot renew the budget.
class MotionWatchdog {
 public:
  enum Result { PAUSED, MOVING, WAITING, REPLAN, EXHAUSTED };
  double window = 4.0, movement = 0.04;
  unsigned budget = 2;
  void reset() { active_ = false; attempts_ = 0; last_ = -1; }
  Result requestRecovery(double now) {
    if (!std::isfinite(now)) return PAUSED;
    if (attempts_ >= budget) return EXHAUSTED;
    ++attempts_;
    active_ = false;
    last_ = now;
    since_ = now;
    return REPLAN;
  }
  Result observe(double now, const Eigen::Vector3d& position, bool enabled) {
    if (!std::isfinite(now) || !position.allFinite() || !enabled ||
        (last_ >= 0 && (now < last_ || now-last_ > 1.0))) {
      active_ = false; last_ = now; return PAUSED;
    }
    last_ = now;
    if (!active_ || (position-anchor_).norm() >= movement) {
      anchor_ = position; since_ = now; active_ = true; return MOVING;
    }
    if (now-since_ < window) return WAITING;
    if (attempts_ >= budget) return EXHAUSTED;
    ++attempts_; since_ = now; return REPLAN;
  }
  unsigned attempts() const { return attempts_; }
  double stagnant(double now) const { return active_ ? now-since_ : 0.0; }
 private:
  bool active_ = false;
  unsigned attempts_ = 0;
  double since_ = 0, last_ = -1;
  Eigen::Vector3d anchor_ = Eigen::Vector3d::Zero();
};
}
