#pragma once

#include <cmath>
#include <cstdint>

namespace fast_planner {

// Debounces traj_server tracking holds and binds them to the exact mission
// goal and Bspline generation. Recovery budgeting remains goal-scoped in the
// FSM's MotionWatchdog; this class only emits once per held generation.
class ServerHoldMonitor {
 public:
  enum Result { IGNORED, CLEAR, WAITING, REPLAN };

  double window = 0.25;
  double max_age = 0.50;

  void reset() {
    active_ = false;
    fired_ = false;
    goal_seq_ = 0;
    traj_id_ = -1;
    traj_start_ns_ = 0;
    since_ = 0.0;
    last_now_ = -1.0;
  }

  Result observe(double now, double stamp, uint32_t expected_goal_seq,
                 int expected_traj_id, uint64_t expected_traj_start_ns,
                 uint32_t message_goal_seq, int message_traj_id,
                 uint64_t message_traj_start_ns, bool tracking_hold) {
    if (!std::isfinite(now) || !std::isfinite(stamp) || now < stamp ||
        now - stamp > max_age || expected_traj_start_ns == 0 ||
        message_goal_seq != expected_goal_seq ||
        message_traj_id != expected_traj_id ||
        message_traj_start_ns != expected_traj_start_ns) {
      return IGNORED;
    }

    const bool generation_changed = goal_seq_ != message_goal_seq ||
        traj_id_ != message_traj_id || traj_start_ns_ != message_traj_start_ns;
    if (generation_changed) {
      active_ = false;
      fired_ = false;
      goal_seq_ = message_goal_seq;
      traj_id_ = message_traj_id;
      traj_start_ns_ = message_traj_start_ns;
      last_now_ = -1.0;
    }

    if (last_now_ >= 0.0 && (now < last_now_ || now - last_now_ > max_age)) {
      active_ = false;
      since_ = 0.0;
    }
    last_now_ = now;

    if (!tracking_hold) {
      active_ = false;
      since_ = 0.0;
      return CLEAR;
    }
    if (fired_) return WAITING;
    if (!active_) {
      active_ = true;
      since_ = now;
      return WAITING;
    }
    if (now - since_ < window) return WAITING;
    fired_ = true;
    return REPLAN;
  }

 private:
  bool active_ = false;
  bool fired_ = false;
  uint32_t goal_seq_ = 0;
  int traj_id_ = -1;
  uint64_t traj_start_ns_ = 0;
  double since_ = 0.0;
  double last_now_ = -1.0;
};

}  // namespace fast_planner
