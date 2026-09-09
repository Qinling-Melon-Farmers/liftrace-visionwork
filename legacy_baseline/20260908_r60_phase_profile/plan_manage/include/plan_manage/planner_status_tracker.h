#ifndef _PLANNER_STATUS_TRACKER_H_
#define _PLANNER_STATUS_TRACKER_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <plan_manage/PlannerStatus.h>
#include <ros/time.h>

namespace fast_planner {

class PlannerStatusTracker {
public:
  PlannerStatusTracker() {
    reset();
  }

  void reset() {
    requested_goal_   = geometry_msgs::PoseStamped();
    effective_goal_   = geometry_msgs::PoseStamped();
    goal_seq_         = 0;
    planning_attempt_ = 0;
    event_seq_        = 0;
    active_           = false;
  }

  std::vector<plan_manage::PlannerStatus> replaceGoal(
      const geometry_msgs::PoseStamped& requested_goal,
      const geometry_msgs::PoseStamped& effective_goal,
      const ros::Time& stamp,
      double cancelled_distance,
      double accepted_distance) {
    std::vector<plan_manage::PlannerStatus> events;
    if (active_) {
      events.push_back(makeEvent(plan_manage::PlannerStatus::CANCELLED,
                                 "superseded_by_new_goal", stamp, cancelled_distance));
    }

    requested_goal_   = requested_goal;
    effective_goal_   = effective_goal;
    goal_seq_         = requested_goal.header.seq;
    effective_goal_.header.seq = goal_seq_;
    planning_attempt_ = 0;
    active_           = true;
    events.push_back(
        makeEvent(plan_manage::PlannerStatus::ACCEPTED, "goal_received", stamp, accepted_distance));
    return events;
  }

  plan_manage::PlannerStatus beginAttempt(uint8_t status,
                                          const std::string& reason,
                                          const ros::Time& stamp,
                                          double distance_to_goal) {
    ++planning_attempt_;
    return makeEvent(status, reason, stamp, distance_to_goal);
  }

  plan_manage::PlannerStatus record(uint8_t status,
                                    const std::string& reason,
                                    const ros::Time& stamp,
                                    double distance_to_goal) {
    return makeEvent(status, reason, stamp, distance_to_goal);
  }

  plan_manage::PlannerStatus finish(const std::string& reason,
                                    const ros::Time& stamp,
                                    double distance_to_goal) {
    plan_manage::PlannerStatus event =
        makeEvent(plan_manage::PlannerStatus::TRAJECTORY_FINISHED, reason, stamp,
                  distance_to_goal);
    active_ = false;
    return event;
  }

  void updateEffectiveGoal(const geometry_msgs::PoseStamped& effective_goal) {
    effective_goal_            = effective_goal;
    effective_goal_.header.seq = goal_seq_;
  }

  bool active() const {
    return active_;
  }

  uint32_t goalSeq() const {
    return goal_seq_;
  }

  uint32_t planningAttempt() const {
    return planning_attempt_;
  }

  const geometry_msgs::PoseStamped& effectiveGoal() const {
    return effective_goal_;
  }

  bool canFinishWithin(double distance_to_goal, double max_distance) const {
    return active_ && std::isfinite(distance_to_goal) &&
           std::isfinite(max_distance) && max_distance >= 0.0 &&
           distance_to_goal <= max_distance;
  }

private:
  plan_manage::PlannerStatus makeEvent(uint8_t status,
                                       const std::string& reason,
                                       const ros::Time& stamp,
                                       double distance_to_goal) {
    plan_manage::PlannerStatus msg;
    msg.header.stamp     = stamp;
    msg.header.frame_id  = effective_goal_.header.frame_id;
    msg.event_seq        = ++event_seq_;
    // roscpp owns the top-level Header.seq and rewrites it when publishing.
    // Keep mission ordering and deduplication on the explicit event_seq field.
    msg.goal_seq         = goal_seq_;
    msg.status           = status;
    msg.planning_attempt = planning_attempt_;
    msg.requested_goal   = requested_goal_;
    msg.effective_goal   = effective_goal_;
    msg.distance_to_goal = distance_to_goal;
    msg.reason           = reason;
    return msg;
  }

  geometry_msgs::PoseStamped requested_goal_, effective_goal_;
  uint32_t goal_seq_, planning_attempt_;
  uint64_t event_seq_;
  bool active_;
};

}  // namespace fast_planner

#endif
