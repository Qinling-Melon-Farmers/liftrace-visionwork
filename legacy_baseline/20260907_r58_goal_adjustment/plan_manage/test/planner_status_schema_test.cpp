#include <gtest/gtest.h>
#include <ros/serialization.h>

#include <plan_manage/PlannerStatus.h>
#include <plan_manage/planner_status_tracker.h>

#include <limits>
#include <vector>

namespace {

geometry_msgs::PoseStamped makeGoal(uint32_t seq, double x) {
  geometry_msgs::PoseStamped goal;
  goal.header.seq      = seq;
  goal.header.frame_id = "camera_init";
  goal.pose.position.x = x;
  goal.pose.orientation.w = 1.0;
  return goal;
}

}  // namespace

TEST(PlannerStatusSchema, StatusValuesRemainStable) {
  EXPECT_EQ(0u, plan_manage::PlannerStatus::ACCEPTED);
  EXPECT_EQ(1u, plan_manage::PlannerStatus::PLANNING);
  EXPECT_EQ(2u, plan_manage::PlannerStatus::TRAJECTORY_READY);
  EXPECT_EQ(3u, plan_manage::PlannerStatus::REPLANNING);
  EXPECT_EQ(4u, plan_manage::PlannerStatus::TRAJECTORY_FINISHED);
  EXPECT_EQ(5u, plan_manage::PlannerStatus::FAILED_ATTEMPT);
  EXPECT_EQ(6u, plan_manage::PlannerStatus::CANCELLED);
}

TEST(PlannerStatusSchema, ExplicitCorrelationAndTransportHeaderSurviveSerialization) {
  plan_manage::PlannerStatus source;
  source.header.seq                = 3;
  source.event_seq                 = 9;
  source.goal_seq                  = 42;
  source.status                    = plan_manage::PlannerStatus::REPLANNING;
  source.planning_attempt          = 3;
  source.requested_goal.header.seq = 42;
  source.requested_goal.pose.position.x = 1.0;
  source.effective_goal.header.seq = 42;
  source.effective_goal.pose.position.x = 1.25;
  source.distance_to_goal          = 0.75;
  source.reason                    = "effective_goal_adjusted";

  const uint32_t length = ros::serialization::serializationLength(source);
  std::vector<uint8_t> buffer(length);
  ros::serialization::OStream output(buffer.data(), length);
  ros::serialization::serialize(output, source);

  plan_manage::PlannerStatus restored;
  ros::serialization::IStream input(buffer.data(), length);
  ros::serialization::deserialize(input, restored);

  EXPECT_EQ(source.header.seq, restored.header.seq);
  EXPECT_EQ(source.event_seq, restored.event_seq);
  EXPECT_EQ(source.goal_seq, restored.goal_seq);
  EXPECT_EQ(source.status, restored.status);
  EXPECT_EQ(source.planning_attempt, restored.planning_attempt);
  EXPECT_EQ(source.requested_goal.header.seq, restored.requested_goal.header.seq);
  EXPECT_DOUBLE_EQ(source.requested_goal.pose.position.x,
                   restored.requested_goal.pose.position.x);
  EXPECT_DOUBLE_EQ(source.effective_goal.pose.position.x,
                   restored.effective_goal.pose.position.x);
  EXPECT_DOUBLE_EQ(source.distance_to_goal, restored.distance_to_goal);
  EXPECT_EQ(source.reason, restored.reason);
}

TEST(PlannerStatusTracker, ReplacementCancelsOldGoalBeforeAcceptingNewGoal) {
  fast_planner::PlannerStatusTracker tracker;
  const ros::Time first_stamp(10, 0);

  const std::vector<plan_manage::PlannerStatus> first =
      tracker.replaceGoal(makeGoal(41, 1.0), makeGoal(41, 1.25), first_stamp, 0.0, 2.0);
  ASSERT_EQ(1u, first.size());
  EXPECT_EQ(plan_manage::PlannerStatus::ACCEPTED, first[0].status);
  EXPECT_EQ(1u, first[0].event_seq);
  EXPECT_EQ(0u, first[0].header.seq);
  EXPECT_EQ(41u, first[0].goal_seq);
  EXPECT_EQ(0u, first[0].planning_attempt);

  const plan_manage::PlannerStatus attempt = tracker.beginAttempt(
      plan_manage::PlannerStatus::PLANNING, "new_trajectory_attempt", ros::Time(11, 0), 1.5);
  EXPECT_EQ(2u, attempt.event_seq);
  EXPECT_EQ(1u, attempt.planning_attempt);

  const std::vector<plan_manage::PlannerStatus> replacement =
      tracker.replaceGoal(makeGoal(42, 2.0), makeGoal(42, 2.25), ros::Time(12, 0), 1.25, 3.0);
  ASSERT_EQ(2u, replacement.size());
  EXPECT_EQ(plan_manage::PlannerStatus::CANCELLED, replacement[0].status);
  EXPECT_EQ(3u, replacement[0].event_seq);
  EXPECT_EQ(41u, replacement[0].goal_seq);
  EXPECT_EQ(1u, replacement[0].planning_attempt);
  EXPECT_EQ(plan_manage::PlannerStatus::ACCEPTED, replacement[1].status);
  EXPECT_EQ(4u, replacement[1].event_seq);
  EXPECT_EQ(42u, replacement[1].goal_seq);
  EXPECT_EQ(0u, replacement[1].planning_attempt);
  EXPECT_LT(replacement[0].event_seq, replacement[1].event_seq);
  EXPECT_EQ(42u, tracker.goalSeq());
  EXPECT_EQ(0u, tracker.planningAttempt());

  const plan_manage::PlannerStatus replacement_attempt = tracker.beginAttempt(
      plan_manage::PlannerStatus::REPLANNING, "trajectory_replan_attempt", ros::Time(13, 0),
      2.5);
  EXPECT_EQ(5u, replacement_attempt.event_seq);
  EXPECT_EQ(42u, replacement_attempt.goal_seq);
  EXPECT_EQ(1u, replacement_attempt.planning_attempt);
}

TEST(PlannerStatusTracker, EffectiveAdjustmentKeepsSequenceAndFinishClosesActiveGoal) {
  fast_planner::PlannerStatusTracker tracker;
  tracker.replaceGoal(makeGoal(73, 1.0), makeGoal(73, 1.0), ros::Time(20, 0), 0.0, 2.0);

  geometry_msgs::PoseStamped adjusted = makeGoal(999, 1.4);
  tracker.updateEffectiveGoal(adjusted);
  const plan_manage::PlannerStatus replanning = tracker.record(
      plan_manage::PlannerStatus::REPLANNING, "effective_goal_adjusted", ros::Time(21, 0), 1.2);
  EXPECT_EQ(73u, replanning.goal_seq);
  EXPECT_EQ(73u, replanning.effective_goal.header.seq);
  EXPECT_DOUBLE_EQ(1.4, replanning.effective_goal.pose.position.x);
  EXPECT_TRUE(tracker.active());

  const plan_manage::PlannerStatus finished =
      tracker.finish("local_trajectory_duration_reached", ros::Time(22, 0), 0.1);
  EXPECT_EQ(plan_manage::PlannerStatus::TRAJECTORY_FINISHED, finished.status);
  EXPECT_EQ(73u, finished.goal_seq);
  EXPECT_FALSE(tracker.active());

  const std::vector<plan_manage::PlannerStatus> next =
      tracker.replaceGoal(makeGoal(74, 2.0), makeGoal(74, 2.0), ros::Time(23, 0), 0.1, 2.0);
  ASSERT_EQ(1u, next.size());
  EXPECT_EQ(plan_manage::PlannerStatus::ACCEPTED, next[0].status);
  EXPECT_EQ(74u, next[0].goal_seq);
}

TEST(PlannerStatusTracker, CompletionRequiresActiveFiniteDistanceWithinThreshold) {
  fast_planner::PlannerStatusTracker tracker;
  tracker.replaceGoal(makeGoal(81, 1.0), makeGoal(81, 1.0),
                      ros::Time(30, 0), 0.0, 1.0);

  EXPECT_FALSE(tracker.canFinishWithin(0.329, 0.1));
  EXPECT_TRUE(tracker.canFinishWithin(0.1, 0.1));
  EXPECT_FALSE(tracker.canFinishWithin(
      std::numeric_limits<double>::quiet_NaN(), 0.1));
  EXPECT_FALSE(tracker.canFinishWithin(
      std::numeric_limits<double>::infinity(), 0.1));
  EXPECT_FALSE(tracker.canFinishWithin(0.05, -1.0));

  tracker.finish("goal_reached_after_local_trajectory", ros::Time(31, 0), 0.1);
  EXPECT_FALSE(tracker.canFinishWithin(0.0, 0.1));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
