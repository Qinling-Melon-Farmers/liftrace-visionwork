#include <gtest/gtest.h>
#include <plan_manage/server_hold_monitor.h>

using fast_planner::ServerHoldMonitor;

TEST(ServerHoldMonitor, RequiresMatchingGoalAndTrajectoryIdentity) {
  ServerHoldMonitor monitor;
  EXPECT_EQ(monitor.observe(1.0, 1.0, 7, 3, 100, 8, 3, 100, true),
            ServerHoldMonitor::IGNORED);
  EXPECT_EQ(monitor.observe(1.0, 1.0, 7, 3, 100, 7, 4, 100, true),
            ServerHoldMonitor::IGNORED);
  EXPECT_EQ(monitor.observe(1.0, 1.0, 7, 3, 100, 7, 3, 101, true),
            ServerHoldMonitor::IGNORED);
}

TEST(ServerHoldMonitor, TransientHoldClearsWithoutRecovery) {
  ServerHoldMonitor monitor;
  EXPECT_EQ(monitor.observe(1.0, 1.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(1.1, 1.1, 7, 3, 100, 7, 3, 100, false),
            ServerHoldMonitor::CLEAR);
  EXPECT_EQ(monitor.observe(1.4, 1.4, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
}

TEST(ServerHoldMonitor, SustainedHoldFiresOncePerGeneration) {
  ServerHoldMonitor monitor;
  EXPECT_EQ(monitor.observe(1.0, 1.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(1.3, 1.3, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::REPLAN);
  EXPECT_EQ(monitor.observe(2.0, 2.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(2.1, 2.1, 7, 4, 200, 7, 4, 200, true),
            ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(2.4, 2.4, 7, 4, 200, 7, 4, 200, true),
            ServerHoldMonitor::REPLAN);
}

TEST(ServerHoldMonitor, RejectsStaleFutureAndClockGapMessages) {
  ServerHoldMonitor monitor;
  EXPECT_EQ(monitor.observe(2.0, 1.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::IGNORED);
  EXPECT_EQ(monitor.observe(1.0, 1.1, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::IGNORED);
  EXPECT_EQ(monitor.observe(2.0, 2.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(3.0, 3.0, 7, 3, 100, 7, 3, 100, true),
            ServerHoldMonitor::WAITING);
}

TEST(ServerHoldMonitor, RosFirstGoalMayHaveZeroTransportSequence) {
  ServerHoldMonitor monitor;
  EXPECT_EQ(monitor.observe(1., 1., 0, 1, 100, 0, 1, 100, true), ServerHoldMonitor::WAITING);
  EXPECT_EQ(monitor.observe(1.3, 1.3, 0, 1, 100, 0, 1, 100, true), ServerHoldMonitor::REPLAN);
  EXPECT_EQ(monitor.observe(1.4, 1.4, 0, 1, 0, 0, 1, 0, true), ServerHoldMonitor::IGNORED);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
