#include <gtest/gtest.h>
#include <plan_manage/motion_watchdog.h>
using fast_planner::MotionWatchdog;
TEST(MotionWatchdog, BoundedRecoveryWithoutTrajectoryReset) {
  MotionWatchdog w;
  int replans=0;
  for (int i=0;i<=121;++i) {
    auto r=w.observe(i*.1,Eigen::Vector3d::Zero(),true);
    if(r==MotionWatchdog::REPLAN) ++replans;
  }
  EXPECT_EQ(replans,2);
  EXPECT_EQ(w.observe(12.2,Eigen::Vector3d::Zero(),true),MotionWatchdog::EXHAUSTED);
  w.reset(); EXPECT_EQ(w.attempts(),0);
}
TEST(MotionWatchdog, DetourAndVerticalMotionCountEvenAwayFromGoal) {
  MotionWatchdog w;
  for(int i=0;i<200;++i) {
    EXPECT_NE(w.observe(i*.1,Eigen::Vector3d(-i*.01,0,i*.01),true),MotionWatchdog::REPLAN);
  }
}
TEST(MotionWatchdog, IntentionalHoldStaleDataAndClockJumpCannotTrigger) {
  MotionWatchdog w;
  for(int i=0;i<100;++i) EXPECT_EQ(w.observe(i*.1,Eigen::Vector3d::Zero(),false),MotionWatchdog::PAUSED);
  w.observe(10,Eigen::Vector3d::Zero(),true);
  EXPECT_EQ(w.observe(30,Eigen::Vector3d::Zero(),true),MotionWatchdog::PAUSED);
  EXPECT_EQ(w.observe(1,Eigen::Vector3d::Zero(),true),MotionWatchdog::PAUSED);
  EXPECT_EQ(w.attempts(),0);
}
TEST(MotionWatchdog, PoseNoiseDoesNotRenewBudget) {
  MotionWatchdog w; int requests=0;
  for(int i=0;i<100;++i) if(w.observe(i*.1,Eigen::Vector3d((i%2)*.005,0,0),true)==MotionWatchdog::REPLAN)++requests;
  EXPECT_EQ(requests,2);
}
TEST(MotionWatchdog, ExplicitServerRecoverySharesGoalBudget) {
  MotionWatchdog w;
  EXPECT_EQ(w.requestRecovery(1.0),MotionWatchdog::REPLAN);
  EXPECT_EQ(w.attempts(),1u);
  EXPECT_EQ(w.requestRecovery(2.0),MotionWatchdog::REPLAN);
  EXPECT_EQ(w.requestRecovery(3.0),MotionWatchdog::EXHAUSTED);
  w.reset();
  EXPECT_EQ(w.requestRecovery(4.0),MotionWatchdog::REPLAN);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
