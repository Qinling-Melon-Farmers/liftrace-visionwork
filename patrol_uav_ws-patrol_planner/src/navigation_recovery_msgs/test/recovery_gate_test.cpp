#include <gtest/gtest.h>
#include <navigation_recovery_msgs/navigation_recovery_gate.h>
using namespace navigation_recovery_msgs;

struct Lease {
  NavigationRecoveryGate gate;
  NavigationRecoveryCommand command;
  NavigationRecoveryContext context;
  Eigen::Vector3d measured{0,0,3.1};
  Lease() {
    gate.enabled=true;
    context.header.stamp=ros::Time(10,0);context.header.frame_id="camera_init";
    context.deadline=ros::Time(30,0);context.active=true;
    command.header.stamp=ros::Time(11,0);command.header.frame_id="camera_init";
    command.goal_stamp=context.header.stamp;command.started_at=ros::Time(10,0);
    command.deadline=ros::Time(18,0);command.recovery_id=1;command.active=true;
    command.pose.position.z=3.08;command.pose.orientation.w=1;command.soft_max_z=2.98;
  }
  bool ok(bool owner=true) {return gate.accepts(command,context,ros::Time(11,0),measured,owner);}
};
TEST(RecoveryGate, ValidOverheightCommandPassesBothReceiverContract) { Lease l;EXPECT_TRUE(l.ok()); }
TEST(RecoveryGate, DefaultDisabled) { Lease l;l.gate.enabled=false;EXPECT_FALSE(l.ok()); }
TEST(RecoveryGate, DeadlineCannotExtendOriginalAction) { Lease l;l.command.deadline=ros::Time(31,0);EXPECT_FALSE(l.ok()); }
TEST(RecoveryGate, LeaseBoundedEvenWithLongActionBudget) { Lease l;l.command.deadline=ros::Time(19,0);EXPECT_FALSE(l.ok()); }
TEST(RecoveryGate, StaleAndFutureCommandsRejected) {
  Lease l;l.command.header.stamp=ros::Time(10,0);EXPECT_FALSE(l.ok());
  l.command.header.stamp=ros::Time(12,0);EXPECT_FALSE(l.ok());
}
TEST(RecoveryGate, GoalAndFrameIdentityMustMatch) {
  Lease l;l.command.goal_stamp=ros::Time(9,0);EXPECT_FALSE(l.ok());
  l.command.goal_stamp=l.context.header.stamp;l.command.header.frame_id="fc";EXPECT_FALSE(l.ok());
}
TEST(RecoveryGate, OwnershipAndCancellationCannotBeExempted) {
  Lease l;EXPECT_FALSE(l.ok(false));l.context.active=false;EXPECT_FALSE(l.ok());
}
TEST(RecoveryGate, HardHeightAndExcessStillBounded) {
  Lease l;l.command.pose.position.z=3.4;l.measured.z()=3.4;EXPECT_FALSE(l.ok());
  l.command.soft_max_z=4.;l.command.pose.position.z=3.6;l.measured.z()=3.6;EXPECT_FALSE(l.ok());
}
TEST(RecoveryGate, SpeedDistanceAndNonFiniteReject) {
  Lease l;l.command.velocity.x=.31;EXPECT_FALSE(l.ok());
  l.command.velocity.x=0;l.command.pose.position.x=.3;EXPECT_FALSE(l.ok());
  l.command.pose.position.x=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(l.ok());
}
int main(int argc,char**argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
