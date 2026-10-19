#include <gtest/gtest.h>
#include <plan_manage/navigation_recovery.h>
#include <bspline/non_uniform_bspline.h>

using namespace fast_planner::recovery;
namespace {
struct Box { Vec low, high; int mask; };
struct World {
  std::vector<Box> boxes;
  bool covered=true;
  MapQuery query() {
    MapQuery m;
    m.sources=[this](const Vec& lo,const Vec& hi){
      int mask=0;
      for (const auto& b:boxes)
        if ((hi.array()>=b.low.array()).all() && (lo.array()<=b.high.array()).all()) mask|=b.mask;
      return mask;
    };
    m.physicalClear=[this](const Vec& lo,const Vec& hi){
      return !(query().sources(lo,hi)&PHYSICAL);
    };
    m.observedFree=[this](const Vec&,const Vec&){return covered;};
    m.canResume=[](const Vec&,double){return true;}; // synthetic world only
    return m;
  }
};
Config config() { Config c;c.enabled=true;c.require_reference_attestation=true;return c; }
Context context() {
  Context x; x.now=10;x.action_deadline=30;x.mission_deadline=100;x.soft_max_z=2.98;
  x.goal=7;x.position=Vec(0,0,2.4);x.goal_position=Vec(2,1,2.4);
  x.offboard=true;x.navigation_owner=true;
  auto& e=x.evidence;e.stamp=e.map_stamp=x.now;e.frame=e.map_frame="task";
  e.reference_verified=e.lio_healthy=e.fc_reset_stream_verified=e.coverage_verified=true;
  e.map_revision=1;return x;
}
World column() {
  World w;
  w.boxes.push_back({Vec(-.1,-.1,1.5),Vec(.1,.1,3.4),COLUMN});
  w.boxes.push_back({Vec(-.1,-.1,.7),Vec(.1,.1,1.5),PHYSICAL});
  return w;
}
bool planned(Config c,Context x,World& w,std::vector<Vec> candidates={Vec(.3,0,2.4)}) {
  Curve curve;std::string reason;return plan(c,x,w.query(),candidates,curve,reason);
}
void refresh(Context& x,double t) { x.now=t;x.evidence.stamp=t;x.evidence.map_stamp=t; }
}

TEST(NavigationRecovery, DisabledByDefault) {
  auto w=column();EXPECT_FALSE(planned(Config(),context(),w));
}
TEST(NavigationRecovery, PureColumnExitPreservesPositionVelocityAndOriginalGoal) {
  auto w=column();auto x=context();x.velocity=Vec(-.08,.02,0);
  Transaction tx(config());ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}))<<tx.reason();
  EXPECT_LT((tx.curve().position(0)-x.position).norm(),1e-12);
  EXPECT_LT((tx.curve().velocity(0)-x.velocity).norm(),1e-12);
  EXPECT_EQ(tx.original().goal,x.goal);
  EXPECT_LT((tx.original().goal_position-x.goal_position).norm(),1e-12);
  EXPECT_EQ(tx.original().action_deadline,30);
}
TEST(NavigationRecovery, NecessaryInflationAtStartCannotBeExempted) {
  auto w=column();w.boxes.push_back({Vec(-.2,-.2,2.3),Vec(.2,.2,2.5),PHYSICAL});
  EXPECT_FALSE(planned(config(),context(),w));
}
TEST(NavigationRecovery, ThinObstacleBetweenClearEndpointsIsRejected) {
  auto w=column();w.boxes.push_back({Vec(.149,-1,2),Vec(.151,1,3),PHYSICAL});
  EXPECT_FALSE(planned(config(),context(),w));
}
TEST(NavigationRecovery, InitialVelocityBrakingMustAlsoBeClear) {
  auto w=column();auto x=context();x.velocity=Vec(-.15,0,0);
  w.boxes.push_back({Vec(-1,-1,2),Vec(-.01,1,3),PHYSICAL});
  EXPECT_FALSE(planned(config(),x,w));
}
TEST(NavigationRecovery, HistoricalPointRecheckedAndNearestReachableChosen) {
  auto w=column();w.boxes.push_back({Vec(.14,-.05,2.3),Vec(.16,.05,2.5),PHYSICAL});
  auto x=context();Curve curve;std::string reason;
  ASSERT_TRUE(plan(config(),x,w.query(),{Vec(.15,0,2.4),Vec(-.3,0,2.4),Vec(0,.6,2.4)},curve,reason));
  EXPECT_LT((curve.controls[3]-Vec(-.3,0,2.4)).norm(),1e-9);
}
TEST(NavigationRecovery, UnknownCoverageIsNotFreeSpace) {
  auto w=column();w.covered=false;EXPECT_FALSE(planned(config(),context(),w));
  auto x=context();x.evidence.coverage_verified=false;w.covered=true;
  EXPECT_FALSE(planned(config(),x,w));
}
TEST(NavigationRecovery, UnknownSourceAndHardBoundaryCannotBeExempted) {
  auto w=column();w.boxes.push_back({Vec(-1,-1,2),Vec(1,1,3),BOUNDARY});
  EXPECT_FALSE(planned(config(),context(),w));
  Curve curve;std::string reason;auto map=w.query();
  map.sources=[](const Vec&,const Vec&){return -1;};
  EXPECT_FALSE(plan(config(),context(),map,{Vec(.3,0,2.4)},curve,reason));
}
TEST(NavigationRecovery, FreshOdometryAloneCannotAuthorizeAnyRecovery) {
  auto w=column();auto x=context();x.evidence=Evidence();
  x.evidence.stamp=x.evidence.map_stamp=x.now;
  EXPECT_FALSE(planned(config(),x,w));
}
TEST(NavigationRecovery, FcOnlyResetDoesNotAuthorizeDescent) {
  World w;auto x=context();x.position.z()=3.10;
  EXPECT_EQ(admission(config(),x),"untrusted_height");
  EXPECT_FALSE(planned(config(),x,w,{Vec(0,0,2.78)}));
  x.evidence.independent_height_verified=true;x.evidence.fc_reset_stream_verified=false;
  EXPECT_EQ(admission(config(),x),"untrusted_reference");
  EXPECT_FALSE(planned(config(),x,w,{Vec(0,0,2.78)}));
}
TEST(NavigationRecovery, TrustedTrueExcessAllowsBrakingThenReturn) {
  World w;auto x=context();x.position.z()=3.10;x.velocity.z()=.08;
  x.evidence.independent_height_verified=true;
  Curve curve;std::string reason;
  ASSERT_TRUE(plan(config(),x,w.query(),{Vec(0,0,2.78)},curve,reason))<<reason;
  EXPECT_NEAR(curve.velocity(0).z(),.08,1e-12);
  EXPECT_GT(curve.position(.05).z(),3.10); // no clipped start/no instant reversal
  EXPECT_NEAR(curve.position(curve.duration).z(),2.78,1e-12);
}
TEST(NavigationRecovery, HardCeilingStillRejectsBrakingOvershoot) {
  World w;auto x=context();x.position.z()=3.10;x.velocity.z()=.08;
  x.evidence.independent_height_verified=true;auto c=config();c.hard_max_z=3.105;
  EXPECT_FALSE(planned(c,x,w,{Vec(0,0,2.78)}));
}
TEST(NavigationRecovery, ObstacleBelowRejectsVerticalReturn) {
  World w;w.boxes.push_back({Vec(-1,-1,2.89),Vec(1,1,2.91),PHYSICAL});
  auto x=context();x.position.z()=3.10;x.evidence.independent_height_verified=true;
  EXPECT_FALSE(planned(config(),x,w,{Vec(0,0,2.78)}));
}
TEST(NavigationRecovery, CanSelectLateralExitInsteadOfOnlyClippingZ) {
  World w;w.boxes.push_back({Vec(-.03,-.2,2.89),Vec(.03,.2,2.91),PHYSICAL});
  auto x=context();x.position.z()=3.10;x.evidence.independent_height_verified=true;
  Curve curve;std::string reason;
  auto c=config();c.tracking_error=.08;
  ASSERT_TRUE(plan(c,x,w.query(),{Vec(0,0,2.78),Vec(.5,0,2.78)},curve,reason));
  EXPECT_NEAR(curve.controls[3].x(),.5,1e-9);
}
TEST(NavigationRecovery, NoReentryAfterLeavingVirtualRegion) {
  auto w=column();w.boxes.push_back({Vec(.19,-1,2),Vec(.21,1,3),COLUMN});
  EXPECT_FALSE(planned(config(),context(),w));
}
TEST(NavigationRecovery, StaleFutureFramesAndGenerationReject) {
  auto x=context();x.evidence.stamp=9;EXPECT_EQ(admission(config(),x),"stale_input");
  x=context();x.evidence.stamp=11;EXPECT_EQ(admission(config(),x),"stale_input");
  x=context();x.evidence.map_stamp=9;EXPECT_EQ(admission(config(),x),"stale_input");
  x=context();x.evidence.map_frame="fc";EXPECT_EQ(admission(config(),x),"untrusted_reference");
  x=context();x.evidence.map_generation=1;EXPECT_EQ(admission(config(),x),"untrusted_reference");
}
TEST(NavigationRecovery, TransactionLockAndTakeoverRejectAdmission) {
  auto x=context();x.release_transaction_active=true;
  EXPECT_EQ(admission(config(),x),"release_transaction_locked");
  x=context();x.takeover=true;EXPECT_EQ(admission(config(),x),"ownership_lost");
}
TEST(NavigationRecovery, ActionAndMissionDeadlinesAreNeverExtended) {
  auto w=column();auto x=context();x.action_deadline=10.5;
  EXPECT_FALSE(planned(config(),x,w));
  x=context();x.mission_deadline=10.5;EXPECT_FALSE(planned(config(),x,w));
}
TEST(NavigationRecovery, InvalidAndFastStatesReject) {
  auto x=context();x.velocity.x()=.6;EXPECT_EQ(admission(config(),x),"entry_speed");
  x=context();x.position.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(admission(config(),x),"invalid_input");
  auto c=config();c.hard_max_z=c.hard_min_z;EXPECT_FALSE(c.valid());
}
TEST(NavigationRecovery, TakeoverStopsWithoutPublishingFabricatedHold) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  x.takeover=true;Vec p,v;EXPECT_EQ(tx.step(x,w.query(),p,v),Transaction::FAILED);
  EXPECT_FALSE(p.allFinite());EXPECT_FALSE(v.allFinite());
}
TEST(NavigationRecovery, FcEpochChangeInvalidatesInFlightCurve) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  ++x.evidence.generation;++x.evidence.map_generation;Vec p,v;
  EXPECT_EQ(tx.step(x,w.query(),p,v),Transaction::FAILED);
  EXPECT_EQ(tx.reason(),"identity_or_constraint_changed");
}
TEST(NavigationRecovery, NewObstacleInvalidatesPreviouslyClearCurve) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  w.boxes.push_back({Vec(.14,-1,2),Vec(.16,1,3),PHYSICAL});
  ++x.evidence.map_revision;Vec p,v;
  EXPECT_EQ(tx.step(x,w.query(),p,v),Transaction::FAILED);
  EXPECT_EQ(tx.reason(),"path_invalidated");
}
TEST(NavigationRecovery, TrackingFailureCannotLoopOnSameGoal) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  x.position.y()=.2;Vec p,v;EXPECT_EQ(tx.step(x,w.query(),p,v),Transaction::FAILED);
  x=context();EXPECT_FALSE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  EXPECT_EQ(tx.reason(),"budget_exhausted");
}
TEST(NavigationRecovery, ClockAndChangedDeadlineAbort) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  refresh(x,9.9);Vec p,v;EXPECT_EQ(tx.step(x,w.query(),p,v),Transaction::FAILED);
  Transaction other(config());x=context();ASSERT_TRUE(other.begin(x,w.query(),{Vec(.3,0,2.4)}));
  x.action_deadline=x.now;EXPECT_EQ(other.step(x,w.query(),p,v),Transaction::FAILED);
}
TEST(NavigationRecovery, ResumeRequiresSettledEndpointAndKeepsOriginalAction) {
  auto w=column();auto x=context();Transaction tx(config());
  ASSERT_TRUE(tx.begin(x,w.query(),{Vec(.3,0,2.4)}));
  Vec p,v;const double duration=tx.curve().duration;
  for (double t=0;t<=duration+.4;t+=.02) {
    refresh(x,10+t);x.position=tx.curve().position(t);x.velocity=tx.curve().velocity(t);
    const auto state=tx.step(x,w.query(),p,v);
    if (t<duration+.2-1e-8) { EXPECT_EQ(state,Transaction::ACTIVE); }
    if (state==Transaction::RESUME) break;
  }
  EXPECT_EQ(tx.state(),Transaction::RESUME)<<tx.reason();
  EXPECT_EQ(tx.original().goal,7u);EXPECT_EQ(tx.original().action_deadline,30);
  EXPECT_LT((tx.original().goal_position-Vec(2,1,2.4)).norm(),1e-12);
}
TEST(NavigationRecovery, ExactCubicCanUseExistingBsplineEvaluatorWithoutRefitting) {
  auto w=column();auto x=context();x.velocity=Vec(.05,0,0);Curve curve;std::string reason;
  ASSERT_TRUE(plan(config(),x,w.query(),{Vec(.3,0,2.4)},curve,reason));
  Eigen::MatrixXd points(4,3);for(int i=0;i<4;++i) points.row(i)=curve.controls[i].transpose();
  fast_planner::NonUniformBspline spline(points,3,curve.duration);
  Eigen::VectorXd knots(8);knots<<0,0,0,0,curve.duration,curve.duration,curve.duration,curve.duration;
  spline.setKnot(knots);auto derivative=spline.getDerivative();
  for(double t=0;t<=curve.duration;t+=.01) {
    EXPECT_LT((spline.evaluateDeBoorT(t)-curve.position(t)).norm(),1e-9);
    EXPECT_LT((derivative.evaluateDeBoorT(t)-curve.velocity(t)).norm(),1e-9);
  }
}

TEST(NavigationRecovery, ProductionHeightRecoveryUsesExistingFreshReference) {
  World w;auto x=context();x.position.z()=3.10;
  x.evidence.reference_verified=x.evidence.lio_healthy=x.evidence.fc_reset_stream_verified=false;
  Config c;c.enabled=true;
  EXPECT_TRUE(planned(c,x,w,{Vec(0,0,2.78)}));
  x.evidence.stamp=8.;EXPECT_FALSE(planned(c,x,w,{Vec(0,0,2.78)}));
}
TEST(NavigationRecovery, OrdinaryBufferCanExitButNecessaryBodyClearanceCannot) {
  World w;w.boxes.push_back({Vec(-.1,-.1,2.1),Vec(.1,.1,2.7),BUFFER});
  Config c;c.enabled=true;auto x=context();
  EXPECT_TRUE(planned(c,x,w));
  w.boxes.push_back({Vec(-.01,-.01,2.3),Vec(.01,.01,2.5),PHYSICAL});
  EXPECT_FALSE(planned(c,x,w));
}
TEST(NavigationRecovery, NormalPlannerThresholdAndEndpointSlackAreNotRelaxed) {
  EXPECT_FALSE(normalPlannerStartClear(0,.19,.20,0.,.05));
  EXPECT_TRUE(normalPlannerStartClear(0,.20,.20,0.,.05));
  EXPECT_FALSE(normalPlannerStartClear(0,.30,.20,.04,.05));
  EXPECT_TRUE(normalPlannerStartClear(0,.35,.20,.04,.05));
  EXPECT_FALSE(normalPlannerStartClear(1,1.,.20,0.,.05));
  EXPECT_FALSE(normalPlannerStartClear(0,std::numeric_limits<double>::quiet_NaN(),.20,0.,.05));
}
TEST(NavigationRecovery, ExitSelectionSkipsFreeButNonResumableNeighbor) {
  auto w=column();auto x=context();auto q=w.query();
  q.canResume=[](const Vec& p,double){return p.x()>=.49;};
  Curve curve;std::string reason;
  ASSERT_TRUE(plan(config(),x,q,{Vec(.3,0,2.4),Vec(.5,0,2.4)},curve,reason));
  EXPECT_NEAR(curve.controls[3].x(),.5,1e-9);
  x.action_deadline=x.now+1.;
  EXPECT_FALSE(plan(config(),x,q,{Vec(.3,0,2.4),Vec(.5,0,2.4)},curve,reason));
}
TEST(NavigationRecovery, ActualHandoffMustMeetNormalClearanceWithoutRenewingDeadline) {
  auto w=column();auto x=context();auto q=w.query();bool actual_clear=false;
  q.canResume=[&](const Vec&,double slack){return slack>0 || actual_clear;};
  Transaction tx(config());ASSERT_TRUE(tx.begin(x,q,{Vec(.3,0,2.4)}));
  Vec p,v;refresh(x,10+tx.curve().duration+.3);x.position=tx.curve().controls[3];x.velocity.setZero();
  EXPECT_EQ(tx.step(x,q,p,v),Transaction::ACTIVE);
  EXPECT_EQ(tx.original().action_deadline,30.);
  actual_clear=true;
  EXPECT_EQ(tx.step(x,q,p,v),Transaction::ACTIVE);
  refresh(x,x.now+.25);EXPECT_EQ(tx.step(x,q,p,v),Transaction::RESUME);
  EXPECT_EQ(tx.original().action_deadline,30.);
  Transaction expired(config());x=context();actual_clear=false;
  ASSERT_TRUE(expired.begin(x,q,{Vec(.3,0,2.4)}));
  refresh(x,18.);x.position=expired.curve().controls[3];x.velocity.setZero();
  EXPECT_EQ(expired.step(x,q,p,v),Transaction::FAILED);
  EXPECT_EQ(expired.reason(),"deadline_or_clock");
}
TEST(NavigationRecovery, ChildVelocityUsesCompleteParentRotation) {
  const double pi=std::acos(-1.);Vec actual;
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(pi/2.,Vec::UnitZ()));
  EXPECT_EQ(parentVelocity(Vec(1,2,3),yaw,"base_link","child",actual),"");
  EXPECT_LT((actual-Vec(-2,1,3)).norm(),1e-12);
  const Eigen::Quaterniond tilted(Eigen::AngleAxisd(pi/6.,Vec::UnitX()));
  EXPECT_EQ(parentVelocity(Vec(1,2,3),tilted,"base_link","child",actual),"");
  EXPECT_LT((actual-Vec(1,2*std::cos(pi/6.)-3*.5,1+3*std::cos(pi/6.))).norm(),1e-12);
  const Eigen::Quaterniond combined=yaw*Eigen::Quaterniond(Eigen::AngleAxisd(pi/2.,Vec::UnitY()));
  EXPECT_EQ(parentVelocity(Vec(1,2,3),combined,"base_link","child",actual),"");
  EXPECT_LT((actual-Vec(-2,3,-1)).norm(),1e-12);
}
TEST(NavigationRecovery, WorldVelocityRemainsUnrotatedForExplicitLegacyFixture) {
  Vec actual;const Eigen::Quaterniond yaw(Eigen::AngleAxisd(std::acos(-1.)/2.,Vec::UnitZ()));
  EXPECT_EQ(parentVelocity(Vec(1,2,3),yaw,"","world",actual),"");
  EXPECT_LT((actual-Vec(1,2,3)).norm(),1e-12);
}
TEST(NavigationRecovery, InvalidChildVelocityCannotAdmitRecovery) {
  Vec actual;auto x=context();const auto valid=Eigen::Quaterniond::Identity();
  x.velocity_error=parentVelocity(Vec(1,0,0),valid,"","child",actual);
  EXPECT_EQ(admission(config(),x),"recovery_child_frame_missing");
  for(const auto& q:{Eigen::Quaterniond(0,0,0,0),Eigen::Quaterniond(2,0,0,0),
                    Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(),0,0,0)}) {
    for(const auto* frame:{"child","world"}) {
      x.velocity_error=parentVelocity(Vec(1,0,0),q,"base_link",frame,actual);
      EXPECT_EQ(admission(config(),x),"recovery_orientation_invalid");
      EXPECT_FALSE(actual.allFinite());
    }
  }
  x.velocity_error=parentVelocity(Vec(std::numeric_limits<double>::quiet_NaN(),0,0),
      valid,"base_link","child",actual);
  EXPECT_EQ(admission(config(),x),"recovery_velocity_nonfinite");
  EXPECT_FALSE(actual.allFinite());
}
int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS(); }
