#include <gtest/gtest.h>
#include <plan_env/local_segment_probe.h>
#include <array>
#include <set>
using namespace fast_planner;

class LocalProbeTest : public ::testing::Test {
 protected:
  LocalCloudView view;
  ProbeLimits limits;
  std::set<std::array<int,3>> occupied;
  void SetUp() override {
    view.valid=true;view.stamp=10.;view.frame="map";view.revision=1;
    view.resolution=1.;view.lower.setZero();view.upper.setConstant(10.);
    limits.max_wall_ms=100.;
  }
  ProbeResult run(std::vector<Segment> segments,double now=10.1) {
    return probeSegments(view,segments,limits,now,[&](const Eigen::Vector3i& id) {
      return occupied.count({id.x(),id.y(),id.z()}) ? 1 : 0;
    });
  }
};

TEST_F(LocalProbeTest, ClearAndOccupiedWithNoMapMutation) {
  occupied.insert({2,1,1});const auto before=occupied;
  auto r=run({{{.5,1.5,1.5},{3.5,1.5,1.5}},{{.5,3.5,1.5},{3.5,3.5,1.5}}});
  ASSERT_TRUE(r.accepted);EXPECT_EQ(r.status[0],OCCUPIED);EXPECT_EQ(r.status[1],CLEAR_IN_INFLATED_MAP);
  EXPECT_EQ(occupied,before);
}
TEST_F(LocalProbeTest, CornerTouchDoesNotTunnel) {
  limits.max_length=6.;occupied.insert({1,0,0});
  auto r=run({{{.5,.5,.5},{3.5,3.5,.5}}});
  ASSERT_TRUE(r.accepted);EXPECT_EQ(r.status[0],OCCUPIED);
}
TEST_F(LocalProbeTest, ParallelBoundaryChecksBothSides) {
  occupied.insert({2,0,0});
  auto r=run({{{.5,1.,.5},{3.5,1.,.5}}});
  EXPECT_EQ(r.status[0],OCCUPIED);
}
TEST_F(LocalProbeTest, StartAndEndFacesIncludedBothDirections) {
  occupied.insert({1,0,0});
  EXPECT_EQ(run({{{2.,.5,.5},{3.5,.5,.5}}}).status[0],OCCUPIED);
  EXPECT_EQ(run({{{3.5,.5,.5},{2.,.5,.5}}}).status[0],OCCUPIED);
}
TEST_F(LocalProbeTest, ZeroLengthAndReverse) {
  EXPECT_EQ(run({{{.5,.5,.5},{.5,.5,.5}}}).status[0],CLEAR_IN_INFLATED_MAP);
  occupied.insert({2,0,0});
  EXPECT_EQ(run({{{3.5,.5,.5},{.5,.5,.5}}}).status[0],OCCUPIED);
}
TEST_F(LocalProbeTest, FreshnessAndUnavailableMapRejectBatch) {
  for (double now : {9.,12.}) EXPECT_FALSE(run({{{.5,.5,.5},{1.5,.5,.5}}},now).accepted);
  view.valid=false;EXPECT_FALSE(run({{{.5,.5,.5},{1.5,.5,.5}}}).accepted);
}
TEST_F(LocalProbeTest, LocalWindowAndLengthLimits) {
  auto r=run({{{.5,.5,.5},{10.5,.5,.5}},{{.5,.5,.5},{8.5,.5,.5}}});
  EXPECT_EQ(r.status[0],OUTSIDE_WINDOW);EXPECT_EQ(r.status[1],TOO_LONG);
}
TEST_F(LocalProbeTest, NoPartialSuccessAfterBudgetExhaustion) {
  limits.max_voxels=1;
  auto r=run({{{.5,.5,.5},{.5,.5,.5}},{{.5,.5,.5},{3.5,.5,.5}}});
  EXPECT_FALSE(r.accepted);EXPECT_EQ(r.reason,"query_budget_exhausted");
  for(auto s:r.status) EXPECT_EQ(s,NOT_CHECKED);
  EXPECT_LE(r.checked_voxels,1u);
}
TEST_F(LocalProbeTest, OversizedAndNonfiniteRejectedBeforeLookup) {
  auto r=run(std::vector<Segment>(33,{{.5,.5,.5},{1.5,.5,.5}}));
  EXPECT_FALSE(r.accepted);EXPECT_EQ(r.checked_voxels,0u);
  r=run({{{NAN,.5,.5},{1.5,.5,.5}}});EXPECT_FALSE(r.accepted);EXPECT_EQ(r.checked_voxels,0u);
}

int main(int argc,char** argv) {
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
