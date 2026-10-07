#include <gtest/gtest.h>
#include <cmath>
#include <path_searching/line_preference.h>
using Eigen::Vector3d;
TEST(LinePreference, DirectionIndependentAndDisabled) {
  Vector3d start(0,0,2),goal(5,0,2),p(2,0,2),v(1,0,0),a(0,0,0);
  EXPECT_DOUBLE_EQ(0.,fast_planner::lineDeviationCost(p,v,a,1,start,goal,2));
  EXPECT_DOUBLE_EQ(0.,fast_planner::lineDeviationCost(p,-v,a,1,goal,start,2));
  p.y()=1;
  EXPECT_DOUBLE_EQ(0.,fast_planner::lineDeviationCost(p,v,a,1,start,goal,0));
  EXPECT_NEAR(2.,fast_planner::lineDeviationCost(p,v,a,1,start,goal,2),1e-9);
}
TEST(LinePreference, FiniteDetourCostAndDiagonal) {
  Vector3d s(0,0,0),g(2,0,2),a(0,0,0);
  EXPECT_NEAR(0.,fast_planner::lineDeviationCost(s,Vector3d(1,0,1),a,1,s,g,2),1e-9);
  EXPECT_GT(fast_planner::lineDeviationCost(s,Vector3d(1,1,1),a,1,s,g,2),0.);
  EXPECT_TRUE(std::isfinite(fast_planner::lineDeviationCost(s,Vector3d(1,0,0),a,1,s,s,2)));
}

int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
