#include <gtest/gtest.h>
#include <plan_env/search_region.h>
TEST(SearchRegion, InsideAndAllOutsideFaces) {
  Eigen::Vector4d b; b << -.1,7.,-4.4,4.4;
  EXPECT_DOUBLE_EQ(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,0.,3.),b),1.1);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(7.1,0.,3.),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(-.2,0.,3.),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,4.5,.5),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,-4.5,.5),b),0.);
  EXPECT_DOUBLE_EQ(fast_planner::searchRegionDistance(Eigen::Vector3d(7.,0.,20.),b),0.);
}
