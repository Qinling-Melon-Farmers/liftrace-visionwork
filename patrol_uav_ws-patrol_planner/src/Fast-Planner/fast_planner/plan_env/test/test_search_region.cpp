#include <gtest/gtest.h>
#include <plan_env/search_region.h>
#include <limits>
TEST(SearchRegion, Seed32RecoveryAndPermittedTargetShareEnvelopeRegion) {
  const auto b = fast_planner::envelopeSearchBounds(
      Eigen::Vector4d(-.5,7.4,-4.8,4.8), .55, 10.);
  EXPECT_GT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.070152,-4.405657,1.091148),b), .025);
  EXPECT_GT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.060856,-4.451313,1.38),b), .025);
  EXPECT_GT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.076097,-4.437315,1.097692),b), .025);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(7.2,0,3),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(1,-4.6,1),b),0.);
}
TEST(SearchRegion, EnvelopePreservesBodyAndSeparateTrackingReserve) {
  const auto b=fast_planner::envelopeSearchBounds(Eigen::Vector4d(-.5,7.4,-4.8,4.8),.55,10.);
  const double reserve=.03;
  EXPECT_NEAR(b[2],-4.481424619063237,1e-12);
  EXPECT_NEAR(fast_planner::searchRegionDistance(Eigen::Vector3d(1,b[2]+reserve,1),b),reserve,1e-12);
  const auto straight=fast_planner::envelopeSearchBounds(Eigen::Vector4d(-2,2,-2,2),.55,0.);
  EXPECT_DOUBLE_EQ(straight[0],-1.725);
}
TEST(SearchRegion, InvalidSharedGeometryRejected) {
  EXPECT_THROW(fast_planner::envelopeSearchBounds(Eigen::Vector4d(0,.1,0,.1),.55,10.),std::invalid_argument);
  EXPECT_THROW(fast_planner::envelopeSearchBounds(Eigen::Vector4d(-2,2,-2,2),.55,46.),std::invalid_argument);
  EXPECT_THROW(fast_planner::envelopeSearchBounds(Eigen::Vector4d(-2,2,-2,2),std::numeric_limits<double>::quiet_NaN(),0.),std::invalid_argument);
}
TEST(SearchRegion, InsideAndAllOutsideFaces) {
  Eigen::Vector4d b; b << -.1,7.,-4.4,4.4;
  EXPECT_DOUBLE_EQ(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,0.,3.),b),1.1);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(7.1,0.,3.),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(-.2,0.,3.),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,4.5,.5),b),0.);
  EXPECT_LT(fast_planner::searchRegionDistance(Eigen::Vector3d(1.,-4.5,.5),b),0.);
  EXPECT_DOUBLE_EQ(fast_planner::searchRegionDistance(Eigen::Vector3d(7.,0.,20.),b),0.);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
