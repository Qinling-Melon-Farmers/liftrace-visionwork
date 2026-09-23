#include <gtest/gtest.h>
#include <plan_env/vertical_obstacle_support.h>

using fast_planner::VerticalObstacleSupport;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(VerticalObstacleSupport, IsolatedAndFlatReturnsDoNotMakeColumns) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .516);
  s.observe(11, 10, .518);
  s.observe(10, 11, .515);
  EXPECT_FALSE(s.supported(10, 10));
}

TEST(VerticalObstacleSupport, NearbyVerticalStructureSupportsObstacle) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .8);
  s.observe(11, 10, 1.0);
  s.observe(10, 11, 1.2);
  EXPECT_TRUE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(20, 20));
}

TEST(VerticalObstacleSupport, RemoteOrTooSparsePointsDoNotSupport) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .5);
  s.observe(10, 11, 1.5);
  s.observe(20, 20, 1.0);
  EXPECT_FALSE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(-1, 10));
}

TEST(VerticalObstacleSupport, LegacyParametersRetainOriginalClassification) {
  VerticalObstacleSupport s(30, 30, 0, 1, 0.0);
  s.observe(10, 10, .5);
  EXPECT_TRUE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(11, 10));
}
