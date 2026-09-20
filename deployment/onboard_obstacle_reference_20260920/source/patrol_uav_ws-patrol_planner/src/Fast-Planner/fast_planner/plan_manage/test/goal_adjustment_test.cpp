#include <gtest/gtest.h>
#include <plan_manage/goal_adjustment.h>

using fast_planner::nearbyFreeGoal;

TEST(GoalAdjustment, FindsNearbyFreeCellWithoutChangingHeight) {
  const Eigen::Vector3d anchor(.587997, 8.16769, 1.4);
  Eigen::Vector3d selected;
  // The captured upper wall occupies this Y cell after ordinary inflation.
  const auto clearance = [](const Eigen::Vector3d& p) {
    return p.y() < 8.15 ? 8.15 - p.y() : -0.05;
  };
  ASSERT_TRUE(nearbyFreeGoal(anchor, .15, .05, .025, clearance, selected));
  EXPECT_NEAR(selected.x(), anchor.x(), 1e-9);
  EXPECT_NEAR(selected.y(), anchor.y() - .05, 1e-9);
  EXPECT_DOUBLE_EQ(selected.z(), anchor.z());
}

TEST(GoalAdjustment, RepeatedRequestsDoNotAccumulateDisplacement) {
  const Eigen::Vector3d anchor(0, 0, 1.4);
  Eigen::Vector3d selected;
  for (int i = 0; i < 5; ++i) {
    const auto clearance = [i](const Eigen::Vector3d& p) {
      return p.x() >= .08 + .05 * i ? .05 : -1.;
    };
    const bool found = nearbyFreeGoal(anchor, .15, .05, .025, clearance, selected);
    if (i < 2) {
      ASSERT_TRUE(found);
      EXPECT_LE((selected - anchor).norm(), .1500001);
    } else {
      EXPECT_FALSE(found);
    }
  }
}

TEST(GoalAdjustment, DoesNotRelaxMapClearanceOrSelectVerticalShortcut) {
  const Eigen::Vector3d anchor(0, 0, 1.4);
  Eigen::Vector3d selected;
  EXPECT_FALSE(nearbyFreeGoal(anchor, .15, .05, .025,
      [](const Eigen::Vector3d&) { return .01; }, selected));
  EXPECT_FALSE(nearbyFreeGoal(anchor, .15, .05, .025,
      [](const Eigen::Vector3d& p) { return p.z() > 1.5 ? 1. : -1.; }, selected));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
