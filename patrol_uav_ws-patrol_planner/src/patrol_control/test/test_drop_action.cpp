#include <gtest/gtest.h>

#include <array>

#include "patrol_control/drop_action.h"

namespace {

TEST(DropActionTest, RejectsInvalidServoSlot) {
  EXPECT_EQ(patrol_control::classifyDropAction(0, true, true),
            patrol_control::DropActionResult::kInvalidServoId);
  EXPECT_EQ(patrol_control::classifyDropAction(4, true, true),
            patrol_control::DropActionResult::kInvalidServoId);
}

TEST(DropActionTest, ReportsServiceFailure) {
  EXPECT_EQ(patrol_control::classifyDropAction(1, false, false),
            patrol_control::DropActionResult::kServiceCallFailed);
}

TEST(DropActionTest, ReportsServoRejection) {
  EXPECT_EQ(patrol_control::classifyDropAction(2, true, false),
            patrol_control::DropActionResult::kRejected);
}

TEST(DropActionTest, AcceptsOnlyPositiveServoAck) {
  EXPECT_EQ(patrol_control::classifyDropAction(3, true, true),
            patrol_control::DropActionResult::kSuccess);
}

TEST(DropActionTest, LegacyModeDoesNotRequireVisionPermission) {
  const patrol_control::DropReleaseGate gate{};
  EXPECT_TRUE(patrol_control::canRequestDrop(false, gate));
}

TEST(DropActionTest, VisionModeRequiresFreshMissionPermission) {
  patrol_control::DropReleaseGate gate{};
  EXPECT_FALSE(patrol_control::canRequestDrop(true, gate));
  gate.mission_permission_active = true;
  EXPECT_FALSE(patrol_control::canRequestDrop(true, gate));
  gate.mission_permission_fresh = true;
  EXPECT_TRUE(patrol_control::canRequestDrop(true, gate));
}

TEST(DropActionTest, ProjectsPixelOffsetThroughConfiguredCameraAxes) {
  const std::array<double, 4> downward_camera_axes{{-1.0, 0.0, 0.0, 1.0}};
  const std::array<double, 2> body_offset =
      patrol_control::projectPixelOffsetToBody(
          100.0, 50.0, 0.0015, downward_camera_axes);
  EXPECT_NEAR(body_offset[0], -0.15, 1e-9);
  EXPECT_NEAR(body_offset[1], 0.075, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
