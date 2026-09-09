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

TEST(DropActionTest, ExternalReleaseUsesOnlyFreshMissionPermission) {
  patrol_control::DropReleaseGate gate{};
  EXPECT_FALSE(patrol_control::dropReleaseReady(true, true, false, gate));
  EXPECT_FALSE(patrol_control::dropReleaseReady(true, true, true, gate));
  gate.mission_permission_active = true;
  gate.mission_permission_fresh = true;
  EXPECT_TRUE(patrol_control::dropReleaseReady(true, false, true, gate));
}

TEST(DropActionTest, LegacyReleaseKeepsGeometryAndOptionalPermission) {
  patrol_control::DropReleaseGate gate{};
  EXPECT_FALSE(patrol_control::dropReleaseReady(false, false, false, gate));
  EXPECT_TRUE(patrol_control::dropReleaseReady(false, true, false, gate));
  EXPECT_FALSE(patrol_control::dropReleaseReady(false, true, true, gate));
  gate.mission_permission_active = true;
  gate.mission_permission_fresh = true;
  EXPECT_TRUE(patrol_control::dropReleaseReady(false, true, true, gate));
}

TEST(DropActionTest, LegacyAlignmentKeepsHistoricalTimeout) {
  EXPECT_TRUE(patrol_control::alignmentWindowOpen(false, 29.9, 30.0));
  EXPECT_TRUE(patrol_control::alignmentWindowOpen(false, 30.0, 30.0));
  EXPECT_FALSE(patrol_control::alignmentWindowOpen(false, 30.1, 30.0));
}

TEST(DropActionTest, ExternalAlignmentUsesMissionManagerDeadline) {
  EXPECT_TRUE(patrol_control::alignmentWindowOpen(true, 30.1, 30.0));
  EXPECT_TRUE(patrol_control::alignmentWindowOpen(true, 120.0, 30.0));
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
