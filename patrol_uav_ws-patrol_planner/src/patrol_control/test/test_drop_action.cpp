#include <gtest/gtest.h>

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

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
