#!/usr/bin/env python3
"""Bounded headless assertion for the real Gazebo truth/image chain."""

import sys
import time

import rospy
from uav_vision_eval.msg import SimTargetArray


class GazeboTruthAssertion:
    def __init__(self):
        self.expected_target_id = rospy.get_param("~expected_target_id", "")
        self.expect_no_targets = rospy.get_param("~expect_no_targets", False)
        self.require_fully_in_frame = rospy.get_param("~require_fully_in_frame", True)
        self.min_messages = int(rospy.get_param("~min_messages", 5))
        self.good_messages = 0
        self.first_logged = False
        rospy.Subscriber(
            rospy.get_param("~truth_topic", "/uav_vision_eval/ground_truth"),
            SimTargetArray, self._callback, queue_size=1,
        )

    def _callback(self, message):
        if self.expect_no_targets:
            if not message.targets:
                self.good_messages += 1
            return
        matches = [target for target in message.targets if target.target_id == self.expected_target_id]
        if len(matches) != 1:
            return
        target = matches[0]
        if not self.first_logged and target.pose_valid:
            rospy.loginfo(
                "Gazebo truth %s: projection=%s full=%s pixel=(%.3f, %.3f) camera=(%.3f, %.3f, %.3f) distance=%.3f",
                target.target_id, target.projection_valid, target.fully_in_frame,
                target.pixel_center.x, target.pixel_center.y,
                target.camera_center.x, target.camera_center.y, target.camera_center.z,
                target.distance_m,
            )
            self.first_logged = True
        valid = target.pose_valid and target.projection_valid
        if self.require_fully_in_frame:
            valid = valid and target.fully_in_frame
        if valid:
            self.good_messages += 1

    def run(self):
        deadline = time.monotonic() + float(rospy.get_param("~timeout_sec", 30.0))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.good_messages >= self.min_messages:
                rospy.loginfo("V-SIM-01 Gazebo truth assertion PASS (%d messages)", self.good_messages)
                return 0
            time.sleep(0.1)
        rospy.logerr(
            "V-SIM-01 Gazebo truth assertion FAIL: expected=%s no_targets=%s good=%d",
            self.expected_target_id, self.expect_no_targets, self.good_messages,
        )
        return 4


if __name__ == "__main__":
    rospy.init_node("gazebo_truth_assertion")
    sys.exit(GazeboTruthAssertion().run())
