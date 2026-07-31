#!/usr/bin/env python3
"""Exit successfully after deterministic truth and detection checks pass."""

import math
import os

import rospy
from uav_vision.msg import TargetDetectionArray
from uav_vision_eval.msg import SimTargetArray


class Assertion:
    def __init__(self):
        self.truth_ok = 0
        self.detection_ok = 0
        self.started = rospy.Time.now()
        rospy.Subscriber("/uav_vision_eval/ground_truth", SimTargetArray, self._truth, queue_size=1)
        rospy.Subscriber("/mock/detections", TargetDetectionArray, self._detections, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._check)

    def _truth(self, message):
        targets = [target for target in message.targets if target.target_id == "tent_1"]
        if len(targets) != 1:
            return
        target = targets[0]
        if target.pose_valid and target.projection_valid and target.fully_in_frame:
            error = math.hypot(target.pixel_center.x - 320.0, target.pixel_center.y - 240.0)
            if error < 1.0e-3:
                self.truth_ok += 1

    def _detections(self, message):
        if len(message.detections) == 1 and message.detections[0].class_name == "tent":
            self.detection_ok += 1

    def _check(self, _event):
        if self.truth_ok >= 5 and self.detection_ok >= 5:
            rospy.loginfo("V-SIM-01/03 mock assertion PASS")
            rospy.signal_shutdown("evaluation assertion passed")
            return
        if (rospy.Time.now() - self.started).to_sec() > 15.0:
            rospy.logerr("evaluation assertion timeout: truth=%d detection=%d", self.truth_ok, self.detection_ok)
            os._exit(3)


if __name__ == "__main__":
    rospy.init_node("eval_pipeline_assertion")
    Assertion()
    rospy.spin()
