#!/usr/bin/env python3
"""验证缺少 TF 的精修观测绝不会升级为候选。"""
import sys

import rospy

from uav_vision.msg import TargetCandidateArray, TargetDetectionArray


class MapRejectionAssertion:
    def __init__(self):
        rospy.init_node("map_rejection_assertion")
        self._invalid_frames = 0
        self._candidate_violation = False
        self._targets_seen = False
        rospy.Subscriber(
            "/uav_vision/detections_mapped", TargetDetectionArray,
            self._on_mapped, queue_size=8)
        rospy.Subscriber(
            "/uav_vision/targets", TargetCandidateArray,
            self._on_targets, queue_size=8)
        rospy.Timer(rospy.Duration(6.0), self._timeout, oneshot=True)

    def _on_mapped(self, message):
        standards = [d for d in message.detections if d.class_name == "panzer"]
        if (standards and standards[0].center_refined and
                standards[0].association_valid and
                not standards[0].map_valid and
                standards[0].reject_reason == "tf_unavailable" and
                standards[0].transform_age_sec < 0.0):
            self._invalid_frames += 1
        self._finish_if_ready()

    def _on_targets(self, message):
        self._targets_seen = True
        if any(target.class_name == "panzer" for target in message.targets):
            self._candidate_violation = True
        self._finish_if_ready()

    def _finish_if_ready(self):
        if (self._invalid_frames >= 3 and self._targets_seen and
                not self._candidate_violation):
            rospy.loginfo("V-CL invalid TF rejection PASS")
            rospy.signal_shutdown("invalid TF rejected")

    def _timeout(self, _event):
        rospy.logerr(
            "V-CL invalid TF rejection FAIL invalid_frames=%d targets_seen=%s "
            "candidate_violation=%s",
            self._invalid_frames, self._targets_seen,
            self._candidate_violation)
        rospy.signal_shutdown("invalid TF rejection failed")
        self._failed = True


if __name__ == "__main__":
    assertion = MapRejectionAssertion()
    assertion._failed = False
    rospy.spin()
    sys.exit(8 if assertion._failed or assertion._candidate_violation else 0)
