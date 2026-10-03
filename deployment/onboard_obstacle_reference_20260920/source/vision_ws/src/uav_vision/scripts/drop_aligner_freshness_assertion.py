#!/usr/bin/env python3
"""回归验证：新鲜圆环必须优先于质量更高但已经过期的圆环。"""

import sys
import time

import rospy
from std_msgs.msg import String

from uav_vision.msg import TargetCandidate, TargetCandidateArray, ReleaseEvidence


CONFIRMED = 2


class FreshnessAssertion:
    def __init__(self):
        self._passed = False
        self._last_evidence = None
        self._mode_pub = rospy.Publisher(
            "/uav_vision/align_mode", String, queue_size=1, latch=True)
        self._targets_pub = rospy.Publisher(
            "/uav_vision/targets", TargetCandidateArray, queue_size=1)
        rospy.Subscriber(
            "/uav_vision/release_evidence", ReleaseEvidence,
            self._on_evidence, queue_size=10)

    def _on_evidence(self, message):
        self._last_evidence = message
        if message.target_id == 2 and message.evidence_valid:
            self._passed = True

    @staticmethod
    def _circle(target_id, now, age_sec, geometry_confidence):
        target = TargetCandidate()
        target.header.stamp = now
        target.id = target_id
        target.class_name = "circle"
        target.class_confidence = 0.95
        target.geometry_confidence = geometry_confidence
        target.center_px.x = 320.0
        target.center_px.y = 240.0
        target.center_px.z = 80.0
        target.center_refined = True
        target.center_source = "circle_geometry"
        target.association_valid = True
        target.state = CONFIRMED
        target.observe_count = 10
        target.consecutive_observe_count = 10
        target.first_seen = now - rospy.Duration(5.0)
        target.last_seen = now - rospy.Duration(age_sec)
        return target

    def run(self):
        deadline = time.monotonic() + 8.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = rospy.Time.now()
            self._mode_pub.publish(String(data="drop_circle"))
            message = TargetCandidateArray()
            message.header.stamp = now
            message.targets = [
                self._circle(1, now, 2.0, 0.99),
                self._circle(2, now, 0.0, 0.80),
            ]
            self._targets_pub.publish(message)
            if self._passed:
                rospy.loginfo(
                    "[DropAlignerFreshness] PASS fresh target selected")
                return 0
            rate.sleep()
        selected = self._last_evidence.target_id if self._last_evidence else None
        reasons = list(self._last_evidence.rejection_reasons) \
            if self._last_evidence else []
        rospy.logerr(
            "[DropAlignerFreshness] FAIL selected=%s reasons=%s",
            selected, reasons)
        return 4


if __name__ == "__main__":
    rospy.init_node("drop_aligner_freshness_assertion")
    sys.exit(FreshnessAssertion().run())
