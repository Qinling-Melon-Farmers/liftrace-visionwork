#!/usr/bin/env python3
"""Simulation-only fresh release evidence for the legacy fixed-drop Gate."""

import rospy

from uav_mission.msg import ReleaseResult
from uav_vision.msg import ReleaseEvidence


class FixedDropEvidenceMock:
    def __init__(self):
        rospy.init_node("fixed_drop_evidence_mock")
        self._target_id = int(rospy.get_param("~first_target_id", 1001))
        self._completed_slots = set()
        self._evidence_pub = rospy.Publisher(
            "/uav_vision/release_evidence", ReleaseEvidence, queue_size=1)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=8)
        rate = float(rospy.get_param("~publish_rate", 20.0))
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(rate, 1.0)), self._publish)
        rospy.logwarn(
            "[FixedDropEvidenceMock] simulation-only evidence enabled")

    def _on_result(self, msg):
        slot = int(msg.payload_slot)
        if not msg.success or slot in self._completed_slots:
            return
        self._completed_slots.add(slot)
        self._target_id += 1

    def _publish(self, _event):
        msg = ReleaseEvidence()
        msg.header.stamp = rospy.Time.now()
        msg.align_mode = "drop_circle"
        msg.target_present = True
        msg.target_id = self._target_id
        msg.target_class = "circle"
        msg.target_confirmed = True
        msg.geometry_verified = True
        msg.center_refined = True
        msg.observation_fresh = True
        msg.observation_age_sec = 0.0
        msg.aligned = True
        msg.stable_frames = 10
        msg.evidence_valid = True
        self._evidence_pub.publish(msg)


if __name__ == "__main__":
    FixedDropEvidenceMock()
    rospy.spin()
