#!/usr/bin/env python3
"""验证中心精修、地图投影和视觉对准结果。"""
import rospy
from uav_vision.msg import (
    DropReady, ReleaseEvidence, TargetCandidateArray, TargetDetectionArray,
)


class MapAssertion:
    def __init__(self):
        rospy.init_node("phase_d_map_assertion")
        self.exit_code = 1
        self._timeout = float(rospy.get_param("~timeout", 8.0))
        self._target_class = rospy.get_param("~target_class", "panzer")
        self._mapped_ok = False
        self._invalid_contract_ok = False
        self._target_ok = False
        self._ready_ok = False
        self._evidence_ok = False
        self._seen_ids = set()
        rospy.Subscriber("/uav_vision/detections_mapped",
                         TargetDetectionArray, self._on_mapped, queue_size=4)
        rospy.Subscriber("/uav_vision/targets",
                         TargetCandidateArray, self._on_targets, queue_size=4)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady,
                         self._on_ready, queue_size=4)
        rospy.Subscriber("/uav_vision/release_evidence", ReleaseEvidence,
                         self._on_evidence, queue_size=4)
        rospy.Timer(rospy.Duration(self._timeout), self._on_timeout, oneshot=True)

    def _on_mapped(self, msg):
        standard = [d for d in msg.detections
                    if d.class_name == self._target_class]
        circles = [d for d in msg.detections if d.class_name == "circle"]
        if standard and circles and standard[0].center_refined and \
                standard[0].map_valid and circles[0].map_valid:
            self._mapped_ok = (
                standard[0].center_source == "circle_geometry" and
                standard[0].association_valid and
                standard[0].reject_reason == "" and
                standard[0].transform_age_sec >= 0.0)
            self._invalid_contract_ok = any(
                not detection.map_valid and
                detection.reject_reason == "circle_association_missing" and
                detection.transform_age_sec < 0.0
                for detection in msg.detections)
            self._finish_if_ready()

    def _on_targets(self, msg):
        standard = [t for t in msg.targets if t.class_name == self._target_class]
        circles = [t for t in msg.targets if t.class_name == "circle"]
        if standard and circles and standard[0].map_valid and \
                standard[0].center_refined and circles[0].map_valid:
            self._seen_ids.add(standard[0].id)
            self._target_ok = len(self._seen_ids) == 1
            self._finish_if_ready()

    def _on_ready(self, msg):
        if msg.ready:
            self._ready_ok = True
            self._finish_if_ready()

    def _on_evidence(self, msg):
        if (msg.evidence_valid and msg.observation_fresh and msg.aligned and
                msg.target_confirmed and msg.geometry_verified):
            self._evidence_ok = True
            self._finish_if_ready()

    def _finish_if_ready(self):
        if self._mapped_ok and self._invalid_contract_ok and self._target_ok and \
                self._ready_ok and self._evidence_ok:
            rospy.loginfo("[PhaseDMapAssertion] success")
            self.exit_code = 0
            rospy.signal_shutdown("map visual regression passed")

    def _on_timeout(self, _event):
        if not (self._mapped_ok and self._invalid_contract_ok and
                self._target_ok and self._ready_ok and self._evidence_ok):
            rospy.logerr("[PhaseDMapAssertion] failed mapped=%s invalid_contract=%s "
                         "target=%s ready=%s evidence=%s ids=%s",
                         self._mapped_ok, self._invalid_contract_ok, self._target_ok,
                         self._ready_ok, self._evidence_ok, sorted(self._seen_ids))
            self.exit_code = 6
            rospy.signal_shutdown("map visual regression failed")


if __name__ == "__main__":
    import sys
    assertion = MapAssertion()
    rospy.spin()
    sys.exit(assertion.exit_code)
