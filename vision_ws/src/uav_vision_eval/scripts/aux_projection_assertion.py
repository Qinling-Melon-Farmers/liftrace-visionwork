#!/usr/bin/env python3
"""验证斜下单目/深度投影、辅助候选和零控制输出契约。"""

import json
import math
import os
import time

import rosgraph
import rospy

from uav_vision.msg import TargetCandidateArray, TargetDetectionArray
from uav_vision_eval.msg import AuxProjectionArray


FORBIDDEN_TOPICS = {
    "/fastplanner/goal",
    "/Servo",
    "/legacy/Servo_raw",
    "/mission/release_permission",
    "/uav_vision/selected_target",
    "/uav_vision/drop_ready",
    "/uav_vision/release_evidence",
    "/detect/waypoint_mark_point",
    "/detect/land_mark_point",
}
AUX_NODE_PREFIXES = (
    "/aux_", "/mock_aux_", "/oblique_aux_",
)


class Assertion:
    def __init__(self):
        self._mono_ok = 0
        self._depth_ok = 0
        self._candidate_ok = 0
        self._depth_sources = set()
        self._start = time.monotonic()
        self._timeout = float(rospy.get_param("~timeout_sec", 15.0))
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
        rospy.Subscriber("/mock/aux/mono_mapped", TargetDetectionArray,
                         self._on_mono, queue_size=1)
        rospy.Subscriber("/mock/aux/depth_mapped", TargetDetectionArray,
                         self._on_depth, queue_size=1)
        rospy.Subscriber("/mock/aux/depth_diagnostics", AuxProjectionArray,
                         self._on_depth_diagnostics, queue_size=1)
        rospy.Subscriber("/uav_vision/aux/targets", TargetCandidateArray,
                         self._on_candidates, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._check)

    @staticmethod
    def _points_ok(message):
        if len(message.detections) != 2:
            return False
        first, second = message.detections
        return (
            first.map_valid and second.map_valid and
            math.hypot(first.map_point.x, first.map_point.y) < 0.03 and
            abs(second.map_point.x - 1.2) < 0.05 and
            abs(second.map_point.y) < 0.03)

    def _on_mono(self, message):
        if self._points_ok(message):
            self._mono_ok += 1

    def _on_depth(self, message):
        if self._points_ok(message):
            self._depth_ok += 1

    def _on_depth_diagnostics(self, message):
        for observation in message.observations:
            if observation.valid:
                self._depth_sources.add(observation.range_source)

    def _on_candidates(self, message):
        if any(target.state >= 2 and target.map_valid for target in message.targets):
            self._candidate_ok += 1

    @staticmethod
    def _aux_forbidden_publishers():
        publishers, _subscribers, _services = rosgraph.Master(
            "/aux_projection_assertion").getSystemState()
        violations = []
        for topic, nodes in publishers:
            if topic not in FORBIDDEN_TOPICS:
                continue
            for node in nodes:
                if node.startswith(AUX_NODE_PREFIXES):
                    violations.append({"topic": topic, "node": node})
        return violations

    def _write(self, status, reason, violations=None):
        payload = {
            "gate": "oblique_aux_projection_l0",
            "status": status,
            "reason": reason,
            "mono_messages": self._mono_ok,
            "depth_messages": self._depth_ok,
            "confirmed_candidate_messages": self._candidate_ok,
            "depth_sources": sorted(self._depth_sources),
            "forbidden_aux_publishers": violations or [],
        }
        directory = os.path.dirname(os.path.abspath(self._report_path))
        os.makedirs(directory, exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self._report_path)

    def _check(self, _event):
        if (self._mono_ok >= 3 and self._depth_ok >= 3 and
                self._candidate_ok >= 2 and
                {"depth_roi", "mono_fallback"}.issubset(self._depth_sources)):
            violations = self._aux_forbidden_publishers()
            if violations:
                self._write("FAIL", "auxiliary_node_publishes_forbidden_topic",
                            violations)
                rospy.signal_shutdown("auxiliary topic contract failed")
                return
            self._write("PASS", "projection_memory_and_topic_contract_passed")
            rospy.loginfo("Oblique auxiliary projection L0 PASS")
            rospy.signal_shutdown("assertion passed")
            return
        if time.monotonic() - self._start > self._timeout:
            self._write("FAIL", "assertion_timeout")
            rospy.logerr(
                "Oblique auxiliary projection L0 timeout mono=%d depth=%d candidate=%d sources=%s",
                self._mono_ok, self._depth_ok, self._candidate_ok,
                sorted(self._depth_sources))
            rospy.signal_shutdown("assertion timeout")


if __name__ == "__main__":
    rospy.init_node("aux_projection_assertion")
    Assertion()
    rospy.spin()
