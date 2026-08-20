#!/usr/bin/env python3
"""验证 Provider 双来源归一化、拒绝原因与零控制输出契约。"""

import json
import os
import time

import rosgraph
import rospy

from uav_vision_eval.msg import AuxProposalArray


FORBIDDEN_TOPICS = {
    "/fastplanner/goal",
    "/Servo",
    "/legacy/Servo_raw",
    "/mission/command",
    "/mission/release_permission",
    "/uav_vision/selected_target",
    "/uav_vision/drop_ready",
    "/uav_vision/release_evidence",
    "/detect/waypoint_mark_point",
    "/detect/land_mark_point",
}


class Assertion:
    def __init__(self):
        self._seen_valid = set()
        self._seen_reasons = set()
        self._messages = 0
        self._started = time.monotonic()
        self._timeout = float(rospy.get_param("~timeout_sec", 10.0))
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
        rospy.Subscriber(
            "/uav_vision/aux/proposals", AuxProposalArray,
            self._on_proposals, queue_size=2)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._check)

    def _on_proposals(self, message):
        self._messages += 1
        for proposal in message.proposals:
            if proposal.valid:
                self._seen_valid.add((proposal.source, proposal.class_hint))
            else:
                self._seen_reasons.add(proposal.reject_reason)

    @staticmethod
    def _forbidden_publishers():
        publishers, _subscribers, _services = rosgraph.Master(
            "/aux_proposal_assertion").getSystemState()
        violations = []
        for topic, nodes in publishers:
            if topic not in FORBIDDEN_TOPICS:
                continue
            for node in nodes:
                if node == "/aux_proposal_provider":
                    violations.append({"topic": topic, "node": node})
        return violations

    def _write(self, status, reason, violations=None):
        payload = {
            "gate": "aux_proposal_provider_l0",
            "status": status,
            "reason": reason,
            "proposal_messages": self._messages,
            "valid_source_classes": [
                list(item) for item in sorted(self._seen_valid)],
            "reject_reasons": sorted(self._seen_reasons),
            "forbidden_provider_publishers": violations or [],
        }
        directory = os.path.dirname(os.path.abspath(self._report_path))
        os.makedirs(directory, exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self._report_path)

    def _check(self, _event):
        expected = {("AUX_CV", "circle"), ("AUX_YOLO", "red_cross")}
        if expected.issubset(self._seen_valid) and \
                "confidence_below_threshold" in self._seen_reasons:
            violations = self._forbidden_publishers()
            if violations:
                self._write("FAIL", "provider_publishes_forbidden_topic",
                            violations)
            else:
                self._write(
                    "PASS", "dual_source_normalization_and_contract_passed")
            rospy.signal_shutdown("assertion complete")
            return
        if time.monotonic() - self._started > self._timeout:
            self._write("FAIL", "assertion_timeout")
            rospy.signal_shutdown("assertion timeout")


if __name__ == "__main__":
    rospy.init_node("aux_proposal_assertion")
    Assertion()
    rospy.spin()
