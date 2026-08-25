#!/usr/bin/env python3
"""导航组搜索 manager 与新视觉三投闭环的运行时断言。"""

import json
import os
import sys
import time

import rosgraph
import rospy
from std_msgs.msg import String


class TargetSearchDeliveryAssertion:
    def __init__(self):
        rospy.init_node("target_search_delivery_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 1800.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._terminal = None
        self._goal_publishers = set()
        self._raw_goal_publishers = set()
        self._nodes_seen = set()
        self._master = rosgraph.Master(rospy.get_name())
        rospy.Subscriber("/mission/target_search_status", String,
                         self._on_status, queue_size=20)

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if payload.get("status") in ("PASS", "FAIL"):
            self._terminal = payload

    def _sample_graph(self):
        try:
            publishers, _subscribers, _services = self._master.getSystemState()
        except Exception as exc:  # ROS master瞬时不可用时继续等待下一次采样。
            rospy.logwarn_throttle(5.0, "graph sample failed: %s", exc)
            return
        for topic, nodes in publishers:
            self._nodes_seen.update(nodes)
            if topic == "/fastplanner/goal":
                self._goal_publishers.update(nodes)
            elif topic == "/navigation/goal_raw":
                self._raw_goal_publishers.update(nodes)

    def _evaluate(self):
        payload = self._terminal or {}
        delivered = payload.get("delivered") or []
        release_results = payload.get("release_results") or []
        slots = [item.get("slot") for item in delivered]
        delivered_ids = [item.get("id") for item in delivered]
        command_sequence = payload.get("command_sequence") or []
        checks = {
            "adapter_pass": payload.get("status") == "PASS",
            "navigation_route_source": (
                "liftrace-controlwork@5144aa8" in
                payload.get("route_source", "")),
            "three_unique_deliveries": (
                len(delivered) == 3 and len(set(delivered_ids)) == 3),
            "sequential_payload_slots": slots == [1, 2, 3],
            "three_positive_release_results": (
                len([item for item in release_results
                     if item.get("success")]) == 3),
            "full_command_flow": (
                command_sequence.count(1) >= 3 and
                command_sequence.count(2) >= 3 and
                command_sequence.count(3) >= 3 and
                4 in command_sequence and 5 in command_sequence),
            "navigation_manager_owned_raw_goal": (
                "/target_search_manager_py" in self._raw_goal_publishers),
            "adapter_owned_planner_goal": (
                "/navigation_visual_delivery_adapter" in
                self._goal_publishers),
            "navigation_source_unmodified": (
                payload.get("navigation_source_modified") is False),
            "temporary_coverage_manager_absent": (
                "/coverage_search_manager" not in self._nodes_seen and
                payload.get("temporary_coverage_manager_active") is False),
            "inside_field_bounds": payload.get("boundary_violations") == 0,
        }
        passed = all(checks.values())
        return {
            "gate": "navigation_search_visual_delivery",
            "status": "PASS" if passed else "FAIL",
            "reason": "all_assertions_passed" if passed
                      else "assertion_failed",
            "checks": checks,
            "manager": payload,
            "goal_publishers_seen": sorted(self._goal_publishers),
            "raw_goal_publishers_seen": sorted(self._raw_goal_publishers),
            "nodes_seen": sorted(self._nodes_seen),
        }

    def _write(self, report):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        with open(self._report_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._sample_graph()
            if self._terminal is not None:
                report = self._evaluate()
                self._write(report)
                rospy.loginfo("[TargetSearchGate] %s", report["status"])
                return 0 if report["status"] == "PASS" else 1
            if time.monotonic() >= self._deadline:
                report = {
                    "gate": "navigation_search_visual_delivery",
                    "status": "FAIL",
                    "reason": "wall_timeout_waiting_for_manager",
                }
                self._write(report)
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(TargetSearchDeliveryAssertion().run())
