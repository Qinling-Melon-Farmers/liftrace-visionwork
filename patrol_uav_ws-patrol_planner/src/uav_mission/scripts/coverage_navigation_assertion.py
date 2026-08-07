#!/usr/bin/env python3
"""Gate the navigation-only full-field coverage SITL."""

import json
import math
import os
import sys
import time

import rosgraph
import rospy
from std_msgs.msg import String, UInt8

from uav_mission.msg import ReleaseResult


class CoverageNavigationAssertion:
    def __init__(self):
        rospy.init_node("coverage_navigation_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 2400.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._gate_name = rospy.get_param(
            "~gate_name", "coverage_navigation")
        self._expected_route_total = int(
            rospy.get_param("~expected_route_total", 20))
        self._min_visited = int(rospy.get_param(
            "~min_visited", math.ceil(0.8 * self._expected_route_total)))
        self._require_north = bool(rospy.get_param("~require_north", True))
        self._required_discovered_classes = set(
            rospy.get_param("~required_discovered_classes", []))
        self._status = None
        self._raw_calls = []
        self._release_results = []
        self._goal_publishers = set()
        self._unexpected_publishers = set()
        self._max_publishers = 0
        self._master = rosgraph.Master(rospy.get_name())
        rospy.Subscriber("/mission/coverage_status", String,
                         self._on_status, queue_size=2)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=4)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release, queue_size=4)
        self._write("RUNNING", "waiting")

    def _on_status(self, msg):
        try:
            self._status = json.loads(msg.data)
        except (TypeError, ValueError):
            self._status = {"status": "FAIL", "reason": "invalid_status_json"}

    def _on_raw_call(self, msg):
        self._raw_calls.append(int(msg.data))

    def _on_release(self, msg):
        self._release_results.append({
            "slot": int(msg.payload_slot),
            "success": bool(msg.success),
            "reason": msg.reason,
        })

    def _sample_publishers(self):
        for topic, nodes in self._master.getSystemState()[0]:
            if topic == "/fastplanner/goal":
                self._max_publishers = max(self._max_publishers, len(nodes))
                self._goal_publishers.update(nodes)
                self._unexpected_publishers.update(
                    name for name in nodes
                    if name != "/coverage_search_manager")
                return

    def _write(self, status, reason):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        payload = {
            "gate": self._gate_name,
            "status": status,
            "reason": reason,
            "manager": self._status,
            "raw_calls": self._raw_calls,
            "release_results": self._release_results,
            "goal_publishers": sorted(self._goal_publishers),
            "unexpected_goal_publishers": sorted(self._unexpected_publishers),
            "max_goal_publishers": self._max_publishers,
        }
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self._sample_publishers()
            if self._unexpected_publishers or self._max_publishers > 1:
                self._write("FAIL", "planner_goal_not_exclusive")
                return 1
            if self._raw_calls or self._release_results:
                self._write("FAIL", "release_occurred_in_navigation_only_gate")
                return 1
            if self._status is not None and self._status.get("status") in (
                    "PASS", "FAIL"):
                if self._status.get("status") != "PASS":
                    self._write("FAIL", "manager_%s" %
                                self._status.get("reason", "failed"))
                    return 1
                checks = [
                    self._status.get("route_total") ==
                    self._expected_route_total,
                    self._status.get("safety_margin") == 0.5,
                    self._status.get("effective_endpoint_margin", 0.0) >= 0.5,
                    self._status.get("route_selected") is True,
                    self._status.get("route_entry") is not None,
                    self._status.get("occupancy_updates", 0) > 0,
                    self._status.get("map_ready_at_start") is True,
                    len(self._status.get("visited", [])) >= self._min_visited,
                    (len(self._status.get("visited", [])) +
                     len(self._status.get("skipped", [])) ==
                     self._expected_route_total),
                    (bool(self._status.get("north_visited")) or
                     not self._require_north),
                    self._status.get("collision_count") == 0,
                    self._status.get("boundary_violations") == 0,
                    self._status.get("navigation_only") is True,
                    self._status.get("execute_candidates") is False,
                    "/coverage_search_manager" in self._goal_publishers,
                ]
                discovered = self._status.get("discovered", [])
                discovered_classes = {
                    item.get("class") for item in discovered}
                discovered_ids = {
                    item.get("id") for item in discovered}
                checks.extend([
                    self._required_discovered_classes.issubset(
                        discovered_classes),
                    len(discovered_ids) ==
                    len(self._required_discovered_classes),
                ])
                if not all(checks):
                    self._write("FAIL", "coverage_contract_failed")
                    return 1
                self._write("PASS", "full_route_return_complete")
                rospy.loginfo("[CoverageNavigationGate] PASS")
                return 0
            if time.monotonic() >= self._deadline:
                self._write("FAIL", "wall_timeout")
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(CoverageNavigationAssertion().run())
