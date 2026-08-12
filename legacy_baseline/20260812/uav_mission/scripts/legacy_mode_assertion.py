#!/usr/bin/env python3
"""Verify default legacy mode still owns and publishes its old route."""

import json
import math
import os
import sys
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped

from patrol_control.msg import MissionCommand


class LegacyModeAssertion:
    def __init__(self):
        rospy.init_node("legacy_mode_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 600.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._expected = rospy.get_param(
            "~expected_first_goal", [-0.602, -1.041, 1.5])
        self._goals = []
        self._external_commands = 0
        self._master = rosgraph.Master(rospy.get_name())
        rospy.Subscriber("/fastplanner/goal", PoseStamped,
                         self._on_goal, queue_size=4)
        rospy.Subscriber("/mission/command", MissionCommand,
                         self._on_command, queue_size=2)

    def _on_goal(self, msg):
        self._goals.append([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ])

    def _on_command(self, _msg):
        self._external_commands += 1

    def _publishers(self):
        for topic, nodes in self._master.getSystemState()[0]:
            if topic == "/fastplanner/goal":
                return sorted(nodes)
        return []

    def _write(self, status, reason, publishers):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        payload = {
            "gate": "legacy_mode_regression",
            "status": status,
            "reason": reason,
            "goal_publishers": publishers,
            "external_commands": self._external_commands,
            "goals": self._goals,
        }
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            publishers = self._publishers()
            if publishers and publishers != ["/patrol_control"]:
                self._write("FAIL", "unexpected_goal_publishers", publishers)
                return 1
            for goal in self._goals:
                error = math.sqrt(sum(
                    (goal[index] - self._expected[index]) ** 2
                    for index in range(3)))
                if error <= 0.05:
                    if self._external_commands:
                        self._write("FAIL", "external_command_in_legacy_mode",
                                    publishers)
                        return 1
                    self._write("PASS", "legacy_first_goal_unchanged",
                                publishers)
                    rospy.loginfo("[LegacyModeGate] PASS")
                    return 0
            if time.monotonic() >= self._deadline:
                self._write("FAIL", "wall_timeout", publishers)
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(LegacyModeAssertion().run())
