#!/usr/bin/env python3
"""Assert the Stage-3 external candidate loop and planner-goal ownership."""

import json
import os
import sys
import time

import rosgraph
import rospy
from std_msgs.msg import String

from patrol_control.msg import MissionCommand
from uav_mission.msg import ReleaseResult


class ExternalCandidateAssertion:
    def __init__(self):
        rospy.init_node("external_candidate_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 900.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._commands = []
        self._release_slots = []
        self._mission_state = "WAITING"
        self._max_goal_publishers = 0
        self._goal_publishers = set()
        self._unexpected_publishers = set()
        self._master = rosgraph.Master(rospy.get_name())

        rospy.Subscriber("/mission/command", MissionCommand,
                         self._on_command, queue_size=8)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release, queue_size=4)
        rospy.Subscriber("/mission/external_candidate_state", String,
                         self._on_state, queue_size=2)
        self._write("RUNNING", "waiting")

    def _on_command(self, msg):
        command = int(msg.command)
        if not self._commands or self._commands[-1] != command:
            self._commands.append(command)

    def _on_release(self, msg):
        if msg.success:
            self._release_slots.append(int(msg.payload_slot))

    def _on_state(self, msg):
        self._mission_state = msg.data

    def _sample_publishers(self):
        try:
            publishers = self._master.getSystemState()[0]
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[ExternalCandidateGate] master: %s", exc)
            return
        names = []
        for topic, nodes in publishers:
            if topic == "/fastplanner/goal":
                names = list(nodes)
                break
        self._max_goal_publishers = max(self._max_goal_publishers, len(names))
        self._goal_publishers.update(names)
        self._unexpected_publishers.update(
            name for name in names if name != "/external_candidate_manager")

    def _write(self, status, reason):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        payload = {
            "gate": "external_candidate",
            "status": status,
            "reason": reason,
            "commands": self._commands,
            "release_slots": self._release_slots,
            "mission_state": self._mission_state,
            "max_goal_publishers": self._max_goal_publishers,
            "goal_publishers": sorted(self._goal_publishers),
            "unexpected_goal_publishers": sorted(self._unexpected_publishers),
        }
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def run(self):
        expected = [
            MissionCommand.SEARCH,
            MissionCommand.APPROACH,
            MissionCommand.ALIGN,
            MissionCommand.RESUME,
        ]
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self._sample_publishers()
            if self._unexpected_publishers or self._max_goal_publishers > 1:
                self._write("FAIL", "planner_goal_not_exclusive")
                return 1
            if self._mission_state == "FAIL":
                self._write("FAIL", "mission_manager_failed")
                return 1
            if self._mission_state == "PASS":
                if self._commands != expected:
                    self._write("FAIL", "command_sequence_%s" % self._commands)
                    return 1
                if self._release_slots != [1]:
                    self._write("FAIL", "release_slots_%s" % self._release_slots)
                    return 1
                if "/external_candidate_manager" not in self._goal_publishers:
                    self._write("FAIL", "manager_goal_publisher_not_observed")
                    return 1
                self._write("PASS", "approach_align_release_resume_complete")
                rospy.loginfo("[ExternalCandidateGate] PASS")
                return 0
            if time.monotonic() >= self._deadline:
                self._write("FAIL", "wall_timeout")
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(ExternalCandidateAssertion().run())
