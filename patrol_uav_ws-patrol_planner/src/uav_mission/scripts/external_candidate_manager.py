#!/usr/bin/env python3
"""Drive one visual candidate through approach, alignment, release and resume."""

import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String

from patrol_control.msg import MissionCommand
from uav_mission.msg import ReleaseResult
from uav_vision.msg import TargetCandidateArray


WEIGHTS = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "tank": 5.0,
}


class ExternalCandidateManager:
    def __init__(self):
        rospy.init_node("external_candidate_manager")
        self._frame = rospy.get_param("~mission_frame", "camera_init")
        self._approach_height = float(rospy.get_param("~approach_height", 1.5))
        self._approach_radius = float(rospy.get_param("~approach_radius", 0.30))
        self._takeoff_height = float(rospy.get_param("~takeoff_height", 1.7))
        self._resume_radius = float(rospy.get_param("~resume_radius", 0.35))
        self._wall_deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 900.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "external_candidate_status.json"))

        self._state = "WAIT_TAKEOFF"
        self._pose = None
        self._control_state = None
        self._candidate = None
        self._release = None
        self._goal_publish_count = 0
        self._command_sequence = []
        self._started_at = time.monotonic()
        self._takeoff_ready_since = None
        self._approach_started_at = None
        self._last_goal_publish_at = None
        self._goal_attempts = 0
        self._minimum_approach_distance = None
        self._planner_command = None
        self._planner_command_count = 0

        self._goal_pub = rospy.Publisher(
            "/fastplanner/goal", PoseStamped, queue_size=1)
        self._command_pub = rospy.Publisher(
            "/mission/command", MissionCommand, queue_size=4)
        self._state_pub = rospy.Publisher(
            "/mission/external_candidate_state", String,
            queue_size=1, latch=True)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber("/detect/point_class", Int8,
                         self._on_control_state, queue_size=2)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=2)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release, queue_size=4)
        rospy.Subscriber("/fastplanner/setpoint_position/local", PoseStamped,
                         self._on_planner_command, queue_size=1)
        self._write_report("RUNNING", "waiting_for_takeoff")

    @staticmethod
    def _fresh(candidate):
        if candidate.last_seen.to_sec() <= 0.0:
            return False
        age = max(0.0, (rospy.Time.now() - candidate.last_seen).to_sec())
        return age <= 0.5

    def _valid(self, candidate):
        return (
            candidate.class_name in WEIGHTS and
            candidate.state >= 2 and
            candidate.map_valid and
            candidate.map_frame == self._frame and
            candidate.association_valid and
            not candidate.reject_reason and
            self._fresh(candidate)
        )

    @staticmethod
    def _rank(candidate):
        return (
            WEIGHTS[candidate.class_name],
            float(candidate.class_confidence),
            -candidate.first_seen.to_sec(),
        )

    def _on_pose(self, msg):
        self._pose = msg

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)

    def _on_targets(self, msg):
        if self._candidate is not None or self._state != "SEARCH":
            return
        candidates = [candidate for candidate in msg.targets
                      if self._valid(candidate)]
        if candidates:
            self._candidate = max(candidates, key=self._rank)
            rospy.loginfo(
                "[ExternalCandidate] selected id=%u class=%s map=(%.2f, %.2f)",
                self._candidate.id, self._candidate.class_name,
                self._candidate.map_point.x, self._candidate.map_point.y)

    def _on_release(self, msg):
        if self._candidate is None or not msg.success:
            return
        self._release = msg
        rospy.loginfo(
            "[ExternalCandidate] guarded ACK slot=%u result_target=%u",
            msg.payload_slot, msg.target_id)

    def _on_planner_command(self, msg):
        self._planner_command = msg
        self._planner_command_count += 1

    def _goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self._frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        return goal

    def _publish_command(self, command, goal, target_id=0, target_class=""):
        msg = MissionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame
        msg.command = command
        msg.target_id = target_id
        msg.target_class = target_class
        msg.goal = goal
        self._command_pub.publish(msg)
        self._command_sequence.append(int(command))

    def _publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self._goal_pub.publish(goal)
        self._goal_publish_count += 1

    def _distance(self, goal):
        if self._pose is None:
            return float("inf")
        p = self._pose.pose.position
        q = goal.pose.position
        return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 +
                         (p.z - q.z) ** 2)

    def _pose_vector(self):
        position = self._pose.pose.position
        return [float(position.x), float(position.y), float(position.z)]

    def _approach_distance(self):
        if self._pose is None or self._candidate is None:
            return None
        position = self._pose.pose.position
        return math.sqrt(
            (position.x - self._candidate.map_point.x) ** 2 +
            (position.y - self._candidate.map_point.y) ** 2 +
            (position.z - self._approach_height) ** 2)

    def _planner_command_vector(self):
        position = self._planner_command.pose.position
        return [float(position.x), float(position.y), float(position.z)]

    def _planner_command_distance(self):
        if self._pose is None or self._planner_command is None:
            return None
        position = self._pose.pose.position
        command = self._planner_command.pose.position
        return math.sqrt(
            (position.x - command.x) ** 2 +
            (position.y - command.y) ** 2 +
            (position.z - command.z) ** 2)

    def _transition(self, state, reason):
        self._state = state
        self._state_pub.publish(String(data=state))
        self._write_report("RUNNING", reason)
        rospy.loginfo("[ExternalCandidate] state=%s reason=%s", state, reason)

    def _write_report(self, status, reason):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        candidate = None
        if self._candidate is not None:
            candidate = {
                "id": int(self._candidate.id),
                "class": self._candidate.class_name,
                "map": [float(self._candidate.map_point.x),
                        float(self._candidate.map_point.y)],
            }
        payload = {
            "gate": "external_candidate",
            "status": status,
            "reason": reason,
            "state": self._state,
            "candidate": candidate,
            "release_slot": (int(self._release.payload_slot)
                             if self._release is not None else None),
            "goal_attempts": self._goal_attempts,
            "command_sequence": self._command_sequence,
            "goal_publish_count": self._goal_publish_count,
            "current_pose": (self._pose_vector()
                             if self._pose is not None else None),
            "approach_distance": self._approach_distance(),
            "minimum_approach_distance": self._minimum_approach_distance,
            "planner_command": (self._planner_command_vector()
                                if self._planner_command is not None else None),
            "planner_command_count": self._planner_command_count,
            "planner_command_distance": self._planner_command_distance(),
            "wall_elapsed": time.monotonic() - self._started_at,
        }
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def run(self):
        rate = rospy.Rate(10)
        approach_goal = None
        resume_goal = self._goal(0.0, 0.0, self._takeoff_height)

        while not rospy.is_shutdown():
            if time.monotonic() >= self._wall_deadline:
                self._write_report("FAIL", "wall_timeout")
                return 1

            if self._state == "WAIT_TAKEOFF":
                takeoff_ready = (
                    self._pose is not None and self._control_state == 1 and
                    self._pose.pose.position.z >= self._takeoff_height)
                if takeoff_ready and self._takeoff_ready_since is None:
                    self._takeoff_ready_since = time.monotonic()
                elif (takeoff_ready and time.monotonic() -
                      self._takeoff_ready_since >= 2.0):
                    self._publish_command(MissionCommand.SEARCH, resume_goal)
                    self._transition("SEARCH", "takeoff_control_stable")
                elif not takeoff_ready:
                    self._takeoff_ready_since = None

            elif self._state == "SEARCH" and self._candidate is not None:
                approach_goal = self._goal(
                    self._candidate.map_point.x,
                    self._candidate.map_point.y,
                    self._approach_height)
                self._publish_goal(approach_goal)
                self._publish_command(
                    MissionCommand.APPROACH, approach_goal,
                    self._candidate.id, self._candidate.class_name)
                self._transition("APPROACH", "candidate_selected")
                self._approach_started_at = rospy.Time.now().to_sec()
                self._last_goal_publish_at = self._approach_started_at
                self._goal_attempts = 1


            elif self._state == "APPROACH":
                now = rospy.Time.now().to_sec()
                distance = self._distance(approach_goal)
                if (self._minimum_approach_distance is None or
                        distance < self._minimum_approach_distance):
                    self._minimum_approach_distance = distance
                if distance <= self._approach_radius:
                    self._publish_command(
                        MissionCommand.ALIGN, approach_goal,
                        self._candidate.id, self._candidate.class_name)
                    self._transition("ALIGN", "approach_reached")
                elif now - self._approach_started_at >= 20.0:
                    self._state = "FAIL"
                    self._state_pub.publish(String(data="FAIL"))
                    self._write_report("FAIL", "approach_unreachable_20s")
                    return 1
                elif (self._goal_attempts < 3 and
                      now - self._last_goal_publish_at >= 5.0):
                    self._publish_goal(approach_goal)
                    self._goal_attempts += 1
                    self._last_goal_publish_at = now
                    self._write_report(
                        "RUNNING", "approach_retry_%d" % self._goal_attempts)
                    rospy.logwarn(
                        "[ExternalCandidate] retry approach goal attempt=%d",
                        self._goal_attempts)

            elif (self._state == "ALIGN" and self._release is not None and
                  self._control_state == 1 and self._pose is not None and
                  self._pose.pose.position.z >= 0.95):
                self._publish_goal(resume_goal)
                self._publish_command(
                    MissionCommand.RESUME, resume_goal,
                    self._candidate.id, self._candidate.class_name)
                self._transition("RESUME", "release_ack_and_climb_complete")

            elif (self._state == "RESUME" and
                  self._distance(resume_goal) <= self._resume_radius):
                self._state = "COMPLETE"
                self._state_pub.publish(String(data="PASS"))
                self._write_report("PASS", "candidate_drop_resume_complete")
                rospy.loginfo("[ExternalCandidate] PASS")
                return 0

            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(ExternalCandidateManager().run())
