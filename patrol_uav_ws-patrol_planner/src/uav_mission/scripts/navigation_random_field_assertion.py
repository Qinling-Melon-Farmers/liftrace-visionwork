#!/usr/bin/env python3
"""Hard Gate for the formal navigation-manager random-field chain."""

import json
import math
import os
import sys
import time

import rosgraph
import rospy
from std_msgs.msg import String

from patrol_control.msg import MissionCommand
from uav_vision.msg import TargetCandidate, TargetCandidateArray


class NavigationRandomFieldAssertion:
    def __init__(self):
        rospy.init_node("navigation_random_field_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 1800.0))
        self._profile = rospy.get_param("~class_profile", "r2026")
        self._nav_feature_profile = rospy.get_param(
            "~nav_feature_profile", "baseline")
        self._allowed = tuple(rospy.get_param(
            "~allowed_classes",
            ["tent", "pillbox", "bridge", "panzer", "red_cross"]))
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
        self._run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")
        self._terminal = None
        self._field_status = None
        self._anchor_status = None
        self._selector_status = None
        self._contact_status = None
        self._adapter_accepting_seen = False
        self._selector_accepting_seen = False
        self._confirmed_ids = set()
        self._profile_selected_ids = set()
        self._profile_selected_classes = []
        self._raw_selected_classes = []
        self._approach_ids = set()
        self._accepted_classes = []
        self._nodes_seen = set()
        self._goal_publishers = set()
        self._raw_goal_publishers = set()
        self._selected_publishers = set()
        self._contact_status_publishers = set()
        self._master = rosgraph.Master(rospy.get_name())

        rospy.Subscriber("/mission/target_search_status", String,
                         self._on_manager, queue_size=20)
        rospy.Subscriber("/mission/random_field_status", String,
                         self._on_field, queue_size=2)
        rospy.Subscriber("/mission/planner_anchor_status", String,
                         self._on_anchor, queue_size=2)
        rospy.Subscriber("/mission/profile_selector_status", String,
                         self._on_selector, queue_size=2)
        rospy.Subscriber("/mission/gazebo_contact_status", String,
                         self._on_contact, queue_size=4)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=2)
        rospy.Subscriber("/mission/profile_selected_target", TargetCandidate,
                         self._on_selected, queue_size=10)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate,
                         self._on_raw_selected, queue_size=10)
        rospy.Subscriber("/mission/command", MissionCommand,
                         self._on_command, queue_size=20)

    @staticmethod
    def _decode(message):
        try:
            return json.loads(message.data)
        except (TypeError, ValueError):
            return None

    def _on_manager(self, message):
        payload = self._decode(message)
        if payload is not None and payload.get("candidate_accepting"):
            self._adapter_accepting_seen = True
        if payload is not None and payload.get("status") in ("PASS", "FAIL"):
            self._terminal = payload

    def _on_field(self, message):
        self._field_status = self._decode(message)

    def _on_anchor(self, message):
        self._anchor_status = self._decode(message)

    def _on_selector(self, message):
        self._selector_status = self._decode(message)
        if (self._selector_status is not None and
                self._selector_status.get("publishing_enabled")):
            self._selector_accepting_seen = True

    def _on_contact(self, message):
        self._contact_status = self._decode(message)

    def _strict_confirmed(self, target):
        point = target.map_point
        now = rospy.Time.now().to_sec()
        last_seen = target.last_seen.to_sec()
        age = now - last_seen
        return (
            target.class_name in self._allowed and
            int(target.state) == 2 and
            int(target.consecutive_observe_count) >= 3 and
            target.map_valid and target.map_frame == "camera_init" and
            target.association_valid and not target.reject_reason and
            last_seen > 0.0 and
            0.0 <= age <= 0.5 and
            all(math.isfinite(value) for value in
                (point.x, point.y, point.z, last_seen,
                 target.class_confidence, target.geometry_confidence,
                 target.map_quality))
        )

    def _on_targets(self, message):
        for target in message.targets:
            if self._strict_confirmed(target):
                self._confirmed_ids.add(int(target.id))

    def _on_selected(self, message):
        self._profile_selected_ids.add(int(message.id))
        self._profile_selected_classes.append(message.class_name)

    def _on_raw_selected(self, message):
        self._raw_selected_classes.append(message.class_name)

    def _on_command(self, message):
        if int(message.command) == int(MissionCommand.APPROACH):
            self._approach_ids.add(int(message.target_id))
            self._accepted_classes.append(message.target_class)

    def _sample_graph(self):
        try:
            publishers, _subscribers, _services = self._master.getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "graph sample failed: %s", exc)
            return
        for topic, nodes in publishers:
            self._nodes_seen.update(nodes)
            if topic == "/fastplanner/goal":
                self._goal_publishers.update(nodes)
            elif topic == "/navigation/goal_raw":
                self._raw_goal_publishers.update(nodes)
            elif topic == "/mission/profile_selected_target":
                self._selected_publishers.update(nodes)
            elif topic == "/mission/gazebo_contact_status":
                self._contact_status_publishers.update(nodes)

    @staticmethod
    def _ready(status, profile):
        return bool(status and status.get("ready") and
                    status.get("status") == "READY" and
                    status.get("profile") == profile)

    def _artifacts_present(self):
        required = (
            "random_field_truth.yaml",
            "red_cross_truth.yaml",
            "random_field_status.json",
            "planner_anchor_status.json",
            "gazebo_contact_status.json",
            "target_search_status.json",
        )
        return all(os.path.isfile(os.path.join(self._run_dir, name))
                   for name in required)

    def _evaluate(self):
        manager = self._terminal or {}
        delivered = manager.get("delivered") or []
        delivered_ids = [int(item.get("id", -1)) for item in delivered]
        delivered_classes = [item.get("class") for item in delivered]
        slots = [item.get("slot") for item in delivered]
        command_events = manager.get("command_events") or []
        approach_transitions = {
            int(item.get("target_id")) for item in command_events
            if (item.get("command") == int(MissionCommand.APPROACH) and
                item.get("from_state") == "SEARCH" and
                item.get("to_state") == "APPROACH" and
                item.get("target_id") is not None)
        }
        status_sequences = [
            int(item.get("sequence", -1))
            for item in (manager.get("status_events") or [])]
        max_altitude = manager.get("max_altitude")
        checks = {
            "manager_pass": manager.get("status") == "PASS",
            "mission_ros_within_600_sec": (
                manager.get("mission_elapsed_ros") is not None and
                float(manager.get("mission_elapsed_ros")) <= 600.0 + 1e-6),
            "mission_wall_within_600_sec": (
                manager.get("mission_elapsed_wall") is not None and
                float(manager.get("mission_elapsed_wall")) <= 600.0 + 1e-6),
            "field_ready": self._ready(self._field_status, self._profile),
            "anchor_ready": self._ready(
                self._anchor_status, self._nav_feature_profile),
            "selector_ready": bool(
                self._selector_status and
                self._selector_status.get("ready") and
                self._selector_status.get("profile") == self._profile and
                tuple(self._selector_status.get("allowed_classes") or []) ==
                self._allowed and self._adapter_accepting_seen and
                self._selector_accepting_seen),
            "profile_contract": (
                manager.get("class_profile") == self._profile and
                tuple(manager.get("allowed_classes") or []) == self._allowed),
            "three_unique_deliveries": (
                len(delivered_ids) == 3 and len(set(delivered_ids)) == 3),
            "sequential_payload_slots": slots == [1, 2, 3],
            "confirm_selected_interrupt_chain": all(
                target_id in self._confirmed_ids and
                target_id in self._profile_selected_ids and
                target_id in self._approach_ids and
                target_id in approach_transitions
                for target_id in delivered_ids),
            "tank_never_selected_or_accepted": (
                "tank" not in self._raw_selected_classes and
                "tank" not in self._profile_selected_classes and
                "tank" not in self._accepted_classes and
                "tank" not in delivered_classes),
            "return_and_land": (
                int(MissionCommand.RETURN_HOME) in
                (manager.get("command_sequence") or []) and
                int(MissionCommand.LAND) in
                (manager.get("command_sequence") or [])),
            "inside_field_bounds": manager.get("boundary_violations") == 0,
            "contact_monitor_ready": bool(
                self._contact_status and
                self._contact_status.get("ready") and
                self._contact_status.get("status") == "READY" and
                int(self._contact_status.get("sample_count", 0)) > 0 and
                self._contact_status.get("last_sample_wall_age") is not None and
                0.0 <= float(self._contact_status.get(
                    "last_sample_wall_age")) <= 1.0 and
                self._contact_status_publishers == {
                    "/gazebo_contact_monitor"}),
            "zero_actual_collisions": bool(
                self._contact_status and
                int(self._contact_status.get(
                    "actual_collision_count", -1)) == 0 and
                int(manager.get("actual_collision_count", -1)) == 0),
            "max_height": (
                max_altitude is not None and
                float(max_altitude) <= 4.0 + 1e-6),
            "event_sequence_monotonic": (
                bool(status_sequences) and status_sequences ==
                list(range(1, len(status_sequences) + 1))),
            "navigation_manager_owned_raw_goal": (
                self._raw_goal_publishers == {"/target_search_manager_py"}),
            "adapter_only_planner_goal": (
                self._goal_publishers == {
                    "/navigation_visual_delivery_adapter"}),
            "selector_only_profile_selected": (
                self._selected_publishers == {"/profile_candidate_selector"}),
            "temporary_coverage_manager_absent": (
                "/coverage_search_manager" not in self._nodes_seen and
                manager.get("temporary_coverage_manager_active") is False),
            "navigation_source_unmodified": (
                manager.get("navigation_source_modified") is False and
                "liftrace-controlwork@5144aa8" in
                manager.get("route_source", "")),
            "required_artifacts": self._artifacts_present(),
        }
        passed = all(checks.values())
        return {
            "gate": "navigation_random_field_visual_delivery",
            "status": "PASS" if passed else "FAIL",
            "reason": "all_assertions_passed" if passed
                      else "assertion_failed",
            "checks": checks,
            "manager": manager,
            "confirmed_ids": sorted(self._confirmed_ids),
            "profile_selected_ids": sorted(self._profile_selected_ids),
            "approach_ids": sorted(self._approach_ids),
            "goal_publishers_seen": sorted(self._goal_publishers),
            "raw_goal_publishers_seen": sorted(self._raw_goal_publishers),
            "selected_publishers_seen": sorted(self._selected_publishers),
            "contact_status_publishers_seen": sorted(
                self._contact_status_publishers),
            "raw_selected_classes": list(self._raw_selected_classes),
            "nodes_seen": sorted(self._nodes_seen),
        }

    def _write(self, report):
        os.makedirs(os.path.dirname(self._report_path) or ".", exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._sample_graph()
            if self._terminal is not None:
                report = self._evaluate()
                self._write(report)
                return 0 if report["status"] == "PASS" else 1
            if time.monotonic() >= self._deadline:
                report = {
                    "gate": "navigation_random_field_visual_delivery",
                    "status": "FAIL",
                    "reason": "wall_timeout_waiting_for_manager",
                }
                self._write(report)
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(NavigationRandomFieldAssertion().run())
