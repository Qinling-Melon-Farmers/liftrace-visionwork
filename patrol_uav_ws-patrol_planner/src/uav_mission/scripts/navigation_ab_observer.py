#!/usr/bin/env python3
"""Collect one fixed-route navigation feature-profile A/B sample."""

import json
import math
import os
import sys
import time

import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from uav_mission.search_policy import SearchPolicy


class NavigationAbObserver:
    def __init__(self):
        rospy.init_node("navigation_ab_observer")
        self._duration = float(rospy.get_param("~duration_sec", 90.0))
        self._startup_timeout = float(
            rospy.get_param("~startup_timeout_sec", 180.0))
        self._planning_timeout = float(
            rospy.get_param("~planning_response_timeout_sec", 5.0))
        self._pose_gap_limit = float(
            rospy.get_param("~pose_gap_limit_sec", 0.5))
        self._arrival = float(rospy.get_param("~arrival_threshold", 0.3))
        self._search_altitude = float(
            rospy.get_param("~search_altitude", 2.2))
        self._max_height = float(rospy.get_param("~max_height", 4.0))
        self._field = (
            float(rospy.get_param("~field/min_x", -3.992)),
            float(rospy.get_param("~field/max_x", 4.008)),
            float(rospy.get_param("~field/min_y", -1.132)),
            float(rospy.get_param("~field/max_y", 8.718)),
        )
        self._profile = rospy.get_param("~class_profile", "r2026")
        self._nav_profile = rospy.get_param(
            "~nav_feature_profile", "baseline")
        self._field_seed = int(rospy.get_param("~field_seed", 11))
        self._world = os.path.abspath(rospy.get_param("~world", ""))
        self._target_model_path = os.path.abspath(
            rospy.get_param("~target_model_path", ""))
        self._metrics_path = rospy.get_param(
            "~metrics_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "ab_metrics.json"))
        self._gate_path = rospy.get_param(
            "~gate_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))

        self._policy = SearchPolicy(
            float(rospy.get_param("~search/min_x", -3.106588)),
            float(rospy.get_param("~search/max_x", 3.093412)),
            float(rospy.get_param("~search/min_y", -0.227310)),
            float(rospy.get_param("~search/max_y", 7.772690)),
            float(rospy.get_param("~search/lane_spacing", 1.2)),
            self._search_altitude)

        self._field_status = None
        self._anchor_status = None
        self._contact_status = None
        self._adapter_status = None
        self._pose = None
        self._last_pose_wall = None
        self._pose_max_gap = 0.0
        self._pose_gap_count = 0
        self._boundary_violation_count = 0
        self._invalid_pose_count = 0
        self._max_altitude_seen = float("-inf")
        self._height_error_sq = []
        self._last_goal = None
        self._last_goal_wall = None
        self._last_bspline_wall = None
        self._current_goal = None
        self._goal_records = []
        self._arrival_wall_times = []
        self._pending_plans = []
        self._planning_failure_count = 0
        self._planning_response_times = []
        self._replan_count = 0
        self._bspline_count = 0
        self._unexpected_goal_count = 0
        self._last_route_index = None
        self._publisher_snapshot = {}
        self._nodes_seen = set()
        self._goal_publishers = set()
        self._raw_goal_publishers = set()
        self._master = rosgraph.Master(rospy.get_name())
        self._measurement_started_wall = None
        self._measurement_started_ros = None
        self._readiness_dropouts = 0
        self._startup_deadline = time.monotonic() + self._startup_timeout

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber("/fastplanner/goal", PoseStamped,
                         self._on_goal, queue_size=10)
        rospy.Subscriber("/planning/bspline", rospy.AnyMsg,
                         self._on_bspline, queue_size=20)
        rospy.Subscriber("/planning/replan", rospy.AnyMsg,
                         self._on_replan, queue_size=50)
        rospy.Subscriber("/mission/random_field_status", String,
                         self._on_field, queue_size=2)
        rospy.Subscriber("/mission/planner_anchor_status", String,
                         self._on_anchor, queue_size=2)
        rospy.Subscriber("/mission/gazebo_contact_status", String,
                         self._on_contact, queue_size=4)
        rospy.Subscriber("/mission/target_search_status", String,
                         self._on_adapter, queue_size=10)

    @staticmethod
    def _decode(message):
        try:
            return json.loads(message.data)
        except (TypeError, ValueError):
            return None

    def _on_field(self, message):
        self._field_status = self._decode(message)

    def _on_anchor(self, message):
        self._anchor_status = self._decode(message)

    def _on_contact(self, message):
        self._contact_status = self._decode(message)

    def _on_adapter(self, message):
        self._adapter_status = self._decode(message)

    def _on_pose(self, message):
        now_wall = time.monotonic()
        self._pose = message
        if self._measurement_started_wall is not None:
            if self._last_pose_wall is not None:
                gap = max(0.0, now_wall - self._last_pose_wall)
                self._pose_max_gap = max(self._pose_max_gap, gap)
                if gap > self._pose_gap_limit:
                    self._pose_gap_count += 1
        self._last_pose_wall = now_wall
        z = float(message.pose.position.z)
        x = float(message.pose.position.x)
        y = float(message.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y, z)):
            self._invalid_pose_count += 1
            return
        if not (self._field[0] <= x <= self._field[1] and
                self._field[2] <= y <= self._field[3]):
            self._boundary_violation_count += 1
        self._max_altitude_seen = max(self._max_altitude_seen, z)
        if (self._measurement_started_wall is not None and
                self._adapter_status is not None and
                self._adapter_status.get("state") == "SEARCH"):
            error = z - self._search_altitude
            self._height_error_sq.append(error * error)
        if self._current_goal is not None:
            current = message.pose.position
            target = self._current_goal.pose.position
            distance = math.sqrt(
                (current.x - target.x) ** 2 +
                (current.y - target.y) ** 2 +
                (current.z - target.z) ** 2)
            if distance <= self._arrival:
                self._arrival_wall_times.append(
                    now_wall - self._measurement_started_wall)
                self._current_goal = None

    @staticmethod
    def _goal_tuple(message):
        point = message.pose.position
        return [round(float(point.x), 6), round(float(point.y), 6),
                round(float(point.z), 6)]

    def _register_goal(self, message, now_wall):
        goal = self._goal_tuple(message)
        if self._goal_records and self._goal_records[-1]["goal"] == goal:
            return
        route = self._route_spec()
        route_index = next((index for index, point in enumerate(route)
                            if all(abs(a - b) <= 1e-4
                                   for a, b in zip(point, goal))), None)
        expected_index = (0 if self._last_route_index is None else
                          self._last_route_index + 1)
        if route_index is None or route_index != expected_index:
            self._unexpected_goal_count += 1
        if route_index is not None:
            self._last_route_index = route_index
        elapsed = max(0.0, now_wall - self._measurement_started_wall)
        self._goal_records.append({
            "goal": goal, "route_index": route_index,
            "wall_time": elapsed})
        self._current_goal = message
        already_resolved = bool(
            self._last_bspline_wall is not None and
            self._last_goal_wall is not None and
            self._last_bspline_wall >= self._last_goal_wall)
        self._pending_plans.append({
            "goal_index": len(self._goal_records) - 1,
            "started_wall": now_wall,
            "resolved": already_resolved,
            "failed": False,
        })

    def _on_goal(self, message):
        now_wall = time.monotonic()
        if (self._measurement_started_wall is not None and
                self._current_goal is not None and
                self._goal_tuple(self._current_goal) !=
                self._goal_tuple(message)):
            # The unchanged upstream manager advances its fixed route only
            # after its own arrival check.  Record that fact even if this
            # observer's pose callback races the next goal callback.
            self._arrival_wall_times.append(
                now_wall - self._measurement_started_wall)
            self._current_goal = None
        self._last_goal = message
        self._last_goal_wall = now_wall
        if self._measurement_started_wall is not None:
            self._register_goal(message, self._last_goal_wall)

    def _on_bspline(self, _message):
        self._last_bspline_wall = time.monotonic()
        if self._measurement_started_wall is None:
            return
        self._bspline_count += 1
        pending = next((item for item in reversed(self._pending_plans)
                        if not item["resolved"] and not item["failed"]), None)
        if pending is not None:
            pending["resolved"] = True
            self._planning_response_times.append(
                max(0.0, time.monotonic() - pending["started_wall"]))

    def _on_replan(self, _message):
        if self._measurement_started_wall is not None:
            self._replan_count += 1

    @staticmethod
    def _ready_status(status, profile):
        return bool(status and status.get("ready") and
                    status.get("status") == "READY" and
                    status.get("profile") == profile)

    def _contact_ready(self):
        status = self._contact_status or {}
        age = status.get("last_sample_wall_age")
        try:
            sample_count = int(status.get("sample_count", 0))
            age = float(age)
        except (TypeError, ValueError):
            return False
        return bool(
            status.get("ready") and status.get("status") == "READY" and
            sample_count > 0 and math.isfinite(age) and 0.0 <= age <= 1.0)

    def _truth_path(self):
        return (self._field_status or {}).get("truth_path", "")

    def _assets_ready(self):
        adapter = self._adapter_status or {}
        field = self._field_status or {}
        try:
            field_seed = int(field.get("seed", -1))
        except (TypeError, ValueError):
            field_seed = -1
        required_topics = (
            "/mavros/local_position/pose",
            "/planning/bspline",
            "/mission/random_field_status",
            "/mission/planner_anchor_status",
            "/mission/gazebo_contact_status",
            "/mission/target_search_status",
        )
        return bool(
            self._ready_status(field, self._profile) and
            field_seed == self._field_seed and
            field.get("footprint_valid") is True and
            self._ready_status(self._anchor_status, self._nav_profile) and
            self._contact_ready() and
            adapter.get("status") == "RUNNING" and
            adapter.get("state") == "SEARCH" and
            adapter.get("operational_ready") is True and
            self._pose is not None and
            os.path.isfile(self._truth_path()) and
            os.path.isfile(self._world) and
            os.path.isfile(self._target_model_path) and
            self._goal_publishers == {
                "/navigation_visual_delivery_adapter"} and
            self._raw_goal_publishers == {"/target_search_manager_py"} and
            all(self._publisher_snapshot.get(topic)
                for topic in required_topics))

    def _sample_graph(self):
        try:
            publishers, _subscribers, _services = self._master.getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "A/B graph sample failed: %s", exc)
            return
        snapshot = {topic: sorted(nodes) for topic, nodes in publishers}
        self._publisher_snapshot = snapshot
        self._nodes_seen.update(
            node for nodes in snapshot.values() for node in nodes)
        self._goal_publishers.update(snapshot.get("/fastplanner/goal", []))
        self._raw_goal_publishers.update(
            snapshot.get("/navigation/goal_raw", []))

    def _start_measurement(self):
        self._measurement_started_wall = time.monotonic()
        self._measurement_started_ros = rospy.Time.now().to_sec()
        self._last_pose_wall = self._measurement_started_wall
        if self._last_goal is not None:
            self._register_goal(
                self._last_goal, self._measurement_started_wall)

    def _update_planning_failures(self):
        now = time.monotonic()
        for item in self._pending_plans:
            if (not item["resolved"] and not item["failed"] and
                    now - item["started_wall"] >= self._planning_timeout):
                item["failed"] = True
                self._planning_failure_count += 1

    def _truth_targets(self):
        with open(self._truth_path(), encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        targets = []
        for item in payload.get("targets") or []:
            targets.append({
                "class": item.get("class"),
                "x": round(float(item.get("x")), 4),
                "y": round(float(item.get("y")), 4),
                "yaw": round(float(item.get("yaw")), 4),
            })
        return sorted(targets, key=lambda item: item["class"])

    def _route_spec(self):
        return [list(point.as_tuple()) for point in self._policy.waypoints]

    @staticmethod
    def _atomic_write(path, payload):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, path)

    def _finish(self, reason):
        now_wall = time.monotonic()
        if self._last_pose_wall is not None:
            self._pose_max_gap = max(
                self._pose_max_gap, now_wall - self._last_pose_wall)
        self._update_planning_failures()
        wall_elapsed = (None if self._measurement_started_wall is None else
                        now_wall - self._measurement_started_wall)
        ros_elapsed = (None if self._measurement_started_ros is None else
                       rospy.Time.now().to_sec() -
                       self._measurement_started_ros)
        height_rms = (None if not self._height_error_sq else
                      math.sqrt(sum(self._height_error_sq) /
                                len(self._height_error_sq)))
        contact_count = int((self._contact_status or {}).get(
            "actual_collision_count", -1))
        assets_ready = self._assets_ready()
        checks = {
            "measurement_completed": (
                wall_elapsed is not None and wall_elapsed >= self._duration),
            "assets_ready": assets_ready,
            "no_readiness_dropout": self._readiness_dropouts == 0,
            "pose_gap": self._pose_max_gap <= self._pose_gap_limit,
            "max_height": (
                self._max_altitude_seen != float("-inf") and
                self._max_altitude_seen <= self._max_height),
            "height_samples_observed": bool(self._height_error_sq),
            "inside_field_bounds": self._boundary_violation_count == 0,
            "finite_pose": self._invalid_pose_count == 0,
            "zero_actual_collisions": contact_count == 0,
            "route_progress_observed": bool(self._arrival_wall_times),
            "fixed_route_goals": (
                bool(self._goal_records) and
                self._unexpected_goal_count == 0),
            "adapter_only_planner_goal": self._goal_publishers == {
                "/navigation_visual_delivery_adapter"},
            "navigation_manager_only_raw_goal": (
                self._raw_goal_publishers == {"/target_search_manager_py"}),
        }
        passed = all(checks.values())
        metrics = {
            "gate": "navigation_feature_ab_run",
            "status": "PASS" if passed else "FAIL",
            "reason": "sample_complete" if passed else reason,
            "checks": checks,
            "assets_ready": assets_ready,
            "field_seed": self._field_seed,
            "class_profile": self._profile,
            "nav_feature_profile": self._nav_profile,
            "world": self._world,
            "target_model_path": self._target_model_path,
            "truth_targets": self._truth_targets() if os.path.isfile(
                self._truth_path()) else [],
            "route_spec": self._route_spec(),
            "route_goals": self._goal_records,
            "arrival_wall_times": self._arrival_wall_times,
            "wall_elapsed": wall_elapsed,
            "ros_elapsed": ros_elapsed,
            "pose_max_gap_wall": self._pose_max_gap,
            "pose_gap_count": self._pose_gap_count,
            "boundary_violation_count": self._boundary_violation_count,
            "invalid_pose_count": self._invalid_pose_count,
            "max_altitude": (None if self._max_altitude_seen == float("-inf")
                             else self._max_altitude_seen),
            "height_drift_rms": height_rms,
            "height_drift_max": (None if not self._height_error_sq else
                                 math.sqrt(max(self._height_error_sq))),
            "planning_failure_count": self._planning_failure_count,
            "planning_response_times": self._planning_response_times,
            "replan_count": self._replan_count,
            "bspline_count": self._bspline_count,
            "unexpected_goal_count": self._unexpected_goal_count,
            "planner_goal_publishers": sorted(self._goal_publishers),
            "raw_goal_publishers": sorted(self._raw_goal_publishers),
            "nodes_seen": sorted(self._nodes_seen),
            "actual_collision_count": contact_count,
            "contact_status": self._contact_status,
            "readiness_dropouts": self._readiness_dropouts,
        }
        self._atomic_write(self._metrics_path, metrics)
        self._atomic_write(self._gate_path, metrics)
        return 0 if passed else 1

    def run(self):
        while not rospy.is_shutdown():
            now = time.monotonic()
            self._sample_graph()
            if self._measurement_started_wall is None:
                if self._assets_ready():
                    self._start_measurement()
                elif now >= self._startup_deadline:
                    return self._finish("startup_timeout")
            else:
                if not self._assets_ready():
                    self._readiness_dropouts += 1
                self._update_planning_failures()
                if now - self._measurement_started_wall >= self._duration:
                    return self._finish("measurement_contract_failed")
            # Do not bind a wall-clock Gate to /clock progress.  Gazebo pause
            # must still reach startup/measurement timeout deterministically.
            time.sleep(0.1)
        return self._finish("ros_shutdown")


if __name__ == "__main__":
    sys.exit(NavigationAbObserver().run())
