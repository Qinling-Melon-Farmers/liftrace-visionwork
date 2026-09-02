#!/usr/bin/env python3
"""Thirty-second fail-closed infrastructure check for the formal random field."""

import json
import math
import os
import sys
import time

import rosgraph
import rospy
import tf2_ros
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String


class NavigationRandomFieldPreflight:
    def __init__(self):
        rospy.init_node("navigation_random_field_preflight")
        self._duration = float(rospy.get_param("~duration_sec", 30.0))
        self._startup_timeout = float(
            rospy.get_param("~startup_timeout_sec", 180.0))
        self._pose_gap_limit = float(
            rospy.get_param("~pose_gap_limit_sec", 0.5))
        self._message_age_limit = float(
            rospy.get_param("~message_age_limit_sec", 1.0))
        self._max_height = float(rospy.get_param("~max_height", 4.0))
        self._profile = rospy.get_param("~class_profile", "r2026")
        self._nav_profile = rospy.get_param(
            "~nav_feature_profile", "baseline")
        self._field_seed = int(rospy.get_param("~field_seed", 11))
        self._allowed_classes = tuple(rospy.get_param(
            "~allowed_classes",
            ["tent", "pillbox", "bridge", "panzer", "red_cross"]))
        self._world = os.path.abspath(rospy.get_param("~world", ""))
        self._target_model_path = os.path.abspath(
            rospy.get_param("~target_model_path", ""))
        self._mission_frame = rospy.get_param(
            "~mission_frame", "camera_init").lstrip("/")
        self._camera_frame = rospy.get_param(
            "~camera_optical_frame",
            "downward_camera_optical_frame").lstrip("/")
        self._image_topic = rospy.get_param(
            "~image_topic", "/downward_camera/image_raw")
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/downward_camera/camera_info")
        self._pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        self._map_topic = rospy.get_param(
            "~map_topic", "/sdf_map/occupancy_inflate")
        self._model_states_topic = rospy.get_param(
            "~model_states_topic", "/gazebo/model_states")
        self._field = (
            float(rospy.get_param("~field/min_x", -3.992)),
            float(rospy.get_param("~field/max_x", 4.008)),
            float(rospy.get_param("~field/min_y", -1.132)),
            float(rospy.get_param("~field/max_y", 8.718)),
        )
        self._search = (
            float(rospy.get_param("~search_region/min_x", -2.007)),
            float(rospy.get_param("~search_region/max_x", 1.993)),
            float(rospy.get_param("~search_region/min_y", 0.273)),
            float(rospy.get_param("~search_region/max_y", 6.273)),
        )
        self._boundary_margin = float(
            rospy.get_param("~spawn/boundary_margin", 0.10))
        self._pair_gap = float(rospy.get_param("~spawn/pair_gap", 0.15))
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
        self._run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")

        self._field_status = None
        self._anchor_status = None
        self._contact_status = None
        self._adapter_status = None
        self._camera_info = None
        self._map_frame = None
        self._model_states = {}
        self._last_message_wall = {}
        self._image_count = 0
        self._map_count = 0
        self._model_states_count = 0
        self._pose = None
        self._last_pose_wall = None
        self._pose_max_gap = 0.0
        self._pose_gap_count = 0
        self._boundary_violation_count = 0
        self._invalid_pose_count = 0
        self._max_altitude_seen = float("-inf")
        self._readiness_dropouts = 0
        self._dropout_check_counts = {}
        self._dropout_events = []
        self._last_dropout_signature = None
        self._measurement_started_wall = None
        self._measurement_started_ros = None
        self._startup_deadline = time.monotonic() + self._startup_timeout
        self._publisher_snapshot = {}
        self._nodes_seen = set()
        self._goal_publishers = set()
        self._raw_goal_publishers = set()
        self._last_graph_sample = 0.0
        self._tf_valid = False
        self._tf_error = "not_checked"

        self._master = rosgraph.Master(rospy.get_name())
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.Subscriber(self._image_topic, Image, self._on_image,
                         queue_size=1)
        rospy.Subscriber(self._camera_info_topic, CameraInfo,
                         self._on_camera_info, queue_size=1)
        rospy.Subscriber(self._pose_topic, PoseStamped, self._on_pose,
                         queue_size=1)
        rospy.Subscriber(self._map_topic, PointCloud2, self._on_map,
                         queue_size=1)
        rospy.Subscriber(self._model_states_topic, ModelStates,
                         self._on_model_states, queue_size=1)
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

    def _stamp(self, key):
        self._last_message_wall[key] = time.monotonic()

    def _on_image(self, _message):
        self._image_count += 1
        self._stamp("image")

    def _on_camera_info(self, message):
        self._camera_info = message
        frame = message.header.frame_id.lstrip("/")
        if frame:
            self._camera_frame = frame
        self._stamp("camera_info")

    def _on_pose(self, message):
        now_wall = time.monotonic()
        self._pose = message
        self._stamp("pose")
        if self._measurement_started_wall is not None:
            if self._last_pose_wall is not None:
                gap = max(0.0, now_wall - self._last_pose_wall)
                self._pose_max_gap = max(self._pose_max_gap, gap)
                if gap > self._pose_gap_limit:
                    self._pose_gap_count += 1
        self._last_pose_wall = now_wall
        point = message.pose.position
        values = (float(point.x), float(point.y), float(point.z))
        if not all(math.isfinite(value) for value in values):
            self._invalid_pose_count += 1
            return
        if not (self._field[0] <= values[0] <= self._field[1] and
                self._field[2] <= values[1] <= self._field[3]):
            self._boundary_violation_count += 1
        self._max_altitude_seen = max(self._max_altitude_seen, values[2])

    def _on_map(self, message):
        self._map_count += 1
        self._map_frame = message.header.frame_id.lstrip("/")
        self._stamp("map")

    def _on_model_states(self, message):
        self._model_states_count += 1
        self._model_states = dict(zip(message.name, message.pose))
        self._stamp("model_states")

    def _on_field(self, message):
        self._field_status = self._decode(message)
        self._stamp("field")

    def _on_anchor(self, message):
        self._anchor_status = self._decode(message)
        self._stamp("anchor")

    def _on_contact(self, message):
        self._contact_status = self._decode(message)
        self._stamp("contact")

    def _on_adapter(self, message):
        self._adapter_status = self._decode(message)
        self._stamp("adapter")

    def _sample_graph(self):
        now = time.monotonic()
        if now - self._last_graph_sample < 0.5:
            return
        self._last_graph_sample = now
        try:
            publishers, _subscribers, _services = self._master.getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "preflight graph sample failed: %s", exc)
            return
        snapshot = {topic: sorted(nodes) for topic, nodes in publishers}
        self._publisher_snapshot = snapshot
        self._nodes_seen.update(
            node for nodes in snapshot.values() for node in nodes)
        self._goal_publishers.update(snapshot.get("/fastplanner/goal", []))
        self._raw_goal_publishers.update(
            snapshot.get("/navigation/goal_raw", []))

    def _sample_tf(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._mission_frame, self._camera_frame, rospy.Time(0),
                # Nonblocking lookup: tf2's timeout follows ROS time and can
                # otherwise hang forever while /clock is paused.
                rospy.Duration(0.0))
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (
                translation.x, translation.y, translation.z,
                rotation.x, rotation.y, rotation.z, rotation.w)
            norm = math.sqrt(sum(value * value for value in values[3:]))
            self._tf_valid = (
                all(math.isfinite(float(value)) for value in values) and
                norm > 1e-6)
            self._tf_error = "" if self._tf_valid else "nonfinite_transform"
        except Exception as exc:
            self._tf_valid = False
            self._tf_error = str(exc)

    def _message_fresh(self, key, limit=None):
        stamp = self._last_message_wall.get(key)
        threshold = self._message_age_limit if limit is None else limit
        return bool(stamp is not None and
                    0.0 <= time.monotonic() - stamp <= threshold)

    @staticmethod
    def _finite_number(value):
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _safe_int(value, default=-1):
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _camera_info_valid(self):
        message = self._camera_info
        if message is None or len(message.K) != 9:
            return False
        values = [float(value) for value in message.K]
        return bool(
            message.width > 0 and message.height > 0 and
            all(math.isfinite(value) for value in values) and
            values[0] > 0.0 and values[4] > 0.0 and
            abs(values[8]) > 1e-9 and
            message.header.frame_id.lstrip("/") == self._camera_frame)

    def _truth_contract(self):
        status = self._field_status or {}
        path = status.get("truth_path", "")
        result = {
            "truth_present": bool(path and os.path.isfile(path)),
            "truth_identity": False,
            "truth_footprints": False,
            "truth_models_match_gazebo": False,
        }
        if not result["truth_present"]:
            return result
        try:
            with open(path, encoding="utf-8") as handle:
                truth = yaml.safe_load(handle) or {}
            targets = list(truth.get("targets") or [])
            classes = tuple(sorted(item.get("class") for item in targets))
            expected_classes = tuple(sorted(self._allowed_classes))
            result["truth_identity"] = bool(
                truth.get("profile") == self._profile and
                self._safe_int(truth.get("seed")) == self._field_seed and
                classes == expected_classes and
                len({item.get("model") for item in targets}) ==
                len(expected_classes))
            footprints = []
            geometry_valid = True
            gazebo_valid = True
            for item in targets:
                values = [item.get(key) for key in (
                    "x", "y", "world_x", "world_y", "yaw",
                    "footprint_radius")]
                if not all(self._finite_number(value) for value in values):
                    geometry_valid = False
                    gazebo_valid = False
                    continue
                x, y, world_x, world_y, _yaw, radius = map(float, values)
                if (radius <= 0.0 or
                        x - radius - self._boundary_margin < self._search[0] or
                        x + radius + self._boundary_margin > self._search[1] or
                        y - radius - self._boundary_margin < self._search[2] or
                        y + radius + self._boundary_margin > self._search[3]):
                    geometry_valid = False
                for other_x, other_y, other_radius in footprints:
                    if math.hypot(x - other_x, y - other_y) < (
                            radius + other_radius + self._pair_gap):
                        geometry_valid = False
                footprints.append((x, y, radius))
                pose = self._model_states.get(item.get("model"))
                if (pose is None or
                        math.hypot(pose.position.x - world_x,
                                   pose.position.y - world_y) > 0.10):
                    gazebo_valid = False
            result["truth_footprints"] = geometry_valid
            result["truth_models_match_gazebo"] = gazebo_valid
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            pass
        return result

    def _anchor_contract(self):
        status = self._anchor_status or {}
        expected = set(status.get("expected_models") or [])
        verified = set(status.get("verified_models") or [])
        spawned = set(status.get("spawned_models") or [])
        return bool(
            status.get("ready") and status.get("status") == "READY" and
            status.get("profile") == self._nav_profile and
            expected == verified == spawned and
            expected.issubset(self._model_states))

    def _contact_ready(self):
        status = self._contact_status or {}
        age = status.get("last_sample_wall_age")
        return bool(
            status.get("ready") and status.get("status") == "READY" and
            self._safe_int(status.get("sample_count"), 0) > 0 and
            self._finite_number(age) and 0.0 <= float(age) <= 1.0)

    def _required_publishers_present(self):
        return not self._missing_required_publishers()

    def _required_publisher_snapshot(self):
        required = (
            self._image_topic,
            self._camera_info_topic,
            self._pose_topic,
            self._map_topic,
            self._model_states_topic,
            "/planning/bspline",
            "/mission/random_field_status",
            "/mission/planner_anchor_status",
            "/mission/gazebo_contact_status",
            "/mission/target_search_status",
        )
        return {
            topic: list(self._publisher_snapshot.get(topic, []))
            for topic in required
        }

    def _missing_required_publishers(self):
        return sorted(
            topic for topic, nodes in
            self._required_publisher_snapshot().items() if not nodes)

    def _stream_wall_ages(self):
        now = time.monotonic()
        keys = ("image", "camera_info", "pose", "map", "model_states",
                "field", "anchor", "contact", "adapter")
        return {
            key: (None if self._last_message_wall.get(key) is None else
                  max(0.0, now - self._last_message_wall[key]))
            for key in keys
        }

    def _artifacts_present(self):
        names = (
            "random_field_truth.yaml",
            "red_cross_truth.yaml",
            "random_field_status.json",
            "planner_anchor_status.json",
            "gazebo_contact_status.json",
            "target_search_status.json",
        )
        return all(os.path.isfile(os.path.join(self._run_dir, name))
                   for name in names)

    def _checks(self):
        field = self._field_status or {}
        adapter = self._adapter_status or {}
        truth_checks = self._truth_contract()
        # field/anchor are immutable latched barriers, not live streams.
        # Their READY payload and publisher ownership are checked separately;
        # wall freshness would incorrectly fail when Gazebo runs slower than
        # real time and their ROS-time heartbeat stretches past two seconds.
        live_keys = ("image", "camera_info", "pose", "map",
                     "model_states", "contact", "adapter")
        checks = {
            "world_present": os.path.isfile(self._world),
            "target_model_present": os.path.isfile(self._target_model_path),
            "field_ready": bool(
                field.get("ready") and field.get("status") == "READY" and
                field.get("profile") == self._profile and
                self._safe_int(field.get("seed")) == self._field_seed and
                field.get("footprint_valid") is True and
                set(field.get("expected_models") or []) ==
                set(field.get("verified_models") or [])),
            "anchor_ready": self._anchor_contract(),
            "contact_monitor_ready": self._contact_ready(),
            "zero_actual_collisions": self._safe_int(
                (self._contact_status or {}).get(
                    "actual_collision_count", -1)) == 0,
            "adapter_search_operational": bool(
                adapter.get("status") == "RUNNING" and
                adapter.get("state") == "SEARCH" and
                adapter.get("operational_ready") is True and
                adapter.get("class_profile") == self._profile and
                adapter.get("nav_feature_profile") == self._nav_profile),
            "camera_info_valid": self._camera_info_valid(),
            "camera_tf_valid": self._tf_valid,
            "image_received": self._image_count > 0,
            "map_received_in_mission_frame": bool(
                self._map_count > 0 and self._map_frame == self._mission_frame),
            "model_states_received": self._model_states_count > 0,
            "live_streams_fresh": all(
                self._message_fresh(key, 2.0 if key in (
                    "field", "anchor", "map") else None)
                for key in live_keys),
            "required_topic_publishers": self._required_publishers_present(),
            "adapter_only_planner_goal": self._goal_publishers == {
                "/navigation_visual_delivery_adapter"},
            "navigation_manager_only_raw_goal": (
                self._raw_goal_publishers == {"/target_search_manager_py"}),
            "temporary_coverage_manager_absent": (
                "/coverage_search_manager" not in self._nodes_seen),
            "required_artifacts": self._artifacts_present(),
        }
        checks.update(truth_checks)
        return checks

    def _start_measurement(self):
        self._measurement_started_wall = time.monotonic()
        self._measurement_started_ros = rospy.Time.now().to_sec()
        self._last_pose_wall = self._measurement_started_wall

    def _record_readiness_dropout(self, checks):
        failed = tuple(sorted(
            name for name, passed in checks.items() if not passed))
        self._readiness_dropouts += 1
        for name in failed:
            self._dropout_check_counts[name] = (
                self._dropout_check_counts.get(name, 0) + 1)
        if (failed != self._last_dropout_signature and
                len(self._dropout_events) < 64):
            self._dropout_events.append({
                "wall_elapsed": max(
                    0.0, time.monotonic() -
                    self._measurement_started_wall),
                "ros_elapsed": max(
                    0.0, rospy.Time.now().to_sec() -
                    self._measurement_started_ros),
                "failed_checks": list(failed),
            })
        self._last_dropout_signature = failed

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
        wall_elapsed = (None if self._measurement_started_wall is None else
                        now_wall - self._measurement_started_wall)
        ros_elapsed = (None if self._measurement_started_ros is None else
                       rospy.Time.now().to_sec() -
                       self._measurement_started_ros)
        checks = self._checks()
        checks.update({
            "measurement_completed": bool(
                wall_elapsed is not None and wall_elapsed >= self._duration),
            "no_readiness_dropout": self._readiness_dropouts == 0,
            "pose_gap": self._pose_max_gap <= self._pose_gap_limit,
            "inside_field_bounds": self._boundary_violation_count == 0,
            "finite_pose": self._invalid_pose_count == 0,
            "max_height": bool(
                self._max_altitude_seen != float("-inf") and
                self._max_altitude_seen <= self._max_height),
        })
        passed = all(checks.values())
        report = {
            "gate": "navigation_random_field_preflight",
            "status": "PASS" if passed else "FAIL",
            "reason": "preflight_complete" if passed else reason,
            "checks": checks,
            "class_profile": self._profile,
            "nav_feature_profile": self._nav_profile,
            "field_seed": self._field_seed,
            "world": self._world,
            "target_model_path": self._target_model_path,
            "camera_info": (None if self._camera_info is None else {
                "width": self._camera_info.width,
                "height": self._camera_info.height,
                "frame_id": self._camera_info.header.frame_id,
                "K": list(self._camera_info.K),
            }),
            "mission_frame": self._mission_frame,
            "camera_frame": self._camera_frame,
            "tf_error": self._tf_error,
            "image_count": self._image_count,
            "map_count": self._map_count,
            "model_states_count": self._model_states_count,
            "wall_elapsed": wall_elapsed,
            "ros_elapsed": ros_elapsed,
            "pose_max_gap_wall": self._pose_max_gap,
            "pose_gap_count": self._pose_gap_count,
            "boundary_violation_count": self._boundary_violation_count,
            "invalid_pose_count": self._invalid_pose_count,
            "max_altitude": (None if self._max_altitude_seen == float("-inf")
                             else self._max_altitude_seen),
            "readiness_dropouts": self._readiness_dropouts,
            "dropout_check_counts": dict(self._dropout_check_counts),
            "dropout_events": list(self._dropout_events),
            "stream_wall_ages": self._stream_wall_ages(),
            "message_age_limit_sec": self._message_age_limit,
            "map_age_limit_sec": 2.0,
            "latched_barriers_freshness_exempt": ["field", "anchor"],
            "required_publisher_snapshot": (
                self._required_publisher_snapshot()),
            "missing_required_publishers": (
                self._missing_required_publishers()),
            "planner_goal_publishers": sorted(self._goal_publishers),
            "raw_goal_publishers": sorted(self._raw_goal_publishers),
            "nodes_seen": sorted(self._nodes_seen),
            "field_status": self._field_status,
            "anchor_status": self._anchor_status,
            "contact_status": self._contact_status,
        }
        self._atomic_write(self._report_path, report)
        return 0 if passed else 1

    def run(self):
        while not rospy.is_shutdown():
            now = time.monotonic()
            self._sample_graph()
            self._sample_tf()
            checks = self._checks()
            ready = all(checks.values())
            if self._measurement_started_wall is None:
                if ready:
                    self._start_measurement()
                elif now >= self._startup_deadline:
                    return self._finish("startup_timeout")
            else:
                if not ready:
                    self._record_readiness_dropout(checks)
                else:
                    self._last_dropout_signature = None
                if now - self._measurement_started_wall >= self._duration:
                    return self._finish("preflight_contract_failed")
            # /clock may stop while Gazebo initializes or pauses; the
            # preflight deadline is deliberately monotonic wall time.
            time.sleep(0.1)
        return self._finish("ros_shutdown")


if __name__ == "__main__":
    sys.exit(NavigationRandomFieldPreflight().run())
