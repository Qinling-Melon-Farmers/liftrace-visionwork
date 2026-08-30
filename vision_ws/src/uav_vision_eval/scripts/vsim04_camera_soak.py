#!/usr/bin/env python3
"""Run a fail-closed, camera-only V-SIM-04 endurance measurement."""

import csv
import json
import math
import os
import re
import sys
import threading
import time

import rosnode
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty

from uav_vision.msg import TargetCandidate, TargetCandidateArray
from uav_vision.msg import TargetDetectionArray
from uav_vision.target_selection_policy import resolve_class_profile
from uav_vision_eval.msg import SimTargetArray
from uav_vision_eval.vsim04_metrics import (
    call_with_monotonic_deadline,
    completed_sources_cover,
)
from uav_vision_eval.vsim04_soak import (
    REQUIRED_SOAK_ARTIFACTS,
    SoakAccounting,
    camera_info_snapshot,
    route_pose,
    selected_candidate_errors,
    validate_soak_config,
)


UNKNOWN_VALUES = {"", "unknown", "unspecified", "none", "null"}


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _atomic_text(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8", newline="") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _atomic_json(path, value):
    _atomic_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def _write_csv(path, fields, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class VSim04CameraSoak:
    REQUIRED_STREAMS = (
        "image", "truth", "mapped_complete", "targets", "perf",
        "camera_pose",
    )
    FRAME_FIELDS = (
        "trial_id", "loop_index", "wall_elapsed_sec", "source_elapsed_sec",
        "monotonic_sec",
        "camera_x_m", "camera_y_m", "camera_z_m", "image_count",
        "complete_mapped_frames", "partial_only_mapped_frames",
        "input_fps", "complete_mapped_fps", "selected_count",
        "tank_selected_count", "disallowed_selected_count",
        "max_heartbeat_gap_sec", "active_errors",
    )
    EVENT_FIELDS = (
        "event_seq", "trial_id", "event", "source_stamp",
        "monotonic_sec", "details",
    )
    PERFORMANCE_FIELDS = (
        "trial_id", "kind", "status", "qualification_status",
        "soak_600s_pass", "requested_duration_sec", "actual_duration_sec",
        "actual_wall_duration_sec", "actual_source_duration_sec",
        "input_fps", "complete_mapped_fps", "complete_mapped_frames",
        "partial_only_mapped_frames", "selected_observations",
        "tank_selected_observations", "disallowed_selected_observations",
        "stale_selected_observations", "p_interrupt", "failure_reasons",
    )

    def __init__(self):
        rospy.init_node("vsim04_camera_soak")
        self._lock = threading.RLock()
        self._config = validate_soak_config({
            name: rospy.get_param("~" + name)
            for name in (
                "duration_sec", "route_period_sec", "route_update_rate_hz",
                "route_center_x_m", "route_center_y_m",
                "route_radius_x_m", "route_radius_y_m", "route_height_m",
                "arena_limit_m", "health_window_sec",
                "heartbeat_timeout_sec", "startup_timeout_sec",
                "bucket_settle_sec", "max_source_lag_sec", "min_input_fps",
                "min_complete_mapped_fps", "max_partial_only_ratio",
                "min_partial_samples", "bad_windows_to_fail",
                "service_call_timeout_sec",
            )
        })
        self._output_dir = os.path.abspath(rospy.get_param("~output_dir"))
        self._camera_model = rospy.get_param(
            "~camera_model", "vision_eval_camera")
        self._set_state_name = rospy.get_param(
            "~set_model_state_service", "/gazebo/set_model_state")
        self._reset_name = rospy.get_param(
            "~reset_service", "/uav_vision/reset_memory")
        self._set_state = rospy.ServiceProxy(
            self._set_state_name, SetModelState)
        self._reset = rospy.ServiceProxy(self._reset_name, Empty)
        self._rpy = [float(value) for value in rospy.get_param(
            "~camera_rpy", [0.0, math.pi / 2.0, 0.0])]
        if len(self._rpy) != 3 or not all(math.isfinite(v) for v in self._rpy):
            raise ValueError("camera_rpy must contain three finite values")
        self._profile, self._allowed_classes = resolve_class_profile(
            rospy.get_param("~class_profile", "r2026"))
        if self._profile != "r2026":
            raise ValueError("camera soak requires class_profile=r2026")
        self._confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self._selected_max_age = float(rospy.get_param(
            "~selected_max_age_sec", 0.5))
        self._required_sources = {
            str(value).strip() for value in rospy.get_param(
                "~required_completed_sources",
                ["target_detector", "circle_detector", "cross_detector"])
            if str(value).strip()
        }
        self._expected_nodes = {
            str(value).strip() for value in rospy.get_param(
                "~expected_nodes", []) if str(value).strip()
        }
        self._node_check_period = float(rospy.get_param(
            "~node_check_period_sec", 1.0))
        self._frame_period = float(rospy.get_param(
            "~frame_record_period_sec", 1.0))
        self._clock_stall_timeout = float(rospy.get_param(
            "~clock_stall_timeout_sec", 3.0))
        self._expected_perf_name = str(rospy.get_param(
            "~expected_perf_name", "uav_vision/target_detector"))
        self._model_path = os.path.abspath(os.path.expanduser(
            str(rospy.get_param("~model_path", "")).strip()))
        self._world_file = os.path.abspath(rospy.get_param("~world_file"))
        self._scenario_file = os.path.abspath(rospy.get_param("~scenario_file"))
        self._target_catalog_file = os.path.abspath(rospy.get_param(
            "~target_catalog_file"))
        self._extrinsic_profile = str(rospy.get_param(
            "~extrinsic_profile", "vision_eval_camera_d435i")).strip()
        self._extrinsic_source = os.path.abspath(rospy.get_param(
            "~extrinsic_source"))
        self._vision_revision = str(rospy.get_param(
            "~vision_revision", "unknown")).strip()
        self._navigation_revision = str(rospy.get_param(
            "~navigation_revision", "unknown")).strip()
        self._validate_provenance()

        self._accounting = SoakAccounting(
            self._config, self.REQUIRED_STREAMS)
        self._frames = []
        self._events = []
        self._event_seq = 0
        self._selected_count = 0
        self._tank_selected_count = 0
        self._disallowed_selected_count = 0
        self._stale_selected_count = 0
        self._last_pose = None
        self._camera_info = None
        self._last_loop_id = ""
        self._started_ros_sec = None
        self._started_wall = None
        self._actual_wall_duration_sec = 0.0
        self._actual_source_duration_sec = 0.0
        self._fatal_exception = ""
        self._subscribe()

    def _validate_provenance(self):
        if not self._model_path or not os.path.isfile(self._model_path):
            raise ValueError("model_path is missing or not a file")
        for name, path in (
                ("world_file", self._world_file),
                ("scenario_file", self._scenario_file),
                ("target_catalog_file", self._target_catalog_file),
                ("extrinsic_source", self._extrinsic_source)):
            if not path or not os.path.isfile(path):
                raise ValueError("{} is missing or not a file".format(name))
        if not self._extrinsic_profile:
            raise ValueError("extrinsic_profile is missing")
        revision_pattern = re.compile(
            r"(?i)(?<![0-9a-f])[0-9a-f]{7,40}(?![0-9a-f])")
        for name, value in (("vision", self._vision_revision),
                            ("navigation", self._navigation_revision)):
            if value.lower() in UNKNOWN_VALUES or not revision_pattern.search(value):
                raise ValueError("{}_revision must contain a git commit".format(name))

    def _subscribe(self):
        rospy.Subscriber(rospy.get_param(
            "~image_topic", "/camera/color/image_raw"),
            Image, self._on_image, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"),
            CameraInfo, self._on_camera_info, queue_size=2)
        rospy.Subscriber(rospy.get_param(
            "~truth_topic", "/uav_vision_eval/ground_truth"),
            SimTargetArray, self._on_truth, queue_size=2)
        rospy.Subscriber(rospy.get_param(
            "~camera_pose_topic", "/uav_vision_eval/camera_pose"),
            PoseStamped, self._on_camera_pose, queue_size=10)
        rospy.Subscriber(rospy.get_param(
            "~mapped_topic", "/uav_vision/detections_mapped"),
            TargetDetectionArray, self._on_mapped, queue_size=20)
        rospy.Subscriber(rospy.get_param(
            "~targets_topic", "/uav_vision/targets"),
            TargetCandidateArray, self._on_targets, queue_size=5)
        rospy.Subscriber(rospy.get_param(
            "~selected_topic", "/uav_vision/selected_target"),
            TargetCandidate, self._on_selected, queue_size=10)
        rospy.Subscriber(rospy.get_param(
            "~perf_topic", "/uav_vision/perf"),
            DiagnosticArray, self._on_perf, queue_size=10)

    @staticmethod
    def _stamp(message):
        return message.header.stamp.to_sec()

    def _note(self, name, message):
        with self._lock:
            self._accounting.note_stream(
                name, time.monotonic(), self._stamp(message))

    def _on_image(self, message):
        self._note("image", message)

    def _on_truth(self, message):
        self._note("truth", message)

    def _on_camera_pose(self, message):
        self._note("camera_pose", message)

    def _on_camera_info(self, message):
        try:
            profile = camera_info_snapshot(message)
        except ValueError:
            with self._lock:
                self._accounting.add_error("camera_info_invalid")
            return
        with self._lock:
            if self._camera_info is None:
                self._camera_info = profile
            elif profile != self._camera_info:
                self._accounting.add_error("camera_info_changed_during_soak")

    def _on_targets(self, message):
        self._note("targets", message)

    def _on_perf(self, message):
        healthy = any(status.name == self._expected_perf_name and
                      int(status.level) == 0
                      for status in message.status)
        with self._lock:
            if not healthy:
                self._accounting.add_error("detector_diagnostic_not_ok")
            self._accounting.note_stream(
                "perf", time.monotonic(), self._stamp(message))

    def _on_mapped(self, message):
        stamp = self._stamp(message)
        complete = completed_sources_cover(
            self._required_sources, message.completed_sources)
        with self._lock:
            self._accounting.note_mapped(
                time.monotonic(), stamp, complete,
                source_now_sec=rospy.Time.now().to_sec())

    def _on_selected(self, message):
        record = {
            "class_name": message.class_name,
            "last_seen_sec": message.last_seen.to_sec(),
            "state": message.state,
            "consecutive_observe_count": message.consecutive_observe_count,
            "map_valid": message.map_valid,
            "association_valid": message.association_valid,
            "reject_reason": message.reject_reason,
        }
        now_source = rospy.Time.now().to_sec()
        errors = selected_candidate_errors(
            record, now_source, self._allowed_classes,
            self._confirm_frames, self._selected_max_age)
        with self._lock:
            self._selected_count += 1
            if message.class_name == "tank":
                self._tank_selected_count += 1
            if message.class_name not in self._allowed_classes:
                self._disallowed_selected_count += 1
            if "selected_stale" in errors:
                self._stale_selected_count += 1
            for reason in errors:
                self._accounting.add_error(reason)
            self._add_event_locked(
                "selected", self._last_loop_id,
                source_stamp=message.header.stamp.to_sec(),
                details={
                    "stable_id": int(message.id),
                    "class_name": message.class_name,
                    "validation_errors": errors,
                })

    def _add_event_locked(self, event, trial_id="", source_stamp=None,
                          details=None):
        self._event_seq += 1
        self._events.append({
            "event_seq": self._event_seq,
            "trial_id": trial_id,
            "event": event,
            "source_stamp": "" if source_stamp is None else source_stamp,
            "monotonic_sec": time.monotonic(),
            "details": json.dumps(details or {}, sort_keys=True),
        })

    def _call_service(self, name, operation):
        return call_with_monotonic_deadline(
            operation, self._config["service_call_timeout_sec"],
            name, rospy.is_shutdown)

    def _set_camera(self, pose):
        state = ModelState()
        state.model_name = self._camera_model
        state.reference_frame = "world"
        state.pose.position.x = pose["x"]
        state.pose.position.y = pose["y"]
        state.pose.position.z = pose["z"]
        quaternion = _quaternion_from_rpy(*self._rpy)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        response = self._call_service(
            "set_model_state", lambda: self._set_state(state))
        if not response.success:
            raise RuntimeError("set_model_state failed: " + response.status_message)
        self._last_pose = dict(pose)

    def _missing_nodes(self):
        live = set(rosnode.get_node_names())
        return sorted(self._expected_nodes - live)

    def _wait_for_startup(self):
        deadline = time.monotonic() + self._config["startup_timeout_sec"]
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            missing_nodes = self._missing_nodes()
            with self._lock:
                missing_streams = [
                    name for name in self.REQUIRED_STREAMS
                    if name not in self._accounting.last_receipt]
                camera_info_missing = self._camera_info is None
            if (not missing_nodes and not missing_streams and
                    not camera_info_missing):
                return
            # A ROS-time Rate can block forever before /clock becomes healthy.
            time.sleep(0.1)
        raise RuntimeError(
            "startup incomplete; nodes={} streams={} camera_info={}".format(
                ",".join(self._missing_nodes()),
                ",".join(name for name in self.REQUIRED_STREAMS
                         if name not in self._accounting.last_receipt),
                "missing" if self._camera_info is None else "ready"))

    def _record_frame_locked(self, pose, wall_elapsed, source_elapsed,
                             now_wall):
        actual_elapsed = max(wall_elapsed, 1.0e-9)
        maximum_gap = max(
            self._accounting.max_heartbeat_gap.values() or [0.0])
        self._frames.append({
            "trial_id": pose["trial_id"],
            "loop_index": pose["loop_index"],
            "wall_elapsed_sec": wall_elapsed,
            "source_elapsed_sec": source_elapsed,
            "monotonic_sec": now_wall,
            "camera_x_m": pose["x"],
            "camera_y_m": pose["y"],
            "camera_z_m": pose["z"],
            "image_count": self._accounting.counts.get("image", 0),
            "complete_mapped_frames": self._accounting.complete_mapped_frames,
            "partial_only_mapped_frames": (
                self._accounting.partial_only_mapped_frames),
            "input_fps": (
                self._accounting.counts.get("image", 0) / actual_elapsed),
            "complete_mapped_fps": (
                self._accounting.complete_mapped_frames / actual_elapsed),
            "selected_count": self._selected_count,
            "tank_selected_count": self._tank_selected_count,
            "disallowed_selected_count": self._disallowed_selected_count,
            "max_heartbeat_gap_sec": maximum_gap,
            "active_errors": json.dumps(self._accounting.errors),
        })

    def run(self):
        timeout = self._config["startup_timeout_sec"]
        rospy.wait_for_service(self._set_state_name, timeout=timeout)
        rospy.wait_for_service(self._reset_name, timeout=timeout)
        initial_pose = route_pose(0.0, self._config)
        self._set_camera(initial_pose)
        self._call_service("reset_memory", self._reset)
        self._wait_for_startup()

        self._started_wall = time.monotonic()
        self._started_ros_sec = rospy.Time.now().to_sec()
        last_ros_sec = self._started_ros_sec
        last_ros_progress = self._started_wall
        next_node_check = self._started_wall
        next_frame = self._started_wall
        with self._lock:
            self._accounting.start(self._started_wall)
            self._add_event_locked(
                "soak_start", initial_pose["trial_id"],
                source_stamp=self._started_ros_sec,
                details={"requested_duration_sec": self._config["duration_sec"]})
        update_wall_period = 1.0 / self._config["route_update_rate_hz"]
        while not rospy.is_shutdown():
            now_wall = time.monotonic()
            now_ros = rospy.Time.now().to_sec()
            if now_ros < last_ros_sec - 1.0e-9:
                raise RuntimeError("ROS source time moved backwards")
            if now_ros > last_ros_sec:
                last_ros_sec = now_ros
                last_ros_progress = now_wall
            elif now_wall - last_ros_progress > self._clock_stall_timeout:
                raise RuntimeError("/clock stalled")
            source_elapsed = max(0.0, now_ros - self._started_ros_sec)
            wall_elapsed = max(0.0, now_wall - self._started_wall)
            self._actual_source_duration_sec = source_elapsed
            self._actual_wall_duration_sec = wall_elapsed
            if wall_elapsed >= self._config["duration_sec"]:
                break
            pose = route_pose(source_elapsed, self._config)
            self._set_camera(pose)
            with self._lock:
                if pose["trial_id"] != self._last_loop_id:
                    self._last_loop_id = pose["trial_id"]
                    self._add_event_locked(
                        "loop_start", pose["trial_id"], source_stamp=now_ros,
                        details={"loop_index": pose["loop_index"]})
                if now_wall >= next_node_check:
                    missing_nodes = self._missing_nodes()
                    if missing_nodes:
                        self._accounting.add_error(
                            "process_missing:" + ",".join(missing_nodes))
                    for reason in self._accounting.heartbeat_errors(now_wall):
                        self._accounting.add_error(reason)
                    self._accounting.evaluate(now_wall)
                    next_node_check = now_wall + self._node_check_period
                if now_wall >= next_frame:
                    self._record_frame_locked(
                        pose, wall_elapsed, source_elapsed, now_wall)
                    next_frame = now_wall + self._frame_period
                if self._accounting.errors:
                    raise RuntimeError(self._accounting.errors[0])
            # Keep the watchdog on monotonic wall time even when /clock stalls.
            time.sleep(update_wall_period)
        if (rospy.is_shutdown() and
                self._actual_wall_duration_sec < self._config["duration_sec"]):
            raise RuntimeError("ROS shutdown before requested duration")

    def _build_manifest(self):
        return {
            "schema_version": 1,
            "evaluation_id": "V-SIM-04-SOAK",
            "measurement_semantics": "camera-only visual endurance",
            "class_profile": self._profile,
            "p_interrupt": None,
            "navigation_events_present": False,
            "route": {
                key: self._config[key] for key in (
                    "route_period_sec", "route_update_rate_hz",
                    "route_center_x_m", "route_center_y_m",
                    "route_radius_x_m", "route_radius_y_m", "route_height_m")
            },
            "thresholds": dict(self._config),
            "required_streams": list(self.REQUIRED_STREAMS),
            "required_completed_sources": sorted(self._required_sources),
            "expected_nodes": sorted(self._expected_nodes),
            "model": {"path": self._model_path, "backend": "ultralytics"},
            "world_file": self._world_file,
            "scenario_file": self._scenario_file,
            "target_catalog_file": self._target_catalog_file,
            "revisions": {
                "vision": self._vision_revision,
                "navigation": self._navigation_revision,
            },
            "camera": {
                "model_name": self._camera_model,
                "rpy": list(self._rpy),
                "camera_info": self._camera_info,
            },
            "extrinsic": {
                "profile": self._extrinsic_profile,
                "source": self._extrinsic_source,
            },
            "route_time_base": "ROS source time with monotonic wall qualification",
        }

    def _write_artifacts(self):
        with self._lock:
            self._accounting.evaluate(time.monotonic(), force=True)
            if self._fatal_exception:
                self._accounting.add_error(
                    "runtime_exception:" + self._fatal_exception)
            summary = self._accounting.final_summary(
                self._config["duration_sec"],
                self._actual_wall_duration_sec,
                self._actual_source_duration_sec)
            summary.update({
                "schema_version": 1,
                "evaluation_id": "V-SIM-04-SOAK",
                "artifact_set_complete": False,
                "selected_observations": self._selected_count,
                "tank_selected_observations": self._tank_selected_count,
                "disallowed_selected_observations": (
                    self._disallowed_selected_count),
                "stale_selected_observations": self._stale_selected_count,
                "unique_trial_ids": sorted({
                    row["trial_id"] for row in self._frames}),
            })
            manifest = self._build_manifest()
            manifest["result"] = {
                "status": summary["status"],
                "qualification_status": summary["qualification_status"],
                "soak_600s_pass": summary["soak_600s_pass"],
            }
            performance = [{
                "trial_id": "soak_total",
                "kind": "camera_only_soak",
                "status": summary["status"],
                "qualification_status": summary["qualification_status"],
                "soak_600s_pass": summary["soak_600s_pass"],
                "requested_duration_sec": summary["requested_duration_sec"],
                "actual_duration_sec": summary["actual_duration_sec"],
                "actual_wall_duration_sec": summary[
                    "actual_wall_duration_sec"],
                "actual_source_duration_sec": summary[
                    "actual_source_duration_sec"],
                "input_fps": summary["input_fps"],
                "complete_mapped_fps": summary["complete_mapped_fps"],
                "complete_mapped_frames": summary["complete_mapped_frames"],
                "partial_only_mapped_frames": summary[
                    "partial_only_mapped_frames"],
                "selected_observations": self._selected_count,
                "tank_selected_observations": self._tank_selected_count,
                "disallowed_selected_observations": (
                    self._disallowed_selected_count),
                "stale_selected_observations": self._stale_selected_count,
                "p_interrupt": "",
                "failure_reasons": json.dumps(summary["errors"]),
            }]
            qualification = summary["qualification_status"]
            report = "\n".join([
                "# V-SIM-04 camera-only soak",
                "",
                "- Status: `{}`".format(summary["status"]),
                "- Qualification: `{}`".format(qualification),
                "- Requested/actual monotonic wall time: `{:.3f}` / `{:.3f}` s".format(
                    summary["requested_duration_sec"],
                    summary["actual_wall_duration_sec"]),
                "- Actual ROS source time: `{:.3f}` s".format(
                    summary["actual_source_duration_sec"]),
                "- 600 s PASS: `{}`".format(
                    str(summary["soak_600s_pass"]).lower()),
                "- Input/complete-mapped throughput: `{:.3f}` / `{:.3f}` FPS".format(
                    summary["input_fps"], summary["complete_mapped_fps"]),
                "- Partial-only mapped stamps: `{}`".format(
                    summary["partial_only_mapped_frames"]),
                "- tank/disallowed/stale selected: `{}` / `{}` / `{}`".format(
                    self._tank_selected_count, self._disallowed_selected_count,
                    self._stale_selected_count),
                "- P_interrupt: `null` (visual-only; no navigation event exists)",
                "- Errors: `{}`".format(
                    ", ".join(summary["errors"]) if summary["errors"] else "none"),
                "",
                ("A successful run shorter than 600 s is SMOKE_ONLY and is not "
                 "a 600 s endurance PASS."),
                "",
            ])

        _atomic_json(os.path.join(self._output_dir, "manifest.json"), manifest)
        _write_csv(os.path.join(self._output_dir, "frames.csv"),
                   self.FRAME_FIELDS, self._frames)
        _write_csv(os.path.join(self._output_dir, "events.csv"),
                   self.EVENT_FIELDS, self._events)
        _write_csv(os.path.join(
            self._output_dir, "vision_search_performance.csv"),
            self.PERFORMANCE_FIELDS, performance)
        _atomic_json(os.path.join(self._output_dir, "summary.json"), summary)
        _atomic_text(os.path.join(self._output_dir, "report.md"), report)
        complete = all(
            os.path.isfile(os.path.join(self._output_dir, name)) and
            os.path.getsize(os.path.join(self._output_dir, name)) > 0
            for name in REQUIRED_SOAK_ARTIFACTS)
        if not complete:
            raise RuntimeError("required soak artifact missing or empty")
        summary["artifact_set_complete"] = True
        _atomic_json(os.path.join(self._output_dir, "summary.json"), summary)
        return summary

    def execute(self):
        try:
            self.run()
        except Exception as error:
            self._fatal_exception = str(error)
            rospy.logerr("V-SIM-04 camera soak failed: %s", error)
        try:
            summary = self._write_artifacts()
        except Exception as error:
            rospy.logerr("V-SIM-04 camera soak artifact failure: %s", error)
            return 9
        if summary["status"] != "SOAK_MEASURED":
            return 8
        rospy.loginfo(
            "V-SIM-04 camera soak complete: %s (%.3f s)",
            summary["qualification_status"],
            summary["actual_wall_duration_sec"])
        return 0


def main():
    try:
        runner = VSim04CameraSoak()
        return runner.execute()
    except Exception as error:
        rospy.logerr("V-SIM-04 camera soak initialization failed: %s", error)
        return 10


if __name__ == "__main__":
    sys.exit(main())
