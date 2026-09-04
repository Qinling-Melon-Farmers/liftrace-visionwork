#!/usr/bin/env python3
"""Record fail-closed V-SIM-04 trial performance in one Gazebo session."""

import copy
import json
import math
import os
import re
import threading
import time
from collections import Counter, OrderedDict

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from uav_vision.msg import (
    TargetCandidate,
    TargetCandidateArray,
    TargetDetectionArray,
)
from uav_vision.target_selection_policy import (
    candidate_is_currently_selectable,
    resolve_class_profile,
)
from uav_vision_eval.msg import SimTargetArray
from uav_vision_eval.vsim04_metrics import (
    REQUIRED_ARTIFACTS,
    STAGE_COUNT_FIELDS,
    STANDARD_CLASSES,
    VISUAL_ONLY_INTERRUPT_REASON,
    annotate_motion_frames,
    annotate_target_lateral_frames,
    CONFIRMED_STATE,
    candidate_audit_observation,
    completed_sources_cover,
    correlate_admission_events,
    detector_diagnostic_errors,
    load_trial_matrix,
    motion_frame_sort_key,
    navigation_metrics_metadata,
    navigation_drain_ready,
    planned_trial_result,
    quaternion_yaw,
    select_trial_matrix,
    trial_output_drain_boundary,
    watermarks_cover_source_stamp,
    write_artifacts,
)
from uav_vision_eval.stamped_pose_buffer import StampedPoseBuffer


UNKNOWN_VALUES = {"", "unknown", "unspecified", "none", "null"}
EXPECTED_TRIAL_COUNT = 23


class VSim04TrialRecorder:
    def __init__(self):
        rospy.init_node("vsim04_trial_recorder")
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._matrix_path = os.path.abspath(rospy.get_param("~matrix_file"))
        self._matrix = select_trial_matrix(
            load_trial_matrix(self._matrix_path),
            rospy.get_param("~trial_selector", ""),
            rospy.get_param("~trial_slice", ""))
        self._evaluation_scope = self._matrix["evaluation_scope"]
        self._dynamic_zero_visibility_policy = self._matrix[
            "dynamic_zero_visibility_policy"]
        self._expected_trial_count = len(self._matrix["trials"])
        self._formal_expected_trial_count = int(self._matrix.get(
            "formal_expected_trial_count", self._expected_trial_count))
        self._scenario_path = os.path.abspath(rospy.get_param(
            "~scenario_file"))
        with open(self._scenario_path, "r", encoding="utf-8") as stream:
            self._scenario = yaml.safe_load(stream)
        if not isinstance(self._scenario, dict):
            raise ValueError("scenario_file must contain a mapping")
        self._target_catalog_path = os.path.abspath(rospy.get_param(
            "~target_catalog_file"))
        with open(self._target_catalog_path, "r", encoding="utf-8") as stream:
            self._target_catalog = yaml.safe_load(stream)
        if not isinstance(self._target_catalog, dict):
            raise ValueError("target_catalog_file must contain a mapping")
        self._trial_specs = {
            trial["trial_id"]: trial for trial in self._matrix["trials"]}
        self._results = OrderedDict(
            (trial["trial_id"], planned_trial_result(trial))
            for trial in self._matrix["trials"])
        self._anchors = self._matrix["target_anchors"]
        self._expected_target_ids = {
            anchor["target_id"] for anchor in self._anchors.values()}
        self._profile, self._allowed_classes = resolve_class_profile(
            rospy.get_param("~class_profile", self._matrix["class_profile"]))
        if self._profile != self._matrix["class_profile"]:
            raise ValueError("recorder class_profile differs from matrix")

        self._confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self._selected_max_age = float(
            rospy.get_param("~selected_max_age", 0.5))
        self._priorities = {
            class_name: float(rospy.get_param(
                "/target_memory/priority_{}".format(class_name), default))
            for class_name, default in {
                "red_cross": 10.0,
                "tank": 5.0,
                "panzer": 2.5,
                "bridge": 2.0,
                "pillbox": 1.5,
                "tent": 1.0,
            }.items()
        }

        self._output_dir = os.path.abspath(rospy.get_param("~output_dir"))
        self._status_topic = rospy.get_param(
            "~status_topic", "/uav_vision_eval/vsim04/status")
        self._active = None
        self._run_complete = False
        self._fatal_error = ""
        self._finalized = False
        self._final_success = False
        self._final_summary_status = "INCOMPLETE"
        self._abort_event_seq = None
        self._terminal_context = {
            "run_complete": False,
            "expected_trial_count": self._expected_trial_count,
            "formal_expected_trial_count": self._formal_expected_trial_count,
            "evaluation_scope": self._evaluation_scope,
            "validation_errors": [],
        }

        self._event_seq = 0
        self._status_seq = 0
        self._events = []
        self._navigation_decisions = []
        self._navigation_results = []
        self._frames = OrderedDict()
        self._truth_centers = {}
        self._mapped_points = {}
        self._candidate_events = {trial_id: [] for trial_id in self._trial_specs}
        self._selected_events = {trial_id: [] for trial_id in self._trial_specs}
        self._candidate_audit_observations = []
        self._image_receipts = OrderedDict()
        self._detector_callback_starts = OrderedDict()
        self._image_stamps = set()
        self._last_image_stamp = None
        self._last_image_receipt = None
        self._consecutive_images = 0
        self._image_count = 0
        self._first_image_stamp_sec = None
        self._first_image_receipt = None
        self._camera_info = None
        self._truth_seen = False
        self._last_truth_receipt = None
        self._mapped_seen = False
        self._last_mapped_receipt = None
        self._last_mapped_completed_sources = []
        self._partial_mapped_frame_count = 0
        self._last_partial_mapped_sources = []
        self._mapped_bucket_status = {}
        self._targets_seen = False
        self._last_targets_receipt = None
        self._last_truth_source_stamp = None
        self._last_mapped_source_stamp = None
        self._last_targets_source_stamp = None
        self._last_perf_source_stamp = None
        self._last_perf_receipt = None
        self._perf_healthy = False
        self._perf_status = None
        self._expected_perf_name = rospy.get_param(
            "~expected_perf_name", "uav_vision/target_detector")
        self._expected_detector_backend = rospy.get_param(
            "~expected_detector_backend", "ultralytics")
        self._required_completed_sources = {
            str(value).strip() for value in rospy.get_param(
                "~required_completed_sources",
                ["target_detector", "circle_detector", "cross_detector"])
            if str(value).strip()
        }
        self._enable_stage_trace = bool(rospy.get_param(
            "~enable_stage_trace", True))
        self._heartbeat_timeout = float(rospy.get_param(
            "~heartbeat_timeout_sec", 2.0))
        self._output_drain_timeout = float(rospy.get_param(
            "~output_drain_timeout_sec", 10.0))
        self._output_quiet_sec = float(rospy.get_param(
            "~output_quiet_sec", 0.25))
        self._status_period = float(rospy.get_param(
            "~status_period_sec", 0.25))
        self._actual_trajectories = {}
        self._camera_pose_frame = str(rospy.get_param(
            "~camera_pose_frame", "world")).strip()
        self._camera_pose_buffer = StampedPoseBuffer(rospy.get_param(
            "~camera_pose_history_length", 4096))
        self._camera_pose_samples = {}
        self._infra_gaps = []
        self._infra_gap_keys = set()
        self._candidate_audit_keys = set()
        self._candidate_event_keys = set()
        self._selected_event_keys = set()
        self._pending_trial_end = None
        self._navigation_metrics = navigation_metrics_metadata(
            rospy.get_param("~navigation_metrics_mode", "visual_only"))
        self._navigation_decision_topic = str(rospy.get_param(
            "~navigation_decision_topic",
            "/navigation/mission_command_raw")).strip()
        self._navigation_result_topic = str(rospy.get_param(
            "~navigation_result_topic",
            "/navigation/mission_result")).strip()
        self._navigation_late_result_quiet_sec = float(rospy.get_param(
            "~navigation_late_result_quiet_sec", 0.5))
        self._navigation_late_result_timeout_sec = float(rospy.get_param(
            "~navigation_late_result_timeout_sec", 3.0))
        self._last_navigation_receipt = None
        self._pending_navigation_finalize = None
        self._navigation_callbacks_frozen = False
        if self._navigation_metrics["mode"] != "visual_only" and (
                not self._navigation_decision_topic or
                not self._navigation_result_topic):
            raise ValueError(
                "typed navigation metrics require both navigation topics")

        model_value = str(rospy.get_param("~model_path", "")).strip()
        model_path = (os.path.abspath(os.path.expanduser(model_value))
                      if model_value else "")
        self._require_model_path = bool(
            rospy.get_param("~require_model_path", True))
        camera_rpy = [float(value) for value in rospy.get_param(
            "~camera_rpy", self._matrix.get("runner", {}).get(
                "camera_rpy", [0.0, math.pi / 2.0, 0.0]))]
        self._manifest = {
            "seed": self._matrix["seed"],
            "class_profile": self._profile,
            "matrix_file": self._matrix_path,
            "scenario_file": self._scenario_path,
            "scenario": copy.deepcopy(self._scenario),
            "world_file": os.path.abspath(rospy.get_param("~world_file", "")),
            "trials": self._matrix["trials"],
            "target_anchors": copy.deepcopy(self._anchors),
            "target_catalog": self._active_target_catalog(),
            "model": {
                "path": model_path,
                "backend": rospy.get_param("~model_backend", "dev_sim"),
            },
            "thresholds": {
                "confirm_frames": self._confirm_frames,
                "selected_max_age_sec": self._selected_max_age,
                "detector_class_confidence": rospy.get_param(
                    "/target_detector/conf_threshold", None),
                "detector_imgsz": rospy.get_param(
                    "/target_detector/imgsz", None),
                "standard_class_confidence": rospy.get_param(
                    "/target_memory/std_class_confidence", None),
                "standard_geometry_confidence": rospy.get_param(
                    "/target_memory/std_geometry_confidence", None),
                "cross_class_confidence": rospy.get_param(
                    "/target_memory/cross_class_confidence", None),
                "cross_geometry_confidence": rospy.get_param(
                    "/target_memory/cross_geometry_confidence", None),
                "allow_latest_tf_fallback": bool(rospy.get_param(
                    "~allow_latest_tf_fallback", True)),
                "max_latest_tf_age_sec": float(rospy.get_param(
                    "~max_latest_tf_age_sec", 0.10)),
                "heartbeat_timeout_sec": self._heartbeat_timeout,
                "output_drain_timeout_sec": self._output_drain_timeout,
                "output_quiet_sec": self._output_quiet_sec,
                "navigation_late_result_quiet_sec":
                    self._navigation_late_result_quiet_sec,
                "navigation_late_result_timeout_sec":
                    self._navigation_late_result_timeout_sec,
                "status_period_sec": self._status_period,
                "priorities": dict(self._priorities),
            },
            "camera_info": None,
            "camera": {
                "model_name": rospy.get_param(
                    "~camera_model", self._matrix.get("runner", {}).get(
                        "camera_model", "vision_eval_camera")),
                "rpy": camera_rpy,
                "pose_frame": self._camera_pose_frame,
                "pose_matching": "latest_at_or_before_image_stamp",
            },
            "extrinsic": {
                "profile": rospy.get_param("~extrinsic_profile", ""),
                "source": os.path.abspath(rospy.get_param(
                    "~extrinsic_source", "")),
            },
            "revisions": {
                "vision": rospy.get_param("~vision_revision", "unknown"),
                "navigation": rospy.get_param(
                    "~navigation_revision", "unknown"),
            },
            "trajectory": {
                "configuration": copy.deepcopy(self._matrix.get("runner", {})),
                "dynamic_configuration": copy.deepcopy(
                    self._matrix.get("dynamic", {})),
                "d50_configuration": {
                    "matrix_kind": self._matrix.get("matrix_kind", ""),
                    "camera": copy.deepcopy(self._matrix.get("camera", {})),
                    "motion": copy.deepcopy(self._matrix.get("motion", {})),
                },
                "actual_trials": {},
            },
            "evaluation_design": {
                "design_id": str(self._matrix.get(
                    "design_id", "formal23")),
                "scope": self._evaluation_scope,
                "trial_selector": list(self._matrix["trial_selector"]),
                "trial_slice": self._matrix.get("trial_slice", ""),
                "dynamic_zero_visibility_policy":
                    self._dynamic_zero_visibility_policy,
                "all_targets_coexist": True,
                "mode": "clutter",
                "score_false_positives": bool(
                    self._scenario.get("score_false_positives", False)),
                "note": (
                    "The seed-11 minimum matrix records co-visible classes; "
                    "every confirmed/selected class is audited even when it "
                    "does not contribute to the expected-class trial score."),
            },
            "performance_contract": copy.deepcopy(
                self._matrix.get("performance_contract", {})),
            "vision_pipeline": {
                "required_mapped_completed_sources": sorted(
                    self._required_completed_sources),
                "stage_trace_enabled": self._enable_stage_trace,
            },
            "navigation_metrics": {
                **copy.deepcopy(self._navigation_metrics),
                "exact_binding_key": [
                    "mission_id", "decision_seq", "target_id",
                    "target_first_seen", "attempt", "payload_slot"],
                "trial_binding_field": "target_observation_stamp",
            },
            "topics": {
                "trial_event": rospy.get_param(
                    "~trial_event_topic",
                    "/uav_vision_eval/vsim04/trial_event"),
                "status": self._status_topic,
                "truth": rospy.get_param(
                    "~truth_topic", "/uav_vision_eval/ground_truth"),
                "detections_mapped": rospy.get_param(
                    "~detections_topic", "/uav_vision/detections_mapped"),
                "detections_raw": rospy.get_param(
                    "~raw_detections_topic", "/uav_vision/detections"),
                "detections_resolved": rospy.get_param(
                    "~resolved_detections_topic",
                    "/uav_vision/detections_resolved"),
                "detections_refined": rospy.get_param(
                    "~refined_detections_topic",
                    "/uav_vision/detections_refined"),
                "targets": rospy.get_param(
                    "~targets_topic", "/uav_vision/targets"),
                "selected_target": rospy.get_param(
                    "~selected_topic", "/uav_vision/selected_target"),
                "image": rospy.get_param(
                    "~image_topic", "/camera/color/image_raw"),
                "camera_info": rospy.get_param(
                    "~camera_info_topic", "/camera/color/camera_info"),
                "camera_pose": rospy.get_param(
                    "~camera_pose_topic",
                    "/uav_vision_eval/camera_pose"),
                "perf": rospy.get_param(
                    "~perf_topic", "/uav_vision/perf"),
                "navigation_decision": self._navigation_decision_topic,
                "navigation_result": self._navigation_result_topic,
            },
        }
        static_errors = self._static_manifest_errors()
        if static_errors:
            self._fatal_error = "static_manifest_invalid: " + "; ".join(
                static_errors)

        self._status_pub = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(
            self._manifest["topics"]["trial_event"], String,
            self._on_trial_event, queue_size=40)
        rospy.Subscriber(
            self._manifest["topics"]["truth"], SimTargetArray,
            self._on_truth, queue_size=40)
        rospy.Subscriber(
            self._manifest["topics"]["detections_mapped"],
            TargetDetectionArray, self._on_detections, queue_size=40)
        if self._enable_stage_trace:
            rospy.Subscriber(
                self._manifest["topics"]["detections_raw"],
                TargetDetectionArray, self._on_raw_detections, queue_size=40)
            rospy.Subscriber(
                self._manifest["topics"]["detections_resolved"],
                TargetDetectionArray, self._on_resolved_detections,
                queue_size=40)
            rospy.Subscriber(
                self._manifest["topics"]["detections_refined"],
                TargetDetectionArray, self._on_refined_detections,
                queue_size=40)
        rospy.Subscriber(
            self._manifest["topics"]["targets"], TargetCandidateArray,
            self._on_targets, queue_size=1)
        rospy.Subscriber(
            self._manifest["topics"]["selected_target"], TargetCandidate,
            self._on_selected, queue_size=1)
        rospy.Subscriber(
            self._manifest["topics"]["image"], Image,
            self._on_image, queue_size=1)
        rospy.Subscriber(
            self._manifest["topics"]["camera_info"], CameraInfo,
            self._on_camera_info, queue_size=1)
        rospy.Subscriber(
            self._manifest["topics"]["camera_pose"], PoseStamped,
            self._on_camera_pose, queue_size=40)
        rospy.Subscriber(
            self._manifest["topics"]["perf"], DiagnosticArray,
            self._on_perf, queue_size=10)
        if self._navigation_metrics["mode"] != "visual_only":
            try:
                from uav_mission.msg import (  # pylint: disable=import-outside-toplevel
                    NavigationDecision, NavigationResult)
            except ImportError as error:
                raise RuntimeError(
                    "typed navigation metrics require uav_mission messages"
                ) from error
            rospy.Subscriber(
                self._navigation_decision_topic, NavigationDecision,
                self._on_navigation_decision, queue_size=40)
            rospy.Subscriber(
                self._navigation_result_topic, NavigationResult,
                self._on_navigation_result, queue_size=40)
        self._timer = rospy.Timer(
            rospy.Duration(self._status_period), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)
        self._write_artifacts_safe()
        self._publish_status()
        rospy.loginfo(
            "[VSim04Recorder] waiting for preflight profile=%s trials=%d output=%s",
            self._profile, len(self._trial_specs), self._output_dir)

    def _active_target_catalog(self):
        entries = [copy.deepcopy(target)
                   for target in self._target_catalog.get("targets", [])
                   if target.get("target_id") in self._expected_target_ids]
        return {
            "source": self._target_catalog_path,
            "world_frame": self._target_catalog.get("world_frame", ""),
            "camera_link_suffix": self._target_catalog.get(
                "camera_link_suffix", ""),
            "active_targets": entries,
        }

    def _static_manifest_errors(self):
        errors = []
        model_path = self._manifest["model"]["path"]
        if self._require_model_path and not os.path.isfile(model_path):
            errors.append("model_path_missing_or_not_file")
        if str(self._manifest["model"]["backend"]).strip().lower() in UNKNOWN_VALUES:
            errors.append("model_backend_missing")
        if (str(self._manifest["model"]["backend"]) !=
                str(self._expected_detector_backend)):
            errors.append("model_backend_diagnostic_expectation_mismatch")
        if str(self._expected_perf_name).strip().lower() in UNKNOWN_VALUES:
            errors.append("expected_perf_name_missing")
        if (not self._required_completed_sources or
                "target_detector" not in self._required_completed_sources):
            errors.append("required_completed_sources_invalid")
        world_file = self._manifest["world_file"]
        if not world_file or not os.path.isfile(world_file):
            errors.append("world_file_missing_or_not_file")
        if not os.path.isfile(self._scenario_path):
            errors.append("scenario_file_missing_or_not_file")
        if (not self._target_catalog_path or
                not os.path.isfile(self._target_catalog_path)):
            errors.append("target_catalog_file_missing_or_not_file")
        if (self._evaluation_scope == "full" and
                len(self._results) != self._formal_expected_trial_count):
            errors.append("trial_count_{}/{}".format(
                len(self._results), self._formal_expected_trial_count))
        elif len(self._results) <= 0:
            errors.append("diagnostic_trial_count_must_be_positive")
        active_targets = set(self._scenario.get("active_targets", []))
        if active_targets != self._expected_target_ids:
            errors.append("scenario_active_targets_do_not_match_matrix")
        active_catalog = self._manifest["target_catalog"]["active_targets"]
        catalog_ids = {target.get("target_id") for target in active_catalog}
        if catalog_ids != self._expected_target_ids:
            errors.append("target_catalog_ids_do_not_match_matrix")
        catalog_by_id = {target.get("target_id"): target
                         for target in active_catalog}
        for class_name, anchor in self._anchors.items():
            target = catalog_by_id.get(anchor.get("target_id"), {})
            if target.get("class_name") != class_name:
                errors.append("target_catalog_class_mismatch:" + class_name)
            fallback = target.get("fallback_center_world", [])
            xyz = anchor.get("xyz", [])
            try:
                finite_anchor = (
                    len(fallback) == 3 and len(xyz) == 3 and
                    all(math.isfinite(float(value))
                        for value in list(fallback) + list(xyz)))
            except (TypeError, ValueError, OverflowError):
                finite_anchor = False
            if not finite_anchor:
                errors.append("target_anchor_invalid:" + class_name)
            elif any(abs(float(left) - float(right)) > 1.0e-6
                     for left, right in zip(fallback, xyz)):
                errors.append("target_anchor_catalog_mismatch:" + class_name)
            if not str(target.get("gazebo_link_name", "")).strip():
                errors.append("target_gazebo_link_missing:" + class_name)
        camera = self._manifest["camera"]
        if not str(camera["model_name"]).strip():
            errors.append("camera_model_missing")
        if not str(camera["pose_frame"]).strip():
            errors.append("camera_pose_frame_missing")
        if (len(camera["rpy"]) != 3 or
                not all(math.isfinite(value) for value in camera["rpy"])):
            errors.append("camera_rpy_invalid")
        extrinsic = self._manifest["extrinsic"]
        if str(extrinsic["profile"]).strip().lower() in UNKNOWN_VALUES:
            errors.append("extrinsic_profile_missing")
        if not extrinsic["source"] or not os.path.isfile(extrinsic["source"]):
            errors.append("extrinsic_source_missing_or_not_file")
        for name, revision in self._manifest["revisions"].items():
            revision_text = str(revision).strip()
            if revision_text.lower() in UNKNOWN_VALUES:
                errors.append("{}_revision_missing".format(name))
            elif re.search(r"(?i)(?<![0-9a-f])[0-9a-f]{7,40}(?![0-9a-f])",
                           revision_text) is None:
                errors.append("{}_revision_not_git_commit".format(name))
        thresholds = self._manifest["thresholds"]
        for name in (
                "confirm_frames", "selected_max_age_sec",
                "detector_class_confidence", "standard_class_confidence",
                "standard_geometry_confidence", "cross_class_confidence",
                "cross_geometry_confidence", "max_latest_tf_age_sec",
                "heartbeat_timeout_sec", "output_drain_timeout_sec",
                "output_quiet_sec", "navigation_late_result_quiet_sec",
                "navigation_late_result_timeout_sec", "status_period_sec"):
            value = thresholds.get(name)
            try:
                valid = value is not None and math.isfinite(float(value))
            except (TypeError, ValueError):
                valid = False
            if not valid:
                errors.append("threshold_{}_invalid".format(name))
        if int(thresholds.get("confirm_frames", 0) or 0) < 1:
            errors.append("threshold_confirm_frames_out_of_range")
        if float(thresholds.get("selected_max_age_sec", 0.0) or 0.0) <= 0.0:
            errors.append("threshold_selected_max_age_sec_out_of_range")
        if float(thresholds.get("heartbeat_timeout_sec", 0.0) or 0.0) <= 0.0:
            errors.append("threshold_heartbeat_timeout_sec_out_of_range")
        for name in (
                "output_drain_timeout_sec", "output_quiet_sec",
                "navigation_late_result_quiet_sec",
                "navigation_late_result_timeout_sec", "status_period_sec"):
            if float(thresholds.get(name, 0.0) or 0.0) <= 0.0:
                errors.append("threshold_{}_out_of_range".format(name))
        if (self._navigation_late_result_timeout_sec <
                self._navigation_late_result_quiet_sec):
            errors.append(
                "threshold_navigation_late_result_timeout_before_quiet")
        for name in (
                "detector_class_confidence", "standard_class_confidence",
                "standard_geometry_confidence", "cross_class_confidence",
                "cross_geometry_confidence"):
            value = thresholds.get(name)
            if value is not None and not 0.0 <= float(value) <= 1.0:
                errors.append("threshold_{}_out_of_range".format(name))
        if not all(math.isfinite(float(value))
                   for value in thresholds.get("priorities", {}).values()):
            errors.append("priority_threshold_invalid")
        if self._manifest["evaluation_design"]["score_false_positives"]:
            errors.append("false_positive_scoring_not_implemented")
        return errors

    @staticmethod
    def _stamp_sec(message):
        return message.header.stamp.to_sec()

    @staticmethod
    def _stamp_key(message):
        return message.header.stamp.to_nsec()

    @staticmethod
    def _point_finite(point):
        return all(math.isfinite(float(getattr(point, axis)))
                   for axis in ("x", "y", "z"))

    def _result_locked(self):
        return self._results[self._active] if self._active else None

    @staticmethod
    def _pose_values_finite(pose):
        return all(math.isfinite(float(value)) for value in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w))

    def _camera_pose_sample_locked(self, stamp_key):
        existing = self._camera_pose_samples.get(stamp_key)
        if existing and existing.get("camera_pose_valid"):
            return existing
        stamp = rospy.Time(
            secs=int(stamp_key // 1000000000),
            nsecs=int(stamp_key % 1000000000))
        message, age_sec = self._camera_pose_buffer.at_or_before(stamp)
        sample = {
            "camera_pose_valid": False,
            "camera_pose_source_stamp": "",
            "camera_pose_age_sec": "",
            "camera_position_x_m": "",
            "camera_position_y_m": "",
            "camera_position_z_m": "",
            "camera_yaw_rad": "",
            "camera_pose_invalid_reason": "camera_pose_missing",
        }
        if message is None:
            self._camera_pose_samples[stamp_key] = sample
            return sample
        if message.header.frame_id != self._camera_pose_frame:
            sample["camera_pose_invalid_reason"] = "camera_pose_frame_mismatch"
            self._camera_pose_samples[stamp_key] = sample
            return sample
        if not self._pose_values_finite(message.pose):
            sample["camera_pose_invalid_reason"] = "camera_pose_nonfinite"
            self._camera_pose_samples[stamp_key] = sample
            return sample
        yaw = quaternion_yaw(
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w)
        if yaw is None or age_sec is None or not math.isfinite(age_sec):
            sample["camera_pose_invalid_reason"] = "camera_pose_invalid"
            self._camera_pose_samples[stamp_key] = sample
            return sample
        sample.update({
            "camera_pose_valid": True,
            "camera_pose_source_stamp": "{:.9f}".format(
                message.header.stamp.to_sec()),
            "camera_pose_age_sec": float(age_sec),
            "camera_position_x_m": float(message.pose.position.x),
            "camera_position_y_m": float(message.pose.position.y),
            "camera_position_z_m": float(message.pose.position.z),
            "camera_yaw_rad": yaw,
            "camera_pose_invalid_reason": "",
        })
        self._camera_pose_samples[stamp_key] = sample
        return sample

    def _apply_camera_pose_locked(self, frame, stamp_key):
        frame.update(self._camera_pose_sample_locked(stamp_key))

    def _frame_locked(self, stamp_key, stamp_sec):
        key = (self._active, stamp_key)
        if key not in self._frames:
            spec = self._trial_specs[self._active]
            anchor = self._anchors[spec["class_name"]]
            self._frames[key] = {
                "trial_id": self._active,
                "stamp": "{:.9f}".format(stamp_sec),
                "target_id": anchor["target_id"],
                "class_name": spec["class_name"],
                "fully_in_frame": False,
                "center_in_frame": False,
                "co_visible_classes": "",
                "camera_pose_valid": False,
                "camera_pose_source_stamp": "",
                "camera_pose_age_sec": "",
                "camera_position_x_m": "",
                "camera_position_y_m": "",
                "camera_position_z_m": "",
                "camera_yaw_rad": "",
                "camera_pose_invalid_reason": "camera_pose_missing",
                "motion_delta_valid": False,
                "actual_linear_speed_mps": "",
                "actual_yaw_rate_radps": "",
                "motion_invalid_reason": "not_derived",
                "path_lateral_offset_m": "",
                "path_lateral_offset_normalized": "",
                "path_lateral_invalid_reason": "not_derived",
                "visibility_profile": str(spec.get(
                    "visibility_profile", "full")),
                "visibility_eligible": False,
                "projection_valid": False,
                "truth_world_x_m": "",
                "truth_world_y_m": "",
                "truth_pixel_u": "",
                "truth_pixel_v": "",
                "target_path_lateral_offset_m": "",
                "target_pixel_offset_x_normalized": "",
                "target_pixel_offset_y_normalized": "",
                "design_kind": spec.get("design_kind", ""),
                "relative_angle_deg": spec.get("relative_angle_deg", ""),
                "motion_profile": spec.get("motion_profile", ""),
                "framing": spec.get("framing", ""),
                "raw_class_present": False,
                "raw_geometry_present": False,
                "raw_class_confidence": "",
                "raw_geometry_confidence": "",
                "resolved_present": False,
                "resolved_class_confidence": "",
                "resolved_geometry_confidence": "",
                "refined_present": False,
                "geometry_verified": False,
                "center_refined": False,
                "association_valid": False,
                "refined_class_confidence": "",
                "refined_geometry_confidence": "",
                "refined_reject_reason": "",
                "detection_present": False,
                "mapped_class_confidence": "",
                "mapped_geometry_confidence": "",
                "map_valid": False,
                "transform_failure": False,
                "reject_reason": "",
                "map_error_xy": "",
                "detector_inference_ms": "",
                "detector_processing_ms": "",
                "detector_callback_start_monotonic_sec": "",
                "detector_callback_end_monotonic_sec": "",
                "detector_perf_receipt_monotonic_sec": "",
                "current_confirmed": False,
                "current_selected": False,
                "stable_id": "",
            }
        self._apply_camera_pose_locked(self._frames[key], stamp_key)
        return self._frames[key]

    def _add_event_locked(self, event, source_stamp=None, stable_id=None,
                          details=None, receipt_monotonic=None,
                          class_name=None):
        self._event_seq += 1
        spec = self._trial_specs.get(self._active, {})
        self._events.append({
            "event_seq": self._event_seq,
            "trial_id": self._active or "",
            "event": event,
            "source_stamp": (
                "{:.9f}".format(float(source_stamp))
                if source_stamp is not None else ""),
            "monotonic_sec": "{:.9f}".format(
                receipt_monotonic if receipt_monotonic is not None
                else time.monotonic()),
            "class_name": (spec.get("class_name", "") if class_name is None
                           else str(class_name)),
            "stable_id": "" if stable_id is None else int(stable_id),
            "details": json.dumps(details or {}, sort_keys=True),
        })

    def _record_infra_gap_locked(self, chain, gap_sec=None, details=None):
        if not self._active:
            return
        key = (self._active, str(chain))
        if key in self._infra_gap_keys:
            return
        self._infra_gap_keys.add(key)
        gap = {
            "trial_id": self._active,
            "chain": str(chain),
            "gap_sec": gap_sec,
            "monotonic_sec": time.monotonic(),
            "details": copy.deepcopy(details or {}),
        }
        self._infra_gaps.append(gap)
        self._add_event_locked(
            "infrastructure_gap", details=gap,
            receipt_monotonic=gap["monotonic_sec"])

    def _note_receipt_gap_locked(self, chain, previous, current):
        result = self._result_locked()
        observation_active = (
            result and result.get("entered_visibility_window") and
            not result.get("left_visibility_window"))
        if (observation_active and previous is not None and
                current - previous > self._heartbeat_timeout):
            self._record_infra_gap_locked(
                chain, current - previous,
                {"heartbeat_timeout_sec": self._heartbeat_timeout})

    def _record_current_missing_locked(self):
        if not self._active:
            return
        result = self._result_locked()
        if (not result.get("entered_visibility_window") or
                result.get("left_visibility_window")):
            return
        for missing in self._readiness_missing_locked():
            self._record_infra_gap_locked(
                missing, None,
                {"heartbeat_timeout_sec": self._heartbeat_timeout})

    def _note_pending_output_locked(self, source_stamp, receipt):
        if self._pending_trial_end is None:
            return
        output_boundary = self._pending_trial_end.get(
            "output_boundary_stamp")
        if (output_boundary is not None and source_stamp is not None and
                math.isfinite(float(source_stamp)) and
                float(source_stamp) <= float(output_boundary)):
            self._pending_trial_end["last_relevant_receipt"] = receipt

    def _readiness_missing_locked(self):
        missing = []
        now = time.monotonic()
        if self._camera_info is None:
            missing.append("camera_info")
        elif not self._camera_info_valid(self._camera_info):
            missing.append("camera_info_invalid")
        if (self._consecutive_images < 3 or
                self._last_image_receipt is None or
                now - self._last_image_receipt > self._heartbeat_timeout):
            missing.append("three_consecutive_images")
        if (not self._truth_seen or self._last_truth_receipt is None or
                now - self._last_truth_receipt > self._heartbeat_timeout):
            missing.append("truth_catalog_poses")
        if (not self._mapped_seen or self._last_mapped_receipt is None or
                now - self._last_mapped_receipt > self._heartbeat_timeout):
            missing.append("mapped_detections_heartbeat")
        if (not self._targets_seen or self._last_targets_receipt is None or
                now - self._last_targets_receipt > self._heartbeat_timeout):
            missing.append("targets_heartbeat")
        if (self._last_perf_receipt is None or
                now - self._last_perf_receipt > self._heartbeat_timeout):
            missing.append("target_detector_diagnostic_heartbeat")
        elif not self._perf_healthy:
            missing.append("target_detector_diagnostic_not_ok")
        return missing

    def _status_payload_locked(self):
        self._status_seq += 1
        missing = self._readiness_missing_locked()
        if self._finalized and self._final_success:
            state = "FINALIZED"
        elif self._fatal_error:
            state = "FAIL"
        elif self._finalized:
            state = "FAIL"
        elif self._run_complete:
            state = "FINALIZING"
        elif self._active:
            state = "RUNNING"
        elif not missing:
            state = "READY"
        else:
            state = "WAITING"
        return {
            "schema_version": 1,
            "evaluation_id": "V-SIM-04",
            "status_seq": self._status_seq,
            "state": state,
            "ready": (not missing and not self._fatal_error and
                      not self._run_complete and not self._finalized),
            "success": self._final_success if self._finalized else False,
            "finalized": self._finalized,
            "summary_status": self._final_summary_status,
            "missing": missing,
            "error": self._fatal_error,
            "active_trial": self._active or "",
            "completed_trial_count": sum(
                result.get("status") == "completed"
                for result in self._results.values()),
            "expected_trial_count": self._expected_trial_count,
            "infra_gap_count": len(self._infra_gaps),
            "pending_trial_end": bool(self._pending_trial_end),
            "pending_navigation_finalize": bool(
                self._pending_navigation_finalize),
            "abort_event_seq": self._abort_event_seq,
            "stamp": rospy.Time.now().to_sec(),
            "monotonic_sec": time.monotonic(),
        }

    def _publish_status(self):
        with self._lock:
            payload = self._status_payload_locked()
        self._status_pub.publish(String(data=json.dumps(
            payload, sort_keys=True)))

    def _on_trial_event(self, message):
        write_after = False
        finalize_after = False
        try:
            event = json.loads(message.data)
            event_name = event["event"]
            with self._lock:
                if self._finalized or self._run_complete:
                    rospy.logwarn(
                        "ignoring V-SIM-04 event after terminal state: %s",
                        event_name)
                    return
                if event_name == "trial_start":
                    self._start_trial_locked(event["trial_id"], event)
                elif event_name == "trial_end":
                    self._request_finish_trial_locked(
                        event["trial_id"], event)
                elif event_name == "run_complete":
                    if self._active:
                        raise RuntimeError("run_complete while trial is active")
                    self._run_complete = True
                    self._add_event_locked("run_complete", details=event)
                    if self._navigation_metrics["mode"] == "visual_only":
                        self._navigation_callbacks_frozen = True
                        finalize_after = True
                    else:
                        requested = time.monotonic()
                        self._pending_navigation_finalize = {
                            "requested_monotonic": requested,
                            "last_receipt_at_request":
                                self._last_navigation_receipt,
                        }
                elif event_name == "run_abort":
                    aborted_trial = self._active
                    if aborted_trial:
                        self._results[aborted_trial]["status"] = "incomplete"
                    self._fatal_error = "runner_abort: {}".format(
                        event.get("error", "unknown"))
                    self._abort_event_seq = int(event.get("event_seq", -1))
                    self._add_event_locked("run_abort", details=event)
                    self._active = None
                    self._pending_trial_end = None
                    self._terminal_context = {
                        "run_complete": False,
                        "expected_trial_count": self._expected_trial_count,
                        "formal_expected_trial_count":
                            self._formal_expected_trial_count,
                        "evaluation_scope": self._evaluation_scope,
                        "validation_errors": [self._fatal_error],
                    }
                    write_after = True
                else:
                    raise ValueError("unknown trial event: " + str(event_name))
        except Exception as error:
            with self._lock:
                self._fatal_error = "trial_event_rejected: {}".format(error)
                self._add_event_locked(
                    "recorder_failure", details={"error": str(error)})
            rospy.logerr("V-SIM-04 trial event rejected: %s", error)
            self._publish_status()
            return

        if write_after:
            self._write_artifacts_safe()
        if finalize_after:
            self._finalize_run()
        self._publish_status()

    def _start_trial_locked(self, trial_id, source_event):
        if self._fatal_error or self._finalized or self._run_complete:
            raise RuntimeError("trial_start after terminal recorder state")
        if self._readiness_missing_locked():
            raise RuntimeError("trial_start before recorder READY")
        if trial_id not in self._trial_specs:
            raise ValueError("unknown trial_id: " + str(trial_id))
        if self._active:
            raise RuntimeError("trial overlap: {} -> {}".format(
                self._active, trial_id))
        if self._results[trial_id].get("status") != "planned":
            raise RuntimeError("trial repeated: " + trial_id)
        self._active = trial_id
        result = self._results[trial_id]
        partial_visibility = (
            str(result.get("visibility_profile", "full")) == "partial")
        result.update({
            "status": "running",
            "p_confirm": None if partial_visibility else False,
            "p_selected": None if partial_visibility else False,
            "p_confirm_visibility": False,
            "p_selected_visibility": False,
            "p_interrupt": None,
            "p_decision": None,
            "p_dispatch": None,
            "p_planner_arrival": None,
            "p_interrupt_reason": (
                VISUAL_ONLY_INTERRUPT_REASON
                if self._navigation_metrics["mode"] == "visual_only" else
                self._navigation_metrics["reason"]),
            "navigation_metrics_mode": self._navigation_metrics["mode"],
            "navigation_target_stage_capability": self._navigation_metrics[
                "target_stage_capability"],
            "navigation_metrics_reason": self._navigation_metrics["reason"],
            "navigation_binding_keys": "[]",
            "navigation_validation_errors": "[]",
            "stable_id": None,
            "selected_target_first_seen_ns": None,
            "selected_target_observation_stamps_ns": [],
            "confirmation_exposure_sec": None,
            "confirmation_processing_ms": None,
            "confirmation_pipeline_ms": None,
            "stage_trace_enabled": self._enable_stage_trace,
            "complete_mapped_frames": 0,
            "partial_only_mapped_frames": 0,
            "partial_source_sets": "{}",
            "detector_inference_ms_samples": [],
            "detector_processing_ms_samples": [],
            "eligible_frames": 0,
            **{field: 0 for field in STAGE_COUNT_FIELDS},
            "detection_frames": 0,
            "map_valid_frames": 0,
            "tf_failure_frames": 0,
            "map_errors_xy": [],
            "entered_fully_in_frame": False,
            "left_fully_in_frame": False,
            "entered_visibility_window": False,
            "left_visibility_window": False,
            "enter_source_stamp": None,
            "leave_source_stamp": None,
            "enter_receipt_monotonic": None,
            "leave_receipt_monotonic": None,
            "expected_duration_sec": None,
            "actual_duration_sec": None,
            "expected_speed_mps": None,
            "actual_speed_mps": None,
            "camera_pose_frame_count": 0,
            "motion_sample_count": 0,
            "lateral_offset_sample_count": 0,
            "actual_linear_speed_mps_samples": [],
            "actual_yaw_rate_radps_samples": [],
            "normalized_lateral_offset_samples": [],
            "mean_actual_linear_speed_mps": None,
            "p95_actual_linear_speed_mps": None,
            "mean_abs_actual_yaw_rate_radps": None,
            "p95_abs_actual_yaw_rate_radps": None,
            "mean_abs_normalized_lateral_offset": None,
            "p95_abs_normalized_lateral_offset": None,
            "target_lateral_sample_count": 0,
            "target_path_lateral_offset_m_samples": [],
            "target_pixel_offset_x_normalized_samples": [],
            "mean_target_path_lateral_offset_m": None,
            "p95_abs_target_path_lateral_offset_m": None,
            "mean_target_pixel_offset_x_normalized": None,
            "p95_abs_target_pixel_offset_x_normalized": None,
        })
        self._add_event_locked("trial_start", details=source_event)

    def _request_finish_trial_locked(self, trial_id, source_event):
        if self._active != trial_id:
            raise RuntimeError("trial_end does not match active trial")
        if self._pending_trial_end is not None:
            raise RuntimeError("trial_end already pending: " + trial_id)
        self._record_current_missing_locked()
        output_boundary, output_boundary_kind = trial_output_drain_boundary(
            self._result_locked(), source_event)
        self._pending_trial_end = {
            "trial_id": trial_id,
            "source_event": copy.deepcopy(source_event),
            "requested_monotonic": time.monotonic(),
            "watermarks_ready_monotonic": None,
            "last_relevant_receipt": time.monotonic(),
            "output_boundary_stamp": output_boundary,
            "output_boundary_kind": output_boundary_kind,
        }
        self._add_event_locked(
            "trial_end_requested", details={
                "runner_event": source_event,
                "required_output_watermarks": [
                    "image", "truth", "mapped_detections", "targets",
                    "target_detector_diagnostic",
                ],
                "output_boundary_stamp": output_boundary,
                "output_boundary_kind": output_boundary_kind,
            })

    def _output_watermarks_locked(self):
        return {
            "image": (
                self._last_image_stamp / 1.0e9
                if self._last_image_stamp is not None else None),
            "truth": self._last_truth_source_stamp,
            "mapped_detections": self._last_mapped_source_stamp,
            "targets": self._last_targets_source_stamp,
            "target_detector_diagnostic": self._last_perf_source_stamp,
        }

    def _maybe_finish_pending_locked(self):
        if self._pending_trial_end is None:
            return False
        pending = self._pending_trial_end
        output_boundary = pending["output_boundary_stamp"]
        watermarks = self._output_watermarks_locked()
        ready = watermarks_cover_source_stamp(watermarks, output_boundary)
        now = time.monotonic()
        if ready:
            if pending["watermarks_ready_monotonic"] is None:
                pending["watermarks_ready_monotonic"] = now
                self._add_event_locked(
                    "output_watermarks_reached", output_boundary,
                    details={
                        "watermarks": watermarks,
                        "output_boundary_kind":
                            pending["output_boundary_kind"],
                    })
            quiet_since = max(
                pending["watermarks_ready_monotonic"],
                pending["last_relevant_receipt"])
            if (now - quiet_since >=
                    self._output_quiet_sec):
                self._finish_trial_locked(
                    pending["trial_id"], pending["source_event"])
                self._pending_trial_end = None
                return True
        elif now - pending["requested_monotonic"] > self._output_drain_timeout:
            self._record_infra_gap_locked(
                "output_watermark_timeout",
                now - pending["requested_monotonic"],
                {"output_boundary_stamp": output_boundary,
                 "output_boundary_kind": pending["output_boundary_kind"],
                 "watermarks": watermarks})
            self._fatal_error = "output_watermark_timeout:{}".format(
                pending["trial_id"])
        return False

    def _finish_trial_locked(self, trial_id, source_event):
        if self._active != trial_id:
            raise RuntimeError("trial_end does not match active trial")
        trajectory = source_event.get("trajectory", {})
        result = self._result_locked()
        result["expected_duration_sec"] = trajectory.get(
            "expected_duration_sec")
        result["actual_duration_sec"] = trajectory.get("actual_duration_sec")
        result["expected_speed_mps"] = trajectory.get("expected_speed_mps")
        result["actual_speed_mps"] = trajectory.get("actual_speed_mps")
        for field in (
                "lateral_bin", "visibility_profile",
                "requested_target_path_lateral_offset_m",
                "requested_pixel_offset_x_normalized",
                "design_kind", "relative_angle_deg", "motion_profile",
                "framing", "relative_angle_measurement",
                "planned_sample_count", "expected_primary_target_id"):
            if field in trajectory:
                result[field] = trajectory[field]
        self._actual_trajectories[trial_id] = copy.deepcopy(trajectory)
        self._derive_frame_metrics_locked(trial_id)
        self._derive_admission_metrics_locked(trial_id)
        result["status"] = "completed"
        self._add_event_locked("trial_end", details={
            "entered_fully_in_frame": result["entered_fully_in_frame"],
            "left_fully_in_frame": result["left_fully_in_frame"],
            "trajectory": trajectory,
        })
        self._active = None

    def _derive_frame_metrics_locked(self, trial_id):
        rows = []
        for (row_trial, stamp_key), row in self._frames.items():
            if row_trial != trial_id:
                continue
            self._apply_camera_pose_locked(row, stamp_key)
            rows.append(row)
        eligible = [row for row in rows if row["visibility_eligible"]]
        stage_row_fields = {
            "raw_class_frames": "raw_class_present",
            "raw_geometry_frames": "raw_geometry_present",
            "resolved_frames": "resolved_present",
            "refined_frames": "refined_present",
            "geometry_verified_frames": "geometry_verified",
            "association_valid_frames": "association_valid",
            "center_refined_frames": "center_refined",
        }
        detected = [row for row in eligible if row["detection_present"]]
        map_valid = [row for row in eligible if row["map_valid"]]
        tf_failures = [row for row in detected if row["transform_failure"]]
        errors = [float(row["map_error_xy"]) for row in eligible
                  if row["map_error_xy"] != ""]
        result = self._results[trial_id]
        result.update(annotate_motion_frames(
            rows, result["kind"],
            self._actual_trajectories.get(trial_id, {})))
        result.update(annotate_target_lateral_frames(
            rows, self._actual_trajectories.get(trial_id, {})))
        result["eligible_frames"] = len(eligible)
        for result_field, row_field in stage_row_fields.items():
            result[result_field] = sum(bool(row[row_field]) for row in eligible)
        result["detection_frames"] = len(detected)
        result["map_valid_frames"] = len(map_valid)
        result["tf_failure_frames"] = len(tf_failures)
        result["map_errors_xy"] = errors
        result["detector_inference_ms_samples"] = [
            float(row["detector_inference_ms"]) for row in eligible
            if row["detector_inference_ms"] != ""]
        result["detector_processing_ms_samples"] = [
            float(row["detector_processing_ms"]) for row in eligible
            if row["detector_processing_ms"] != ""]
        buckets = [
            value for (row_trial, _stamp), value
            in self._mapped_bucket_status.items()
            if row_trial == trial_id]
        result["complete_mapped_frames"] = sum(
            bool(value["complete"]) for value in buckets)
        partial_only = [value for value in buckets
                        if not value["complete"] and
                        value["partial_source_sets"]]
        result["partial_only_mapped_frames"] = len(partial_only)
        source_sets = Counter(
            "+".join(source_set) if source_set else "empty"
            for value in partial_only
            for source_set in value["partial_source_sets"])
        result["partial_source_sets"] = json.dumps(
            dict(sorted(source_sets.items())), sort_keys=True)

    def _derive_admission_metrics_locked(self, trial_id):
        result = self._results[trial_id]
        result.update(correlate_admission_events(
            self._candidate_events[trial_id],
            self._selected_events[trial_id], result,
            self._image_receipts, self._detector_callback_starts))

    def _on_camera_pose(self, message):
        if message.header.frame_id != self._camera_pose_frame:
            rospy.logerr_throttle(
                5.0, "V-SIM-04 camera pose frame mismatch: %s",
                message.header.frame_id)
            return
        with self._lock:
            self._camera_pose_buffer.add(message)

    def _on_image(self, message):
        receipt = time.monotonic()
        with self._lock:
            stamp_key = self._stamp_key(message)
            stamp_sec = self._stamp_sec(message)
            self._note_receipt_gap_locked(
                "image_heartbeat", self._last_image_receipt, receipt)
            self._image_receipts[stamp_key] = receipt
            self._image_stamps.add(stamp_sec)
            if self._first_image_receipt is None:
                self._first_image_receipt = receipt
                self._first_image_stamp_sec = stamp_sec
            self._image_count += 1
            stamp_delta = (
                (stamp_key - self._last_image_stamp) / 1.0e9
                if self._last_image_stamp is not None else None)
            if (stamp_delta is None or 0.0 < stamp_delta <= 0.5):
                self._consecutive_images += 1
            else:
                self._consecutive_images = 1
            self._last_image_stamp = stamp_key
            self._last_image_receipt = receipt
            key = (self._active, stamp_key)
            if self._active and key in self._frames:
                self._apply_camera_pose_locked(self._frames[key], stamp_key)
            self._note_pending_output_locked(stamp_sec, receipt)
            while len(self._image_receipts) > 4000:
                self._image_receipts.popitem(last=False)

    def _on_camera_info(self, message):
        candidate = {
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "K": [float(value) for value in message.K],
            "D": [float(value) for value in message.D],
            "P": [float(value) for value in message.P],
            "frame_id": message.header.frame_id,
        }
        with self._lock:
            if (self._camera_info is not None and
                    self._camera_info_valid(self._camera_info)):
                return
            self._camera_info = candidate

    @staticmethod
    def _camera_info_valid(camera_info):
        return bool(
            camera_info and camera_info["width"] > 0 and
            camera_info["height"] > 0 and
            str(camera_info["frame_id"]).strip() and
            len(camera_info["K"]) == 9 and
            len(camera_info["P"]) == 12 and
            camera_info["K"][0] > 0.0 and
            camera_info["K"][4] > 0.0 and
            all(math.isfinite(value) for field in ("K", "D", "P")
                for value in camera_info[field]))

    def _on_perf(self, message):
        receipt = time.monotonic()
        matching = [status for status in message.status
                    if status.name == self._expected_perf_name]
        if not matching:
            return
        status = matching[-1]
        values = {item.key: item.value for item in status.values}
        expected_model = self._manifest["model"]["path"]
        reasons = detector_diagnostic_errors(
            status.level, values, self._expected_detector_backend,
            expected_model)
        source_stamp = self._stamp_sec(message)
        with self._lock:
            stamp_key = self._stamp_key(message)
            callback_start = self._finite_diagnostic_value(
                values, "callback_start_monotonic_sec")
            callback_end = self._finite_diagnostic_value(
                values, "callback_end_monotonic_sec")
            detector_inference = self._finite_diagnostic_value(
                values, "inference_ms")
            detector_processing = self._finite_diagnostic_value(
                values, "processing_ms")
            if callback_start is not None:
                self._detector_callback_starts[stamp_key] = callback_start
                while len(self._detector_callback_starts) > 4000:
                    self._detector_callback_starts.popitem(last=False)
            if self._active:
                frame = self._frame_locked(stamp_key, source_stamp)
                frame["detector_inference_ms"] = (
                    "" if detector_inference is None else
                    detector_inference)
                frame["detector_processing_ms"] = (
                    "" if detector_processing is None else
                    detector_processing)
                frame["detector_callback_start_monotonic_sec"] = (
                    "" if callback_start is None else callback_start)
                frame["detector_callback_end_monotonic_sec"] = (
                    "" if callback_end is None else callback_end)
                frame["detector_perf_receipt_monotonic_sec"] = receipt
            self._note_receipt_gap_locked(
                "target_detector_diagnostic_heartbeat",
                self._last_perf_receipt, receipt)
            self._last_perf_receipt = receipt
            self._last_perf_source_stamp = source_stamp
            self._note_pending_output_locked(source_stamp, receipt)
            self._perf_healthy = not reasons
            self._perf_status = {
                "name": status.name,
                "level": int(status.level),
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": values,
                "expected_backend": self._expected_detector_backend,
                "expected_model_path": expected_model,
                "healthy": self._perf_healthy,
                "validation_errors": reasons,
                "source_stamp": source_stamp,
                "receipt_monotonic": receipt,
            }
            if self._active and reasons:
                self._record_infra_gap_locked(
                    "target_detector_diagnostic_not_ok", None,
                    {"validation_errors": reasons,
                     "diagnostic": copy.deepcopy(self._perf_status)})

    @staticmethod
    def _finite_diagnostic_value(values, key):
        try:
            value = float(values.get(key, ""))
        except (TypeError, ValueError, OverflowError):
            return None
        return value if math.isfinite(value) else None

    def _on_truth(self, message):
        receipt = time.monotonic()
        with self._lock:
            target_ids = {target.target_id for target in message.targets
                          if target.pose_valid and
                          self._point_finite(target.world_center)}
            if self._expected_target_ids.issubset(target_ids):
                self._note_receipt_gap_locked(
                    "truth_heartbeat", self._last_truth_receipt, receipt)
                self._truth_seen = True
                self._last_truth_receipt = receipt
                self._last_truth_source_stamp = self._stamp_sec(message)
                self._note_pending_output_locked(
                    self._last_truth_source_stamp, receipt)
            if not self._active:
                return
            spec = self._trial_specs[self._active]
            target_id = self._anchors[spec["class_name"]]["target_id"]
            targets = [target for target in message.targets
                       if target.target_id == target_id]
            if len(targets) != 1:
                return
            target = targets[0]
            stamp_key = self._stamp_key(message)
            stamp_sec = self._stamp_sec(message)
            frame = self._frame_locked(stamp_key, stamp_sec)
            frame["fully_in_frame"] = bool(target.fully_in_frame)
            frame["center_in_frame"] = bool(target.center_in_frame)
            frame["projection_valid"] = bool(target.projection_valid)
            frame["truth_world_x_m"] = float(target.world_center.x)
            frame["truth_world_y_m"] = float(target.world_center.y)
            if target.projection_valid:
                frame["truth_pixel_u"] = float(target.pixel_center.x)
                frame["truth_pixel_v"] = float(target.pixel_center.y)
                if self._camera_info_valid(self._camera_info):
                    half_width = float(self._camera_info["width"]) / 2.0
                    half_height = float(self._camera_info["height"]) / 2.0
                    frame["target_pixel_offset_x_normalized"] = (
                        (float(target.pixel_center.x) -
                         float(self._camera_info["K"][2])) / half_width)
                    frame["target_pixel_offset_y_normalized"] = (
                        (float(target.pixel_center.y) -
                         float(self._camera_info["K"][5])) / half_height)
            visibility_profile = str(spec.get(
                "visibility_profile", "full"))
            visibility_eligible = (
                bool(target.fully_in_frame)
                if visibility_profile != "partial" else
                bool(target.projection_valid and target.center_in_frame and
                     not target.fully_in_frame))
            frame["visibility_profile"] = visibility_profile
            frame["visibility_eligible"] = visibility_eligible
            frame["co_visible_classes"] = ";".join(sorted({
                item.class_name for item in message.targets
                if item.fully_in_frame}))
            self._truth_centers[(self._active, stamp_key)] = (
                float(target.world_center.x), float(target.world_center.y))
            self._reconcile_map_error_locked((self._active, stamp_key))
            result = self._result_locked()
            if target.fully_in_frame and not result["entered_fully_in_frame"]:
                result["entered_fully_in_frame"] = True
                self._add_event_locked(
                    "target_entered_fully_in_frame", stamp_sec,
                    receipt_monotonic=receipt,
                    details={"co_visible_classes":
                             frame["co_visible_classes"]})
            elif (not target.fully_in_frame and
                  result["entered_fully_in_frame"] and
                  not result["left_fully_in_frame"]):
                result["left_fully_in_frame"] = True
                self._add_event_locked(
                    "target_left_fully_in_frame", stamp_sec,
                    receipt_monotonic=receipt)
            if (visibility_eligible and
                    not result["entered_visibility_window"]):
                result["entered_visibility_window"] = True
                result["enter_source_stamp"] = stamp_sec
                result["enter_receipt_monotonic"] = receipt
                self._add_event_locked(
                    "target_entered_visibility_window", stamp_sec,
                    receipt_monotonic=receipt,
                    details={
                        "visibility_profile": visibility_profile,
                        "fully_in_frame": bool(target.fully_in_frame),
                    })
            elif (not visibility_eligible and
                  result["entered_visibility_window"] and
                  not result["left_visibility_window"]):
                result["left_visibility_window"] = True
                result["leave_source_stamp"] = stamp_sec
                result["leave_receipt_monotonic"] = receipt
                self._add_event_locked(
                    "target_left_visibility_window", stamp_sec,
                    receipt_monotonic=receipt,
                    details={"visibility_profile": visibility_profile})

    @staticmethod
    def _is_transform_failure(reason):
        value = str(reason).lower()
        return "transform" in value or value.startswith("tf_")

    @staticmethod
    def _best_detection(detections):
        if not detections:
            return None
        return max(detections, key=lambda detection: (
            int(bool(detection.association_valid)),
            int(bool(detection.geometry_verified)),
            int(bool(detection.center_refined)),
            float(detection.geometry_confidence),
            float(detection.class_confidence),
        ))

    @staticmethod
    def _max_confidence(detections, field):
        values = [float(getattr(detection, field))
                  for detection in detections
                  if math.isfinite(float(getattr(detection, field)))]
        return max(values) if values else ""

    def _on_raw_detections(self, message):
        with self._lock:
            if not self._active:
                return
            frame = self._frame_locked(
                self._stamp_key(message), self._stamp_sec(message))
            expected_class = self._trial_specs[self._active]["class_name"]
            if message.source == "target_detector":
                classifiers = [
                    detection for detection in message.detections
                    if detection.class_name == expected_class
                ]
                if classifiers:
                    frame["raw_class_present"] = True
                    confidence = self._max_confidence(
                        classifiers, "class_confidence")
                    if confidence != "":
                        previous = frame["raw_class_confidence"]
                        frame["raw_class_confidence"] = (
                            confidence if previous == "" else
                            max(previous, confidence))
            geometry = []
            if (expected_class in STANDARD_CLASSES and
                    message.source == "circle_detector"):
                geometry = [
                    detection for detection in message.detections
                    if detection.class_name == "circle" and
                    detection.geometry_verified
                ]
            elif (expected_class == "red_cross" and
                  message.source == "cross_detector"):
                geometry = [
                    detection for detection in message.detections
                    if detection.class_name == "red_cross" and
                    detection.geometry_verified
                ]
            if geometry:
                frame["raw_geometry_present"] = True
                confidence = self._max_confidence(
                    geometry, "geometry_confidence")
                if confidence != "":
                    previous = frame["raw_geometry_confidence"]
                    frame["raw_geometry_confidence"] = (
                        confidence if previous == "" else
                        max(previous, confidence))

    def _on_resolved_detections(self, message):
        with self._lock:
            if not self._active:
                return
            frame = self._frame_locked(
                self._stamp_key(message), self._stamp_sec(message))
            expected_class = self._trial_specs[self._active]["class_name"]
            detections = [
                detection for detection in message.detections
                if detection.class_name == expected_class
            ]
            if not detections:
                return
            frame["resolved_present"] = True
            frame["resolved_class_confidence"] = self._max_confidence(
                detections, "class_confidence")
            frame["resolved_geometry_confidence"] = self._max_confidence(
                detections, "geometry_confidence")

    def _on_refined_detections(self, message):
        with self._lock:
            if not self._active:
                return
            frame = self._frame_locked(
                self._stamp_key(message), self._stamp_sec(message))
            expected_class = self._trial_specs[self._active]["class_name"]
            detections = [
                detection for detection in message.detections
                if detection.class_name == expected_class
            ]
            best = self._best_detection(detections)
            if best is None:
                return
            frame["refined_present"] = True
            frame["geometry_verified"] = bool(best.geometry_verified)
            frame["center_refined"] = bool(best.center_refined)
            frame["association_valid"] = bool(best.association_valid)
            frame["refined_class_confidence"] = float(
                best.class_confidence)
            frame["refined_geometry_confidence"] = float(
                best.geometry_confidence)
            frame["refined_reject_reason"] = ";".join(sorted({
                detection.reject_reason for detection in detections
                if detection.reject_reason
            }))

    def _on_detections(self, message):
        receipt = time.monotonic()
        with self._lock:
            completed_sources = sorted(set(message.completed_sources))
            sources_complete = completed_sources_cover(
                self._required_completed_sources, completed_sources)
            if self._active:
                bucket_key = (self._active, self._stamp_key(message))
                bucket = self._mapped_bucket_status.setdefault(bucket_key, {
                    "complete": False,
                    "partial_source_sets": set(),
                })
                if sources_complete:
                    bucket["complete"] = True
                else:
                    bucket["partial_source_sets"].add(
                        tuple(completed_sources))
            if not sources_complete:
                # The lightweight geometry detectors run on every image while
                # target_detector deliberately uses queue_size=1 and may skip
                # source frames under load.  A timed-out auxiliary-only fusion
                # bucket is therefore expected and must not invalidate a
                # trial.  Only complete buckets advance the mapped heartbeat
                # and output watermark; losing those for heartbeat_timeout_sec
                # remains a hard infrastructure failure.
                self._partial_mapped_frame_count += 1
                self._last_partial_mapped_sources = completed_sources
                return
            self._last_mapped_completed_sources = completed_sources
            self._note_receipt_gap_locked(
                "mapped_detections_heartbeat",
                self._last_mapped_receipt, receipt)
            self._mapped_seen = True
            self._last_mapped_receipt = receipt
            self._last_mapped_source_stamp = self._stamp_sec(message)
            self._note_pending_output_locked(
                self._last_mapped_source_stamp, receipt)
            if not self._active:
                return
            stamp_key = self._stamp_key(message)
            stamp_sec = self._stamp_sec(message)
            frame = self._frame_locked(stamp_key, stamp_sec)
            expected_class = self._trial_specs[self._active]["class_name"]
            detections = [detection for detection in message.detections
                          if detection.class_name == expected_class]
            if not detections:
                return
            frame["detection_present"] = True
            best = self._best_detection(detections)
            frame["mapped_class_confidence"] = float(
                best.class_confidence)
            frame["mapped_geometry_confidence"] = float(
                best.geometry_confidence)
            reasons = sorted({detection.reject_reason
                              for detection in detections
                              if detection.reject_reason})
            frame["reject_reason"] = ";".join(reasons)
            frame["transform_failure"] = any(
                self._is_transform_failure(reason) for reason in reasons)
            valid = []
            invalid_map_payload = False
            for detection in detections:
                valid_payload = (
                    detection.map_valid and
                    self._point_finite(detection.map_point) and
                    math.isfinite(float(detection.map_quality)) and
                    bool(str(detection.map_frame).strip()))
                if valid_payload:
                    valid.append(detection)
                elif detection.map_valid:
                    invalid_map_payload = True
            if invalid_map_payload:
                reasons.append("invalid_map_payload")
                frame["reject_reason"] = ";".join(sorted(set(reasons)))
            frame["map_valid"] = bool(valid)
            self._mapped_points[(self._active, stamp_key)] = [
                (float(detection.map_point.x),
                 float(detection.map_point.y)) for detection in valid]
            self._reconcile_map_error_locked((self._active, stamp_key))

    def _reconcile_map_error_locked(self, key):
        truth = self._truth_centers.get(key)
        mapped = self._mapped_points.get(key, [])
        frame = self._frames.get(key)
        if truth is None or not mapped or frame is None:
            return
        frame["map_error_xy"] = min(
            math.hypot(point[0] - truth[0], point[1] - truth[1])
            for point in mapped)

    def _audit_candidate_locked(self, event_kind, candidate, now, receipt):
        expected_class = (
            self._trial_specs[self._active]["class_name"]
            if self._active else "")
        policy_selectable = candidate_is_currently_selectable(
            candidate, now, self._confirm_frames, self._selected_max_age,
            self._priorities, self._allowed_classes)
        source_stamp = candidate.last_seen.to_sec()
        record = candidate_audit_observation(
            event_kind, candidate.class_name, candidate.id, expected_class,
            self._allowed_classes, state=candidate.state,
            policy_selectable=policy_selectable,
            trial_id=self._active or "", source_stamp=source_stamp)
        audit_key = (
            self._active or "", str(event_kind), int(candidate.id),
            candidate.last_seen.to_nsec(), str(candidate.class_name),
            int(candidate.state), bool(policy_selectable),
        )
        if audit_key not in self._candidate_audit_keys:
            self._candidate_audit_keys.add(audit_key)
            self._candidate_audit_observations.append(record)
            self._add_event_locked(
                "candidate_{}_audit".format(event_kind), source_stamp,
                candidate.id, details=record, receipt_monotonic=receipt,
                class_name=candidate.class_name)
        return record

    def _on_targets(self, message):
        receipt = time.monotonic()
        with self._lock:
            self._note_receipt_gap_locked(
                "targets_heartbeat", self._last_targets_receipt, receipt)
            self._targets_seen = True
            self._last_targets_receipt = receipt
            self._last_targets_source_stamp = self._stamp_sec(message)
            self._note_pending_output_locked(
                self._last_targets_source_stamp, receipt)
            for candidate in message.targets:
                if int(candidate.state) == CONFIRMED_STATE:
                    self._audit_candidate_locked(
                        "confirmed", candidate, message.header.stamp, receipt)
            if not self._active:
                return
            expected_class = self._trial_specs[self._active]["class_name"]
            candidates = [candidate for candidate in message.targets
                          if candidate.class_name == expected_class]
            eligible = [candidate for candidate in candidates
                        if candidate_is_currently_selectable(
                            candidate, message.header.stamp,
                            self._confirm_frames, self._selected_max_age,
                            self._priorities, self._allowed_classes)]
            for candidate in eligible:
                event = {
                    "source_stamp": candidate.last_seen.to_sec(),
                    "stamp_key": candidate.last_seen.to_nsec(),
                    "receipt_monotonic": receipt,
                    "stable_id": int(candidate.id),
                    "target_first_seen_ns": candidate.first_seen.to_nsec(),
                }
                event_key = (
                    self._active, int(candidate.id),
                    candidate.last_seen.to_nsec())
                first_observation = event_key not in self._candidate_event_keys
                if first_observation:
                    self._candidate_event_keys.add(event_key)
                    self._candidate_events[self._active].append(event)
                frame = self._frame_locked(
                    event["stamp_key"], event["source_stamp"])
                frame["current_confirmed"] = True
                frame["stable_id"] = int(candidate.id)
                if first_observation:
                    self._add_event_locked(
                        "candidate_currently_admissible",
                        event["source_stamp"], candidate.id,
                        receipt_monotonic=receipt,
                        details={
                            "consecutive_observe_count": int(
                                candidate.consecutive_observe_count),
                            "map_valid": bool(candidate.map_valid),
                            "association_valid": bool(
                                candidate.association_valid),
                            "reject_reason": candidate.reject_reason,
                        })

    def _on_selected(self, candidate):
        receipt = time.monotonic()
        with self._lock:
            audit = self._audit_candidate_locked(
                "selected", candidate, candidate.header.stamp, receipt)
            if audit["disallowed_by_profile"]:
                rospy.logerr_throttle(
                    1.0,
                    "V-SIM-04 hard failure: profile %s published selected %s id=%s",
                    self._profile, candidate.class_name, candidate.id)
            if not self._active:
                return
            self._note_pending_output_locked(
                candidate.last_seen.to_sec(), receipt)
            if candidate.class_name != self._trial_specs[
                    self._active]["class_name"]:
                return
            if not candidate_is_currently_selectable(
                    candidate, candidate.header.stamp,
                    self._confirm_frames, self._selected_max_age,
                    self._priorities, self._allowed_classes):
                self._add_event_locked(
                    "selected_target_rejected_by_recorder_policy",
                    candidate.last_seen.to_sec(), candidate.id,
                    receipt_monotonic=receipt)
                return
            event = {
                "source_stamp": candidate.last_seen.to_sec(),
                "stamp_key": candidate.last_seen.to_nsec(),
                "receipt_monotonic": receipt,
                "stable_id": int(candidate.id),
                "target_first_seen_ns": candidate.first_seen.to_nsec(),
            }
            event_key = (
                self._active, int(candidate.id), candidate.last_seen.to_nsec())
            first_observation = event_key not in self._selected_event_keys
            if first_observation:
                self._selected_event_keys.add(event_key)
                self._selected_events[self._active].append(event)
            frame = self._frame_locked(
                event["stamp_key"], event["source_stamp"])
            frame["current_selected"] = True
            frame["stable_id"] = int(candidate.id)
            if first_observation:
                self._add_event_locked(
                    "candidate_selected_observed", event["source_stamp"],
                    candidate.id, receipt_monotonic=receipt,
                    details={
                        "target_first_seen_ns":
                            candidate.first_seen.to_nsec(),
                    })

    def _on_navigation_decision(self, message):
        receipt = time.monotonic()
        record = {
            "schema_version": int(message.schema_version),
            "mission_id": str(message.mission_id),
            "decision_seq": int(message.decision_seq),
            "header_seq": int(message.header.seq),
            "header_stamp_ns": message.header.stamp.to_nsec(),
            "header_frame_id": str(message.header.frame_id),
            "deadline_ns": message.deadline.to_nsec(),
            "command": int(message.command),
            "class_profile": str(message.class_profile),
            "has_goal": bool(message.has_goal),
            "has_target": bool(message.has_target),
            "target_id": int(message.target_id),
            "target_first_seen_ns": message.target_first_seen.to_nsec(),
            "target_observation_stamp_ns":
                message.target_observation_stamp.to_nsec(),
            "target_class": str(message.target_class),
            "attempt": int(message.attempt),
            "payload_slot": int(message.payload_slot),
            "goal_stamp_ns": message.goal.header.stamp.to_nsec(),
            "goal_frame_id": str(message.goal.header.frame_id),
            "goal_x": float(message.goal.pose.position.x),
            "goal_y": float(message.goal.pose.position.y),
            "goal_z": float(message.goal.pose.position.z),
            "goal_qx": float(message.goal.pose.orientation.x),
            "goal_qy": float(message.goal.pose.orientation.y),
            "goal_qz": float(message.goal.pose.orientation.z),
            "goal_qw": float(message.goal.pose.orientation.w),
            "reason": str(message.reason),
            "receipt_monotonic": receipt,
        }
        with self._lock:
            if self._finalized or self._navigation_callbacks_frozen:
                return
            self._last_navigation_receipt = receipt
            record["trial_id_at_receipt"] = self._active or ""
            self._navigation_decisions.append(record)
            self._note_pending_output_locked(
                message.header.stamp.to_sec(), receipt)
            self._add_event_locked(
                "navigation_decision_observed",
                message.header.stamp.to_sec(),
                message.target_id if message.has_target else None,
                details=record, receipt_monotonic=receipt,
                class_name=message.target_class if message.has_target else "")

    def _on_navigation_result(self, message):
        receipt = time.monotonic()
        record = {
            "schema_version": int(message.schema_version),
            "mission_id": str(message.mission_id),
            "executor_id": str(message.executor_id),
            "event_seq": int(message.event_seq),
            "decision_seq": int(message.decision_seq),
            "header_seq": int(message.header.seq),
            "header_stamp_ns": message.header.stamp.to_nsec(),
            "header_frame_id": str(message.header.frame_id),
            "command": int(message.command),
            "status": int(message.status),
            "stage": int(message.stage),
            "terminal": bool(message.terminal),
            "retryable": bool(message.retryable),
            "payload_committed": bool(message.payload_committed),
            "has_target": bool(message.has_target),
            "target_id": int(message.target_id),
            "target_first_seen_ns": message.target_first_seen.to_nsec(),
            "target_class": str(message.target_class),
            "attempt": int(message.attempt),
            "payload_slot": int(message.payload_slot),
            "reason": str(message.reason),
            "evidence_source": str(message.evidence_source),
            "receipt_monotonic": receipt,
        }
        with self._lock:
            if self._finalized or self._navigation_callbacks_frozen:
                return
            self._last_navigation_receipt = receipt
            record["trial_id_at_receipt"] = self._active or ""
            self._navigation_results.append(record)
            self._note_pending_output_locked(
                message.header.stamp.to_sec(), receipt)
            self._add_event_locked(
                "navigation_result_observed",
                message.header.stamp.to_sec(),
                message.target_id if message.has_target else None,
                details=record, receipt_monotonic=receipt,
                class_name=message.target_class if message.has_target else "")

    def _actual_fps_locked(self):
        duration = (
            self._last_image_receipt - self._first_image_receipt
            if self._image_count >= 2 and
            self._last_image_receipt is not None and
            self._first_image_receipt is not None else 0.0)
        return ((self._image_count - 1) / duration
                if duration > 0.0 else None)

    def _actual_source_fps_locked(self):
        last_stamp = (
            self._last_image_stamp / 1.0e9
            if self._last_image_stamp is not None else None)
        duration = (
            last_stamp - self._first_image_stamp_sec
            if self._image_count >= 2 and last_stamp is not None and
            self._first_image_stamp_sec is not None else 0.0)
        return ((self._image_count - 1) / duration
                if duration > 0.0 else None)

    def _terminal_errors_locked(self):
        errors = []
        if not self._run_complete:
            errors.append("run_complete_event_missing")
        completed = [result for result in self._results.values()
                     if result.get("status") == "completed"]
        if len(self._results) != self._expected_trial_count:
            errors.append("trial_count_{}/{}".format(
                len(self._results), self._expected_trial_count))
        if (self._evaluation_scope == "full" and
                self._expected_trial_count !=
                self._formal_expected_trial_count):
            errors.append("full_trial_count_must_match_formal_design")
        if len(completed) != len(self._results):
            errors.append("completed_trials_{}/{}".format(
                len(completed), len(self._results)))
        for trial_id, result in self._results.items():
            if result.get("status") != "completed":
                continue
            count_sampling_miss = (
                self._dynamic_zero_visibility_policy ==
                "count_as_failure" and result.get("kind") == "dynamic")
            if (not result.get("entered_visibility_window") and
                    not count_sampling_miss):
                errors.append(
                    "{}:never_entered_visibility_window".format(trial_id))
            if (not result.get("left_visibility_window") and
                    not count_sampling_miss):
                errors.append(
                    "{}:never_left_visibility_window".format(trial_id))
            if (int(result.get("eligible_frames", 0)) <= 0 and
                    not count_sampling_miss):
                errors.append("{}:no_eligible_frames".format(trial_id))
            if (result.get("p_confirm_visibility") and
                    result.get("confirmation_pipeline_ms") is None):
                errors.append("{}:confirmation_pipeline_missing".format(
                    trial_id))
            for field in ("expected_duration_sec", "actual_duration_sec"):
                value = result.get(field)
                if (value is None or not math.isfinite(float(value)) or
                        float(value) <= 0.0):
                    errors.append("{}:{}_invalid".format(trial_id, field))
            if result.get("kind") == "dynamic":
                for field in ("expected_speed_mps", "actual_speed_mps"):
                    value = result.get(field)
                    if (value is None or not math.isfinite(float(value)) or
                            float(value) <= 0.0):
                        errors.append("{}:{}_invalid".format(trial_id, field))
                if int(result.get("camera_pose_frame_count", 0)) <= 0:
                    errors.append("{}:camera_pose_frames_missing".format(
                        trial_id))
                if int(result.get("motion_sample_count", 0)) <= 0:
                    errors.append("{}:motion_samples_missing".format(
                        trial_id))
                if int(result.get("lateral_offset_sample_count", 0)) <= 0:
                    errors.append("{}:lateral_offset_samples_missing".format(
                        trial_id))
                if (result.get("lateral_bin") and
                        int(result.get("target_lateral_sample_count", 0)) <= 0):
                    errors.append(
                        "{}:target_lateral_samples_missing".format(trial_id))
        if self._camera_info is None:
            errors.append("camera_info_missing")
        elif not self._camera_info_valid(self._camera_info):
            errors.append("camera_info_invalid")
        fps = self._actual_fps_locked()
        if fps is None or not math.isfinite(fps) or fps <= 0.0:
            errors.append("actual_image_receipt_fps_invalid")
        source_fps = self._actual_source_fps_locked()
        if (source_fps is None or not math.isfinite(source_fps) or
                source_fps <= 0.0):
            errors.append("actual_image_source_fps_invalid")
        for missing in self._readiness_missing_locked():
            errors.append("terminal_chain_missing:" + missing)
        if self._pending_trial_end is not None:
            errors.append("trial_end_still_pending")
        for gap in self._infra_gaps:
            errors.append("{}:infra_gap:{}".format(
                gap["trial_id"], gap["chain"]))
        errors.extend(self._static_manifest_errors())
        if self._fatal_error:
            errors.append(self._fatal_error)
        return sorted(set(errors))

    def _snapshot(self, terminal_context=None):
        with self._lock:
            manifest = copy.deepcopy(self._manifest)
            manifest["camera_info"] = copy.deepcopy(self._camera_info)
            manifest["trajectory"]["actual_trials"] = copy.deepcopy(
                self._actual_trajectories)
            manifest["runtime"] = {
                "target_detector_diagnostic": copy.deepcopy(
                    self._perf_status),
                "infrastructure_gaps": copy.deepcopy(self._infra_gaps),
                "image_receipt_fps": self._actual_fps_locked(),
                "image_source_fps": self._actual_source_fps_locked(),
                "camera_pose_matched_frame_stamps": sum(
                    bool(sample.get("camera_pose_valid"))
                    for sample in self._camera_pose_samples.values()),
                "camera_pose_matching":
                    "latest_at_or_before_image_stamp",
                "last_mapped_completed_sources": list(
                    self._last_mapped_completed_sources),
                "partial_mapped_frame_count": int(
                    self._partial_mapped_frame_count),
                "last_partial_mapped_sources": list(
                    self._last_partial_mapped_sources),
                "active_trial_mapped_buckets": {
                    "complete": sum(
                        bool(value["complete"])
                        for value in self._mapped_bucket_status.values()),
                    "partial_only": sum(
                        not value["complete"] and
                        bool(value["partial_source_sets"])
                        for value in self._mapped_bucket_status.values()),
                },
            }
            frames = copy.deepcopy(list(self._frames.values()))
            trial_order = {
                trial_id: index
                for index, trial_id in enumerate(self._trial_specs)
            }
            frames.sort(key=lambda row: (
                trial_order.get(row.get("trial_id"), len(trial_order)),
                motion_frame_sort_key(row)))
            events = copy.deepcopy(self._events)
            results = copy.deepcopy(list(self._results.values()))
            actual_fps = self._actual_fps_locked()
            actual_source_fps = self._actual_source_fps_locked()
            context = copy.deepcopy(
                terminal_context if terminal_context is not None
                else self._terminal_context)
            context["class_profile"] = self._profile
            context["allowed_classes"] = sorted(self._allowed_classes)
            context["candidate_audit_observations"] = copy.deepcopy(
                self._candidate_audit_observations)
            context["performance_contract"] = copy.deepcopy(
                self._matrix.get("performance_contract", {}))
            context["dynamic_zero_visibility_policy"] = \
                self._dynamic_zero_visibility_policy
            context["navigation_metrics_mode"] = self._navigation_metrics[
                "mode"]
            context["navigation_decision_topic"] = \
                self._navigation_decision_topic
            context["navigation_result_topic"] = \
                self._navigation_result_topic
            context["navigation_decision_records"] = copy.deepcopy(
                self._navigation_decisions)
            context["navigation_result_records"] = copy.deepcopy(
                self._navigation_results)
        return (manifest, frames, events, results, actual_fps,
                actual_source_fps, context)

    def _write_artifacts_safe(self, terminal_context=None):
        with self._write_lock:
            snapshot = self._snapshot(terminal_context)
            try:
                return write_artifacts(
                    self._output_dir, snapshot[0], snapshot[1], snapshot[2],
                    snapshot[3], "ros_{}".format(
                        self._navigation_metrics["mode"]),
                    snapshot[4], snapshot[6],
                    snapshot[5])
            except Exception as error:
                with self._lock:
                    self._fatal_error = "artifact_write_failed: {}".format(
                        error)
                rospy.logerr("V-SIM-04 artifact write failed: %s", error)
                return None

    def _finalize_run(self):
        with self._lock:
            if self._finalized:
                return
            self._navigation_callbacks_frozen = True
            errors = self._terminal_errors_locked()
            context = {
                "run_complete": True,
                "expected_trial_count": self._expected_trial_count,
                "formal_expected_trial_count":
                    self._formal_expected_trial_count,
                "evaluation_scope": self._evaluation_scope,
                "dynamic_zero_visibility_policy":
                    self._dynamic_zero_visibility_policy,
                "validation_errors": list(errors),
            }
            self._terminal_context = copy.deepcopy(context)
        summary = self._write_artifacts_safe(context)
        missing_artifacts = [name for name in REQUIRED_ARTIFACTS
                             if not os.path.isfile(os.path.join(
                                 self._output_dir, name))]
        if missing_artifacts:
            errors.append("missing_artifacts:" +
                          ",".join(sorted(missing_artifacts)))
            context["validation_errors"] = sorted(set(errors))
            with self._lock:
                self._terminal_context = copy.deepcopy(context)
            summary = self._write_artifacts_safe(context)
        expected_status = (
            "MEASURED" if self._evaluation_scope == "full" else
            "DIAGNOSTIC")
        measurement_success = bool(
            summary and summary.get("status") == expected_status and
            not context["validation_errors"])
        hard_performance_failure = bool(
            summary and summary.get("performance_verdict", {}).get(
                "hard_failure"))
        success = measurement_success and not hard_performance_failure
        if not measurement_success and not context["validation_errors"]:
            context["validation_errors"] = [
                self._fatal_error or "recorder_finalization_failed"]
            with self._lock:
                self._terminal_context = copy.deepcopy(context)
            summary = self._write_artifacts_safe(context) or summary
        with self._lock:
            self._finalized = True
            self._final_success = success
            self._final_summary_status = (
                summary.get("status", "INVALID") if summary else "INVALID")
            if not success and not self._fatal_error:
                if hard_performance_failure:
                    reasons = summary["performance_verdict"].get(
                        "hard_failure_reasons", [])
                    self._fatal_error = "performance_hard_failure:" + ",".join(
                        reasons)
                else:
                    self._fatal_error = "terminal_validation_failed"
        if success:
            rospy.loginfo(
                "V-SIM-04 recorder finalized %s (%d/%d)",
                expected_status, len(self._results),
                self._expected_trial_count)
        else:
            rospy.logerr("V-SIM-04 recorder finalization failed: %s",
                         "; ".join(context["validation_errors"]) or
                         self._fatal_error or "unknown")
        self._publish_status()

    def _on_timer(self, _event):
        write_after = False
        finalize_after = False
        with self._lock:
            self._record_current_missing_locked()
            write_after = self._maybe_finish_pending_locked()
            finalize_after = self._maybe_finish_navigation_finalize_locked()
        if write_after:
            self._write_artifacts_safe()
        if finalize_after:
            self._finalize_run()
        self._publish_status()

    def _maybe_finish_navigation_finalize_locked(self):
        pending = self._pending_navigation_finalize
        if pending is None:
            return False
        now = time.monotonic()
        ready, timed_out = navigation_drain_ready(
            now, pending["requested_monotonic"],
            self._last_navigation_receipt,
            self._navigation_late_result_quiet_sec,
            self._navigation_late_result_timeout_sec)
        if not ready:
            return False
        self._navigation_callbacks_frozen = True
        self._pending_navigation_finalize = None
        self._add_event_locked(
            "navigation_late_result_drain_complete",
            details={
                "timed_out": timed_out,
                "elapsed_sec": now - pending["requested_monotonic"],
                "decision_event_count": len(self._navigation_decisions),
                "result_event_count": len(self._navigation_results),
            })
        return True

    def _on_shutdown(self):
        with self._lock:
            self._navigation_callbacks_frozen = True
        self._write_artifacts_safe()


def main():
    VSim04TrialRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
