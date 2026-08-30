#!/usr/bin/env python3
"""Capture bounded raw frames for explicitly selected V-SIM diagnostics."""

import json
import math
import os
import threading

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from uav_vision_eval.failure_capture import (
    CAPTURE_DATASET_KIND,
    CAPTURE_SCHEMA_VERSION,
    ExactStampPairBuffer,
    align_sampling_plan,
    allocate_trial_quotas,
    build_capture_status,
    build_frame_record,
    configure_sampling_plan,
    freeze_camera_info_profile,
    resolve_capture_output_dir,
    sampling_plan,
    sampling_timing,
    select_truth_target,
    stamp_key,
    validate_capture_config,
    validate_capture_manifest,
)
from uav_vision_eval.msg import SimTargetArray
from uav_vision_eval.vsim04_metrics import (
    load_trial_matrix,
    select_trial_matrix,
)


class VSim04FailureCapture:
    def __init__(self):
        rospy.init_node("vsim04_failure_capture")
        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._enabled = bool(rospy.get_param("~enabled", False))
        self._selector = str(rospy.get_param("~trial_selector", "")).strip()
        self._slice = str(rospy.get_param("~trial_slice", "")).strip()
        self._max_frames = int(rospy.get_param("~max_frames", 30))
        self._max_sampling_lateness = float(rospy.get_param(
            "~max_sampling_lateness_sec", 0.25))
        if (not math.isfinite(self._max_sampling_lateness) or
                self._max_sampling_lateness <= 0.0):
            raise ValueError("max_sampling_lateness_sec must be positive")
        output_value = str(rospy.get_param("~output_dir", "")).strip()
        output_root = str(rospy.get_param("~output_root", "")).strip()
        validate_capture_config(
            self._enabled, self._selector, self._slice, self._max_frames,
            output_value)
        self._output_dir = resolve_capture_output_dir(
            output_root, output_value)
        if not self._enabled:
            raise ValueError("disabled capture node must not be launched")

        matrix_path = os.path.abspath(rospy.get_param("~matrix_file"))
        selected = select_trial_matrix(
            load_trial_matrix(matrix_path), self._selector, self._slice)
        if selected.get("evaluation_scope") != "diagnostic":
            raise ValueError("failure capture is diagnostic-only")
        self._trials = {
            trial["trial_id"]: trial for trial in selected["trials"]}
        self._trial_quotas = allocate_trial_quotas(
            self._trials.keys(), self._max_frames)
        self._sampling_plans = {
            trial_id: sampling_plan(
                selected, trial, self._trial_quotas[trial_id])
            for trial_id, trial in self._trials.items()
        }
        self._target_ids = {
            class_name: anchor["target_id"]
            for class_name, anchor in selected["target_anchors"].items()
        }
        self._pair_buffer = ExactStampPairBuffer(int(rospy.get_param(
            "~max_pending_pairs", 64)))
        self._camera_info = None
        self._camera_profile = None
        self._last_image = None
        self._readiness_pair_buffer = ExactStampPairBuffer(int(
            rospy.get_param("~max_pending_pairs", 64)))
        self._readiness_pair = None
        self._capture_ready = False
        self._ready_pair_stamp = None
        self._ready_before_first_trial = False
        self._active_trial = None
        self._active_event = ""
        self._active_event_seq = 0
        self._last_capture_control_seq = 0
        self._completed_trial_count = 0
        self._sampling_started_trial_ids = []
        self._trial_start_stamp_sec = None
        self._last_considered_elapsed = None
        self._latest_eligible_pair = None
        self._captured_stamps = {
            trial_id: set() for trial_id in self._trials}
        self._captured = 0
        self._trial_counts = {trial_id: 0 for trial_id in self._trials}
        self._records = []
        self._fatal_error = ""
        self._run_complete = False
        self._capture_status_topic = rospy.get_param(
            "~capture_status_topic",
            "/uav_vision_eval/vsim04/failure_capture/status")
        self._trial_event_topic = rospy.get_param(
            "~trial_event_topic", "/uav_vision_eval/vsim04/trial_event")
        self._capture_control_topic = rospy.get_param(
            "~capture_control_topic",
            "/uav_vision_eval/vsim04/failure_capture/control")
        resolved_topics = [rospy.resolve_name(value) for value in (
            self._capture_status_topic, self._trial_event_topic,
            self._capture_control_topic)]
        if len(resolved_topics) != len(set(resolved_topics)):
            raise ValueError("failure capture topics must be distinct")
        os.makedirs(self._output_dir, exist_ok=True)
        if os.listdir(self._output_dir):
            raise ValueError(
                "failure capture output_dir must be empty: {}".format(
                    self._output_dir))

        self._status_publisher = rospy.Publisher(
            self._capture_status_topic,
            String, queue_size=1, latch=True)
        self._publish_status("STARTING")

        rospy.Subscriber(
            rospy.get_param("~camera_info_topic",
                            "/camera/color/camera_info"),
            CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image, self._on_image, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~truth_topic",
                            "/uav_vision_eval/ground_truth"),
            SimTargetArray, self._on_truth, queue_size=10)
        rospy.Subscriber(
            self._trial_event_topic,
            String, self._on_trial_event, queue_size=20)
        rospy.Subscriber(
            self._capture_control_topic,
            String, self._on_capture_control, queue_size=4)
        rospy.on_shutdown(self._write_manifest)

    def _publish_status(self, state, ready=None, **values):
        if self._fatal_error and state != "FAIL":
            return
        if self._run_complete and state not in ("FINALIZED", "FAIL"):
            return
        if ready is None:
            ready = self._capture_ready
        payload = build_capture_status(
            self._trials.keys(), state, ready=ready,
            active_trial=values.get(
                "active_trial",
                self._active_trial["trial_id"]
                if self._active_trial is not None else ""),
            active_event=values.get("active_event", self._active_event),
            active_event_seq=values.get(
                "active_event_seq", self._active_event_seq),
            completed_trial_count=self._completed_trial_count,
            run_complete=self._run_complete,
            error=values.get("error", self._fatal_error))
        self._status_publisher.publish(String(data=json.dumps(
            payload, sort_keys=True)))

    def _maybe_mark_ready(self):
        if self._capture_ready or self._camera_info is None:
            return
        if self._readiness_pair is None:
            return
        image, _ = self._readiness_pair
        freeze_camera_info_profile(
            self._camera_profile, self._camera_info, image)
        self._capture_ready = True
        self._ready_pair_stamp = stamp_key(image)
        self._readiness_pair_buffer.clear()
        self._publish_status("READY", ready=True)

    def _fail(self, error):
        with self._lock:
            if self._fatal_error:
                return
            self._fatal_error = str(error)
            rospy.logfatal("V-SIM failure capture failed closed: %s", error)
            try:
                if hasattr(self, "_status_publisher"):
                    self._publish_status(
                        "FAIL", ready=False, error=str(error))
            except Exception as status_error:
                rospy.logerr(
                    "failure capture could not publish FAIL status: %s",
                    status_error)
            try:
                self._write_manifest()
            except Exception as manifest_error:
                rospy.logerr(
                    "failure capture could not persist FAIL manifest: %s",
                    manifest_error)
        rospy.signal_shutdown(self._fatal_error)

    def _on_camera_info(self, message):
        try:
            with self._lock:
                if self._fatal_error or self._run_complete:
                    return
                self._camera_profile = freeze_camera_info_profile(
                    self._camera_profile, message, self._last_image)
                self._camera_info = message
                self._maybe_mark_ready()
        except Exception as error:
            self._fail(error)

    def _on_trial_event(self, message):
        self._on_event(message, {
            "trial_start", "trial_end", "run_abort", "run_complete"})

    def _on_capture_control(self, message):
        self._on_event(message, {"sampling_start"})

    def _on_event(self, message, allowed_events):
        try:
            payload = json.loads(message.data)
            event = payload.get("event")
            if event not in allowed_events:
                raise RuntimeError(
                    "event arrived on the wrong failure capture topic")
            trial_id = payload.get("trial_id", "")
            with self._lock:
                if self._fatal_error or self._run_complete:
                    return
                if event == "trial_start":
                    if not self._capture_ready:
                        raise RuntimeError(
                            "trial_start received before failure capture ready")
                    if self._active_trial is not None:
                        raise RuntimeError("overlapping failure capture trials")
                    if trial_id not in self._trials:
                        raise RuntimeError(
                            "unknown selected trial: {}".format(trial_id))
                    trial_start = float(payload.get("stamp"))
                    if not math.isfinite(trial_start) or trial_start < 0.0:
                        raise RuntimeError("invalid trial_start source stamp")
                    self._pair_buffer.clear()
                    self._active_trial = self._trials[trial_id]
                    self._active_event_seq = int(payload.get("event_seq", 0))
                    if self._active_event_seq <= 0:
                        raise RuntimeError("invalid trial_start event sequence")
                    if self._completed_trial_count == 0:
                        self._ready_before_first_trial = True
                    self._active_event = "trial_start"
                    self._trial_start_stamp_sec = None
                    self._last_considered_elapsed = None
                    self._latest_eligible_pair = None
                    self._publish_status("RUNNING", ready=True)
                elif event == "sampling_start":
                    if (self._active_trial is None or
                            self._active_trial["trial_id"] != trial_id):
                        raise RuntimeError(
                            "sampling_start does not match active trial")
                    sampling_start = float(payload.get(
                        "sampling_start_stamp_sec"))
                    if (not math.isfinite(sampling_start) or
                            sampling_start <= rospy.Time.now().to_sec()):
                        raise RuntimeError(
                            "sampling_start source stamp is not in the future")
                    event_seq = int(payload.get("event_seq", 0))
                    if event_seq <= self._last_capture_control_seq:
                        raise RuntimeError(
                            "sampling_start event sequence is not increasing")
                    if trial_id in self._sampling_started_trial_ids:
                        raise RuntimeError(
                            "sampling_start repeated for selected trial")
                    expected_duration = float(payload.get(
                        "sampling_expected_duration_sec"))
                    target_center = payload.get(
                        "sampling_target_center_offset_sec")
                    trial_plan = self._sampling_plans[trial_id]
                    self._sampling_plans[trial_id] = configure_sampling_plan(
                        trial_plan, expected_duration,
                        target_center if self._active_trial["kind"] ==
                        "dynamic" else None, sampling_start)
                    self._pair_buffer.clear()
                    self._trial_start_stamp_sec = sampling_start
                    self._last_considered_elapsed = None
                    self._latest_eligible_pair = None
                    self._active_event_seq = event_seq
                    self._active_event = "sampling_start"
                    self._last_capture_control_seq = event_seq
                    self._sampling_started_trial_ids.append(trial_id)
                    self._publish_status("RUNNING", ready=True)
                elif event == "trial_end":
                    if (self._active_trial is None or
                            self._active_trial["trial_id"] != trial_id):
                        raise RuntimeError(
                            "trial_end does not match active capture trial")
                    self._capture_trial_end_fallback()
                    if (self._trial_counts[trial_id] !=
                            self._trial_quotas[trial_id]):
                        raise RuntimeError(
                            "selected trial capture quota incomplete: "
                            "{} count={} quota={}".format(
                                trial_id,
                                self._trial_counts[trial_id],
                                self._trial_quotas[trial_id]))
                    self._active_trial = None
                    self._active_event = ""
                    self._active_event_seq = 0
                    self._trial_start_stamp_sec = None
                    self._last_considered_elapsed = None
                    self._latest_eligible_pair = None
                    self._pair_buffer.clear()
                    self._completed_trial_count += 1
                    self._publish_status("READY", ready=True)
                elif event == "run_abort":
                    raise RuntimeError(
                        "V-SIM run aborted: {}".format(
                            payload.get("error", "unknown")))
                elif event == "run_complete":
                    if self._active_trial is not None:
                        raise RuntimeError(
                            "run_complete received during active trial")
                    if self._trial_counts != dict(self._trial_quotas):
                        raise RuntimeError(
                            "one or more selected trials missed capture quota")
                    if self._captured != self._max_frames:
                        raise RuntimeError(
                            "captured frame total does not match max_frames")
                    self._active_trial = None
                    self._pair_buffer.clear()
                    self._run_complete = True
                    self._write_manifest()
                    manifest_path = os.path.join(
                        self._output_dir, "dataset_manifest.json")
                    with open(manifest_path, "r", encoding="utf-8") as stream:
                        validate_capture_manifest(
                            json.load(stream), self._output_dir)
                    self._publish_status("FINALIZED", ready=True)
        except Exception as error:
            self._fail(error)

    def _on_image(self, message):
        try:
            with self._lock:
                if self._fatal_error or self._run_complete:
                    return
                self._last_image = message
                if self._camera_info is not None:
                    freeze_camera_info_profile(
                        self._camera_profile, self._camera_info, message)
                if not self._capture_ready:
                    ready_pair = self._readiness_pair_buffer.add_image(message)
                    if ready_pair is not None:
                        self._readiness_pair = ready_pair
                    self._maybe_mark_ready()
                if not self._active_trial_has_budget():
                    return
                pair = self._pair_buffer.add_image(message)
                if pair is not None:
                    self._consider_pair(*pair)
        except Exception as error:
            self._fail(error)

    def _on_truth(self, message):
        try:
            with self._lock:
                if self._fatal_error or self._run_complete:
                    return
                if not self._capture_ready:
                    ready_pair = self._readiness_pair_buffer.add_truth(message)
                    if ready_pair is not None:
                        self._readiness_pair = ready_pair
                    self._maybe_mark_ready()
                if not self._active_trial_has_budget():
                    return
                pair = self._pair_buffer.add_truth(message)
                if pair is not None:
                    self._consider_pair(*pair)
        except Exception as error:
            self._fail(error)

    @staticmethod
    def _atomic_json(path, payload):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(
                payload, stream, indent=2, sort_keys=True,
                allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)

    def _active_trial_has_budget(self):
        trial = self._active_trial
        if (trial is None or self._trial_start_stamp_sec is None or
                self._captured >= self._max_frames):
            return False
        trial_id = trial["trial_id"]
        return self._trial_counts[trial_id] < self._trial_quotas[trial_id]

    @staticmethod
    def _source_stamp_sec(message):
        secs, nsecs = stamp_key(message)
        return float(secs) + float(nsecs) * 1.0e-9

    def _consider_pair(self, image, truth):
        if self._camera_info is None:
            return
        trial = self._active_trial
        if not self._active_trial_has_budget():
            return
        target = select_truth_target(
            truth, trial["class_name"],
            self._target_ids.get(trial["class_name"], ""))
        if target is None:
            return
        elapsed = self._source_stamp_sec(image) - self._trial_start_stamp_sec
        if elapsed < 0.0:
            return
        if (self._last_considered_elapsed is not None and
                elapsed <= self._last_considered_elapsed + 1.0e-9):
            return
        self._last_considered_elapsed = elapsed
        self._latest_eligible_pair = (image, truth, target, elapsed)
        trial_id = trial["trial_id"]
        sample_index = self._trial_counts[trial_id]
        plan = self._sampling_plans[trial_id]
        if not plan.get("aligned"):
            plan = align_sampling_plan(plan, elapsed)
            self._sampling_plans[trial_id] = plan
        if elapsed + 1.0e-6 < plan["sample_offsets_sec"][sample_index]:
            return
        self._capture_pair(image, truth, target, elapsed, False)

    def _capture_trial_end_fallback(self):
        trial = self._active_trial
        if trial is None or not self._active_trial_has_budget():
            return
        trial_id = trial["trial_id"]
        remaining = (
            self._trial_quotas[trial_id] - self._trial_counts[trial_id])
        if remaining != 1:
            return
        latest = self._latest_eligible_pair
        if latest is None:
            return
        if stamp_key(latest[0]) in self._captured_stamps[trial_id]:
            return
        self._capture_pair(*latest, used_trial_end_fallback=True)

    def _capture_pair(self, image, truth, target, elapsed,
                      used_trial_end_fallback=False):
        trial = self._active_trial
        trial_id = trial["trial_id"]
        if stamp_key(image) in self._captured_stamps[trial_id]:
            raise RuntimeError("duplicate source stamp selected for capture")
        capture_index = self._captured + 1
        stem = "{:04d}_{}_{}_{:09d}".format(
            capture_index, trial["trial_id"],
            int(image.header.stamp.secs), int(image.header.stamp.nsecs))
        image_name = stem + ".png"
        metadata_name = stem + ".json"
        image_path = os.path.join(self._output_dir, image_name)
        temporary_image_path = image_path + ".tmp.png"
        plan = self._sampling_plans[trial_id]
        sample_index = self._trial_counts[trial_id]
        timing = sampling_timing(
            elapsed, plan["sample_offsets_sec"][sample_index],
            self._max_sampling_lateness, used_trial_end_fallback)
        record = build_frame_record(
            trial, image, truth, target, self._camera_info,
            image_name, metadata_name, capture_index, {
                "policy": plan["policy"],
                "sample_index": sample_index,
                "planned_fraction": plan["sample_fractions"][sample_index],
                "planned_offset_sec": plan["sample_offsets_sec"][sample_index],
                "actual_offset_sec": elapsed,
                "expected_duration_sec": plan["expected_duration_sec"],
                "sampling_start_stamp_sec":
                    plan["sampling_start_stamp_sec"],
                "window_start_offset_sec":
                    plan["window_start_offset_sec"],
                "window_duration_sec": plan["window_duration_sec"],
                **timing,
                "used_trial_end_fallback": used_trial_end_fallback,
            })
        cv_image = self._bridge.imgmsg_to_cv2(
            image, desired_encoding="bgr8")
        if not cv2.imwrite(temporary_image_path, cv_image):
            raise IOError("cv2.imwrite failed: {}".format(image_path))
        os.replace(temporary_image_path, image_path)
        self._atomic_json(
            os.path.join(self._output_dir, metadata_name), record)
        self._records.append(record)
        self._captured = capture_index
        self._trial_counts[trial_id] += 1
        self._captured_stamps[trial_id].add(stamp_key(image))
        self._write_manifest()

    def _write_manifest(self):
        with self._lock:
            if not self._output_dir:
                return
            payload = {
                "schema_version": CAPTURE_SCHEMA_VERSION,
                "dataset_kind": CAPTURE_DATASET_KIND,
                "status": (
                    "FAIL" if self._fatal_error else
                    "DIAGNOSTIC" if self._run_complete else "INCOMPLETE"),
                "error": self._fatal_error,
                "run_complete": self._run_complete,
                "trial_selector": self._selector,
                "trial_slice": self._slice,
                "selected_trial_ids": list(self._trials),
                "sampling_policy": (
                    "source-stamp deterministic over first-visible window; "
                    "singleton=45%, multi-sample=0..90%"),
                "sampling_plans": self._sampling_plans,
                "max_sampling_lateness_sec":
                    self._max_sampling_lateness,
                "readiness": {
                    "camera_profile_frozen":
                        self._camera_profile is not None,
                    "exact_pair_observed":
                        self._ready_pair_stamp is not None,
                    "ready_before_first_trial":
                        self._ready_before_first_trial,
                    "ready_pair_stamp": (
                        None if self._ready_pair_stamp is None else {
                            "secs": self._ready_pair_stamp[0],
                            "nsecs": self._ready_pair_stamp[1],
                        }),
                    "sampling_started_trial_ids":
                        list(self._sampling_started_trial_ids),
                },
                "max_frames": self._max_frames,
                "captured_frames": self._captured,
                "trial_counts": dict(self._trial_counts),
                "trial_quotas": dict(self._trial_quotas),
                "records": self._records,
            }
            self._atomic_json(
                os.path.join(self._output_dir, "dataset_manifest.json"),
                payload)


def main():
    node = None
    try:
        node = VSim04FailureCapture()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("V-SIM failure capture startup failed: %s", error)
        return 8
    if node is not None and node._fatal_error:
        return 8
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
