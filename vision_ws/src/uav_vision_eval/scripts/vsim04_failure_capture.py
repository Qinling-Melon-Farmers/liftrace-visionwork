#!/usr/bin/env python3
"""Capture bounded raw frames for explicitly selected V-SIM diagnostics."""

import json
import os
import threading

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from uav_vision_eval.failure_capture import (
    CAPTURE_SCHEMA_VERSION,
    ExactStampPairBuffer,
    allocate_trial_quotas,
    build_frame_record,
    select_truth_target,
    validate_capture_config,
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
        self._max_frames = int(rospy.get_param("~max_frames", 30))
        output_value = str(rospy.get_param("~output_dir", "")).strip()
        validate_capture_config(
            self._enabled, self._selector, self._max_frames,
            output_value)
        self._output_dir = os.path.abspath(os.path.expanduser(output_value))
        if not self._enabled:
            raise ValueError("disabled capture node must not be launched")

        matrix_path = os.path.abspath(rospy.get_param("~matrix_file"))
        selected = select_trial_matrix(
            load_trial_matrix(matrix_path), self._selector)
        if selected.get("evaluation_scope") != "diagnostic":
            raise ValueError("failure capture is diagnostic-only")
        self._trials = {
            trial["trial_id"]: trial for trial in selected["trials"]}
        self._trial_quotas = allocate_trial_quotas(
            self._trials.keys(), self._max_frames)
        self._target_ids = {
            class_name: anchor["target_id"]
            for class_name, anchor in selected["target_anchors"].items()
        }
        self._pair_buffer = ExactStampPairBuffer(int(rospy.get_param(
            "~max_pending_pairs", 64)))
        self._camera_info = None
        self._active_trial = None
        self._captured = 0
        self._trial_counts = {trial_id: 0 for trial_id in self._trials}
        self._records = []
        self._fatal_error = ""
        self._run_complete = False
        os.makedirs(self._output_dir, exist_ok=True)
        if os.listdir(self._output_dir):
            raise ValueError(
                "failure capture output_dir must be empty: {}".format(
                    self._output_dir))

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
            rospy.get_param("~trial_event_topic",
                            "/uav_vision_eval/vsim04/trial_event"),
            String, self._on_event, queue_size=20)
        rospy.on_shutdown(self._write_manifest)

    def _fail(self, error):
        if self._fatal_error:
            return
        self._fatal_error = str(error)
        rospy.logfatal("V-SIM failure capture failed closed: %s", error)
        self._write_manifest()
        rospy.signal_shutdown(self._fatal_error)

    def _on_camera_info(self, message):
        with self._lock:
            self._camera_info = message

    def _on_event(self, message):
        try:
            payload = json.loads(message.data)
            event = payload.get("event")
            trial_id = payload.get("trial_id", "")
            with self._lock:
                if event == "trial_start":
                    self._pair_buffer.clear()
                    self._active_trial = self._trials.get(trial_id)
                elif event == "trial_end" and self._active_trial is not None:
                    if self._active_trial["trial_id"] == trial_id:
                        if self._trial_counts[trial_id] == 0:
                            raise RuntimeError(
                                "selected trial produced no exact-stamp "
                                "truth/image capture: {}".format(trial_id))
                        self._active_trial = None
                        self._pair_buffer.clear()
                elif event == "run_abort":
                    raise RuntimeError(
                        "V-SIM run aborted: {}".format(
                            payload.get("error", "unknown")))
                elif event == "run_complete":
                    if any(count == 0 for count in self._trial_counts.values()):
                        raise RuntimeError(
                            "one or more selected trials produced no capture")
                    self._active_trial = None
                    self._pair_buffer.clear()
                    self._run_complete = True
                    self._write_manifest()
        except Exception as error:
            self._fail(error)

    def _on_image(self, message):
        try:
            with self._lock:
                if not self._active_trial_has_budget():
                    return
                pair = self._pair_buffer.add_image(message)
                if pair is not None:
                    self._capture_pair(*pair)
        except Exception as error:
            self._fail(error)

    def _on_truth(self, message):
        try:
            with self._lock:
                if not self._active_trial_has_budget():
                    return
                pair = self._pair_buffer.add_truth(message)
                if pair is not None:
                    self._capture_pair(*pair)
        except Exception as error:
            self._fail(error)

    @staticmethod
    def _atomic_json(path, payload):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)

    def _active_trial_has_budget(self):
        trial = self._active_trial
        if trial is None or self._captured >= self._max_frames:
            return False
        trial_id = trial["trial_id"]
        return self._trial_counts[trial_id] < self._trial_quotas[trial_id]

    def _capture_pair(self, image, truth):
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
        capture_index = self._captured + 1
        stem = "{:04d}_{}_{}_{:09d}".format(
            capture_index, trial["trial_id"],
            int(image.header.stamp.secs), int(image.header.stamp.nsecs))
        image_name = stem + ".png"
        image_path = os.path.join(self._output_dir, image_name)
        temporary_image_path = image_path + ".tmp.png"
        record = build_frame_record(
            trial, image, truth, target, self._camera_info,
            image_name, capture_index)
        cv_image = self._bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough")
        if not cv2.imwrite(temporary_image_path, cv_image):
            raise IOError("cv2.imwrite failed: {}".format(image_path))
        os.replace(temporary_image_path, image_path)
        self._atomic_json(os.path.join(
            self._output_dir, stem + ".json"), record)
        self._records.append(record)
        self._captured = capture_index
        self._trial_counts[trial["trial_id"]] += 1
        self._write_manifest()

    def _write_manifest(self):
        with self._lock:
            if not self._output_dir:
                return
            payload = {
                "schema_version": CAPTURE_SCHEMA_VERSION,
                "dataset_kind": "sim-small-target",
                "status": (
                    "FAIL" if self._fatal_error else
                    "DIAGNOSTIC" if self._run_complete else "INCOMPLETE"),
                "error": self._fatal_error,
                "run_complete": self._run_complete,
                "trial_selector": self._selector,
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
