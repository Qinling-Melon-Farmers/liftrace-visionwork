#!/usr/bin/env python3
"""Record V-SIM-04 trial-level confirmation/selection performance."""

import json
import math
import os
import threading
import time
from collections import OrderedDict

import rospy
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
    load_trial_matrix,
    planned_trial_result,
    write_artifacts,
)


class VSim04TrialRecorder:
    def __init__(self):
        rospy.init_node("vsim04_trial_recorder")
        self._lock = threading.RLock()
        self._matrix_path = os.path.abspath(rospy.get_param("~matrix_file"))
        self._matrix = load_trial_matrix(self._matrix_path)
        self._trial_specs = {
            trial["trial_id"]: trial for trial in self._matrix["trials"]}
        self._results = OrderedDict(
            (trial["trial_id"], planned_trial_result(trial))
            for trial in self._matrix["trials"])
        self._anchors = self._matrix["target_anchors"]
        self._profile, self._allowed_classes = resolve_class_profile(
            rospy.get_param(
                "~class_profile", self._matrix["class_profile"]))
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
        self._active = None
        self._window_open = False
        self._entered = False
        self._exposure_start_stamp = None
        self._confirmed_id = None
        self._event_seq = 0
        self._events = []
        self._frames = OrderedDict()
        self._eligible_keys = set()
        self._detection_keys = set()
        self._truth_centers = {}
        self._image_receipts = OrderedDict()
        self._image_stamps = set()
        self._camera_info = None
        self._run_complete = False
        model_path = rospy.get_param("~model_path", "")
        if (bool(rospy.get_param("~require_model_path", True)) and
                not str(model_path).strip()):
            raise ValueError(
                "V-SIM-04 requires an explicit non-empty model_path")
        self._manifest = {
            "seed": self._matrix["seed"],
            "class_profile": self._profile,
            "matrix_file": self._matrix_path,
            "trials": self._matrix["trials"],
            "model": {
                "path": model_path,
                "backend": rospy.get_param("~model_backend", "dev_sim"),
            },
            "thresholds": {
                "confirm_frames": self._confirm_frames,
                "selected_max_age_sec": self._selected_max_age,
                "detector_class_confidence": rospy.get_param(
                    "/target_detector/conf_threshold", None),
                "standard_class_confidence": rospy.get_param(
                    "/target_memory/std_class_confidence", None),
                "standard_geometry_confidence": rospy.get_param(
                    "/target_memory/std_geometry_confidence", None),
                "cross_class_confidence": rospy.get_param(
                    "/target_memory/cross_class_confidence", None),
                "cross_geometry_confidence": rospy.get_param(
                    "/target_memory/cross_geometry_confidence", None),
            },
            "camera_info": None,
            "extrinsic_profile": rospy.get_param(
                "~extrinsic_profile", "unspecified"),
            "revisions": {
                "vision": rospy.get_param("~vision_revision", "unknown"),
                "navigation": rospy.get_param(
                    "~navigation_revision", "unknown"),
            },
            "topics": {
                "trial_event": rospy.get_param(
                    "~trial_event_topic",
                    "/uav_vision_eval/vsim04/trial_event"),
                "truth": rospy.get_param(
                    "~truth_topic", "/uav_vision_eval/ground_truth"),
                "detections_mapped": rospy.get_param(
                    "~detections_topic", "/uav_vision/detections_mapped"),
                "targets": rospy.get_param(
                    "~targets_topic", "/uav_vision/targets"),
                "selected_target": rospy.get_param(
                    "~selected_topic", "/uav_vision/selected_target"),
            },
        }

        rospy.Subscriber(
            self._manifest["topics"]["trial_event"], String,
            self._on_trial_event, queue_size=20)
        rospy.Subscriber(
            self._manifest["topics"]["truth"], SimTargetArray,
            self._on_truth, queue_size=20)
        rospy.Subscriber(
            self._manifest["topics"]["detections_mapped"],
            TargetDetectionArray, self._on_detections, queue_size=20)
        rospy.Subscriber(
            self._manifest["topics"]["targets"], TargetCandidateArray,
            self._on_targets, queue_size=20)
        rospy.Subscriber(
            self._manifest["topics"]["selected_target"], TargetCandidate,
            self._on_selected, queue_size=20)
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image, self._on_image, queue_size=20)
        rospy.Subscriber(
            rospy.get_param(
                "~camera_info_topic", "/camera/color/camera_info"),
            CameraInfo, self._on_camera_info, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(1.0), self._on_timer)
        rospy.on_shutdown(self._write)
        self._write()
        rospy.loginfo(
            "[VSim04Recorder] ready profile=%s trials=%d output=%s",
            self._profile, len(self._trial_specs), self._output_dir)

    @staticmethod
    def _stamp_sec(message):
        return message.header.stamp.to_sec()

    @staticmethod
    def _stamp_key(message):
        return message.header.stamp.to_nsec()

    def _result(self):
        return self._results[self._active] if self._active else None

    def _frame(self, stamp_key, stamp_sec):
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
                "detection_present": False,
                "map_valid": False,
                "transform_failure": False,
                "reject_reason": "",
                "map_error_xy": "",
                "current_confirmed": False,
                "current_selected": False,
                "stable_id": "",
            }
        return self._frames[key]

    def _add_event(self, event, source_stamp=None, stable_id=None, details=None):
        self._event_seq += 1
        spec = self._trial_specs.get(self._active, {})
        self._events.append({
            "event_seq": self._event_seq,
            "trial_id": self._active or "",
            "event": event,
            "source_stamp": (
                "{:.9f}".format(float(source_stamp))
                if source_stamp is not None else ""),
            "monotonic_sec": "{:.9f}".format(time.monotonic()),
            "class_name": spec.get("class_name", ""),
            "stable_id": "" if stable_id is None else int(stable_id),
            "details": json.dumps(details or {}, sort_keys=True),
        })

    def _on_trial_event(self, message):
        with self._lock:
            try:
                event = json.loads(message.data)
                event_name = event["event"]
                if event_name == "trial_start":
                    self._start_trial(event["trial_id"])
                elif event_name == "trial_end":
                    self._finish_trial(event["trial_id"])
                elif event_name == "run_complete":
                    if self._active:
                        self._finish_trial(self._active)
                    self._run_complete = True
                    self._add_event("run_complete", details=event)
                    self._write()
            except Exception as error:
                rospy.logerr("V-SIM-04 trial event rejected: %s", error)

    def _start_trial(self, trial_id):
        if trial_id not in self._trial_specs:
            raise ValueError("unknown trial_id: " + str(trial_id))
        if self._active:
            raise RuntimeError("trial overlap: {} -> {}".format(
                self._active, trial_id))
        self._active = trial_id
        result = self._results[trial_id]
        result.update({
            "status": "running",
            "p_confirm": False,
            "p_selected": False,
            "p_interrupt": None,
            "stable_id": None,
            "confirmation_exposure_sec": None,
            "confirmation_processing_ms": None,
            "eligible_frames": 0,
            "detection_frames": 0,
            "map_valid_frames": 0,
            "tf_failure_frames": 0,
            "map_errors_xy": [],
        })
        self._window_open = False
        self._entered = False
        self._exposure_start_stamp = None
        self._confirmed_id = None
        self._add_event("trial_start")

    def _finish_trial(self, trial_id):
        if self._active != trial_id:
            raise RuntimeError("trial_end does not match active trial")
        result = self._result()
        result["status"] = "completed"
        self._add_event("trial_end", details={
            "entered_fully_in_frame": self._entered,
            "window_open": self._window_open,
        })
        self._active = None
        self._window_open = False
        self._write()

    def _on_image(self, message):
        with self._lock:
            key = self._stamp_key(message)
            self._image_receipts[key] = time.monotonic()
            self._image_stamps.add(self._stamp_sec(message))
            while len(self._image_receipts) > 2000:
                self._image_receipts.popitem(last=False)

    def _on_camera_info(self, message):
        with self._lock:
            if self._camera_info is not None:
                return
            self._camera_info = {
                "width": int(message.width),
                "height": int(message.height),
                "distortion_model": message.distortion_model,
                "K": [float(value) for value in message.K],
                "D": [float(value) for value in message.D],
                "P": [float(value) for value in message.P],
                "frame_id": message.header.frame_id,
            }
            self._manifest["camera_info"] = self._camera_info

    def _on_truth(self, message):
        with self._lock:
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
            frame = self._frame(stamp_key, stamp_sec)
            frame["fully_in_frame"] = bool(target.fully_in_frame)
            frame["center_in_frame"] = bool(target.center_in_frame)
            self._truth_centers[(self._active, stamp_key)] = (
                float(target.world_center.x), float(target.world_center.y))
            if target.fully_in_frame:
                if (self._active, stamp_key) not in self._eligible_keys:
                    self._eligible_keys.add((self._active, stamp_key))
                    self._result()["eligible_frames"] += 1
                if not self._entered:
                    self._entered = True
                    self._window_open = True
                    self._exposure_start_stamp = stamp_sec
                    self._add_event("target_entered_fully_in_frame", stamp_sec)
            elif self._window_open:
                self._window_open = False
                self._add_event("target_left_fully_in_frame", stamp_sec)

    @staticmethod
    def _is_transform_failure(reason):
        value = str(reason).lower()
        return "transform" in value or value.startswith("tf_")

    def _on_detections(self, message):
        with self._lock:
            if not self._active:
                return
            stamp_key = self._stamp_key(message)
            frame = self._frames.get((self._active, stamp_key))
            if not frame or not frame["fully_in_frame"]:
                return
            count_key = (self._active, stamp_key)
            expected_class = self._trial_specs[self._active]["class_name"]
            detections = [detection for detection in message.detections
                          if detection.class_name == expected_class]
            if not detections:
                return
            result = self._result()
            if count_key not in self._detection_keys:
                self._detection_keys.add(count_key)
                result["detection_frames"] += 1
            frame["detection_present"] = True
            reasons = sorted(set(filter(None, (
                frame["reject_reason"].split(";") if frame["reject_reason"]
                else []))) | {
                    detection.reject_reason for detection in detections
                    if detection.reject_reason})
            frame["reject_reason"] = ";".join(reasons)
            transform_failure = any(
                self._is_transform_failure(reason) for reason in reasons)
            if transform_failure and not frame["transform_failure"]:
                result["tf_failure_frames"] += 1
            frame["transform_failure"] = transform_failure
            valid = [detection for detection in detections
                     if detection.map_valid]
            if not valid:
                return
            if not frame["map_valid"]:
                result["map_valid_frames"] += 1
            frame["map_valid"] = True
            truth_center = self._truth_centers.get(count_key)
            if truth_center is None:
                return
            errors = [math.hypot(
                float(detection.map_point.x) - truth_center[0],
                float(detection.map_point.y) - truth_center[1])
                for detection in valid]
            error = min(errors)
            previous = frame["map_error_xy"]
            if previous == "" or error < float(previous):
                if previous != "":
                    result["map_errors_xy"].remove(float(previous))
                frame["map_error_xy"] = error
                result["map_errors_xy"].append(error)

    def _on_targets(self, message):
        with self._lock:
            if not self._active or not self._window_open:
                return
            expected_class = self._trial_specs[self._active]["class_name"]
            candidates = [candidate for candidate in message.targets
                          if candidate.class_name == expected_class]
            eligible = [candidate for candidate in candidates
                        if candidate_is_currently_selectable(
                            candidate, message.header.stamp,
                            self._confirm_frames, self._selected_max_age,
                            self._priorities, self._allowed_classes)]
            if not eligible:
                return
            candidate = max(eligible, key=lambda item: (
                item.consecutive_observe_count, item.class_confidence,
                item.map_quality, -item.id))
            last_seen = candidate.last_seen.to_sec()
            if (self._exposure_start_stamp is None or
                    last_seen < self._exposure_start_stamp):
                return
            result = self._result()
            stamp_key = message.header.stamp.to_nsec()
            frame = self._frame(stamp_key, message.header.stamp.to_sec())
            frame["current_confirmed"] = True
            frame["stable_id"] = int(candidate.id)
            if result["p_confirm"]:
                return
            result["p_confirm"] = True
            result["stable_id"] = int(candidate.id)
            result["confirmation_exposure_sec"] = max(
                0.0, last_seen - self._exposure_start_stamp)
            receipt = self._image_receipts.get(candidate.last_seen.to_nsec())
            if receipt is not None:
                result["confirmation_processing_ms"] = max(
                    0.0, (time.monotonic() - receipt) * 1000.0)
            self._confirmed_id = int(candidate.id)
            self._add_event(
                "candidate_confirmed", last_seen, candidate.id,
                details={
                    "consecutive_observe_count": int(
                        candidate.consecutive_observe_count),
                    "map_valid": bool(candidate.map_valid),
                    "association_valid": bool(candidate.association_valid),
                    "reject_reason": candidate.reject_reason,
                })

    def _on_selected(self, candidate):
        with self._lock:
            if (not self._active or not self._window_open or
                    self._confirmed_id is None or
                    int(candidate.id) != self._confirmed_id or
                    candidate.class_name !=
                    self._trial_specs[self._active]["class_name"]):
                return
            result = self._result()
            stamp_key = candidate.last_seen.to_nsec()
            frame = self._frame(stamp_key, candidate.last_seen.to_sec())
            frame["current_selected"] = True
            frame["stable_id"] = int(candidate.id)
            if result["p_selected"]:
                return
            result["p_selected"] = True
            self._add_event(
                "candidate_selected", candidate.last_seen.to_sec(),
                candidate.id)

    def _actual_fps(self):
        stamps = sorted(self._image_stamps)
        duration = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
        return ((len(stamps) - 1) / duration
                if duration > 0.0 else None)

    def _write(self):
        with self._lock:
            try:
                write_artifacts(
                    self._output_dir, self._manifest,
                    list(self._frames.values()), self._events,
                    list(self._results.values()), "ros_visual_only",
                    self._actual_fps())
            except Exception as error:
                rospy.logerr_throttle(
                    5.0, "V-SIM-04 artifact write failed: %s", error)

    def _on_timer(self, _event):
        self._write()


def main():
    VSim04TrialRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
