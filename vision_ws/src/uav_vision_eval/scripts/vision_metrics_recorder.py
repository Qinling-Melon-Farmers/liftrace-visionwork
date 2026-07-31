#!/usr/bin/env python3
"""Match vision detections to independent simulation truth and write metrics."""

import csv
import json
import math
import os
import statistics
import threading
import time
from collections import Counter, defaultdict
from collections import OrderedDict

import rospy
import yaml

from uav_vision.msg import TargetDetectionArray
from uav_vision_eval.msg import SimTargetArray
from sensor_msgs.msg import Image


FIELDS = [
    "stamp", "scenario_id", "frame_index", "match_status", "target_id", "truth_class",
    "detection_index", "detection_class", "class_confidence", "geometry_confidence",
    "geometry_verified", "center_refined", "truth_x", "truth_y", "det_x", "det_y",
    "pixel_error", "truth_fully_in_frame", "map_valid", "map_frame", "map_x", "map_y",
    "map_z", "map_error_xy", "latency_ms",
]


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


class MetricsRecorder:
    def __init__(self):
        scenario_path = rospy.get_param("~scenario_file")
        with open(scenario_path, "r", encoding="utf-8") as stream:
            self.scenario = yaml.safe_load(stream)
        self.scenario_id = self.scenario["scenario_id"]
        self.require_fully_in_frame = bool(self.scenario.get("require_fully_in_frame", True))
        self.max_pair_age = float(rospy.get_param("~max_pair_age_sec", 0.25))
        self.max_match_distance = float(rospy.get_param("~max_match_distance_px", 160.0))
        # phase_d 的几何分支与 YOLO 分支会为同一图像时间戳异步到达。
        # 指标必须按图像帧聚合一次，不能把早到/晚到的中间数组重复计分。
        self.aggregation_delay = float(
            rospy.get_param("~aggregation_delay_sec", 0.8))
        self.detection_dedup_distance = float(
            rospy.get_param("~detection_dedup_distance_px", 30.0))
        self.ignored_detection_classes = set(
            rospy.get_param("~ignored_detection_classes", ["circle"]))
        self.require_geometry_verified = bool(
            rospy.get_param("~require_geometry_verified_detections", False))
        self.required_scoring_source = rospy.get_param(
            "~required_scoring_source", "").strip()
        self.output_dir = os.path.abspath(rospy.get_param("~output_dir", "/tmp/uav_vision_eval/latest"))
        self.warmup_frames = int(rospy.get_param("~warmup_frames", 0))
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, "frames.csv")
        self.summary_path = os.path.join(self.output_dir, "summary.json")
        self.manifest_path = os.path.join(self.output_dir, "manifest.json")
        self.lock = threading.RLock()
        self.truth_messages = []
        self.image_receipts = OrderedDict()
        self.pending_detections = []
        self.detection_groups = {}
        self.rows = []
        self.frame_latencies = []
        self.frame_count = 0
        self.received_frame_count = 0
        self.stage_topics = {
            "raw": rospy.get_param(
                "~raw_detections_topic", "/uav_vision/detections"),
            "resolved": rospy.get_param(
                "~resolved_detections_topic", "/uav_vision/detections_resolved"),
            "refined": rospy.get_param(
                "~refined_detections_topic", "/uav_vision/detections_refined"),
            "mapped": rospy.get_param(
                "~detections_topic", "/uav_vision/detections_mapped"),
        }
        self.stage_audit = {
            stage: {
                "messages": 0,
                "nonempty_messages": 0,
                "class_detections": Counter(),
                "class_unique_stamps": defaultdict(set),
                "unique_stamps": set(),
                "geometry_verified": 0,
                "center_refined": 0,
                "map_valid": 0,
                "sources": Counter(),
                "completed_source_sets": Counter(),
            }
            for stage in self.stage_topics
        }
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.csv_writer.writeheader()

        self.gates = {
            "min_scored_frames": int(rospy.get_param("~min_scored_frames", 1)),
            "min_precision": float(rospy.get_param("~min_precision", 0.0)),
            "min_recall": float(rospy.get_param("~min_recall", 0.0)),
            "min_input_coverage": float(
                rospy.get_param("~min_input_coverage", 0.0)),
            "max_mean_pixel_error": float(rospy.get_param("~max_mean_pixel_error", 1.0e9)),
            "max_p95_pixel_error": float(rospy.get_param("~max_p95_pixel_error", 1.0e9)),
            "max_mean_map_error_xy": float(rospy.get_param("~max_mean_map_error_xy", 1.0e9)),
            "max_p95_map_error_xy": float(rospy.get_param("~max_p95_map_error_xy", 1.0e9)),
            "max_p95_latency_ms": float(rospy.get_param("~max_p95_latency_ms", 1.0e9)),
            "max_false_positives": int(rospy.get_param("~max_false_positives", 2147483647)),
            "require_pixel_error": bool(rospy.get_param("~require_pixel_error", True)),
            "require_map_error": bool(rospy.get_param("~require_map_error", True)),
            "require_latency": bool(rospy.get_param("~require_latency", True)),
        }
        self._write_json(self.manifest_path, {
            "scenario": self.scenario,
            "evaluation_seed": int(rospy.get_param("~evaluation_seed", -1)),
            "gate_profile": rospy.get_param("~gate_profile", "custom"),
            "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
            "gazebo_master_uri": os.environ.get("GAZEBO_MASTER_URI", ""),
            "camera_pose": {
                "x": float(rospy.get_param("~camera_x", 0.0)),
                "y": float(rospy.get_param("~camera_y", 0.0)),
                "z": float(rospy.get_param("~camera_z", 0.0)),
                "roll": float(rospy.get_param("~camera_roll", 0.0)),
                "pitch": float(rospy.get_param("~camera_pitch", 0.0)),
                "yaw": float(rospy.get_param("~camera_yaw", 0.0)),
            },
            "scenario_file": os.path.abspath(scenario_path),
            "truth_topic": rospy.get_param("~truth_topic", "/uav_vision_eval/ground_truth"),
            "detections_topic": rospy.get_param("~detections_topic", "/uav_vision/detections_mapped"),
            "stage_topics": self.stage_topics,
            "gates": self.gates,
            "ignored_detection_classes": sorted(self.ignored_detection_classes),
            "warmup_frames": self.warmup_frames,
            "require_geometry_verified_detections": self.require_geometry_verified,
            "required_scoring_source": self.required_scoring_source,
            "aggregation_delay_sec": self.aggregation_delay,
            "detection_dedup_distance_px": self.detection_dedup_distance,
            "latency_clock": "perf_counter_from_image_receipt_to_mapped_receipt",
        })

        rospy.Subscriber(
            rospy.get_param("~truth_topic", "/uav_vision_eval/ground_truth"),
            SimTargetArray, self._truth_callback, queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image, self._image_callback, queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~detections_topic", "/uav_vision/detections_mapped"),
            TargetDetectionArray, self._detections_callback, queue_size=10,
        )
        for stage in ("raw", "resolved", "refined"):
            rospy.Subscriber(
                self.stage_topics[stage], TargetDetectionArray,
                self._stage_callback, callback_args=stage, queue_size=10,
            )
        self.flush_timer = rospy.Timer(rospy.Duration(0.05), self._flush_timer_callback)
        self.summary_timer = rospy.Timer(rospy.Duration(1.0), self._timer_callback)
        rospy.on_shutdown(self.close)

    @staticmethod
    def _stamp(message):
        return message.header.stamp.to_sec()

    def _truth_callback(self, message):
        with self.lock:
            if self.csv_file.closed:
                return
            self.truth_messages.append(message)
            self.truth_messages = self.truth_messages[-100:]
            pending = self.pending_detections
            self.pending_detections = []
            for detection_message, receipt_time in pending:
                if not self._score_if_truth_available(detection_message, receipt_time):
                    self.pending_detections.append((detection_message, receipt_time))

    def _image_callback(self, message):
        with self.lock:
            key = message.header.stamp.to_nsec()
            self.image_receipts[key] = time.perf_counter()
            while len(self.image_receipts) > 500:
                self.image_receipts.popitem(last=False)

    def _detections_callback(self, message):
        with self.lock:
            if self.csv_file.closed:
                return
            self._record_stage("mapped", message)
            if self.required_scoring_source and self.required_scoring_source not in \
                    message.completed_sources:
                return
            key = message.header.stamp.to_nsec()
            group = self.detection_groups.setdefault(key, [])
            group.append((message, time.perf_counter()))
            self._flush_mature_groups(rospy.Time.now())

    def _stage_callback(self, message, stage):
        with self.lock:
            if self.csv_file.closed:
                return
            self._record_stage(stage, message)

    def _record_stage(self, stage, message):
        audit = self.stage_audit[stage]
        stamp = message.header.stamp.to_nsec()
        audit["messages"] += 1
        audit["sources"][message.source or "<empty>"] += 1
        completed_key = ",".join(sorted(message.completed_sources)) or "<empty>"
        audit["completed_source_sets"][completed_key] += 1
        audit["unique_stamps"].add(stamp)
        if message.detections:
            audit["nonempty_messages"] += 1
        for detection in message.detections:
            class_name = detection.class_name or "<empty>"
            audit["class_detections"][class_name] += 1
            audit["class_unique_stamps"][class_name].add(stamp)
            audit["geometry_verified"] += int(detection.geometry_verified)
            audit["center_refined"] += int(detection.center_refined)
            audit["map_valid"] += int(detection.map_valid)

    @staticmethod
    def _detection_quality(detection):
        return (
            bool(detection.geometry_verified), bool(detection.center_refined),
            bool(detection.map_valid), float(detection.geometry_confidence),
            float(detection.class_confidence),
        )

    def _merge_detection_group(self, entries):
        messages = [entry[0] for entry in entries]
        merged = TargetDetectionArray()
        merged.header = messages[0].header
        selected = []
        for message in messages:
            for detection in message.detections:
                duplicate_index = None
                for index, existing in enumerate(selected):
                    if existing.class_name != detection.class_name:
                        continue
                    distance = math.hypot(
                        existing.center_px.x - detection.center_px.x,
                        existing.center_px.y - detection.center_px.y,
                    )
                    if distance <= self.detection_dedup_distance:
                        duplicate_index = index
                        break
                if duplicate_index is None:
                    selected.append(detection)
                elif self._detection_quality(detection) > self._detection_quality(
                        selected[duplicate_index]):
                    selected[duplicate_index] = detection
        merged.detections = selected
        receipt_time = max(entry[1] for entry in entries)
        return merged, receipt_time

    def _flush_mature_groups(self, now):
        if now.to_sec() <= 0.0:
            return
        mature_keys = []
        for key, entries in self.detection_groups.items():
            stamp = entries[0][0].header.stamp
            if (now - stamp).to_sec() >= self.aggregation_delay:
                mature_keys.append(key)
        for key in sorted(mature_keys):
            merged, receipt_time = self._merge_detection_group(
                self.detection_groups.pop(key))
            if not self._score_if_truth_available(merged, receipt_time):
                self.pending_detections.append((merged, receipt_time))
                self.pending_detections = self.pending_detections[-100:]

    def _score_if_truth_available(self, detections, receipt_time):
        if not self.truth_messages:
            return False
        detection_stamp = self._stamp(detections)
        truth = min(self.truth_messages, key=lambda item: abs(self._stamp(item) - detection_stamp))
        if abs(self._stamp(truth) - detection_stamp) > self.max_pair_age:
            return False
        self._score_frame(truth, detections, receipt_time)
        return True

    def _eligible_truth(self, truth):
        result = []
        for target in truth.targets:
            if not target.pose_valid or not target.projection_valid:
                continue
            if self.require_fully_in_frame and not target.fully_in_frame:
                continue
            result.append(target)
        return result

    @staticmethod
    def _pixel_error(target, detection):
        return math.hypot(
            detection.center_px.x - target.pixel_center.x,
            detection.center_px.y - target.pixel_center.y,
        )

    def _base_row(self, truth, detections, latency_ms):
        return {
            "stamp": "{:.9f}".format(self._stamp(detections)),
            "scenario_id": truth.scenario_id or self.scenario_id,
            "frame_index": self.frame_count,
            "latency_ms": latency_ms,
        }

    def _score_frame(self, truth, detections, receipt_time):
        if self.csv_file.closed:
            return
        self.received_frame_count += 1
        if self.received_frame_count <= self.warmup_frames:
            return
        self.frame_count += 1
        # Aggregation deliberately waits before scoring, but that recorder-side
        # wait is not algorithm latency. Measure when the last mapped output
        # for this source image actually arrived at the recorder.
        image_receipt = self.image_receipts.get(
            detections.header.stamp.to_nsec())
        latency_ms = (
            max(0.0, (receipt_time - image_receipt) * 1000.0)
            if image_receipt is not None else None
        )
        if latency_ms is not None:
            self.frame_latencies.append(latency_ms)
        truths = self._eligible_truth(truth)
        unmatched_truth = set(range(len(truths)))
        scored_detections = [
            (original_index, detection)
            for original_index, detection in enumerate(detections.detections)
            if detection.class_name not in self.ignored_detection_classes and
            (not self.require_geometry_verified or
             (detection.geometry_verified and detection.center_refined))
        ]
        unmatched_detection = set(range(len(scored_detections)))
        pairs = []
        candidates = []
        for truth_index, target in enumerate(truths):
            for detection_index, (_original_index, detection) in enumerate(scored_detections):
                if target.class_name != detection.class_name:
                    continue
                error = self._pixel_error(target, detection)
                if error <= self.max_match_distance:
                    candidates.append((error, truth_index, detection_index))
        for error, truth_index, detection_index in sorted(candidates):
            if truth_index in unmatched_truth and detection_index in unmatched_detection:
                unmatched_truth.remove(truth_index)
                unmatched_detection.remove(detection_index)
                pairs.append((truth_index, detection_index, error))

        new_rows = []
        for truth_index, detection_index, pixel_error in pairs:
            target = truths[truth_index]
            original_index, detection = scored_detections[detection_index]
            row = self._base_row(truth, detections, latency_ms)
            row.update({
                "match_status": "true_positive", "target_id": target.target_id,
                "truth_class": target.class_name, "detection_index": original_index,
                "detection_class": detection.class_name,
                "class_confidence": detection.class_confidence,
                "geometry_confidence": detection.geometry_confidence,
                "geometry_verified": detection.geometry_verified,
                "center_refined": detection.center_refined,
                "truth_x": target.pixel_center.x, "truth_y": target.pixel_center.y,
                "det_x": detection.center_px.x, "det_y": detection.center_px.y,
                "pixel_error": pixel_error,
                "truth_fully_in_frame": target.fully_in_frame,
                "map_valid": detection.map_valid, "map_frame": detection.map_frame,
                "map_x": detection.map_point.x, "map_y": detection.map_point.y,
                "map_z": detection.map_point.z,
                "map_error_xy": math.hypot(
                    detection.map_point.x - target.world_center.x,
                    detection.map_point.y - target.world_center.y,
                ) if detection.map_valid and detection.map_frame == "world" else "",
            })
            new_rows.append(row)

        for truth_index in sorted(unmatched_truth):
            target = truths[truth_index]
            row = self._base_row(truth, detections, latency_ms)
            row.update({
                "match_status": "false_negative", "target_id": target.target_id,
                "truth_class": target.class_name, "truth_x": target.pixel_center.x,
                "truth_y": target.pixel_center.y,
                "truth_fully_in_frame": target.fully_in_frame,
            })
            new_rows.append(row)

        for detection_index in sorted(unmatched_detection):
            original_index, detection = scored_detections[detection_index]
            row = self._base_row(truth, detections, latency_ms)
            row.update({
                "match_status": "false_positive", "detection_index": original_index,
                "detection_class": detection.class_name,
                "class_confidence": detection.class_confidence,
                "geometry_confidence": detection.geometry_confidence,
                "geometry_verified": detection.geometry_verified,
                "center_refined": detection.center_refined,
                "det_x": detection.center_px.x, "det_y": detection.center_px.y,
                "map_valid": detection.map_valid, "map_frame": detection.map_frame,
                "map_x": detection.map_point.x, "map_y": detection.map_point.y,
                "map_z": detection.map_point.z,
            })
            new_rows.append(row)

        if not new_rows:
            row = self._base_row(truth, detections, latency_ms)
            row.update({"match_status": "true_negative"})
            new_rows.append(row)

        for row in new_rows:
            normalized = {field: row.get(field, "") for field in FIELDS}
            self.rows.append(normalized)
            self.csv_writer.writerow(normalized)
        self.csv_file.flush()

    def _summary(self):
        true_positive = sum(row["match_status"] == "true_positive" for row in self.rows)
        false_positive = sum(row["match_status"] == "false_positive" for row in self.rows)
        false_negative = sum(row["match_status"] == "false_negative" for row in self.rows)
        pixel_errors = [float(row["pixel_error"]) for row in self.rows if row["pixel_error"] != ""]
        map_errors = [float(row["map_error_xy"]) for row in self.rows if row["map_error_xy"] != ""]
        latencies = list(self.frame_latencies)
        raw_sources = self.stage_audit["raw"]["sources"]
        reference_messages = max(raw_sources.values()) if raw_sources else 0
        scoring_source_messages = raw_sources.get(
            self.required_scoring_source, 0) if self.required_scoring_source else 0
        coverage = (
            scoring_source_messages / float(reference_messages)
            if self.required_scoring_source and reference_messages else None
        )
        precision = true_positive / float(true_positive + false_positive) if true_positive + false_positive else 1.0
        recall = true_positive / float(true_positive + false_negative) if true_positive + false_negative else 1.0
        metrics = {
            "scored_frames": self.frame_count,
            "true_positive": true_positive,
            "false_positive": false_positive,
            "false_negative": false_negative,
            "precision": precision,
            "recall": recall,
            "input_coverage": coverage,
            "mean_pixel_error": statistics.mean(pixel_errors) if pixel_errors else None,
            "p95_pixel_error": _percentile(pixel_errors, 95),
            "mean_map_error_xy": statistics.mean(map_errors) if map_errors else None,
            "p95_map_error_xy": _percentile(map_errors, 95),
            "p95_latency_ms": _percentile(latencies, 95),
        }
        checks = {
            "scored_frames": self.frame_count >= self.gates["min_scored_frames"],
            "precision": precision >= self.gates["min_precision"],
            "recall": recall >= self.gates["min_recall"],
            "input_coverage": (
                coverage is not None and
                coverage >= self.gates["min_input_coverage"]),
            "false_positive": false_positive <= self.gates["max_false_positives"],
            "mean_pixel_error": (not self.gates["require_pixel_error"]) or (
                metrics["mean_pixel_error"] is not None and
                metrics["mean_pixel_error"] <= self.gates["max_mean_pixel_error"]),
            "p95_pixel_error": (not self.gates["require_pixel_error"]) or (
                metrics["p95_pixel_error"] is not None and
                metrics["p95_pixel_error"] <= self.gates["max_p95_pixel_error"]),
            "mean_map_error_xy": (not self.gates["require_map_error"]) or (
                metrics["mean_map_error_xy"] is not None and
                metrics["mean_map_error_xy"] <= self.gates["max_mean_map_error_xy"]),
            "p95_map_error_xy": (not self.gates["require_map_error"]) or (
                metrics["p95_map_error_xy"] is not None and
                metrics["p95_map_error_xy"] <= self.gates["max_p95_map_error_xy"]),
            "p95_latency_ms": (not self.gates["require_latency"]) or (
                metrics["p95_latency_ms"] is not None and
                metrics["p95_latency_ms"] <= self.gates["max_p95_latency_ms"]),
        }
        stage_audit = {}
        for stage, audit in self.stage_audit.items():
            stage_audit[stage] = {
                "messages": audit["messages"],
                "nonempty_messages": audit["nonempty_messages"],
                "unique_stamps": len(audit["unique_stamps"]),
                "detections_total": sum(audit["class_detections"].values()),
                "class_detections": dict(sorted(audit["class_detections"].items())),
                "class_unique_stamps": {
                    class_name: len(stamps)
                    for class_name, stamps in sorted(
                        audit["class_unique_stamps"].items())
                },
                "geometry_verified": audit["geometry_verified"],
                "center_refined": audit["center_refined"],
                "map_valid": audit["map_valid"],
                "sources": dict(sorted(audit["sources"].items())),
                "completed_source_sets": dict(sorted(
                    audit["completed_source_sets"].items())),
            }
        input_coverage = {
            "required_scoring_source": self.required_scoring_source,
            "reference_camera_frames": reference_messages,
            "processed_frames": scoring_source_messages,
            "coverage": coverage,
        }
        return {
            "scenario_id": self.scenario_id,
            "metrics": metrics,
            "gates": self.gates,
            "checks": checks,
            "passed": all(checks.values()),
            "stage_audit": stage_audit,
            "input_coverage": input_coverage,
            "files": {"csv": self.csv_path, "manifest": self.manifest_path},
        }

    @staticmethod
    def _write_json(path, value):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)

    def _timer_callback(self, _event):
        with self.lock:
            self._write_json(self.summary_path, self._summary())

    def _flush_timer_callback(self, _event):
        with self.lock:
            if not self.csv_file.closed:
                self._flush_mature_groups(rospy.Time.now())

    def close(self):
        with self.lock:
            if self.csv_file.closed:
                return
            self._write_json(self.summary_path, self._summary())
            self.csv_file.flush()
            self.csv_file.close()


if __name__ == "__main__":
    rospy.init_node("vision_metrics_recorder")
    MetricsRecorder()
    rospy.spin()
