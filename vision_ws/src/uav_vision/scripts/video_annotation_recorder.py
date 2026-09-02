#!/usr/bin/env python3
"""Draw current ROS pixel-chain evidence into one annotated MP4."""

import json
import os
import threading
from collections import defaultdict

import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, UInt32

from uav_vision.msg import TargetDetectionArray
from uav_vision.video_replay_contract import (
    frame_is_complete,
    mapped_message_is_fail_closed,
    validate_perf_status,
    validate_video_metadata,
)


YOLO_COLOR = (0, 220, 0)
GEOMETRY_COLOR = (255, 220, 0)
FUSED_SEARCH_COLOR = (255, 0, 255)
FUSED_LANDING_COLOR = (255, 120, 0)
REFINED_COLOR = (0, 140, 255)
REJECTED_COLOR = (0, 255, 255)


def _stamp_key(header):
    return int(header.stamp.secs), int(header.stamp.nsecs)


def _decode_bgr8(message):
    if message.encoding.lower() != "bgr8":
        raise ValueError("video recorder requires bgr8, got %s" % message.encoding)
    expected_row = int(message.width) * 3
    if int(message.step) < expected_row:
        raise ValueError("invalid image step")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.step) * int(message.height)
    if raw.size < expected_size:
        raise ValueError("short image buffer")
    rows = raw[:expected_size].reshape((int(message.height), int(message.step)))
    return rows[:, :expected_row].reshape(
        (int(message.height), int(message.width), 3)).copy()


def _roi_points(detection):
    x1 = int(detection.roi.x_offset)
    y1 = int(detection.roi.y_offset)
    x2 = x1 + int(detection.roi.width)
    y2 = y1 + int(detection.roi.height)
    return x1, y1, x2, y2


def _center(detection):
    return int(round(detection.center_px.x)), int(round(detection.center_px.y))


def _put_label(image, text, origin, color, scale=0.50):
    x = max(0, min(int(origin[0]), image.shape[1] - 1))
    y = max(18, min(int(origin[1]), image.shape[0] - 4))
    (width, height), baseline = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(
        image,
        (x, max(0, y - height - baseline - 3)),
        (min(image.shape[1] - 1, x + width + 4), min(image.shape[0] - 1, y + 3)),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        image, text, (x + 2, y), cv2.FONT_HERSHEY_SIMPLEX,
        scale, color, 1, cv2.LINE_AA)


def _draw_raw(image, raw_by_source):
    target_message = raw_by_source.get("target_detector")
    if target_message is not None:
        for detection in target_message.detections:
            x1, y1, x2, y2 = _roi_points(detection)
            cv2.rectangle(image, (x1, y1), (x2, y2), YOLO_COLOR, 2)
            cx, cy = _center(detection)
            cv2.drawMarker(
                image, (cx, cy), YOLO_COLOR, cv2.MARKER_CROSS, 10, 1)
            _put_label(
                image,
                "YOLO %s %.2f" % (
                    detection.class_name, detection.class_confidence),
                (x1, max(20, y1 - 4)), YOLO_COLOR)

    for source, prefix in (
            ("circle_detector", "RING"),
            ("cross_detector", "CROSS"),
            ("landing_detector", "H")):
        message = raw_by_source.get(source)
        if message is None:
            continue
        for detection in message.detections:
            cx, cy = _center(detection)
            radius = max(0, int(round(detection.center_px.z)))
            if radius > 0:
                cv2.circle(image, (cx, cy), radius, GEOMETRY_COLOR, 2)
            else:
                x1, y1, x2, y2 = _roi_points(detection)
                cv2.rectangle(image, (x1, y1), (x2, y2), GEOMETRY_COLOR, 2)
            cv2.circle(image, (cx, cy), 5, GEOMETRY_COLOR, -1)
            _put_label(
                image,
                "%s geom %.2f" % (prefix, detection.geometry_confidence),
                (cx + 7, cy - 7), GEOMETRY_COLOR)


def _draw_fused(image, message, prefix, color):
    for detection in message.detections:
        if detection.class_name == "circle":
            continue
        cx, cy = _center(detection)
        cv2.circle(image, (cx, cy), 9, color, 2)
        _put_label(
            image,
            "%s %s %s" % (
                prefix, detection.class_name,
                detection.center_source or "bbox"),
            (cx + 10, cy + 18), color)


def _draw_refined(image, message):
    standard_classes = {"bridge", "panzer", "pillbox", "tent", "tank"}
    for detection in message.detections:
        if detection.class_name not in standard_classes:
            continue
        cx, cy = _center(detection)
        x1, y1, x2, y2 = _roi_points(detection)
        raw_center = ((x1 + x2) // 2, (y1 + y2) // 2)
        if detection.center_refined and detection.association_valid:
            cv2.line(image, raw_center, (cx, cy), REFINED_COLOR, 2)
            cv2.circle(image, (cx, cy), 8, REFINED_COLOR, -1)
            _put_label(
                image,
                "REFINED %s circle_geometry" % detection.class_name,
                (cx + 10, cy - 10), REFINED_COLOR)
        else:
            cv2.circle(image, (cx, cy), 7, REJECTED_COLOR, 2)
            reason = detection.reject_reason or "not_refined"
            _put_label(
                image,
                "RAW-ONLY %s %s" % (detection.class_name, reason),
                (cx + 9, cy + 20), REJECTED_COLOR)


def draw_annotation(frame, frame_index, fps, raw_by_source,
                    resolved_search, resolved_landing, refined,
                    mapped=None, perf_backend=""):
    output = frame.copy()
    _draw_raw(output, raw_by_source)
    _draw_fused(output, resolved_search, "FUSED-SEARCH", FUSED_SEARCH_COLOR)
    _draw_fused(output, resolved_landing, "FUSED-LANDING", FUSED_LANDING_COLOR)
    _draw_refined(output, refined)

    extended = mapped is not None or bool(perf_backend)
    panel_height = 143 if extended else 116
    overlay = output.copy()
    cv2.rectangle(overlay, (0, 0), (output.shape[1], panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.68, output, 0.32, 0.0, output)
    raw_counts = ",".join(
        "%s:%d" % (source, len(message.detections))
        for source, message in sorted(raw_by_source.items()))
    lines = [
        "%s  frame=%d  media=%.3fs  fps=%.3f" % (
            "ROS RKNN CHAIN" if extended else "ROS PIXEL CHAIN",
            frame_index, frame_index / fps, fps),
        "raw {%s}  fused_search=%d fused_landing=%d refined=%d" % (
            raw_counts, len(resolved_search.detections),
            len(resolved_landing.detections), len(refined.detections)),
    ]
    if extended:
        mapped_detections = list(mapped.detections) if mapped is not None else []
        mapped_valid = sum(1 for detection in mapped_detections
                           if detection.map_valid)
        lines.extend([
            "mapped valid=%d invalid=%d  perf=%s" % (
                mapped_valid, len(mapped_detections) - mapped_valid,
                perf_backend or "missing"),
            "NO TF BY DESIGN: map must remain fail-closed; no stable-ID/selected-target",
            "green=YOLO  cyan=geometry  magenta/blue=fused  orange=refined  yellow=raw-only",
        ])
    else:
        lines.extend([
            "green=YOLO  cyan=geometry  magenta/blue=fused stage  orange=ring-refined  yellow=raw-only",
            "PIXEL ONLY: no TF/map/stable-ID/selected-target",
        ])
    for row, text in enumerate(lines):
        cv2.putText(
            output, text, (12, 24 + row * 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255),
            1, cv2.LINE_AA)
    return output


class VideoAnnotationRecorder:
    def __init__(self):
        rospy.init_node("video_annotation_recorder")
        self._output_video = os.path.abspath(
            os.path.expanduser(rospy.get_param("~output_video", "")))
        self._codec = rospy.get_param("~codec", "mp4v")
        self._overwrite = bool(rospy.get_param("~overwrite", False))
        self._require_mapped_perf = bool(
            rospy.get_param("~require_mapped_perf", False))
        self._require_map_fail_closed = bool(
            rospy.get_param("~require_map_fail_closed", False))
        self._expected_perf_name = str(
            rospy.get_param("~expected_perf_name", ""))
        self._expected_perf_backend = str(
            rospy.get_param("~expected_perf_backend", ""))
        if self._require_map_fail_closed and not self._require_mapped_perf:
            raise RuntimeError(
                "require_map_fail_closed requires require_mapped_perf")
        if not self._output_video:
            raise RuntimeError("output_video is required")
        if os.path.abspath(self._output_video).lower().endswith(".mp4") is False:
            raise RuntimeError("output_video must use the .mp4 extension")
        if len(self._codec) != 4:
            raise RuntimeError("codec must contain exactly four characters")
        if os.path.exists(self._output_video) and not self._overwrite:
            raise RuntimeError(
                "output already exists; choose a new path or set overwrite:=true: %s" %
                self._output_video)
        os.makedirs(os.path.dirname(self._output_video), exist_ok=True)

        self._lock = threading.RLock()
        self._metadata = None
        self._writer = None
        self._states = defaultdict(
            lambda: {
                "image": None,
                "frame_index": None,
                "raw": {},
                "resolved_search": None,
                "resolved_landing": None,
                "refined": None,
                "mapped": None,
                "perf": None,
                "perf_backend": "",
            })
        self._finished_keys = set()
        self._images_received = 0
        self._frames_written = 0
        self._perf_messages = 0
        self._mapped_detections = 0
        self._invalid_mapped_detections = 0
        self._failed = False
        self._closed = False

        metadata_topic = rospy.get_param(
            "~metadata_topic", "/uav_vision/video_replay/metadata")
        image_topic = rospy.get_param(
            "~image_topic", "/uav_vision/video_replay/image_raw")
        raw_topic = rospy.get_param(
            "~raw_topic", "/uav_vision/video_replay/detections_raw")
        resolved_search_topic = rospy.get_param(
            "~resolved_search_topic",
            "/uav_vision/video_replay/detections_resolved_search")
        resolved_landing_topic = rospy.get_param(
            "~resolved_landing_topic",
            "/uav_vision/video_replay/detections_resolved_landing")
        refined_topic = rospy.get_param(
            "~refined_topic", "/uav_vision/video_replay/detections_refined")
        mapped_topic = rospy.get_param(
            "~mapped_topic", "/uav_vision/video_replay/detections_mapped")
        perf_topic = rospy.get_param(
            "~perf_topic", "/uav_vision/video_replay/perf")
        input_done_topic = rospy.get_param(
            "~input_done_topic", "/uav_vision/video_replay/input_done")
        frame_done_topic = rospy.get_param(
            "~frame_done_topic", "/uav_vision/video_replay/frame_done")
        output_done_topic = rospy.get_param(
            "~output_done_topic", "/uav_vision/video_replay/output_done")
        error_topic = rospy.get_param(
            "~error_topic", "/uav_vision/video_replay/error")

        self._frame_done_pub = rospy.Publisher(
            frame_done_topic, UInt32, queue_size=10)
        self._output_done_pub = rospy.Publisher(
            output_done_topic, Bool, queue_size=1, latch=True)
        self._error_pub = rospy.Publisher(
            error_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(metadata_topic, String, self._on_metadata, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._on_image,
                         queue_size=1, buff_size=2 ** 25)
        rospy.Subscriber(raw_topic, TargetDetectionArray,
                         self._on_raw, queue_size=32)
        rospy.Subscriber(resolved_search_topic, TargetDetectionArray,
                         self._on_resolved_search, queue_size=4)
        rospy.Subscriber(resolved_landing_topic, TargetDetectionArray,
                         self._on_resolved_landing, queue_size=4)
        rospy.Subscriber(refined_topic, TargetDetectionArray,
                         self._on_refined, queue_size=4)
        if self._require_mapped_perf:
            rospy.Subscriber(mapped_topic, TargetDetectionArray,
                             self._on_mapped, queue_size=4)
            rospy.Subscriber(perf_topic, DiagnosticArray,
                             self._on_perf, queue_size=4)
        rospy.Subscriber(input_done_topic, UInt32,
                         self._on_input_done, queue_size=1)
        rospy.on_shutdown(self._close_writer)

    def _fail(self, reason):
        with self._lock:
            if self._failed:
                return
            self._failed = True
            rospy.logfatal("[VideoAnnotationRecorder] %s", reason)
            self._error_pub.publish(String(data=str(reason)))
            self._close_writer()

    def _on_metadata(self, message):
        try:
            metadata = json.loads(message.data)
            width, height, fps = validate_video_metadata(metadata)
            input_video = os.path.abspath(
                os.path.expanduser(str(metadata.get("input_video", ""))))
            if input_video == self._output_video:
                raise RuntimeError("output_video must not overwrite the input video")
            with self._lock:
                if self._metadata is not None:
                    return
                writer = cv2.VideoWriter(
                    self._output_video,
                    cv2.VideoWriter_fourcc(*self._codec),
                    fps,
                    (width, height),
                )
                if not writer.isOpened():
                    raise RuntimeError(
                        "unable to create output video: %s" % self._output_video)
                self._metadata = metadata
                self._writer = writer
                rospy.loginfo(
                    "[VideoAnnotationRecorder] output=%s %dx%d @ %.6f FPS",
                    self._output_video, width, height, fps)
        except Exception as error:
            self._fail("invalid replay metadata: %s" % error)

    def _on_image(self, message):
        try:
            frame = _decode_bgr8(message)
            key = _stamp_key(message.header)
            with self._lock:
                if key in self._finished_keys:
                    return
                state = self._states[key]
                if state["image"] is None:
                    # ROS publishers own Header.seq and may rewrite it.  The
                    # replay contract permits only one in-flight image, so the
                    # image topic's receive order is the stable frame index.
                    state["frame_index"] = self._images_received
                    self._images_received += 1
                state["image"] = frame
                self._maybe_write(key)
        except Exception as error:
            self._fail("image callback failed: %s" % error)

    def _on_raw(self, message):
        key = _stamp_key(message.header)
        with self._lock:
            if key in self._finished_keys:
                return
            if message.source:
                self._states[key]["raw"][message.source] = message
            self._maybe_write(key)

    def _set_stage(self, message, stage):
        key = _stamp_key(message.header)
        with self._lock:
            if key in self._finished_keys:
                return
            self._states[key][stage] = message
            self._maybe_write(key)

    def _on_resolved_search(self, message):
        self._set_stage(message, "resolved_search")

    def _on_resolved_landing(self, message):
        self._set_stage(message, "resolved_landing")

    def _on_refined(self, message):
        self._set_stage(message, "refined")

    def _on_mapped(self, message):
        key = _stamp_key(message.header)
        with self._lock:
            if key in self._finished_keys:
                return
            if (self._require_map_fail_closed and
                    not mapped_message_is_fail_closed(message)):
                self._fail(
                    "file replay unexpectedly produced a valid map point")
                return
            state = self._states[key]
            if state["mapped"] is None:
                detections = list(message.detections)
                self._mapped_detections += len(detections)
                self._invalid_mapped_detections += sum(
                    1 for detection in detections if not detection.map_valid)
            state["mapped"] = message
            self._maybe_write(key)

    def _on_perf(self, message):
        try:
            backend = validate_perf_status(
                message,
                expected_name=self._expected_perf_name,
                expected_backend=self._expected_perf_backend,
            )
        except ValueError as error:
            self._fail("invalid detector perf evidence: %s" % error)
            return
        key = _stamp_key(message.header)
        with self._lock:
            if key in self._finished_keys:
                return
            state = self._states[key]
            if state["perf"] is None:
                self._perf_messages += 1
            state["perf"] = message
            state["perf_backend"] = backend
            self._maybe_write(key)

    def _maybe_write(self, key):
        if self._failed or self._writer is None:
            return
        state = self._states.get(key)
        if not frame_is_complete(
                state, require_mapped_perf=self._require_mapped_perf):
            return
        frame_index = int(state["frame_index"])
        if frame_index != self._frames_written:
            self._fail(
                "non-sequential frame completion: got %d expected %d" % (
                    frame_index, self._frames_written))
            return
        try:
            annotated = draw_annotation(
                state["image"], frame_index, float(self._metadata["fps"]),
                state["raw"], state["resolved_search"],
                state["resolved_landing"], state["refined"],
                mapped=state["mapped"],
                perf_backend=state["perf_backend"])
            expected_size = (
                int(self._metadata["height"]), int(self._metadata["width"]))
            if annotated.shape[:2] != expected_size:
                raise RuntimeError("annotated frame dimensions changed")
            self._writer.write(annotated)
            self._frames_written += 1
            self._finished_keys.add(key)
            self._states.pop(key, None)
            self._frame_done_pub.publish(UInt32(data=frame_index))
        except Exception as error:
            self._fail("failed to annotate frame %d: %s" % (frame_index, error))

    def _on_input_done(self, message):
        expected_frames = int(message.data)
        with self._lock:
            if self._failed or self._closed:
                return
            if self._frames_written != expected_frames or self._states:
                self._fail(
                    "input finished with incomplete frames: wrote=%d expected=%d pending=%d" % (
                        self._frames_written, expected_frames, len(self._states)))
                return
            self._close_writer()
            self._output_done_pub.publish(Bool(data=True))
            rospy.loginfo(
                "[VideoAnnotationRecorder] finalized %d frames: %s "
                "perf=%d mapped=%d invalid_mapped=%d",
                self._frames_written, self._output_video,
                self._perf_messages,
                self._mapped_detections,
                self._invalid_mapped_detections)

    def _close_writer(self):
        with self._lock:
            if self._closed:
                return
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            self._closed = True


def main():
    VideoAnnotationRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
