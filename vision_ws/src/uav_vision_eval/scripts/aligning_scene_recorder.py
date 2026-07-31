#!/usr/bin/env python3
"""Capture actual flight-camera frames when the old controller aligns.

Images are synchronized with independent Gazebo projection truth. The node
is diagnostic-only: it publishes no planner, controller, or actuator output.
"""
import json
import os
import threading

import cv2
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, String

from uav_vision_eval.msg import SimTargetArray


class CaptureGate:
    """Pure state gate used by the recorder and deterministic regression."""

    def __init__(self, trigger_modes, required_control_state,
                 frames_per_episode, min_interval_sec):
        self._trigger_modes = set(trigger_modes)
        self._required_control_state = int(required_control_state)
        self._frames_per_episode = max(1, int(frames_per_episode))
        self._min_interval_sec = max(0.0, float(min_interval_sec))
        self._active = False
        self._episode = 0
        self._frame = 0
        self._last_capture_stamp = None

    def update(self, align_mode, control_state):
        active = (align_mode in self._trigger_modes and
                  control_state == self._required_control_state)
        if active and not self._active:
            self._episode += 1
            self._frame = 0
            self._last_capture_stamp = None
        self._active = active

    def request_capture(self, stamp_sec):
        stamp_sec = float(stamp_sec)
        if not self._active or self._frame >= self._frames_per_episode:
            return None
        if self._last_capture_stamp is not None:
            delta = stamp_sec - self._last_capture_stamp
            if 0.0 <= delta < self._min_interval_sec:
                return None
        self._frame += 1
        self._last_capture_stamp = stamp_sec
        return self._episode, self._frame


def _to_bgr(message):
    encoding = message.encoding.lower()
    channels = {
        "mono8": 1, "rgb8": 3, "bgr8": 3,
        "rgba8": 4, "bgra8": 4,
    }.get(encoding)
    if channels is None:
        raise ValueError("unsupported encoding: " + message.encoding)
    row_bytes = message.width * channels
    raw = np.frombuffer(message.data, dtype=np.uint8)
    rows = raw[:message.step * message.height].reshape(
        (message.height, message.step))
    pixels = rows[:, :row_bytes]
    if channels == 1:
        return cv2.cvtColor(
            pixels.reshape((message.height, message.width)),
            cv2.COLOR_GRAY2BGR)
    image = pixels.reshape((message.height, message.width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


class AligningSceneRecorder:
    def __init__(self):
        rospy.init_node("aligning_scene_recorder")
        self._output_dir = os.path.abspath(rospy.get_param(
            "~output_dir", "/tmp/uav_vision_eval/aligning_scene"))
        self._manifest_path = os.path.join(self._output_dir, "manifest.json")
        self._lock = threading.RLock()
        self._align_mode = "disabled"
        self._control_state = None
        self._pose = None
        self._captures = []
        self._gate = CaptureGate(
            trigger_modes=rospy.get_param(
                "~trigger_modes", ["drop_circle", "drop_cross"]),
            required_control_state=rospy.get_param(
                "~required_control_state", 2),
            frames_per_episode=rospy.get_param("~frames_per_episode", 5),
            min_interval_sec=rospy.get_param("~min_interval_sec", 0.5),
        )

        rospy.Subscriber(
            rospy.get_param("~align_mode_topic", "/uav_vision/align_mode"),
            String, self._on_align_mode, queue_size=4)
        rospy.Subscriber(
            rospy.get_param("~control_state_topic", "/detect/point_class"),
            Int8, self._on_control_state, queue_size=4)
        rospy.Subscriber(
            rospy.get_param("~pose_topic", "/mavros/local_position/pose"),
            PoseStamped, self._on_pose, queue_size=4)

        image_sub = message_filters.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image, queue_size=2, buff_size=2 ** 24)
        truth_sub = message_filters.Subscriber(
            rospy.get_param(
                "~truth_topic", "/uav_vision_eval/flight_ground_truth"),
            SimTargetArray, queue_size=2)
        self._sync = message_filters.TimeSynchronizer(
            [image_sub, truth_sub], queue_size=20)
        self._sync.registerCallback(self._on_image_truth)

        os.makedirs(self._output_dir, exist_ok=True)
        self._write_manifest()
        rospy.on_shutdown(self._write_manifest)
        rospy.loginfo(
            "[AligningSceneRecorder] ready output=%s", self._output_dir)

    def _update_gate(self):
        self._gate.update(self._align_mode, self._control_state)

    def _on_align_mode(self, message):
        with self._lock:
            self._align_mode = message.data.strip()
            self._update_gate()

    def _on_control_state(self, message):
        with self._lock:
            self._control_state = int(message.data)
            self._update_gate()

    def _on_pose(self, message):
        with self._lock:
            self._pose = message

    @staticmethod
    def _point(point):
        return [float(point.x), float(point.y), float(point.z)]

    @classmethod
    def _truth_target(cls, target):
        return {
            "target_id": target.target_id,
            "class_name": target.class_name,
            "gazebo_link_name": target.gazebo_link_name,
            "pose_valid": bool(target.pose_valid),
            "projection_valid": bool(target.projection_valid),
            "center_in_frame": bool(target.center_in_frame),
            "fully_in_frame": bool(target.fully_in_frame),
            "world_center": cls._point(target.world_center),
            "camera_center": cls._point(target.camera_center),
            "pixel_center": cls._point(target.pixel_center),
            "roi": {
                "x": int(target.roi.x_offset),
                "y": int(target.roi.y_offset),
                "width": int(target.roi.width),
                "height": int(target.roi.height),
            },
            "distance_m": float(target.distance_m),
        }

    @staticmethod
    def _pose_dict(message, image_stamp):
        if message is None:
            return None
        stamp = message.header.stamp.to_sec()
        return {
            "stamp": stamp,
            "age_sec": max(0.0, image_stamp - stamp) if stamp > 0.0 else None,
            "frame_id": message.header.frame_id,
            "position": [
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ],
            "orientation_xyzw": [
                float(message.pose.orientation.x),
                float(message.pose.orientation.y),
                float(message.pose.orientation.z),
                float(message.pose.orientation.w),
            ],
        }

    def _on_image_truth(self, image, truth):
        with self._lock:
            capture = self._gate.request_capture(image.header.stamp.to_sec())
            if capture is None:
                return
            episode, frame = capture
            episode_dir = os.path.join(
                self._output_dir, "episode_%03d" % episode)
            os.makedirs(episode_dir, exist_ok=True)
            relative_path = os.path.join(
                "episode_%03d" % episode, "frame_%03d.png" % frame)
            absolute_path = os.path.join(self._output_dir, relative_path)
            try:
                if not cv2.imwrite(absolute_path, _to_bgr(image)):
                    raise RuntimeError("cv2.imwrite returned false")
                record = {
                    "episode": episode,
                    "frame": frame,
                    "image_file": relative_path,
                    "image_stamp": image.header.stamp.to_sec(),
                    "image_frame_id": image.header.frame_id,
                    "image_size": [int(image.width), int(image.height)],
                    "align_mode": self._align_mode,
                    "control_state": self._control_state,
                    "vehicle_pose": self._pose_dict(
                        self._pose, image.header.stamp.to_sec()),
                    "truth_scenario": truth.scenario_id,
                    "truth_targets": [
                        self._truth_target(target) for target in truth.targets
                    ],
                }
                self._captures.append(record)
                self._write_manifest()
                rospy.loginfo(
                    "[AligningSceneRecorder] saved episode=%d frame=%d %s",
                    episode, frame, absolute_path)
            except Exception as error:  # diagnostic must not affect control
                rospy.logerr(
                    "[AligningSceneRecorder] capture failed: %s", error)

    def _write_manifest(self):
        with self._lock:
            os.makedirs(self._output_dir, exist_ok=True)
            document = {
                "schema_version": 1,
                "capture_count": len(self._captures),
                "captures": self._captures,
            }
            temporary = self._manifest_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(document, handle, ensure_ascii=False,
                          indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, self._manifest_path)


def main():
    AligningSceneRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
