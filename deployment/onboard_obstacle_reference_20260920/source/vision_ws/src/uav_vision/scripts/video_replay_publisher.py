#!/usr/bin/env python3
"""Reliably publish an MP4 one frame at a time into the ROS pixel chain.

The publisher waits for the annotation recorder's per-frame acknowledgement.
Processing may therefore be slower than real time, while media timestamps and
the generated video's FPS remain identical to the decoded source video.
"""

import json
import os
import threading
import time

import cv2
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Header, String, UInt32


class VideoReplayPublisher:
    def __init__(self):
        rospy.init_node("video_replay_publisher")
        self._video_path = os.path.abspath(
            os.path.expanduser(rospy.get_param("~video_path", "")))
        self._image_topic = rospy.get_param(
            "~image_topic", "/uav_vision/video_replay/image_raw")
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/uav_vision/video_replay/camera_info")
        self._metadata_topic = rospy.get_param(
            "~metadata_topic", "/uav_vision/video_replay/metadata")
        self._frame_done_topic = rospy.get_param(
            "~frame_done_topic", "/uav_vision/video_replay/frame_done")
        self._input_done_topic = rospy.get_param(
            "~input_done_topic", "/uav_vision/video_replay/input_done")
        self._output_done_topic = rospy.get_param(
            "~output_done_topic", "/uav_vision/video_replay/output_done")
        self._error_topic = rospy.get_param(
            "~error_topic", "/uav_vision/video_replay/error")
        self._frame_id = rospy.get_param(
            "~frame_id", "video_replay_camera")
        self._startup_timeout = float(
            rospy.get_param("~startup_timeout_sec", 120.0))
        self._frame_timeout = float(
            rospy.get_param("~frame_timeout_sec", 60.0))
        self._finish_timeout = float(
            rospy.get_param("~finish_timeout_sec", 30.0))
        self._min_image_subscribers = int(
            rospy.get_param("~min_image_subscribers", 5))
        self._min_camera_info_subscribers = int(
            rospy.get_param("~min_camera_info_subscribers", 3))
        self._max_frames = int(rospy.get_param("~max_frames", 0))
        self._orientation_auto = bool(
            rospy.get_param("~orientation_auto", False))
        self._map_projection_mode = str(
            rospy.get_param("~map_projection_mode", "disabled"))

        if not self._video_path or not os.path.isfile(self._video_path):
            raise RuntimeError("video_path is not a readable file: %s" % self._video_path)
        if self._max_frames < 0:
            raise RuntimeError("max_frames must be zero or positive")
        if self._map_projection_mode not in (
                "disabled", "fail_closed_no_tf"):
            raise RuntimeError(
                "unsupported map_projection_mode: %s" %
                self._map_projection_mode)

        self._condition = threading.Condition()
        self._last_frame_done = -1
        self._output_done = False
        self._remote_error = ""

        self._image_pub = rospy.Publisher(
            self._image_topic, Image, queue_size=1)
        self._camera_info_pub = rospy.Publisher(
            self._camera_info_topic, CameraInfo, queue_size=1, latch=True)
        self._metadata_pub = rospy.Publisher(
            self._metadata_topic, String, queue_size=1, latch=True)
        self._input_done_pub = rospy.Publisher(
            self._input_done_topic, UInt32, queue_size=1, latch=True)
        rospy.Subscriber(
            self._frame_done_topic, UInt32, self._on_frame_done, queue_size=10)
        rospy.Subscriber(
            self._output_done_topic, Bool, self._on_output_done, queue_size=1)
        rospy.Subscriber(
            self._error_topic, String, self._on_error, queue_size=1)

    def _on_frame_done(self, message):
        with self._condition:
            self._last_frame_done = max(self._last_frame_done, int(message.data))
            self._condition.notify_all()

    def _on_output_done(self, message):
        if not message.data:
            return
        with self._condition:
            self._output_done = True
            self._condition.notify_all()

    def _on_error(self, message):
        with self._condition:
            self._remote_error = message.data or "video replay recorder failed"
            self._condition.notify_all()

    @staticmethod
    def _image_message(frame, header):
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError("decoded frame is not BGR8")
        message = Image()
        message.header = header
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(frame.shape[1] * 3)
        message.data = frame.tobytes()
        return message

    def _camera_info(self, width, height, header):
        # The geometry nodes only require an initialized CameraInfo model for
        # their pixel algorithms.  No projector is launched, so these neutral
        # values are never presented as calibrated intrinsics or map evidence.
        focal = float(max(width, height))
        cx = (float(width) - 1.0) * 0.5
        cy = (float(height) - 1.0) * 0.5
        message = CameraInfo()
        message.header = header
        message.width = int(width)
        message.height = int(height)
        message.distortion_model = "plumb_bob"
        message.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.K = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        message.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.P = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _wait_for_connections(self, info_message):
        deadline = time.monotonic() + self._startup_timeout
        while not rospy.is_shutdown():
            self._camera_info_pub.publish(info_message)
            image_count = self._image_pub.get_num_connections()
            info_count = self._camera_info_pub.get_num_connections()
            if (image_count >= self._min_image_subscribers and
                    info_count >= self._min_camera_info_subscribers):
                # Give the latched CameraInfo callback time to initialize each
                # C++ image_geometry model before the first image arrives.
                rospy.sleep(0.5)
                return
            if self._remote_error:
                raise RuntimeError(self._remote_error)
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "startup timeout: image subscribers=%d/%d camera_info=%d/%d" % (
                        image_count, self._min_image_subscribers,
                        info_count, self._min_camera_info_subscribers))
            rospy.sleep(0.1)
        raise RuntimeError("ROS shutdown before replay subscribers became ready")

    def _wait_for_frame(self, frame_index):
        deadline = time.monotonic() + self._frame_timeout
        with self._condition:
            while not rospy.is_shutdown() and self._last_frame_done < frame_index:
                if self._remote_error:
                    raise RuntimeError(self._remote_error)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        "frame %d did not complete the pixel chain within %.1fs" % (
                            frame_index, self._frame_timeout))
                self._condition.wait(timeout=min(remaining, 0.5))
        if rospy.is_shutdown() and self._last_frame_done < frame_index:
            raise RuntimeError("ROS shutdown while waiting for frame %d" % frame_index)

    def _wait_for_output(self):
        deadline = time.monotonic() + self._finish_timeout
        with self._condition:
            while not rospy.is_shutdown() and not self._output_done:
                if self._remote_error:
                    raise RuntimeError(self._remote_error)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("annotation recorder did not finalize output")
                self._condition.wait(timeout=min(remaining, 0.5))

    def run(self):
        capture = cv2.VideoCapture(self._video_path)
        if not capture.isOpened():
            raise RuntimeError("unable to open video: %s" % self._video_path)
        if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
            capture.set(
                cv2.CAP_PROP_ORIENTATION_AUTO,
                1.0 if self._orientation_auto else 0.0)

        ok, frame = capture.read()
        if not ok or frame is None:
            capture.release()
            raise RuntimeError("video contains no decodable frame")

        fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
        if fps <= 0.0:
            capture.release()
            raise RuntimeError("video reports an invalid FPS")
        height, width = frame.shape[:2]
        reported_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        metadata = {
            "input_video": self._video_path,
            "width": int(width),
            "height": int(height),
            "fps": fps,
            "reported_frame_count": reported_frames,
            "orientation_auto": self._orientation_auto,
            "camera_info_semantics": "pixel_chain_initialization_only_not_calibrated",
            "map_projection": self._map_projection_mode != "disabled",
            "map_projection_mode": self._map_projection_mode,
            "map_valid_expected": False,
            "selected_target": False,
        }
        self._metadata_pub.publish(String(data=json.dumps(metadata, sort_keys=True)))

        base_stamp = rospy.Time.now()
        if base_stamp.to_sec() <= 0.0:
            base_stamp = rospy.Time.from_sec(1.0)
        first_header = Header(stamp=base_stamp, frame_id=self._frame_id)
        info_message = self._camera_info(width, height, first_header)
        self._wait_for_connections(info_message)

        frame_index = 0
        try:
            while ok and frame is not None and not rospy.is_shutdown():
                if self._max_frames and frame_index >= self._max_frames:
                    break
                if frame.shape[0] != height or frame.shape[1] != width:
                    raise RuntimeError(
                        "decoded frame dimensions changed at frame %d" % frame_index)

                image_message = Image()
                image_message.header.seq = frame_index
                image_message.header.stamp = base_stamp + rospy.Duration.from_sec(
                    frame_index / fps)
                image_message.header.frame_id = self._frame_id
                image_message = self._image_message(frame, image_message.header)
                info_message = self._camera_info(width, height, image_message.header)
                self._camera_info_pub.publish(info_message)
                self._image_pub.publish(image_message)
                self._wait_for_frame(frame_index)

                frame_index += 1
                if frame_index % 100 == 0:
                    rospy.loginfo(
                        "[VideoReplayPublisher] completed %d frames (media %.2fs)",
                        frame_index, frame_index / fps)
                ok, frame = capture.read()
        finally:
            capture.release()

        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown before video replay completed")
        self._input_done_pub.publish(UInt32(data=frame_index))
        self._wait_for_output()
        rospy.loginfo(
            "[VideoReplayPublisher] replay complete frames=%d fps=%.6f duration=%.3fs",
            frame_index, fps, frame_index / fps)


def main():
    node = VideoReplayPublisher()
    node.run()


if __name__ == "__main__":
    main()
