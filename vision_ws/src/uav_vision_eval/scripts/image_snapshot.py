#!/usr/bin/env python3
"""Save one evaluated ROS image as a reproducible scene artifact."""

import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def _to_bgr(message):
    encoding = message.encoding.lower()
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError("unsupported encoding: " + message.encoding)
    row_bytes = message.width * channels
    raw = np.frombuffer(message.data, dtype=np.uint8)
    rows = raw[:message.step * message.height].reshape((message.height, message.step))
    pixels = rows[:, :row_bytes]
    if channels == 1:
        return cv2.cvtColor(pixels.reshape((message.height, message.width)), cv2.COLOR_GRAY2BGR)
    image = pixels.reshape((message.height, message.width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


class Snapshot:
    def __init__(self):
        self.output_file = os.path.abspath(rospy.get_param("~output_file"))
        self.saved = False
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image, self._callback, queue_size=1, buff_size=2 ** 24,
        )

    def _callback(self, message):
        if self.saved:
            return
        try:
            image = _to_bgr(message)
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            if not cv2.imwrite(self.output_file, image):
                raise RuntimeError("cv2.imwrite returned false")
            self.saved = True
            rospy.loginfo("uav_vision_eval scene snapshot: %s", self.output_file)
        except Exception as error:
            rospy.logerr_throttle(5.0, "uav_vision_eval snapshot failed: %s", error)


if __name__ == "__main__":
    rospy.init_node("image_snapshot")
    Snapshot()
    rospy.spin()
