#!/usr/bin/env python3
"""最小空白图像发布器，用于 Phase D detector/perf 回归。"""
import rospy
from sensor_msgs.msg import Image


class MockImagePublisher:
    def __init__(self):
        rospy.init_node("mock_image_publisher")
        self._image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self._width = int(rospy.get_param("~width", 32))
        self._height = int(rospy.get_param("~height", 32))
        self._encoding = rospy.get_param("~encoding", "bgr8")
        self._rate_hz = float(rospy.get_param("~rate", 5.0))
        self._fill_value = max(0, min(255, int(rospy.get_param("~fill_value", 0))))
        self._pub = rospy.Publisher(self._image_topic, Image, queue_size=1)

        if self._encoding == "bgr8":
            self._channels = 3
        elif self._encoding == "mono8":
            self._channels = 1
        else:
            rospy.logwarn("[MockImagePublisher] unsupported encoding=%s, fallback to bgr8",
                          self._encoding)
            self._encoding = "bgr8"
            self._channels = 3

        rospy.loginfo("[MockImagePublisher] ready topic=%s size=%dx%d encoding=%s rate=%.1f",
                      self._image_topic, self._width, self._height, self._encoding,
                      self._rate_hz)

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        payload = bytes([self._fill_value] * (self._width * self._height * self._channels))
        step = self._width * self._channels
        while not rospy.is_shutdown():
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.height = self._height
            msg.width = self._width
            msg.encoding = self._encoding
            msg.is_bigendian = 0
            msg.step = step
            msg.data = payload
            self._pub.publish(msg)
            rate.sleep()


def main():
    MockImagePublisher().run()


if __name__ == "__main__":
    main()
