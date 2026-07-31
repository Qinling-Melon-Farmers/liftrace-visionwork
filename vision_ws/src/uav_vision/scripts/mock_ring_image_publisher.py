#!/usr/bin/env python3
"""发布一张包含两个蓝色圆环的确定性测试图像。"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


class MockRingImagePublisher:
    def __init__(self):
        rospy.init_node("mock_ring_image_publisher")
        self._topic = rospy.get_param("~topic", "/camera/image_raw")
        self._frame_id = rospy.get_param("~frame_id", "camera")
        self._width = int(rospy.get_param("~width", 1280))
        self._height = int(rospy.get_param("~height", 1024))
        self._pub = rospy.Publisher(self._topic, Image, queue_size=1)

    def spin(self):
        image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        cv2.circle(image, (320, 300), 150, (255, 0, 0), 28)
        cv2.circle(image, (960, 700), 210, (255, 0, 0), 32)
        msg = Image()
        msg.header.frame_id = self._frame_id
        msg.height = self._height
        msg.width = self._width
        msg.encoding = "bgr8"
        msg.step = self._width * 3
        msg.data = image.tobytes()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self._pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    MockRingImagePublisher().spin()
