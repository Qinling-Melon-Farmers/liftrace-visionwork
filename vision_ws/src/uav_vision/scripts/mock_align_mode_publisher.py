#!/usr/bin/env python3
"""周期发布固定 /uav_vision/align_mode，用于 Phase D mock 回归。"""
import rospy
from std_msgs.msg import String


class MockAlignModePublisher:
    def __init__(self):
        rospy.init_node("mock_align_mode_publisher")
        self._topic = rospy.get_param("~topic", "/uav_vision/align_mode")
        self._mode = rospy.get_param("~mode", "disabled")
        self._rate = float(rospy.get_param("~rate", 5.0))
        self._pub = rospy.Publisher(self._topic, String, queue_size=1, latch=True)
        rospy.loginfo("[MockAlignModePublisher] topic=%s mode=%s rate=%.1f",
                      self._topic, self._mode, self._rate)

    def spin(self):
        msg = String(data=self._mode)
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self._pub.publish(msg)
            rate.sleep()


def main():
    MockAlignModePublisher().spin()


if __name__ == "__main__":
    main()
