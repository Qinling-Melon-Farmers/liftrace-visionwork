#!/usr/bin/env python3
"""发布固定 UAV Pose / Odom，用于不依赖规划器的 Phase D 逻辑联调。"""
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class MockUavPose:
    def __init__(self):
        rospy.init_node("mock_uav_pose")
        self._frame_id = rospy.get_param("~frame_id", "camera_init")
        self._x = rospy.get_param("~x", 0.0)
        self._y = rospy.get_param("~y", 0.0)
        self._z = rospy.get_param("~z", 2.0)
        self._yaw = rospy.get_param("~yaw", 0.0)
        self._rate = rospy.get_param("~rate", 20.0)
        self._pose_topic = rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        self._odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")

        self._pose_pub = rospy.Publisher(self._pose_topic, PoseStamped, queue_size=1)
        self._odom_pub = rospy.Publisher(self._odom_topic, Odometry, queue_size=1)
        rospy.loginfo("[MockUavPose] pose_topic=%s odom_topic=%s pose=(%.2f, %.2f, %.2f, yaw=%.2f)",
                      self._pose_topic, self._odom_topic, self._x, self._y, self._z, self._yaw)

    def spin(self):
        rate = rospy.Rate(self._rate)
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self._frame_id
            pose.pose.position.x = self._x
            pose.pose.position.y = self._y
            pose.pose.position.z = self._z
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self._pose_pub.publish(pose)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self._frame_id
            odom.child_frame_id = "base_link"
            odom.pose.pose = pose.pose
            self._odom_pub.publish(odom)

            rate.sleep()


def main():
    MockUavPose().spin()


if __name__ == "__main__":
    main()
