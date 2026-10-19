#!/usr/bin/env python3
"""Broadcast a configurable PoseStamped as a TF transform."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class PoseTfBroadcaster:
    def __init__(self):
        rospy.init_node("pose_tf_broadcaster")
        self._parent = rospy.get_param("~parent_frame", "camera_init")
        self._child = rospy.get_param("~child_frame", "vision_body")
        topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        self._broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(topic, PoseStamped, self._on_pose, queue_size=1)
        rospy.loginfo(
            "[PoseTfBroadcaster] topic=%s parent=%s child=%s",
            topic, self._parent, self._child)

    def _on_pose(self, msg):
        orientation = msg.pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w)
        if not math.isfinite(norm) or abs(norm - 1.0) > 0.05:
            rospy.logwarn_throttle(
                2.0, "[PoseTfBroadcaster] invalid pose quaternion")
            return
        transform = TransformStamped()
        transform.header.stamp = (
            msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else
            rospy.Time.now())
        transform.header.frame_id = self._parent
        transform.child_frame_id = self._child
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = orientation
        self._broadcaster.sendTransform(transform)


if __name__ == "__main__":
    PoseTfBroadcaster()
    rospy.spin()
