#!/usr/bin/env python3
"""Broadcast the evaluated Gazebo camera pose as an eval-only world TF."""

import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image


def _multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


class GazeboCameraTf:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.camera_link_suffix = rospy.get_param("~camera_link_suffix", "D435i::camera_color_frame")
        self.child_frame_override = rospy.get_param("~child_frame", "")
        self.camera_pose_is_optical = rospy.get_param("~camera_pose_is_optical", False)
        self.pose = None
        self.link_name = ""
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(
            rospy.get_param("~link_states_topic", "/gazebo/link_states"),
            LinkStates,
            self._link_states_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image,
            self._image_callback,
            queue_size=1,
        )

    def _link_states_callback(self, message):
        matches = [
            (name, pose) for name, pose in zip(message.name, message.pose)
            if name.endswith(self.camera_link_suffix)
        ]
        if len(matches) == 1:
            self.link_name, self.pose = matches[0]
        else:
            self.pose = None

    def _image_callback(self, image):
        if self.pose is None:
            rospy.logwarn_throttle(5.0, "uav_vision_eval: no unique Gazebo camera pose for TF")
            return
        transform = TransformStamped()
        transform.header.stamp = image.header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.child_frame_override or image.header.frame_id
        transform.transform.translation.x = self.pose.position.x
        transform.transform.translation.y = self.pose.position.y
        transform.transform.translation.z = self.pose.position.z
        raw = (
            self.pose.orientation.x, self.pose.orientation.y,
            self.pose.orientation.z, self.pose.orientation.w,
        )
        # camera_link -> optical: RPY(-pi/2, 0, -pi/2).
        optical = raw if self.camera_pose_is_optical else _multiply(
            raw, (-0.5, 0.5, -0.5, 0.5)
        )
        transform.transform.rotation.x = optical[0]
        transform.transform.rotation.y = optical[1]
        transform.transform.rotation.z = optical[2]
        transform.transform.rotation.w = optical[3]
        self.broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("gazebo_camera_tf")
    GazeboCameraTf()
    rospy.spin()
