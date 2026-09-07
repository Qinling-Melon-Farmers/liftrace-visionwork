#!/usr/bin/env python3
"""Broadcast the evaluated Gazebo camera pose as an eval-only world TF."""

import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from uav_vision.msg import TargetDetectionArray

from uav_vision_eval.stamped_pose_buffer import StampedPoseBuffer


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
        self.use_stamped_camera_pose = rospy.get_param(
            "~use_stamped_camera_pose", False)
        self.pose = None
        self.link_name = ""
        self.pose_buffer = StampedPoseBuffer(rospy.get_param(
            "~pose_history_length", 512))
        self.broadcaster = tf2_ros.TransformBroadcaster()
        if self.use_stamped_camera_pose:
            rospy.Subscriber(
                rospy.get_param(
                    "~camera_pose_topic", "/uav_vision_eval/camera_pose"),
                PoseStamped, self._camera_pose_callback, queue_size=40)
        else:
            rospy.Subscriber(
                rospy.get_param("~link_states_topic", "/gazebo/link_states"),
                LinkStates, self._link_states_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image,
            self._image_callback,
            queue_size=1,
        )
        detection_trigger_topic = rospy.get_param(
            "~detection_trigger_topic", "")
        self._detection_trigger_source = rospy.get_param(
            "~detection_trigger_source", "target_detector")
        if detection_trigger_topic:
            rospy.Subscriber(
                detection_trigger_topic,
                TargetDetectionArray,
                self._detection_callback,
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

    def _camera_pose_callback(self, message):
        if message.header.frame_id != self.world_frame:
            rospy.logerr_throttle(
                5.0, "uav_vision_eval: stamped camera pose frame mismatch")
            return
        self.pose_buffer.add(message)

    def _image_callback(self, image):
        self._broadcast_for_header(image.header)

    def _detection_callback(self, detections):
        if detections.source != self._detection_trigger_source:
            return
        self._broadcast_for_header(detections.header)

    def _broadcast_for_header(self, header):
        pose = self.pose
        if self.use_stamped_camera_pose:
            stamped_pose, _age_sec = self.pose_buffer.at_or_before(
                header.stamp)
            pose = stamped_pose.pose if stamped_pose is not None else None
        if pose is None:
            rospy.logwarn_throttle(5.0, "uav_vision_eval: no unique Gazebo camera pose for TF")
            return
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.child_frame_override or header.frame_id
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        raw = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
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
