#!/usr/bin/env python3
"""Publish one stamped Gazebo camera-link pose source for eval consumers."""

import math

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped


class GazeboCameraPoseStamper:
    def __init__(self):
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._camera_link_suffix = rospy.get_param(
            "~camera_link_suffix", "D435i::camera_color_frame")
        max_publish_rate_hz = float(rospy.get_param(
            "~max_publish_rate_hz", 0.0))
        if not math.isfinite(max_publish_rate_hz) or max_publish_rate_hz < 0.0:
            raise ValueError("max_publish_rate_hz must be finite and non-negative")
        self._min_publish_period_sec = (
            1.0 / max_publish_rate_hz if max_publish_rate_hz > 0.0 else 0.0)
        self._last_publish_stamp_sec = None
        self._publisher = rospy.Publisher(
            rospy.get_param(
                "~output_topic", "/uav_vision_eval/camera_pose"),
            PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(
            rospy.get_param("~link_states_topic", "/gazebo/link_states"),
            LinkStates, self._on_link_states, queue_size=1)

    def _on_link_states(self, message):
        matches = [
            (name, pose) for name, pose in zip(message.name, message.pose)
            if name.endswith(self._camera_link_suffix)
        ]
        if len(matches) != 1:
            rospy.logwarn_throttle(
                5.0, "uav_vision_eval: no unique Gazebo camera pose")
            return
        name, pose = matches[0]
        stamp = rospy.Time.now()
        stamp_sec = stamp.to_sec()
        if self._last_publish_stamp_sec is not None:
            elapsed = stamp_sec - self._last_publish_stamp_sec
            if 0.0 <= elapsed < self._min_publish_period_sec:
                return
        self._last_publish_stamp_sec = stamp_sec
        output = PoseStamped()
        output.header.stamp = stamp
        output.header.frame_id = self._world_frame
        output.pose = pose
        self._publisher.publish(output)
        rospy.logdebug_throttle(
            5.0, "uav_vision_eval: stamped camera pose from %s", name)


if __name__ == "__main__":
    rospy.init_node("gazebo_camera_pose_stamper")
    GazeboCameraPoseStamper()
    rospy.spin()
