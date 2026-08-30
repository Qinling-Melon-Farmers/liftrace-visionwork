#!/usr/bin/env python3
"""Publish one stamped Gazebo camera-link pose source for eval consumers."""

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped


class GazeboCameraPoseStamper:
    def __init__(self):
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._camera_link_suffix = rospy.get_param(
            "~camera_link_suffix", "D435i::camera_color_frame")
        self._publisher = rospy.Publisher(
            rospy.get_param(
                "~output_topic", "/uav_vision_eval/camera_pose"),
            PoseStamped, queue_size=40, latch=True)
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
        output = PoseStamped()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = self._world_frame
        output.pose = pose
        self._publisher.publish(output)
        rospy.logdebug_throttle(
            5.0, "uav_vision_eval: stamped camera pose from %s", name)


if __name__ == "__main__":
    rospy.init_node("gazebo_camera_pose_stamper")
    GazeboCameraPoseStamper()
    rospy.spin()
