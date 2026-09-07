#!/usr/bin/env python3
"""uav_vision 占位节点（Phase A 接口验证用，后续替换为正式节点）。"""
import rospy
from uav_vision.msg import (
    TargetDetection,
    TargetDetectionArray,
    TargetCandidate,
    TargetCandidateArray,
    DropOffset,
    DropReady,
)


def main():
    rospy.init_node("uav_vision")

    image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
    camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
    odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
    enable_debug = rospy.get_param("~enable_debug_image", False)
    debug_topic = rospy.get_param("~debug_image_topic", "/uav_vision/debug_image")

    rospy.loginfo("uav_vision placeholder started")
    rospy.loginfo("  image_topic:       %s", image_topic)
    rospy.loginfo("  camera_info_topic:  %s", camera_info_topic)
    rospy.loginfo("  odom_topic:         %s", odom_topic)
    rospy.loginfo("  enable_debug_image: %s", enable_debug)
    rospy.loginfo("  debug_image_topic:  %s", debug_topic)

    rospy.spin()


if __name__ == "__main__":
    main()
