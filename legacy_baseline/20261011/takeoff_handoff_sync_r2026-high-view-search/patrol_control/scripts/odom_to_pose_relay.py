#!/usr/bin/env python3
"""Relay from nav_msgs/Odometry to geometry_msgs/PoseStamped.
Bridges the internal simulator's /state_ukf/odom (Odometry) to the
topic that patrol_control expects (/mavros/local_position/pose as PoseStamped).
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def callback(msg):
    out = PoseStamped()
    out.header = msg.header
    out.pose = msg.pose.pose
    pub.publish(out)


rospy.init_node("odom_to_pose_relay", anonymous=True)

in_topic = rospy.get_param("~in_topic", "/state_ukf/odom")
out_topic = rospy.get_param("~out_topic", "/mavros/local_position/pose")

pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
sub = rospy.Subscriber(in_topic, Odometry, callback, queue_size=10)

rospy.loginfo("Relaying %s -> %s", in_topic, out_topic)
rospy.spin()
