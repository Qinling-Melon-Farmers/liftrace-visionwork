#!/usr/bin/env python3
"""按安装俯角发布机体到斜下相机 ROS 光学坐标系的静态外参。"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def pitch_quaternion(pitch):
    return (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))


def main():
    rospy.init_node("aux_camera_extrinsic_tf")
    parent = rospy.get_param("~parent_frame", "vision_body")
    child = rospy.get_param("~child_frame", "aux_camera_optical_frame")
    xyz = [float(value) for value in rospy.get_param("~xyz", [0.12, 0.0, 0.0])]
    angle = float(rospy.get_param("~depression_angle_deg", 55.0))
    if len(xyz) != 3 or angle not in (45.0, 55.0, 60.0):
        raise ValueError("xyz must have 3 values and angle must be 45/55/60")
    # Gazebo 相机物理 link 沿 +X 看；link -> ROS optical 为 RPY(-90,0,-90)。
    quaternion = multiply(
        pitch_quaternion(math.radians(angle)),
        (-0.5, 0.5, -0.5, 0.5))
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.loginfo(
        "[AuxCameraExtrinsicTF] %s -> %s angle=%.1f xyz=%s",
        parent, child, angle, xyz)
    rospy.spin()


if __name__ == "__main__":
    main()
