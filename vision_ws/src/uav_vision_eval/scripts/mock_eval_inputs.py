#!/usr/bin/env python3
"""Publish deterministic camera, Gazebo pose and detection inputs."""

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest

from uav_vision.msg import TargetDetection, TargetDetectionArray


class MockInputs:
    def __init__(self):
        self.camera_info_pub = rospy.Publisher("/mock/camera_info", CameraInfo, queue_size=1)
        self.image_pub = rospy.Publisher("/mock/image", Image, queue_size=1)
        self.link_states_pub = rospy.Publisher("/mock/link_states", LinkStates, queue_size=1)
        self.detections_pub = rospy.Publisher("/mock/detections", TargetDetectionArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish)

    @staticmethod
    def _pose(x, y, z):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        return pose

    def _publish(self, _event):
        stamp = rospy.Time.now()
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = "camera_color_frame"
        camera_info.width, camera_info.height = 640, 480
        camera_info.K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        camera_info.P = [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)

        link_states = LinkStates()
        link_states.name = ["mock::D435i::camera_color_frame", "3::zhangpeng::link"]
        link_states.pose = [self._pose(0.0, 0.0, 0.0), self._pose(0.0, 0.0, 3.0)]
        self.link_states_pub.publish(link_states)

        image = Image()
        image.header = camera_info.header
        image.width, image.height = 640, 480
        image.encoding = "rgb8"
        image.step = 640 * 3
        image.data = bytes(image.step * image.height)
        self.image_pub.publish(image)

        detection = TargetDetection()
        detection.header = image.header
        detection.class_name = "tent"
        detection.class_confidence = 0.9
        detection.geometry_confidence = 0.85
        detection.geometry_verified = True
        detection.roi = RegionOfInterest(x_offset=250, y_offset=170, width=140, height=140)
        detection.center_px.x, detection.center_px.y = 324.0, 243.0
        detection.center_refined = True
        detection.map_valid = True
        detection.map_frame = "world"
        detection.map_point.x, detection.map_point.y, detection.map_point.z = 0.02, -0.01, 0.0
        detection.map_quality = 0.9
        array = TargetDetectionArray()
        array.header = image.header
        array.detections = [detection]
        self.detections_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("mock_eval_inputs")
    MockInputs()
    rospy.spin()
