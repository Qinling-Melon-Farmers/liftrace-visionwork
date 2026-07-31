#!/usr/bin/env python3
"""Project Gazebo target link geometry into the simulated camera image.

The detector output is intentionally never used to construct ground truth.
Target and camera poses come from /gazebo/link_states, while projection uses
the CameraInfo that belongs to the evaluated image stream.
"""

import math

import rospy
import yaml
from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Point, Pose
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest

from uav_vision_eval.msg import SimTarget, SimTargetArray


def _rotate(quaternion, vector):
    """Rotate vector by quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _inverse_rotate(quaternion, vector):
    qx, qy, qz, qw = quaternion
    return _rotate((-qx, -qy, -qz, qw), vector)


def _quaternion_tuple(orientation):
    return (orientation.x, orientation.y, orientation.z, orientation.w)


def _point(values):
    result = Point()
    result.x, result.y, result.z = values
    return result


class GroundTruthProjector:
    def __init__(self):
        catalog_path = rospy.get_param("~catalog_file")
        scenario_path = rospy.get_param("~scenario_file")
        with open(catalog_path, "r", encoding="utf-8") as stream:
            catalog = yaml.safe_load(stream)
        with open(scenario_path, "r", encoding="utf-8") as stream:
            scenario = yaml.safe_load(stream)

        active = set(scenario.get("active_targets", []))
        self.targets = [target for target in catalog["targets"] if target["target_id"] in active]
        self.scenario_id = scenario["scenario_id"]
        self.world_frame = catalog.get("world_frame", "world")
        self.camera_link_suffix = rospy.get_param(
            "~camera_link_suffix", catalog.get("camera_link_suffix", "D435i::camera_color_frame")
        )
        self.allow_fallback_pose = rospy.get_param("~allow_fallback_pose", False)
        self.camera_pose_is_optical = rospy.get_param("~camera_pose_is_optical", False)
        self.model = PinholeCameraModel()
        self.camera_info = None
        self.link_poses = {}
        self.model_poses = {}

        output_topic = rospy.get_param("~output_topic", "/uav_vision_eval/ground_truth")
        self.publisher = rospy.Publisher(output_topic, SimTargetArray, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/camera/color/camera_info"),
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~link_states_topic", "/gazebo/link_states"),
            LinkStates,
            self._link_states_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~model_states_topic", "/gazebo/model_states"),
            ModelStates,
            self._model_states_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/color/image_raw"),
            Image,
            self._image_callback,
            queue_size=1,
        )

    def _camera_info_callback(self, message):
        self.camera_info = message
        self.model.fromCameraInfo(message)

    def _link_states_callback(self, message):
        self.link_poses = dict(zip(message.name, message.pose))

    def _model_states_callback(self, message):
        self.model_poses = dict(zip(message.name, message.pose))

    @staticmethod
    def _find_pose(poses, requested_name):
        if requested_name in poses:
            return requested_name, poses[requested_name]
        suffix = "::" + requested_name
        matches = [(name, pose) for name, pose in poses.items() if name.endswith(suffix)]
        return matches[0] if len(matches) == 1 else (None, None)

    def _camera_pose(self):
        matches = [
            (name, pose) for name, pose in self.link_poses.items()
            if name.endswith(self.camera_link_suffix)
        ]
        return matches[0] if len(matches) == 1 else (None, None)

    def _target_pose(self, target):
        resolved_name, pose = self._find_pose(self.link_poses, target["gazebo_link_name"])
        if pose is not None:
            return resolved_name, pose
        model_name = target.get("gazebo_model_name")
        local_center = target.get("local_center_in_model")
        model_pose = self.model_poses.get(model_name)
        if model_pose is None or local_center is None:
            return None, None
        synthetic = Pose()
        center_world = self._world_point(model_pose, tuple(local_center))
        synthetic.position.x, synthetic.position.y, synthetic.position.z = center_world
        synthetic.orientation = model_pose.orientation
        return "model:{}:local:{}".format(model_name, target["target_id"]), synthetic

    @staticmethod
    def _world_point(pose, local_point):
        rotated = _rotate(_quaternion_tuple(pose.orientation), local_point)
        return (
            pose.position.x + rotated[0],
            pose.position.y + rotated[1],
            pose.position.z + rotated[2],
        )

    @staticmethod
    def _camera_link_point(camera_pose, world_point):
        delta = (
            world_point[0] - camera_pose.position.x,
            world_point[1] - camera_pose.position.y,
            world_point[2] - camera_pose.position.z,
        )
        return _inverse_rotate(_quaternion_tuple(camera_pose.orientation), delta)

    def _camera_point(self, camera_pose, world_point):
        point = self._camera_link_point(camera_pose, world_point)
        if self.camera_pose_is_optical:
            return point
        # Gazebo camera sensor convention (x forward, y left, z up) to
        # ROS optical convention (z forward, x right, y down).
        return (-point[1], -point[2], point[0])

    def _project(self, camera_point):
        if camera_point[2] <= 1.0e-3:
            return None
        pixel = self.model.project3dToPixel(camera_point)
        if not all(math.isfinite(value) for value in pixel):
            return None
        return pixel

    def _target_message(self, target, camera_pose, image):
        result = SimTarget()
        result.header = image.header
        result.target_id = target["target_id"]
        result.class_name = target["class_name"]
        result.gazebo_link_name = target["gazebo_link_name"]

        resolved_name, target_pose = self._target_pose(target)
        if target_pose is None and self.allow_fallback_pose:
            center_world = tuple(target["fallback_center_world"])
            result.gazebo_link_name = "fallback:" + target["gazebo_link_name"]
        elif target_pose is None:
            return result
        else:
            center_world = self._world_point(target_pose, (0.0, 0.0, 0.0))
            result.gazebo_link_name = resolved_name
        result.pose_valid = True
        result.world_center = _point(center_world)

        center_camera = self._camera_point(camera_pose, center_world)
        result.camera_center = _point(center_camera)
        result.distance_m = math.sqrt(sum(value * value for value in center_camera))
        center_pixel = self._project(center_camera)
        if center_pixel is None:
            return result

        size_x, size_y = target["size_m"]
        if target_pose is None:
            corners_world = [
                (center_world[0] + dx, center_world[1] + dy, center_world[2])
                for dx in (-size_x / 2.0, size_x / 2.0)
                for dy in (-size_y / 2.0, size_y / 2.0)
            ]
        else:
            corners_world = [
                self._world_point(target_pose, (dx, dy, 0.0))
                for dx in (-size_x / 2.0, size_x / 2.0)
                for dy in (-size_y / 2.0, size_y / 2.0)
            ]
        corners_pixel = [self._project(self._camera_point(camera_pose, corner)) for corner in corners_world]
        if any(pixel is None for pixel in corners_pixel):
            return result

        xs = [pixel[0] for pixel in corners_pixel]
        ys = [pixel[1] for pixel in corners_pixel]
        left = max(0, int(math.floor(min(xs))))
        top = max(0, int(math.floor(min(ys))))
        right = min(image.width, int(math.ceil(max(xs))))
        bottom = min(image.height, int(math.ceil(max(ys))))
        result.pixel_center = _point((center_pixel[0], center_pixel[1], 0.0))
        result.center_in_frame = 0.0 <= center_pixel[0] < image.width and 0.0 <= center_pixel[1] < image.height
        result.fully_in_frame = (
            min(xs) >= 0.0 and max(xs) < image.width and min(ys) >= 0.0 and max(ys) < image.height
        )
        result.roi = RegionOfInterest(
            x_offset=left,
            y_offset=top,
            width=max(0, right - left),
            height=max(0, bottom - top),
            do_rectify=False,
        )
        result.projection_valid = result.center_in_frame and result.roi.width > 0 and result.roi.height > 0
        return result

    def _image_callback(self, image):
        output = SimTargetArray()
        output.header = image.header
        output.scenario_id = self.scenario_id
        if self.camera_info is None:
            rospy.logwarn_throttle(5.0, "uav_vision_eval: waiting for CameraInfo")
            self.publisher.publish(output)
            return
        _, camera_pose = self._camera_pose()
        if camera_pose is None:
            rospy.logwarn_throttle(5.0, "uav_vision_eval: camera link is absent or ambiguous")
            self.publisher.publish(output)
            return
        output.targets = [self._target_message(target, camera_pose, image) for target in self.targets]
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("sim_ground_truth_projector")
    GroundTruthProjector()
    rospy.spin()
