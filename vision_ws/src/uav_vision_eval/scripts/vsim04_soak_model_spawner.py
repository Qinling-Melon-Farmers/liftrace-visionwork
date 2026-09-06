#!/usr/bin/env python3
"""Spawn the camera and red-cross models serially, then latch readiness."""

import math
import os
import sys
import time

import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from uav_vision_eval.vsim04_metrics import call_with_monotonic_deadline


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _pose(values):
    if len(values) != 6:
        raise ValueError("model pose must contain x y z roll pitch yaw")
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("model pose must be finite")
    result = Pose()
    result.position.x, result.position.y, result.position.z = numbers[:3]
    quaternion = _quaternion_from_rpy(*numbers[3:])
    result.orientation.x = quaternion[0]
    result.orientation.y = quaternion[1]
    result.orientation.z = quaternion[2]
    result.orientation.w = quaternion[3]
    return result


class SequentialSoakModelSpawner:
    def __init__(self):
        rospy.init_node("vsim04_soak_model_spawner")
        self._startup_timeout = float(rospy.get_param(
            "~startup_timeout_sec", 45.0))
        self._call_timeout = float(rospy.get_param(
            "~service_call_timeout_sec", 5.0))
        if (not math.isfinite(self._startup_timeout) or
                not math.isfinite(self._call_timeout) or
                self._startup_timeout <= 0.0 or self._call_timeout <= 0.0):
            raise ValueError("spawn timeouts must be finite and positive")
        self._spawn_name = rospy.get_param(
            "~spawn_service", "/gazebo/spawn_sdf_model")
        self._state_name = rospy.get_param(
            "~get_state_service", "/gazebo/get_model_state")
        self._world_name = rospy.get_param(
            "~get_world_properties_service", "/gazebo/get_world_properties")
        self._spawn = rospy.ServiceProxy(self._spawn_name, SpawnModel)
        self._get_state = rospy.ServiceProxy(self._state_name, GetModelState)
        self._get_world = rospy.ServiceProxy(
            self._world_name, GetWorldProperties)
        self._ready = rospy.Publisher(
            rospy.get_param(
                "~ready_topic", "/uav_vision_eval/soak_models_ready"),
            Bool, queue_size=1, latch=True)
        self._models = (
            (
                str(rospy.get_param("~camera_model_name")),
                os.path.abspath(rospy.get_param("~camera_sdf_file")),
                _pose(rospy.get_param(
                    "~camera_pose", [2.6, 0.5, 2.4, 0.0,
                                     math.pi / 2.0, 0.0])),
            ),
            (
                str(rospy.get_param("~red_cross_model_name")),
                os.path.abspath(rospy.get_param("~red_cross_sdf_file")),
                _pose(rospy.get_param(
                    "~red_cross_pose", [-2.5, -2.5, 0.0, 0.0, 0.0, 0.0])),
            ),
        )
        for name, path, _model_pose in self._models:
            if not name.strip() or not os.path.isfile(path):
                raise ValueError(
                    "invalid model provenance: {} {}".format(name, path))

    def _call(self, name, operation):
        return call_with_monotonic_deadline(
            operation, self._call_timeout, name, rospy.is_shutdown)

    def _exists(self, model_name):
        try:
            return bool(self._call(
                "get_" + model_name,
                lambda: self._get_state(model_name, "world")).success)
        except Exception:
            return False

    def _spawn_one(self, model_name, sdf_file, initial_pose):
        with open(sdf_file, "r", encoding="utf-8") as stream:
            model_xml = stream.read()
        deadline = time.monotonic() + self._startup_timeout
        last_error = ""
        if self._exists(model_name):
            return
        # A timed-out Gazebo spawn request cannot be cancelled.  Submit it
        # exactly once; a retry could create ``model_name_0`` while the first
        # worker is still completing inside Gazebo.
        try:
            response = call_with_monotonic_deadline(
                lambda: self._spawn(
                    model_name, model_xml, "", initial_pose, "world"),
                self._startup_timeout, "spawn_" + model_name,
                rospy.is_shutdown)
            if response.success:
                return
            last_error = str(response.status_message)
        except Exception as error:
            last_error = str(error)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._exists(model_name):
                return
            time.sleep(0.25)
        raise RuntimeError(
            "failed to spawn {}: {}".format(model_name, last_error))

    def _verify_unique_world_models(self):
        response = self._call("get_world_properties", self._get_world)
        if not response.success:
            raise RuntimeError(
                "get_world_properties failed: " + response.status_message)
        world_names = {str(value) for value in response.model_names}
        for model_name, _path, _pose_value in self._models:
            matching = sorted(
                value for value in world_names
                if value == model_name or value.startswith(model_name + "_"))
            if matching != [model_name]:
                raise RuntimeError(
                    "model uniqueness failed for {}: {}".format(
                        model_name, ",".join(matching)))

    def run(self):
        rospy.wait_for_service(
            self._spawn_name, timeout=self._startup_timeout)
        rospy.wait_for_service(
            self._state_name, timeout=self._startup_timeout)
        rospy.wait_for_service(
            self._world_name, timeout=self._startup_timeout)
        for model in self._models:
            self._spawn_one(*model)
        if not all(self._exists(model[0]) for model in self._models):
            raise RuntimeError("spawned model verification failed")
        self._verify_unique_world_models()
        self._ready.publish(Bool(data=True))
        rospy.loginfo(
            "V-SIM-04 soak models ready: %s",
            ",".join(model[0] for model in self._models))
        rospy.spin()


def main():
    try:
        SequentialSoakModelSpawner().run()
        return 0
    except Exception as error:
        rospy.logerr("V-SIM-04 sequential model spawn failed: %s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
