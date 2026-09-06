#!/usr/bin/env python3
"""Spawn a selected planner-feature profile and publish a READY barrier."""

import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from uav_mission.planner_anchor_policy import (
    resolve_model_sdf,
    validate_anchor_profile,
)


class PlannerAnchorSpawner:
    def __init__(self):
        rospy.init_node("planner_anchor_spawner")
        self._profile_name = rospy.get_param(
            "~nav_feature_profile", "baseline")
        self._profiles = rospy.get_param("~profiles", {})
        self._status_path = rospy.get_param(
            "~status_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"),
                "planner_anchor_status.json"))
        self._status_pub = rospy.Publisher(
            rospy.get_param(
                "~status_topic", "/mission/planner_anchor_status"),
            String, queue_size=1, latch=True)
        raw_roots = rospy.get_param(
            "~model_roots", os.environ.get("GAZEBO_MODEL_PATH", ""))
        self._model_roots = [
            os.path.abspath(root)
            for root in str(raw_roots).split(os.pathsep) if root]
        self._verify_timeout = float(
            rospy.get_param("~verification_timeout", 20.0))
        self._verify_tolerance = float(
            rospy.get_param("~verification_tolerance", 0.05))
        self._status = {
            "component": "planner_anchor_spawner",
            "profile": self._profile_name,
            "status": "INIT",
            "ready": False,
            "reason": "initializing",
        }
        self._publish_status()

    def _atomic_write(self, content):
        os.makedirs(os.path.dirname(self._status_path) or ".", exist_ok=True)
        temporary = self._status_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.replace(temporary, self._status_path)

    def _publish_status(self):
        self._status["updated_wall_time"] = time.time()
        encoded = json.dumps(self._status, sort_keys=True)
        self._status_pub.publish(String(data=encoded))
        self._atomic_write(json.dumps(
            self._status, indent=2, sort_keys=True) + "\n")

    def _set_status(self, status, reason, **fields):
        self._status.update(fields)
        self._status.update({
            "status": status,
            "ready": status == "READY",
            "reason": reason,
        })
        self._publish_status()

    @staticmethod
    def _quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _resolve_model(self, model_name):
        return resolve_model_sdf(model_name, self._model_roots)

    @staticmethod
    def _model_states(timeout):
        try:
            message = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates, timeout=timeout)
        except rospy.ROSException:
            raise RuntimeError("gazebo model_states unavailable")
        return {name: pose for name, pose in zip(message.name, message.pose)}

    def _spawn(self, anchors):
        if not anchors:
            return {}
        if not self._model_roots:
            raise ValueError("model_roots is empty for non-baseline profile")
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=180.0)
        except rospy.ROSException:
            raise RuntimeError("spawn_sdf_model service unavailable")
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        expected = {}
        for anchor in anchors:
            path = self._resolve_model(anchor["model"])
            with open(path, encoding="utf-8") as handle:
                sdf = handle.read()
            x, y, z, roll, pitch, yaw = anchor["pose"]
            qx, qy, qz, qw = self._quaternion(roll, pitch, yaw)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
            response = spawn(anchor["name"], sdf, "", pose, "")
            if not response.success:
                raise RuntimeError(
                    "spawn anchor %s failed: %s" %
                    (anchor["name"], response.status_message))
            expected[anchor["name"]] = (x, y, z)
        return expected

    def _verify(self, expected):
        if not expected:
            return
        deadline = time.monotonic() + self._verify_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            states = self._model_states(min(
                2.0, max(0.1, deadline - time.monotonic())))
            valid = True
            for name, expected_pose in expected.items():
                actual = states.get(name)
                if actual is None:
                    valid = False
                    break
                error = math.sqrt(
                    (actual.position.x - expected_pose[0]) ** 2 +
                    (actual.position.y - expected_pose[1]) ** 2 +
                    (actual.position.z - expected_pose[2]) ** 2)
                if error > self._verify_tolerance:
                    valid = False
                    break
            if valid:
                return
            rospy.sleep(0.2)
        raise RuntimeError("planner anchor ModelStates verification failed")

    def run(self):
        try:
            profile = validate_anchor_profile(
                self._profile_name, self._profiles)
            self._set_status(
                "SPAWNING", "profile_valid",
                source_revision=profile["source_revision"],
                external_feature_dependency=profile[
                    "external_feature_dependency"],
                expected_models=[item["name"] for item in profile["anchors"]],
                model_roots=self._model_roots)
            expected = self._spawn(profile["anchors"])
            self._verify(expected)
            self._set_status(
                "READY", "anchors_verified",
                spawned_models=sorted(expected),
                verified_models=sorted(expected))
            return 0
        except Exception as exc:
            rospy.logerr("[PlannerAnchor] %s", exc)
            self._set_status("FAIL", str(exc))
            return 1

    def serve_status(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._publish_status()
            rate.sleep()


if __name__ == "__main__":
    node = PlannerAnchorSpawner()
    exit_code = node.run()
    node.serve_status()
    sys.exit(exit_code)
