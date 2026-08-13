#!/usr/bin/env python3
"""random_red_cross_spawner: 在搜索区域内随机摆放红十字（模拟随机投放区）。

真值只写入 run 目录 red_cross_truth.yaml 供复盘评分，不进入控制链；manager 与视觉
必须靠搜索独立发现。摆放时避开既有模型（墙/树/箱/机架）并保证边界净空。
"""
import math
import os
import random
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

TRUTH_FILE = "red_cross_truth.yaml"


class RandomRedCrossSpawner:
    def __init__(self):
        rospy.init_node("random_red_cross_spawner")
        self._min_x = float(rospy.get_param("~search_region/min_x", -2.0))
        self._max_x = float(rospy.get_param("~search_region/max_x", 2.0))
        self._min_y = float(rospy.get_param("~search_region/min_y", 0.5))
        self._max_y = float(rospy.get_param("~search_region/max_y", 6.0))
        self._margin = float(rospy.get_param("~margin", 0.6))
        self._min_clearance = float(rospy.get_param("~min_clearance", 1.2))
        self._max_attempts = int(rospy.get_param("~max_attempts", 200))
        self._seed = int(rospy.get_param("~seed", 0))
        self._model_sdf_path = rospy.get_param(
            "~model_sdf",
            "/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/"
            "sitl_gazebo-classic/models/red_cross/model.sdf")
        self._model_name = rospy.get_param("~model_name", "red_cross_random")
        self._run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")
        self._rng = random.Random(self._seed if self._seed > 0 else None)

    def _collect_models(self, timeout=30.0):
        models = []
        received = [False]

        def _callback(msg):
            for name, pose in zip(msg.name, msg.pose):
                if name in ("ground_plane", "sun"):
                    continue
                models.append((name, pose.position.x, pose.position.y))
            received[0] = True

        subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, _callback, queue_size=1)
        deadline = time.time() + timeout
        while (not received[0] and time.time() < deadline and
                not rospy.is_shutdown()):
            rospy.sleep(0.2)
        subscriber.unregister()
        return models

    def _wait_spawn_service(self, timeout=180.0):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model",
                                   timeout=timeout)
            return rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        except rospy.ROSException:
            rospy.logerr("spawn_sdf_model service unavailable")
            return None

    def run(self):
        spawn = self._wait_spawn_service()
        if spawn is None:
            return 1
        models = self._collect_models()
        sdf = open(self._model_sdf_path).read()

        pose = None
        attempts = 0
        while pose is None and attempts < self._max_attempts and \
                not rospy.is_shutdown():
            attempts += 1
            x = self._rng.uniform(self._min_x + self._margin,
                                  self._max_x - self._margin)
            y = self._rng.uniform(self._min_y + self._margin,
                                  self._max_y - self._margin)
            yaw = self._rng.uniform(-math.pi, math.pi)
            clear = True
            for _name, mx, my in models:
                if math.hypot(x - mx, y - my) < self._min_clearance:
                    clear = False
                    break
            if clear:
                pose = (x, y, yaw)
        if pose is None:
            rospy.logerr("no clear pose found after %d attempts", attempts)
            return 1

        x, y, yaw = pose
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = 0.02
        initial_pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.orientation.w = math.cos(yaw / 2.0)
        response = spawn(self._model_name, sdf, "", initial_pose, "")
        if not response.success:
            rospy.logerr("spawn failed: %s", response.status_message)
            return 1

        truth = os.path.join(self._run_dir, TRUTH_FILE)
        with open(truth, "w") as handle:
            handle.write("# 随机红十字摆放真值（仅复盘，不进入控制链）\n")
            handle.write("model: %s\n" % self._model_name)
            handle.write("x: %.4f\n" % x)
            handle.write("y: %.4f\n" % y)
            handle.write("yaw: %.4f\n" % yaw)
            handle.write("seed: %d\n" % self._seed)
        rospy.loginfo("red_cross spawned at (%.3f, %.3f) yaw=%.3f truth=%s",
                      x, y, yaw, truth)
        return 0


if __name__ == "__main__":
    import sys
    sys.exit(RandomRedCrossSpawner().run())
