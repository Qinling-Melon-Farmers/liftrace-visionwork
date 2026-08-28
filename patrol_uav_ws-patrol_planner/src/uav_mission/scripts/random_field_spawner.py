#!/usr/bin/env python3
"""random_field_spawner: 在投递区搜索域内随机摆放全部任务靶标。

2026 规则书场地形态：标准投放区与随机投放区坐标均不给定，需自主搜索。
本节点把 profile 内的标准靶（1m 板）与随机红十字（0.35m）按参数化搜索域
随机摆放，避开既有模型并保证两两净空；真值写入 run 目录供复盘评分，
不进入控制链，manager 与视觉必须靠搜索独立发现。

与 random_red_cross_spawner.py 的关系：本节点面向"全随机场地"（标准靶也
随机），同时向后兼容地写出 red_cross_truth.yaml，使 coverage_r6 断言的
red_cross_required 门控继续成立；旧入口不受影响。
"""
import math
import os
import random
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# 类目 -> Gazebo 模型目录名（材质与几何均为仓库外 GAZEBO_MODEL_PATH 内
# 既有的静态板模型；拼音名来自 2025 资产，不在本节点硬编码路径）。
CLASS_MODEL_NAMES = {
    "tent": "zhangpeng",
    "pillbox": "dibao",
    "bridge": "qiaoliang",
    "panzer": "zhuangjiache",
    "tank": "tanke",
    "red_cross": "red_cross",
}

TRUTH_FILE = "random_field_truth.yaml"
RED_CROSS_TRUTH_FILE = "red_cross_truth.yaml"


class RandomFieldSpawner:
    def __init__(self):
        rospy.init_node("random_field_spawner")
        self._min_x = float(rospy.get_param("~search_region/min_x", -2.0))
        self._max_x = float(rospy.get_param("~search_region/max_x", 2.0))
        self._min_y = float(rospy.get_param("~search_region/min_y", 0.5))
        self._max_y = float(rospy.get_param("~search_region/max_y", 6.0))
        self._margin = float(rospy.get_param("~margin", 0.6))
        # 1m 标准板两两净空按板面不重叠留观测余量；红十字沿用 0.6m 既有口径。
        # 实际 pairwise 约束取双方净空的较大者，避免小板贴上大板板面。
        self._min_clearance_standard = float(
            rospy.get_param("~min_clearance_standard", 1.2))
        self._min_clearance_red_cross = float(
            rospy.get_param("~min_clearance_red_cross", 0.6))
        self._max_attempts = int(rospy.get_param("~max_attempts", 4000))
        self._seed = int(rospy.get_param("~seed", 0))
        self._model_dir = rospy.get_param(
            "~model_dir",
            "/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/"
            "sitl_gazebo-classic/models")
        # 标准靶类目列表（逗号分隔）；默认为 r2026 profile 的四类标准靶。
        self._standard_classes = [
            class_name.strip() for class_name in
            rospy.get_param("~standard_classes",
                            "panzer,bridge,pillbox,tent").split(",")
            if class_name.strip()]
        self._spawn_red_cross = bool(
            rospy.get_param("~spawn_red_cross", True))
        self._run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")
        self._rng = random.Random(self._seed if self._seed > 0 else None)

    def _collect_models(self, timeout=30.0):
        models = []
        received = [False]

        def _callback(msg):
            for name, pose in zip(msg.name, msg.pose):
                if name in ("ground_plane", "sun"):
                    continue
                models.append((name, pose.position.x, pose.position.y, 0.0))
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

    def _clear(self, x, y, clearance, occupied):
        for _name, mx, my, mclear in occupied:
            need = max(clearance, mclear)
            if math.hypot(x - mx, y - my) < need:
                return False
        return True

    def _sample(self, clearance, occupied):
        for _attempt in range(self._max_attempts):
            x = self._rng.uniform(self._min_x + self._margin,
                                  self._max_x - self._margin)
            y = self._rng.uniform(self._min_y + self._margin,
                                  self._max_y - self._margin)
            if self._clear(x, y, clearance, occupied):
                return x, y
        return None

    def run(self):
        spawn = self._wait_spawn_service()
        if spawn is None:
            return 1
        occupied = self._collect_models()
        placements = []

        plan = [(class_name, CLASS_MODEL_NAMES[class_name],
                 self._min_clearance_standard)
                for class_name in self._standard_classes]
        if self._spawn_red_cross:
            plan.append(("red_cross", CLASS_MODEL_NAMES["red_cross"],
                         self._min_clearance_red_cross))

        for class_name, model_name, clearance in plan:
            sdf_path = os.path.join(self._model_dir, model_name, "model.sdf")
            if not os.path.isfile(sdf_path):
                rospy.logerr("model.sdf missing for %s: %s",
                             class_name, sdf_path)
                return 1
            pose = self._sample(clearance, occupied)
            if pose is None:
                rospy.logerr(
                    "no clear pose for %s after %d attempts",
                    class_name, self._max_attempts)
                return 1
            x, y = pose
            yaw = self._rng.uniform(-math.pi, math.pi)
            initial_pose = Pose()
            initial_pose.position.x = x
            initial_pose.position.y = y
            initial_pose.position.z = 0.02
            initial_pose.orientation.z = math.sin(yaw / 2.0)
            initial_pose.orientation.w = math.cos(yaw / 2.0)
            model_sdf = open(sdf_path).read()
            spawned_name = "random_%s" % class_name
            response = spawn(spawned_name, model_sdf, "",
                             initial_pose, "")
            if not response.success:
                rospy.logerr("spawn %s failed: %s",
                             class_name, response.status_message)
                return 1
            occupied.append((spawned_name, x, y, clearance))
            placements.append({
                "class": class_name,
                "model": spawned_name,
                "source": model_name,
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 4),
            })
            rospy.loginfo("spawned %s at (%.3f, %.3f) yaw=%.3f",
                          class_name, x, y, yaw)

        truth_path = os.path.join(self._run_dir, TRUTH_FILE)
        with open(truth_path, "w", encoding="utf-8") as handle:
            handle.write("# 随机场地摆放真值（仅复盘，不进入控制链）\n")
            handle.write("seed: %d\n" % self._seed)
            handle.write("search_region:\n")
            handle.write("  min_x: %.4f\n" % self._min_x)
            handle.write("  max_x: %.4f\n" % self._max_x)
            handle.write("  min_y: %.4f\n" % self._min_y)
            handle.write("  max_y: %.4f\n" % self._max_y)
            handle.write("targets:\n")
            for item in placements:
                handle.write("  - class: %s\n" % item["class"])
                handle.write("    model: %s\n" % item["model"])
                handle.write("    source: %s\n" % item["source"])
                handle.write("    x: %.4f\n" % item["x"])
                handle.write("    y: %.4f\n" % item["y"])
                handle.write("    yaw: %.4f\n" % item["yaw"])

        cross = next((item for item in placements
                      if item["class"] == "red_cross"), None)
        if cross is not None:
            # 向后兼容：coverage_r6 断言以 red_cross_truth.yaml 的存在与否
            # 判定"本轮摆放了随机十字、必须发现"。
            legacy_path = os.path.join(self._run_dir, RED_CROSS_TRUTH_FILE)
            with open(legacy_path, "w", encoding="utf-8") as handle:
                handle.write("# 随机红十字摆放真值（仅复盘，不进入控制链）\n")
                handle.write("model: %s\n" % cross["model"])
                handle.write("x: %.4f\n" % cross["x"])
                handle.write("y: %.4f\n" % cross["y"])
                handle.write("yaw: %.4f\n" % cross["yaw"])
                handle.write("seed: %d\n" % self._seed)
        rospy.loginfo("random field truth written to %s", truth_path)
        return 0


if __name__ == "__main__":
    import sys
    sys.exit(RandomFieldSpawner().run())
