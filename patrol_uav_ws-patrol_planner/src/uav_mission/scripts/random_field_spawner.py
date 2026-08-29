#!/usr/bin/env python3
"""Spawn a reproducible, footprint-safe random task field.

Truth remains evaluation-only. Runtime consumers synchronize exclusively on
the continuously published, latched JSON status; READY is emitted only after
all models are present at their requested Gazebo poses and truth is durable.
"""

import json
import math
import os
import random
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from coverage_policy import profile_standard_classes
from uav_mission.random_field_policy import (
    Footprint,
    RED_CROSS_FOOTPRINT_RADIUS,
    STANDARD_FOOTPRINT_RADIUS,
    plan_footprint_layout,
    validate_bounds,
    validate_seed,
    validate_standard_classes,
)


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
STATUS_FILE = "random_field_status.json"


class RandomFieldSpawner:
    def __init__(self):
        rospy.init_node("random_field_spawner")
        self._run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")
        self._status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/mission/random_field_status"),
            String, queue_size=1, latch=True)
        self._status_path = rospy.get_param(
            "~status_path", os.path.join(self._run_dir, STATUS_FILE))
        self._status = {
            "component": "random_field_spawner",
            "status": "INIT",
            "reason": "initializing",
            "ready": False,
            "updated_wall_time": time.time(),
        }

        self._profile = rospy.get_param("~class_profile", "r2026")
        self._seed = int(rospy.get_param("~seed", 0))
        self._search_bounds = (
            rospy.get_param("~search_region/min_x", -2.0),
            rospy.get_param("~search_region/max_x", 2.0),
            rospy.get_param("~search_region/min_y", 0.5),
            rospy.get_param("~search_region/max_y", 6.0),
        )
        self._field_bounds = (
            rospy.get_param("~field/min_x", -3.992),
            rospy.get_param("~field/max_x", 4.008),
            rospy.get_param("~field/min_y", -1.132),
            rospy.get_param("~field/max_y", 8.718),
        )
        self._boundary_margin = float(
            rospy.get_param("~spawn/boundary_margin", 0.10))
        self._pair_gap = float(rospy.get_param("~spawn/pair_gap", 0.15))
        self._static_model_radius = float(
            rospy.get_param("~spawn/static_model_radius", 0.55))
        self._max_attempts = int(rospy.get_param("~max_attempts", 4000))
        self._layout_attempts = int(rospy.get_param(
            "~spawn/layout_attempts", 64))
        self._verification_tolerance = float(
            rospy.get_param("~spawn/verification_tolerance", 0.05))
        self._verification_timeout = float(
            rospy.get_param("~spawn/verification_timeout", 20.0))
        self._offset_x = float(rospy.get_param("~spawn_offset/x", 0.0))
        self._offset_y = float(rospy.get_param("~spawn_offset/y", 0.0))
        self._spawn_red_cross = bool(
            rospy.get_param("~spawn_red_cross", True))

        raw_classes = rospy.get_param(
            "~standard_classes", "panzer,bridge,pillbox,tent")
        self._standard_classes = tuple(
            item.strip() for item in raw_classes.split(",") if item.strip())
        raw_roots = rospy.get_param(
            "~model_roots", os.environ.get("GAZEBO_MODEL_PATH", ""))
        if isinstance(raw_roots, list):
            self._model_roots = [os.path.abspath(root) for root in raw_roots
                                 if root]
        else:
            self._model_roots = [
                os.path.abspath(root)
                for root in str(raw_roots).split(os.pathsep) if root]
        raw_ignore = rospy.get_param(
            "~ignore_model_names", "ground_plane,sun,toudi2,3")
        self._ignore_model_names = {
            item.strip() for item in raw_ignore.split(",") if item.strip()}
        self._static_model_radii = rospy.get_param(
            "~static_model_radii", {})
        self._static_exclusions = rospy.get_param("~static_exclusions", [])
        self._rng = None
        self._publish_status()

    def _atomic_write(self, path, content):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.replace(temporary, path)

    def _publish_status(self):
        self._status["updated_wall_time"] = time.time()
        payload = json.dumps(self._status, sort_keys=True)
        self._status_pub.publish(String(data=payload))
        self._atomic_write(
            self._status_path,
            json.dumps(self._status, indent=2, sort_keys=True) + "\n")

    def _set_status(self, status, reason, **fields):
        self._status.update(fields)
        self._status.update({
            "status": status,
            "reason": reason,
            "ready": status == "READY",
        })
        self._publish_status()

    def _validate(self):
        self._seed = validate_seed(self._seed)
        self._search_bounds = validate_bounds(
            self._search_bounds, "search_region")
        self._field_bounds = validate_bounds(self._field_bounds, "field")
        required = profile_standard_classes(self._profile)
        self._standard_classes = validate_standard_classes(
            self._standard_classes, required)
        if (self._boundary_margin < 0.0 or self._pair_gap < 0.0 or
                self._static_model_radius < 0.0 or self._max_attempts <= 0 or
                self._layout_attempts <= 0):
            raise ValueError("spawn margins/radii/attempts are invalid")
        if not self._model_roots:
            raise ValueError("model_roots is empty; pass GAZEBO_MODEL_PATH")
        normalized_radii = {}
        for name, raw_radius in self._static_model_radii.items():
            radius = float(raw_radius)
            if not math.isfinite(radius) or radius < 0.0:
                raise ValueError(
                    "static model radius must be finite and nonnegative: %s" %
                    name)
            normalized_radii[str(name)] = radius
        self._static_model_radii = normalized_radii
        for item in self._static_exclusions:
            for key in ("name", "world_x", "world_y", "radius"):
                if key not in item:
                    raise ValueError(
                        "static exclusion is missing %s: %r" % (key, item))
            values = (float(item["world_x"]), float(item["world_y"]),
                      float(item["radius"]))
            if not all(math.isfinite(value) for value in values):
                raise ValueError("static exclusion must be finite: %r" % item)
            if values[2] < 0.0:
                raise ValueError("static exclusion radius must be nonnegative")
        self._rng = random.Random(self._seed)
        self._set_status(
            "VALIDATED", "configuration_valid",
            profile=self._profile, seed=self._seed,
            allowed_classes=list(self._standard_classes) +
            (["red_cross"] if self._spawn_red_cross else []),
            layout_attempts=self._layout_attempts,
            model_roots=self._model_roots,
            static_model_radii=self._static_model_radii,
            compound_exclusions=[
                str(item["name"]) for item in self._static_exclusions])

    def _collect_models(self, timeout=30.0):
        try:
            message = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates, timeout=timeout)
        except rospy.ROSException:
            raise RuntimeError("gazebo model_states unavailable")
        return {name: pose for name, pose in zip(message.name, message.pose)}

    def _wait_spawn_service(self, timeout=180.0):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=timeout)
        except rospy.ROSException:
            raise RuntimeError("spawn_sdf_model service unavailable")
        return rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    def _resolve_model_sdf(self, model_name):
        checked = []
        for root in self._model_roots:
            path = os.path.join(root, model_name, "model.sdf")
            checked.append(path)
            if os.path.isfile(path):
                return path
        raise RuntimeError(
            "model.sdf missing for %s under %s" %
            (model_name, ", ".join(checked)))

    def _occupied(self, model_states):
        occupied = []
        for name, pose in model_states.items():
            if name in self._ignore_model_names or name.startswith("random_"):
                continue
            occupied.append(Footprint(
                name, float(pose.position.x), float(pose.position.y),
                self._static_model_radii.get(
                    name, self._static_model_radius)))
        for item in self._static_exclusions:
            occupied.append(Footprint(
                str(item["name"]), float(item["world_x"]),
                float(item["world_y"]), float(item["radius"])))
        return occupied

    def _verify_spawned(self, expected):
        deadline = time.monotonic() + self._verification_timeout
        last_missing = sorted(expected)
        last_wrong_pose = []
        while time.monotonic() < deadline and not rospy.is_shutdown():
            states = self._collect_models(timeout=min(
                2.0, max(0.1, deadline - time.monotonic())))
            missing = []
            wrong_pose = []
            for name, (expected_x, expected_y) in expected.items():
                pose = states.get(name)
                if pose is None:
                    missing.append(name)
                    continue
                error = math.hypot(
                    pose.position.x - expected_x,
                    pose.position.y - expected_y)
                if error > self._verification_tolerance:
                    wrong_pose.append({"model": name, "error_m": error})
            if not missing and not wrong_pose:
                return
            last_missing = missing
            last_wrong_pose = wrong_pose
            rospy.sleep(0.2)
        raise RuntimeError(
            "spawn verification failed; missing=%r wrong_pose=%r" %
            (last_missing, last_wrong_pose))

    def _truth_content(self, placements):
        lines = [
            "# Random-field truth (evaluation only; never a control input)",
            "profile: %s" % self._profile,
            "seed: %d" % self._seed,
            "spawn_offset:",
            "  x: %.4f" % self._offset_x,
            "  y: %.4f" % self._offset_y,
            "search_region:",
            "  min_x: %.4f" % self._search_bounds[0],
            "  max_x: %.4f" % self._search_bounds[1],
            "  min_y: %.4f" % self._search_bounds[2],
            "  max_y: %.4f" % self._search_bounds[3],
            "targets:",
        ]
        for item in placements:
            lines.extend([
                "  - class: %s" % item["class"],
                "    model: %s" % item["model"],
                "    source: %s" % item["source"],
                "    footprint_radius: %.4f" % item["footprint_radius"],
                "    x: %.4f" % item["x"],
                "    y: %.4f" % item["y"],
                "    world_x: %.4f" % item["world_x"],
                "    world_y: %.4f" % item["world_y"],
                "    yaw: %.4f" % item["yaw"],
            ])
        return "\n".join(lines) + "\n"

    def _write_truth(self, placements):
        truth_path = os.path.join(self._run_dir, TRUTH_FILE)
        self._atomic_write(truth_path, self._truth_content(placements))
        cross = next((item for item in placements
                      if item["class"] == "red_cross"), None)
        if cross is not None:
            legacy = "\n".join([
                "# Random red-cross truth; x/y are mission-frame coordinates",
                "model: %s" % cross["model"],
                "x: %.4f" % cross["x"],
                "y: %.4f" % cross["y"],
                "world_x: %.4f" % cross["world_x"],
                "world_y: %.4f" % cross["world_y"],
                "yaw: %.4f" % cross["yaw"],
                "seed: %d" % self._seed,
                "",
            ])
            self._atomic_write(
                os.path.join(self._run_dir, RED_CROSS_TRUTH_FILE), legacy)
        return truth_path

    def _execute(self):
        self._validate()
        spawn = self._wait_spawn_service()
        occupied = self._occupied(self._collect_models())
        plan = [(class_name, CLASS_MODEL_NAMES[class_name],
                 STANDARD_FOOTPRINT_RADIUS)
                for class_name in self._standard_classes]
        if self._spawn_red_cross:
            plan.append(("red_cross", CLASS_MODEL_NAMES["red_cross"],
                         RED_CROSS_FOOTPRINT_RADIUS))
        self._set_status(
            "SPAWNING", "placing_models",
            expected_models=["random_%s" % item[0] for item in plan],
            occupied_footprint_count=len(occupied))

        layout = plan_footprint_layout(
            self._rng,
            [(class_name, radius)
             for class_name, _model_name, radius in plan],
            occupied, self._search_bounds, self._field_bounds,
            self._boundary_margin, self._pair_gap,
            self._offset_x, self._offset_y,
            self._max_attempts, self._layout_attempts)
        if layout is None:
            raise RuntimeError(
                "no footprint-safe full layout after %d restarts "
                "(%d attempts per target)" %
                (self._layout_attempts, self._max_attempts))

        placements = []
        expected = {}
        for (class_name, model_name, radius), (
                planned_class, lx, ly) in zip(plan, layout):
            if planned_class != class_name:
                raise RuntimeError("planned class order changed unexpectedly")
            sdf_path = self._resolve_model_sdf(model_name)
            wx, wy = lx + self._offset_x, ly + self._offset_y
            yaw = self._rng.uniform(-math.pi, math.pi)
            initial_pose = Pose()
            initial_pose.position.x = wx
            initial_pose.position.y = wy
            initial_pose.position.z = 0.02
            initial_pose.orientation.z = math.sin(yaw / 2.0)
            initial_pose.orientation.w = math.cos(yaw / 2.0)
            with open(sdf_path, encoding="utf-8") as handle:
                model_sdf = handle.read()
            spawned_name = "random_%s" % class_name
            response = spawn(
                spawned_name, model_sdf, "", initial_pose, "")
            if not response.success:
                raise RuntimeError(
                    "spawn %s failed: %s" %
                    (class_name, response.status_message))
            expected[spawned_name] = (wx, wy)
            placements.append({
                "class": class_name,
                "model": spawned_name,
                "source": model_name,
                "footprint_radius": radius,
                "x": lx, "y": ly,
                "world_x": wx, "world_y": wy,
                "yaw": yaw,
            })

        self._set_status(
            "VERIFYING", "checking_model_states",
            spawned_models=sorted(expected))
        self._verify_spawned(expected)
        truth_path = self._write_truth(placements)
        self._set_status(
            "READY", "models_verified_and_truth_written",
            spawned_models=sorted(expected),
            verified_models=sorted(expected),
            truth_path=truth_path,
            footprint_valid=True)

    def run(self):
        try:
            self._execute()
            return 0
        except Exception as exc:
            rospy.logerr("[RandomField] %s", exc)
            self._set_status("FAIL", str(exc))
            return 1

    def serve_status(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._publish_status()
            rate.sleep()


if __name__ == "__main__":
    node = RandomFieldSpawner()
    exit_code = node.run()
    node.serve_status()
    sys.exit(exit_code)
