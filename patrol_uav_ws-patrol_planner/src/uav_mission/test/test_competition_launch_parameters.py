"""Expand final launch parameters offline; never start ROS nodes or hardware."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import roslaunch
import rospkg
import yaml
from uav_mission.competition_config import generate

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "patrol_uav_ws-patrol_planner/src/uav_mission"


class CompetitionLaunchParameters(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix="competition-launch-")
        self.addCleanup(self.folder.cleanup)
        self.out = Path(self.folder.name) / "generated with spaces"
        self.settings = yaml.safe_load(
            (ROOT / "deployment/competition/field.example.yaml").read_text())
        self.settings.update(
            site_confirmed=True,
            corridor_waypoints=[
                dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
                dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)],
            landing_xy=[8.5, -4.2])
        rig = yaml.safe_load(
            (PACKAGE / "config/competition/known_rig.yaml").read_text())
        self.reference = generate(ROOT, self.out, self.settings, (0., 0., 0.), rig)
        self.rospack = rospkg.RosPack(ros_paths=[
            str(ROOT / "vision_ws/src"),
            str(ROOT / "patrol_uav_ws-patrol_planner/src"),
            "/opt/ros/noetic/share"])

    def load(self, path, args):
        config = roslaunch.config.ROSLaunchConfig()
        with patch.object(roslaunch.substitution_args, "_rospack", self.rospack):
            roslaunch.xmlloader.XmlLoader().load(
                str(path), config, argv=args, verbose=False)
        return {key: parameter.value for key, parameter in config.params.items()}

    def application(self, enabled):
        return self.load(PACKAGE / "launch/competition_application.launch", [
            "enable_control_output:=" + str(enabled).lower(),
            "model_path:=/tmp/offline-not-loaded.rknn",
            "generated_dir:=" + str(self.out),
            "ground_z:=" + str(self.reference["ground_z"]),
            "low_z:=" + str(self.reference["low_z"]),
            "cruise_speed:=" + str(self.settings["cruise_speed"]),
            "cruise_acceleration:=" + str(self.settings["cruise_acceleration"])])

    def test_flight_uses_generated_limit_and_preserves_speed_parameters(self):
        values = self.application(True)
        control = yaml.safe_load((self.out / "control.yaml").read_text())
        self.assertEqual(control["px4_max_distance"], .4)
        self.assertEqual(values["/px4_max_distance"], control["px4_max_distance"])
        self.assertEqual(values["/traj_server/traj_server/target_dist"], .4)
        self.assertEqual(values["/external_planner_start_max_distance"], 1.2)
        for namespace, velocity, acceleration in (
                ("manager", "max_vel", "max_acc"),
                ("search", "max_vel", "max_acc"),
                ("optimization", "max_vel", "max_acc"),
                ("bspline", "limit_vel", "limit_acc")):
            self.assertEqual(values["/fast_planner_node/" + namespace + "/" + velocity],
                             self.settings["cruise_speed"])
            self.assertEqual(values["/fast_planner_node/" + namespace + "/" + acceleration],
                             self.settings["cruise_acceleration"])

    def test_generated_limit_is_authoritative_and_is_the_only_changed_parameter(self):
        before = self.application(True)
        control = yaml.safe_load((self.out / "control.yaml").read_text())
        control["px4_max_distance"] = .63
        (self.out / "control.yaml").write_text(yaml.safe_dump(control))
        after = self.application(True)
        self.assertEqual(after["/px4_max_distance"], .63)
        changed = {key for key in before.keys() | after.keys()
                   if before.get(key) != after.get(key)}
        self.assertEqual(changed, {"/px4_max_distance"})

    def test_preview_keeps_control_limit_absent(self):
        values = self.application(False)
        self.assertNotIn("/px4_max_distance", values)
        self.assertEqual(values["/traj_server/traj_server/target_dist"], .4)

    def test_legacy_include_keeps_its_explicit_limit(self):
        values = self.load(
            ROOT / "patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_control_px4_sim.launch",
            ["rviz:=false", "start_waypoint_generator:=false",
             "waypoint_config:=" + str(self.out / "control.yaml"),
             "px4_max_distance:=0.25"])
        self.assertEqual(values["/px4_max_distance"], .25)


if __name__ == "__main__":
    unittest.main(verbosity=2)
