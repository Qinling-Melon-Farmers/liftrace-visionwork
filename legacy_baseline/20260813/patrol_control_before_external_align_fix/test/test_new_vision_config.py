#!/usr/bin/env python3
"""Static regression checks for the new-vision fixed-drop compatibility route."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
CONTROL_CPP = ROOT / "src/patrol_control/src/patrol_control.cpp"
CONFIG = ROOT / "src/patrol_control/config/patrol_toudi4_new_vision.yaml"
LAUNCH = ROOT / "src/patrol_control/launch/toudi3_full_competition_sim_new_vision.launch"
FULL_LAUNCH = ROOT / "src/patrol_control/launch/patrol_full_competition_sim.launch"
CONTROL_LAUNCH = ROOT / "src/patrol_control/launch/patrol_control_px4_sim.launch"
PLANNER_LAUNCH = ROOT / "src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_px4_sim.launch"


class NewVisionConfigTest(unittest.TestCase):
    def test_controller_has_compatibility_switch_with_legacy_default(self):
        source = CONTROL_CPP.read_text(encoding="utf-8")
        self.assertIn(
            'nh_.param("uav_vision/update_goal_from_selected_target", true)',
            source,
        )
        self.assertIn(
            "if (update_goal_from_selected_target_) {",
            source,
        )
        self.assertIn(
            "if (update_goal_from_selected_target_ &&",
            source,
        )

    def test_new_vision_config_accepts_all_standard_classes_and_disables_rewrite(self):
        config = CONFIG.read_text(encoding="utf-8")
        parsed = yaml.safe_load(config)
        self.assertIn(
            'goal_list: ["bridge", "panzer", "pillbox", "tent", "tank"]',
            config,
        )
        self.assertIn("update_goal_from_selected_target: false", config)
        zero_offsets = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.assertEqual(parsed["drop_system"]["slot_offsets"], zero_offsets)
        self.assertEqual(
            parsed["drop_system"]["dynamic_slot_offsets"], zero_offsets)
        self.assertIn(
            "pixel_to_body_matrix: [0.0, -1.0, -1.0, 0.0]",
            config,
        )

    def test_toudi4_launch_defaults_are_consistent(self):
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("toudi4_copy.world", launch)
        self.assertIn("iris_mid360_downward_camera/model.sdf", launch)
        self.assertIn('default="/downward_camera/image_raw"', launch)
        self.assertIn('default="/downward_camera/camera_info"', launch)
        self.assertIn("patrol_toudi4_new_vision.yaml", launch)

    def test_new_vision_requires_fresh_mission_release_permission(self):
        source = CONTROL_CPP.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        self.assertIn(
            'nh_.param("uav_vision/require_release_permission", false)',
            source,
        )
        self.assertIn("require_release_permission: true", config)
        self.assertIn(
            "release_permission_state_topic: /mission/release_permission_active",
            config,
        )
        self.assertIn("release_permission_timeout: 0.20", config)
        self.assertGreaterEqual(source.count("canRequestDrop("), 3)

    def test_new_vision_launch_passes_camera_model_and_map_parameters(self):
        launch = LAUNCH.read_text(encoding="utf-8")
        for arg in (
            'name="camera_image_topic"',
            'name="camera_info_topic"',
            'name="target_model_path"',
            'name="map_frame"',
            'name="enable_debug_image"',
            'name="drop_stable_frames"',
            'name="waypoint_config"',
        ):
            self.assertIn(arg, launch)
        for arg in (
            'arg name="camera_info_topic" value="$(arg camera_info_topic)"',
            'arg name="target_model_path" value="$(arg target_model_path)"',
            'arg name="map_frame" value="$(arg map_frame)"',
        ):
            self.assertIn(arg, launch)

    def test_new_vision_limits_planner_map_without_changing_legacy_defaults(self):
        new_vision_launch = LAUNCH.read_text(encoding="utf-8")
        full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
        control_launch = CONTROL_LAUNCH.read_text(encoding="utf-8")

        for name, value in (
            ("planner_map_size_x", "14.0"),
            ("planner_map_size_y", "14.0"),
            ("planner_map_size_z", "6.0"),
        ):
            self.assertIn(
                f'<arg name="{name}" default="{value}" />',
                new_vision_launch,
            )
            self.assertIn(
                f'<arg name="{name}" value="$(arg {name})" />',
                new_vision_launch,
            )

        for launch in (full_launch, control_launch):
            for name, value in (
                    ("planner_map_size_x", "100.0"),
                    ("planner_map_size_y", "100.0"),
                    ("planner_map_size_z", "50.0")):
                self.assertIn(
                    f'<arg name="{name}" default="{value}" />',
                    launch,
                )

        for name in (
                "planner_map_size_x",
                "planner_map_size_y",
                "planner_map_size_z"):
            self.assertIn(
                f'<arg name="{name}" value="$(arg {name})" />',
                full_launch,
            )

        for axis in ("x", "y", "z"):
            name = f"planner_map_size_{axis}"
            self.assertIn(
                f'<arg name="map_size_{axis}" value="$(arg {name})" />',
                control_launch,
            )

        planner_launch = PLANNER_LAUNCH.read_text(encoding="utf-8")
        for axis, value in (("x", "100.0"), ("y", "100.0"), ("z", "50.0")):
            self.assertIn(
                f'<arg name="map_size_{axis}" default="{value}"/>',
                planner_launch,
            )


if __name__ == "__main__":
    unittest.main()
