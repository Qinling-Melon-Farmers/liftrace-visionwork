#!/usr/bin/env python3
"""Static regression checks for the new-vision fixed-drop compatibility route."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
CONTROL_CPP = ROOT / "src/patrol_control/src/patrol_control.cpp"
CONFIG = ROOT / "src/patrol_control/config/patrol_toudi4_new_vision.yaml"
TOUDI3_CONFIG = ROOT / "src/patrol_control/config/patrol_toudi3_new_vision.yaml"
BRIDGE_CONFIG = ROOT / "src/uav_mission/config/vcl06_planner_bridge.yaml"
LAUNCH = ROOT / "src/patrol_control/launch/toudi3_full_competition_sim_new_vision.launch"
FULL_LAUNCH = ROOT / "src/patrol_control/launch/patrol_full_competition_sim.launch"
CONTROL_LAUNCH = ROOT / "src/patrol_control/launch/patrol_control_px4_sim.launch"
PLANNER_LAUNCH = ROOT / "src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_px4_sim.launch"
INTERNAL_PLANNER_LAUNCH = ROOT / "src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_sim.launch"
PLANNER_SIM_XML = ROOT / "src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_sim.xml"
PLANNER_REAL_XML = ROOT / "src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_real.xml"


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
        full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("toudi4_copy.world", launch)
        self.assertIn("iris_mid360_downward_camera/model.sdf", launch)
        self.assertIn('default="/downward_camera/image_raw"', launch)
        self.assertIn('default="/downward_camera/camera_info"', launch)
        self.assertIn("patrol_toudi4_new_vision.yaml", launch)
        for text in (launch, full_launch):
            self.assertIn('name="spawn_x" default="-0.493412"', text)
            self.assertIn('name="spawn_y" default="-1.772690"', text)

    def test_toudi4_waypoints_are_local_to_main_h(self):
        parsed = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        points = parsed["waypoints"]
        self.assertEqual(
            [(point["x"], point["y"]) for point in points[:4]],
            [(0.0, 0.0), (-0.081, 2.293),
             (-1.6, 2.477), (1.769, 2.354)],
        )
        self.assertEqual((points[-1]["x"], points[-1]["y"]), (0.0, 0.0))

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
        self.assertGreaterEqual(source.count("dropReleaseReady("), 3)
        self.assertNotIn("uav_pose.pose.position.z <= 0.17", source)
        self.assertNotIn("ttt <= 0.15", source)

    def test_external_release_and_recovery_have_one_authority(self):
        source = CONTROL_CPP.read_text(encoding="utf-8")
        bridge = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))
        for path in (CONFIG, TOUDI3_CONFIG):
            with self.subTest(path=path.name):
                config = yaml.safe_load(path.read_text(encoding="utf-8"))
                drop = config["drop_system"]
                vision = config["uav_vision"]
                self.assertEqual(drop["position_threshold"], 0.15)
                self.assertEqual(drop["release_setpoint_height"], 0.10)
                self.assertEqual(
                    vision["recovery_height"],
                    bridge["target"]["recovery_height"])
        self.assertNotIn("external_alignment_timeout", source)
        self.assertIn(
            'current_align_mode_ == "drop_circle" && uav_drop_ready_',
            source,
        )
        self.assertIn(
            'current_align_mode_ == "drop_cross" && uav_drop_ready_',
            source,
        )

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

    def test_simulation_planner_uses_bounded_arena_map_profile(self):
        new_vision_launch = LAUNCH.read_text(encoding="utf-8")
        full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
        control_launch = CONTROL_LAUNCH.read_text(encoding="utf-8")

        for launch in (new_vision_launch, full_launch, control_launch):
            for name, value in (
                    ("planner_map_size_x", "20.0"),
                    ("planner_map_size_y", "20.0"),
                    ("planner_map_size_z", "5.0")):
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
                new_vision_launch,
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
        for axis, value in (("x", "20.0"), ("y", "20.0"), ("z", "5.0")):
            self.assertIn(
                f'<arg name="map_size_{axis}" default="{value}"/>',
                planner_launch,
            )
        self.assertIn(
            '<arg name="obstacles_inflation" default="0.25"/>',
            planner_launch,
        )

    def test_external_planner_setpoint_height_guard_is_wired_end_to_end(self):
        source = CONTROL_CPP.read_text(encoding="utf-8")
        self.assertIn(
            'nh_.param("external_planner_max_command_z", 3.5)', source)
        self.assertIn(
            "mavros_point_cmd.pose.position.z >", source)
        self.assertIn("capping command height", source)
        self.assertIn("preserving horizontal progress", source)
        distance_limit = source.index(
            "if (distance_to_target > px4_max_distance)")
        final_guard = source.index("enforcing final command height")
        publish = source.index("mavros_point_cmd_pub.publish")
        self.assertLess(distance_limit, final_guard)
        self.assertLess(final_guard, publish)
        self.assertIn(
            "mavros_point_cmd.pose.position.z = "
            "external_planner_max_command_z_",
            source[final_guard:publish],
        )

        for path in (LAUNCH, FULL_LAUNCH, CONTROL_LAUNCH):
            launch = path.read_text(encoding="utf-8")
            self.assertIn(
                'name="external_planner_max_command_z" default="3.5"',
                launch)
        self.assertIn(
            '<param name="external_planner_max_command_z"',
            CONTROL_LAUNCH.read_text(encoding="utf-8"))
        for path in (LAUNCH, FULL_LAUNCH):
            self.assertIn(
                'arg name="external_planner_max_command_z"',
                path.read_text(encoding="utf-8"))

    def test_simulation_inflation_profile_is_complete_and_real_is_unchanged(self):
        sim_root = ET.parse(str(PLANNER_SIM_XML)).getroot()
        sim_node = sim_root.find("node")
        self.assertIsNotNone(sim_node)
        sim_params = {param.attrib["name"]: param.attrib.get("value")
                      for param in sim_node.findall("param")}
        self.assertEqual(
            sim_params["sdf_map/obstacles_inflation"],
            "$(arg obstacles_inflation)",
        )
        self.assertEqual(sim_params["sdf_map/obstacles_inflation_up"], "0.2")
        self.assertEqual(sim_params["sdf_map/obstacles_inflation_down"], "0.1")
        self.assertEqual(sim_params["sdf_map/local_update_range_z"], "4.5")

        internal_root = ET.parse(str(INTERNAL_PLANNER_LAUNCH)).getroot()
        internal_args = {arg.attrib["name"]: arg.attrib.get("value")
                         for arg in internal_root.findall("arg")}
        self.assertEqual(internal_args["map_size_x"], "20.0")
        self.assertEqual(internal_args["map_size_y"], "20.0")
        self.assertEqual(internal_args["map_size_z"], "5.0")
        self.assertEqual(internal_args["obstacles_inflation"], "0.25")
        include = internal_root.find("include")
        self.assertIsNotNone(include)
        passed_args = {arg.attrib["name"]: arg.attrib.get("value")
                       for arg in include.findall("arg")}
        self.assertEqual(
            passed_args["obstacles_inflation"],
            "$(arg obstacles_inflation)",
        )
        self.assertEqual(
            passed_args["clearance_threshold"],
            "$(arg clearance_threshold)",
        )

        real_root = ET.parse(str(PLANNER_REAL_XML)).getroot()
        real_node = real_root.find("node")
        self.assertIsNotNone(real_node)
        real_params = {param.attrib["name"]: param.attrib.get("value")
                       for param in real_node.findall("param")}
        # This navigation repository keeps its existing real-aircraft profile;
        # only the bounded simulation profile is changed by this integration.
        self.assertEqual(real_params["sdf_map/obstacles_inflation"], "0.25")
        self.assertEqual(real_params["sdf_map/obstacles_inflation_up"], "0.2")
        self.assertEqual(real_params["sdf_map/obstacles_inflation_down"], "0.1")


if __name__ == "__main__":
    unittest.main()
