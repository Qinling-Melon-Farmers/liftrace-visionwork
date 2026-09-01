#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
PATROL = WORKSPACE_SRC / "patrol_control"


class ExternalMissionContractTest(unittest.TestCase):
    def test_mission_command_contract_is_exact(self):
        message = (PATROL / "msg" / "MissionCommand.msg").read_text(
            encoding="utf-8").strip().splitlines()
        self.assertEqual(message, [
            "uint8 SEARCH=0",
            "uint8 APPROACH=1",
            "uint8 ALIGN=2",
            "uint8 RESUME=3",
            "uint8 RETURN_HOME=4",
            "uint8 LAND=5",
            "Header header",
            "uint8 command",
            "uint32 target_id",
            "string target_class",
            "geometry_msgs/PoseStamped goal",
        ])

    def test_external_mode_defaults_false_through_launch_chain(self):
        launch_files = [
            PATROL / "launch" / "patrol_control_px4_sim.launch",
            PATROL / "launch" / "patrol_full_competition_sim.launch",
            PATROL / "launch" / "toudi3_full_competition_sim_new_vision.launch",
            PACKAGE / "launch" / "toudi3_visual_delivery_guarded.launch",
        ]
        for launch_file in launch_files:
            with self.subTest(launch_file=launch_file.name):
                root = ET.parse(str(launch_file)).getroot()
                args = {element.attrib.get("name"): element.attrib
                        for element in root.findall("arg")}
                self.assertEqual(
                    args["external_mission_mode"].get("default"), "false")

    def test_patrol_goal_publisher_and_next_point_are_guarded(self):
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("if (!external_mission_mode_)", source)
        self.assertIn("NextPoint disabled in external mission mode", source)
        self.assertIn("planner goal publisher disabled", source)
        self.assertIn("hasValidExternalPlannerCommand", source)
        self.assertIn("position.z <= 0.05", source)
        self.assertIn("external_planner_start_max_distance", source)
        self.assertIn(
            'nh_.param("switch/flag_planner_px4", true)', source)
        self.assertEqual(source.count(
            'advertise<geometry_msgs::PoseStamped>("/fastplanner/goal"'), 1)

    def test_control_readiness_is_latched_after_takeoff(self):
        header = (PATROL / "include" / "patrol_control" /
                  "patrol_control.h").read_text(encoding="utf-8")
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("ros::Publisher control_ready_pub_", header)
        self.assertIn("bool control_ready_latched_ = false", header)
        self.assertIn('control_ready_topic_ = "/mission/control_ready"',
                      header)
        self.assertIn(
            "advertise<std_msgs::Bool>(control_ready_topic_, 1, true)",
            source)
        self.assertIn("publishControlReady(false)", source)
        self.assertIn("flag_takeoff_done = 1", source)
        self.assertIn("publishControlReady(true)", source)
        self.assertLess(
            source.index("flag_takeoff_done = 1"),
            source.index("publishControlReady(true)"))
        ready_window = source[
            source.index("flag_takeoff_done = 1"):
            source.index("void LLController::publishControlReady")]
        self.assertIn("if (external_mission_mode_)", ready_window)

    def test_external_gate_enables_mode_and_manager_owns_goal(self):
        launch = ET.parse(str(PACKAGE / "launch" /
                              "external_candidate.launch")).getroot()
        include_args = {
            arg.attrib["name"]: arg.attrib.get("value")
            for include in launch.findall("include")
            for arg in include.findall("arg")
        }
        self.assertEqual(include_args["external_mission_mode"], "true")
        self.assertEqual(include_args["waypoint_mode"], "false")
        manager = (PACKAGE / "scripts" /
                   "external_candidate_manager.py").read_text(encoding="utf-8")
        self.assertEqual(manager.count(
            'rospy.Publisher(\n            "/fastplanner/goal"'), 1)


if __name__ == "__main__":
    unittest.main()
