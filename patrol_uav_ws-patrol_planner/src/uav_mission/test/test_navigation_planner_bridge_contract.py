#!/usr/bin/env python3

import ast
from pathlib import Path
import stat
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "navigation_planner_bridge.py"
LAUNCH = PACKAGE_ROOT / "launch" / "navigation_planner_bridge.launch"
CONFIG = PACKAGE_ROOT / "config" / "vcl06_planner_bridge.yaml"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


class NavigationPlannerBridgeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCRIPT.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.launch = ET.parse(str(LAUNCH)).getroot()
        cls.package = ET.parse(str(PACKAGE_XML)).getroot()

    def test_script_has_only_fenced_output_types(self):
        self.assertTrue(SCRIPT.stat().st_mode & stat.S_IXUSR)
        publisher_types = set()
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if (isinstance(function, ast.Attribute) and
                    function.attr == "Publisher" and len(node.args) >= 2 and
                    isinstance(node.args[1], ast.Name)):
                publisher_types.add(node.args[1].id)
        self.assertEqual(
            publisher_types, {"PoseStamped", "NavigationResult", "String"})
        for forbidden in (
                "Servo", "actuator_pwm", "rospy.ServiceProxy",
                "/planning/replan", "mavros", "arming"):
            self.assertNotIn(forbidden, self.source)

    def test_live_planner_topic_is_hard_blocked(self):
        self.assertIn(
            'LIVE_PLANNER_GOAL_TOPIC = "/fastplanner/goal"', self.source)
        self.assertIn(
            'return False, "live_goal_output_blocked_without_cancel_hold_ack"',
            self.source)
        self.assertIn('"live_goal_output_supported": False', self.source)
        self.assertIn("DIAGNOSTIC_ONLY_INTENTS", self.source)
        self.assertIn("CANCEL_PLANNER_GOAL", self.source)
        self.assertIn("START_TARGET_TRANSACTION", self.source)
        self.assertIn("LAND_EXTERNAL", self.source)
        self.assertIn("ABORT_SAFE", self.source)

    def test_goal_is_published_before_dispatch_acceptance(self):
        apply_start = self.source.index("def _apply_outcome")
        apply_end = self.source.index("def _handle_callback_exception")
        body = self.source[apply_start:apply_end]
        self.assertLess(
            body.index("self._publish_planner_goal"),
            body.index("self._result_pub.publish"),
        )
        self.assertIn(
            'raise RuntimeError("dispatch acceptance precedes goal publish")',
            body,
        )

    def test_raw_decision_and_planner_identity_are_checked(self):
        for token in (
                "message.header.seq", "message.decision_seq",
                "nested goal sequence mismatch", "nested goal stamp mismatch",
                "decision goal flag does not match command",
                "decision target flag does not match command",
                "targetless decision carries target identity",
                "requested_goal=requested", "effective_goal=effective",
                "planner nested goal sequence mismatch",
                "planner nested goal stamps differ"):
            self.assertIn(token, self.source)
        self.assertIn(
            'raise ValueError("ALIGN requires the target transaction executor")',
            self.source)

    def test_r2026_limits_are_exact_and_tank_is_absent(self):
        execution = self.config["execution"]
        self.assertFalse(execution["enabled"])
        self.assertFalse(execution["allow_live_goal_output"])
        self.assertEqual(execution["mission_frame"], "camera_init")
        self.assertEqual(execution["class_profile"], "r2026")
        self.assertEqual(execution["max_goal_z"], 4.0)
        self.assertEqual(execution["payload_slots"], 3)
        self.assertEqual(
            execution["allowed_target_classes"],
            ["tent", "pillbox", "bridge", "panzer", "red_cross"],
        )
        self.assertNotIn("tank", execution["allowed_target_classes"])
        self.assertEqual(execution["effective_goal_max_offset"], 1.10)
        self.assertEqual(execution["planner_accept_timeout"], 2.0)

    def test_launch_defaults_to_disabled_isolated_output(self):
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.launch.findall("arg")
        }
        self.assertEqual(arguments["execution_enabled"], "false")
        self.assertEqual(arguments["allow_live_goal_output"], "false")
        self.assertEqual(arguments["kino_planner_confirmed"], "false")
        self.assertEqual(arguments["manual_target_confirmed"], "false")
        self.assertEqual(
            arguments["planner_goal_topic"],
            "/navigation/fastplanner_goal",
        )
        self.assertNotEqual(
            arguments["planner_goal_topic"], "/fastplanner/goal")

    def test_ros_dependencies_are_declared(self):
        for dependency_kind in (
                "build_depend", "build_export_depend", "exec_depend"):
            dependencies = {
                item.text for item in self.package.findall(dependency_kind)
            }
            self.assertIn("nav_msgs", dependencies)
            self.assertIn("plan_manage", dependencies)

    def test_all_callbacks_fail_closed(self):
        self.assertIn("threading.RLock()", self.source)
        self.assertIn("def _handle_callback_exception", self.source)
        for source in ("decision", "planner_status", "odom", "timer"):
            self.assertIn(
                '_handle_callback_exception("%s"' % source,
                self.source,
            )


if __name__ == "__main__":
    unittest.main()
