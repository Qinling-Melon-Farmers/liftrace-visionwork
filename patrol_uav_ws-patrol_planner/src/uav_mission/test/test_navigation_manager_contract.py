#!/usr/bin/env python3

import ast
from pathlib import Path
import stat
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "navigation_mission_manager.py"
LAUNCH = PACKAGE_ROOT / "launch" / "navigation_mission_manager.launch"
CONFIG = PACKAGE_ROOT / "config" / "vcl06_runtime.yaml"
FORMAL_CONFIG = PACKAGE_ROOT / "config" / "vcl06_random_field_runtime.yaml"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


class NavigationManagerContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCRIPT.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.formal_config = yaml.safe_load(
            FORMAL_CONFIG.read_text(encoding="utf-8"))
        cls.launch = ET.parse(str(LAUNCH)).getroot()
        cls.package = ET.parse(str(PACKAGE_XML)).getroot()

    def test_script_is_manual_start_and_raw_decision_only(self):
        self.assertTrue(SCRIPT.stat().st_mode & stat.S_IXUSR)
        self.assertIn('"start_mission", Trigger', self.source)
        self.assertIn('"abort_mission", Trigger', self.source)
        self.assertIn(
            '"mission_command_raw", NavigationDecision', self.source)
        self.assertIn("threading.RLock()", self.source)
        self.assertNotIn("auto_start", self.source)
        for forbidden in (
                "/fastplanner/goal", "Servo", "actuator_pwm", "arming"):
            self.assertNotIn(forbidden, self.source)

        publisher_types = set()
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if (isinstance(function, ast.Attribute) and
                    function.attr == "Publisher" and len(node.args) >= 2 and
                    isinstance(node.args[1], ast.Name)):
                publisher_types.add(node.args[1].id)
        self.assertEqual(publisher_types, {"NavigationDecision", "String"})

    def test_decision_maps_lease_and_exact_target_identity(self):
        self.assertIn("message.deadline =", self.source)
        self.assertIn("message.decision_seq =", self.source)
        self.assertIn("message.header.seq =", self.source)
        self.assertIn("_ns_to_stamp(target.first_seen_ns)", self.source)
        self.assertIn("target.last_seen_ns", self.source)
        self.assertIn("TargetCandidateArray", self.source)

    def test_runtime_limits_and_profile_are_frozen(self):
        self.assertEqual(self.config["profile"]["name"], "r2026")
        mission = self.config["mission"]
        self.assertEqual(mission["forced_return_at"], 510.0)
        self.assertEqual(mission["timeout"], 600.0)
        self.assertEqual(mission["max_target_z"], 4.0)
        self.assertEqual(mission["approach_altitude"], 1.2)
        self.assertEqual(mission["return_altitude"], 2.2)
        self.assertNotIn("classes", self.config)

    def test_formal_runtime_has_ordered_three_door_route_to_final_h(self):
        mission = self.formal_config["mission"]
        route = mission["post_delivery_route"]
        self.assertEqual(mission["frame"], "camera_init")
        self.assertEqual(mission["forced_return_at"], 420.0)
        self.assertEqual(mission["return_land_reserve"], 180.0)
        self.assertEqual(
            mission["post_delivery_route_revision"],
            "toudi3-random-three-door-h-r1",
        )
        self.assertEqual(len(route), 11)
        self.assertEqual(route[1], [-2.386703, 6.072270, 1.0])
        self.assertEqual(route[5], [-0.112003, 8.053133, 1.2])
        self.assertEqual(route[8], [2.123022, 8.009650, 1.2])
        self.assertEqual(route[-1][:2], mission["landing_xy"])
        self.assertLessEqual(max(point[2] for point in route), 4.0)
        self.assertIn("GoalSnapshot", self.source)
        self.assertIn("post_delivery_route_index", self.source)

    def test_launch_exposes_only_contract_topics(self):
        nodes = self.launch.findall(".//node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(
            nodes[0].attrib["type"], "navigation_mission_manager.py")
        remaps = {
            item.attrib["from"]: item.attrib["to"]
            for item in nodes[0].findall("remap")
        }
        self.assertIn("mission_command_raw", remaps)
        self.assertIn("mission_result", remaps)
        self.assertIn("candidates", remaps)
        self.assertNotIn("goal", remaps)
        self.assertNotIn(
            "/fastplanner/goal", LAUNCH.read_text(encoding="utf-8"))

    def test_r2026_readiness_and_callbacks_fail_closed(self):
        self.assertTrue(self.config["readiness"]["require_map"])
        self.assertIn("r2026 requires map readiness", self.source)
        self.assertIn('return False, "map_empty"', self.source)
        self.assertIn('return False, "map_layout_invalid"', self.source)
        self.assertIn('return False, "map_data_truncated"', self.source)
        self.assertIn("def _handle_callback_exception", self.source)
        for source in ("candidates", "result", "timer"):
            self.assertIn(
                '_handle_callback_exception("%s"' % source,
                self.source,
            )
        self.assertNotIn('abort("executor_changed"', self.source)

    def test_runtime_declares_yaml_dependency(self):
        dependencies = {
            item.text for item in self.package.findall("exec_depend")
        }
        self.assertIn("python3-yaml", dependencies)


if __name__ == "__main__":
    unittest.main()
