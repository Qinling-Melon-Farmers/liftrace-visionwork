#!/usr/bin/env python3

import ast
import copy
import math
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
        self.assertEqual(mission["motion_action_timeout"], 90.0)
        self.assertIn(
            '"~mission/motion_action_timeout", 90.0', self.source)
        self.assertNotIn("classes", self.config)

    def test_formal_runtime_has_ordered_three_door_route_to_final_h(self):
        mission = self.formal_config["mission"]
        route = mission["post_delivery_route"]
        self.assertEqual(mission["frame"], "camera_init")
        self.assertEqual(mission["forced_return_at"], 420.0)
        self.assertEqual(mission["return_land_reserve"], 180.0)
        self.assertEqual(mission["target_action_timeout"], 120.0)
        self.assertEqual(mission["motion_action_timeout"], 90.0)
        self.assertEqual(
            mission["post_delivery_route_revision"],
            "toudi3-random-three-door-h-r4",
        )
        self.assertEqual(len(route), 9)
        self.assertEqual(route[0], [-2.386703, 4.672270, 2.2])
        self.assertEqual(route[1], [-2.386703, 5.672270, 0.9])
        self.assertEqual(route[2], [-2.386703, 6.472270, 0.9])
        self.assertEqual(route[4], [-0.512003, 8.053133, 1.2])
        self.assertEqual(route[5], [0.287997, 8.053133, 1.2])
        self.assertEqual(route[6], [1.723022, 8.009650, 1.2])
        self.assertEqual(route[7], [2.523022, 8.009650, 1.2])
        self.assertEqual(route[-1][:2], mission["landing_xy"])
        self.assertLessEqual(max(point[2] for point in route), 4.0)
        self.assertGreater(route[0][2], 1.5)
        self.assertEqual(route[0][0], route[1][0])
        self.assertLess(route[0][1], route[1][1])
        self.assertAlmostEqual(route[1][1] - route[0][1], 1.0)
        doors = self.formal_config["post_delivery_gate"]["doors"]
        self.assertEqual(
            [door["route_indices"] for door in doors], [[3], [6], [8]])
        for door in doors:
            axis = 0 if door["axis"] == "x" else 1
            crossing_index = door["route_indices"][0] - 1
            previous = route[crossing_index - 1][axis]
            current = route[crossing_index][axis]
            self.assertLess(previous, door["coordinate"])
            self.assertGreater(current, door["coordinate"])
            self.assertNotEqual(current, door["coordinate"])
        wall_15 = doors[0]
        self.assertGreater(wall_15["coordinate"] - route[1][1], 0.3)
        self.assertGreater(route[2][1] - wall_15["coordinate"], 0.3)
        self.assertIn("GoalSnapshot", self.source)
        self.assertIn("post_delivery_route_index", self.source)

        search = self.formal_config["search"]
        self.assertEqual(
            [search["min_x"], search["max_x"],
             search["min_y"], search["max_y"]],
            [-2.007, 1.993, 0.273, 6.273],
        )
        self.assertEqual(
            search["route_revision"],
            "toudi3-random-spawn-envelope-r2",
        )

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
        launch_args = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.launch.findall("arg")}
        self.assertEqual(launch_args["start_mode"], "full")
        params = {
            item.attrib["name"]: item.attrib.get("value")
            for item in nodes[0].findall("param")}
        self.assertEqual(params["runtime/start_mode"],
                         "$(arg start_mode)")

    def test_post_delivery_start_is_explicit_and_reported(self):
        self.assertIn(
            'self._start_mode not in ("full", "post_delivery")',
            self.source)
        self.assertIn("runtime.start_post_delivery_validation(",
                      self.source)
        self.assertIn('"start_mode": self._start_mode', self.source)

    def test_r2026_readiness_and_callbacks_fail_closed(self):
        self.assertTrue(self.config["readiness"]["require_map"])
        for config in (self.config, self.formal_config):
            readiness = config["readiness"]
            self.assertEqual(readiness["pose_max_age"], 0.5)
            self.assertEqual(readiness["stamp_future_tolerance"], 0.05)
        self.assertIn('return "future"', self.source)
        self.assertIn('return False, "pose_stamp_in_future"', self.source)
        self.assertIn('return False, "map_stamp_in_future"', self.source)
        self.assertIn('self._stamp_future_tolerance > 0.1', self.source)
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
        self.assertIn("TRANSIENT_READINESS_FAILURES", self.source)
        self.assertIn(
            'self._last_reason = "runtime_waiting_for_%s" % reason',
            self.source,
        )

    def test_readiness_enforces_ceiling_not_local_frame_floor(self):
        self.assertIn("if position.z > 4.0:", self.source)
        self.assertNotIn("position.z < 0.0", self.source)
        self.assertIn(
            'return False, "pose_altitude_out_of_bounds"', self.source)

    def test_readiness_accepts_only_bounded_future_stamp_jitter(self):
        manager = next(
            node for node in self.tree.body
            if isinstance(node, ast.ClassDef) and
            node.name == "NavigationMissionManager"
        )
        function = copy.deepcopy(next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and
            node.name == "_age_state"
        ))
        function.decorator_list = []
        module = ast.fix_missing_locations(
            ast.Module(body=[function], type_ignores=[]))
        namespace = {"math": math}
        exec(compile(module, str(SCRIPT), "exec"), namespace)

        class Stamp:
            def __init__(self, seconds):
                self._seconds = seconds

            def to_sec(self):
                return self._seconds

        age_state = namespace["_age_state"]
        self.assertEqual(age_state(Stamp(10.001), 10.0, 0.5, 0.05),
                         "fresh")
        self.assertEqual(age_state(Stamp(10.051), 10.0, 0.5, 0.05),
                         "future")
        self.assertEqual(age_state(Stamp(9.499), 10.0, 0.5, 0.05),
                         "stale")

    def test_runtime_declares_yaml_dependency(self):
        dependencies = {
            item.text for item in self.package.findall("exec_depend")
        }
        self.assertIn("python3-yaml", dependencies)


if __name__ == "__main__":
    unittest.main()
