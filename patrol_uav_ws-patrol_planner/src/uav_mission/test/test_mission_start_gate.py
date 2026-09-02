#!/usr/bin/env python3

import ast
from pathlib import Path
import stat
import tempfile
import unittest
import xml.etree.ElementTree as ET

from uav_mission.mission_start_gate import (
    MissionStartGate,
    MissionStartPolicy,
    R2026_TARGET_CLASSES,
    R2026_TARGET_MODELS,
    StartGateConfig,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "navigation_mission_start_gate.py"
LAUNCH = PACKAGE_ROOT / "launch" / "navigation_mission_start_gate.launch"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


def valid_documents(truth_path):
    classes = ["tent", "pillbox", "bridge", "panzer", "red_cross"]
    models = ["random_%s" % item for item in classes]
    targets = []
    for index, (class_name, model_name) in enumerate(zip(classes, models)):
        targets.append({
            "class": class_name,
            "model": model_name,
            "x": float(index),
            "y": float(index) + 0.1,
            "world_x": float(index) - 0.5,
            "world_y": float(index) - 1.5,
            "yaw": 0.1 * index,
            "footprint_radius": 0.25,
        })
    field = {
        "status": "READY",
        "ready": True,
        "profile": "r2026",
        "seed": 11,
        "allowed_classes": classes,
        "expected_models": models,
        "spawned_models": list(reversed(models)),
        "verified_models": models,
        "truth_path": truth_path,
        "footprint_valid": True,
    }
    truth = {"profile": "r2026", "seed": 11, "targets": targets}
    anchor = {
        "status": "READY",
        "ready": True,
        "profile": "baseline",
        "expected_models": [],
        "spawned_models": [],
        "verified_models": [],
    }
    manager = {
        "phase": "IDLE",
        "profile": "r2026",
        "mission_id": "",
        "manual_start_required": True,
    }
    return field, truth, anchor, manager


class MissionStartPolicyTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.truth_path = str(
            Path(self.temporary.name) / "random_field_truth.yaml")
        self.config = StartGateConfig(
            expected_seed=11,
            expected_truth_path=self.truth_path,
            retry_initial_sec=0.5,
            retry_max_sec=2.0,
        )
        self.field, self.truth, self.anchor, self.manager = (
            valid_documents(self.truth_path))

    def tearDown(self):
        self.temporary.cleanup()

    def evaluate(self):
        return MissionStartPolicy(self.config).evaluate(
            self.field, self.truth, True, self.anchor, self.manager)

    def test_exact_r2026_contract_is_ready(self):
        result = self.evaluate()
        self.assertTrue(result.ready, result.reason)
        self.assertEqual(result.reason, "ready_to_start")
        self.assertEqual(set(self.field["allowed_classes"]),
                         R2026_TARGET_CLASSES)
        self.assertEqual(set(self.field["verified_models"]),
                         R2026_TARGET_MODELS)
        self.assertTrue(all(result.checks.values()))

    def test_field_ready_requires_both_status_and_boolean(self):
        self.field["ready"] = False
        self.assertEqual(self.evaluate().reason, "field_not_ready")
        self.field["ready"] = True
        self.field["status"] = "VERIFYING"
        self.assertEqual(self.evaluate().reason, "field_not_ready")

    def test_field_identity_and_all_five_lists_are_exact(self):
        self.field["seed"] = 12
        self.assertEqual(self.evaluate().reason, "field_identity_mismatch")
        self.field["seed"] = 11.0
        self.assertEqual(self.evaluate().reason, "field_identity_mismatch")
        self.field["seed"] = 11
        self.field["allowed_classes"].append("tank")
        self.assertEqual(
            self.evaluate().reason, "field_target_classes_mismatch")
        self.field["allowed_classes"].pop()
        self.field["verified_models"].pop()
        self.assertEqual(
            self.evaluate().reason, "field_target_models_mismatch")

    def test_truth_must_be_exact_path_durable_and_match_manifest(self):
        self.field["truth_path"] = self.truth_path + ".stale"
        self.assertEqual(self.evaluate().reason, "truth_path_mismatch")
        self.field["truth_path"] = self.truth_path
        result = MissionStartPolicy(self.config).evaluate(
            self.field, self.truth, False, self.anchor, self.manager)
        self.assertEqual(result.reason, "truth_not_durable")
        self.truth["targets"][0]["model"] = "random_tank"
        self.assertEqual(
            self.evaluate().reason, "truth_target_manifest_mismatch")

    def test_truth_requires_positive_finite_footprint_manifest(self):
        self.truth["targets"][0]["footprint_radius"] = 0.0
        self.assertEqual(
            self.evaluate().reason, "truth_footprint_manifest_invalid")
        self.truth["targets"][0]["footprint_radius"] = float("nan")
        self.assertEqual(
            self.evaluate().reason, "truth_footprint_manifest_invalid")

    def test_anchor_profile_and_model_sets_must_match(self):
        self.anchor["profile"] = "a68925d"
        self.assertEqual(self.evaluate().reason, "anchor_profile_mismatch")
        self.anchor["profile"] = "baseline"
        self.anchor["verified_models"] = ["planner_oak_0"]
        self.assertEqual(self.evaluate().reason, "anchor_models_mismatch")

    def test_manager_must_be_matching_idle_manager(self):
        self.manager["profile"] = "full"
        self.assertEqual(self.evaluate().reason, "manager_profile_mismatch")
        self.manager["profile"] = "r2026"
        self.manager["phase"] = "SEARCH"
        self.assertEqual(self.evaluate().reason, "manager_not_idle")

    def test_invalid_json_sentinel_fails_closed(self):
        self.field = {"_decode_error": "JSONDecodeError"}
        result = self.evaluate()
        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "field_status_missing_or_invalid")


class MissionStartStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        truth_path = str(
            Path(self.temporary.name) / "random_field_truth.yaml")
        self.config = StartGateConfig(
            expected_seed=11,
            expected_truth_path=truth_path,
            retry_initial_sec=0.5,
            retry_max_sec=2.0,
        )
        self.field, self.truth, self.anchor, self.manager = (
            valid_documents(truth_path))

    def tearDown(self):
        self.temporary.cleanup()

    def ready_gate(self, enabled=True):
        gate = MissionStartGate(self.config, enabled=enabled)
        gate.update_field(self.field, self.truth, True)
        gate.update_anchor(self.anchor)
        gate.update_manager(self.manager)
        return gate

    def test_disabled_and_pre_ready_gate_never_calls_service(self):
        disabled = self.ready_gate(enabled=False)
        self.assertFalse(disabled.begin_service_call(0.0))
        self.assertEqual(disabled.service_call_count, 0)
        self.assertEqual(disabled.status(0.0)["status"], "DISABLED")

        waiting = MissionStartGate(self.config, enabled=True)
        self.assertFalse(waiting.begin_service_call(0.0))
        self.assertFalse(waiting.record_service_unavailable(
            0.0, "not registered"))
        self.assertEqual(waiting.service_call_count, 0)
        self.assertEqual(waiting.service_unavailable_count, 0)

    def test_success_is_permanently_latched(self):
        gate = self.ready_gate()
        self.assertTrue(gate.begin_service_call(10.0))
        self.assertEqual(gate.service_call_count, 1)
        gate.complete_service_call(10.1, True, "vcl06-1")
        gate.update_manager({"phase": "IDLE", "profile": "r2026"})
        gate.update_field({"status": "FAIL"}, None, False)
        self.assertFalse(gate.begin_service_call(999.0))
        self.assertEqual(gate.service_call_count, 1)
        status = gate.status(999.0)
        self.assertEqual(status["status"], "STARTED")
        self.assertTrue(status["started_latched"])

    def test_manager_active_prevents_calls_without_changing_count(self):
        gate = self.ready_gate()
        gate.update_manager({"phase": "APPROACH", "profile": "r2026"})
        self.assertFalse(gate.begin_service_call(0.0))
        self.assertEqual(gate.service_call_count, 0)
        self.assertEqual(gate.status(0.0)["reason"], "manager_not_idle")

    def test_unavailable_and_rejection_use_capped_backoff(self):
        gate = self.ready_gate()
        self.assertTrue(gate.record_service_unavailable(
            0.0, "service unavailable"))
        self.assertEqual(gate.service_call_count, 0)
        self.assertFalse(gate.begin_service_call(0.49))
        self.assertTrue(gate.begin_service_call(0.5))
        gate.complete_service_call(0.5, False, "pose_missing")
        self.assertFalse(gate.begin_service_call(1.49))
        self.assertTrue(gate.begin_service_call(1.5))
        gate.complete_service_call(1.5, False, "map_missing")
        self.assertFalse(gate.begin_service_call(3.49))
        self.assertTrue(gate.begin_service_call(3.5))
        gate.complete_service_call(3.5, False, "map_stale")
        self.assertAlmostEqual(gate.next_retry_at, 5.5)
        self.assertEqual(gate.service_call_count, 3)
        self.assertEqual(gate.service_unavailable_count, 1)
        self.assertEqual(gate.service_rejection_count, 3)


class NavigationMissionStartGateContractTest(unittest.TestCase):
    def test_ros_shell_does_not_duplicate_pose_or_map_readiness(self):
        source = SCRIPT.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertTrue(SCRIPT.stat().st_mode & stat.S_IXUSR)
        self.assertIn("rospy.wait_for_service", source)
        self.assertIn("latch=True", source)
        self.assertIn("internal_error_fail_closed", source)
        self.assertIn("/mission/random_field_status", source)
        self.assertIn("/mission/planner_anchor_status", source)
        self.assertIn("/navigation/mission_status", source)
        self.assertIn("/navigation/start_mission", source)
        for forbidden in (
                "PoseStamped", "PointCloud2", "/mavros/",
                "/freedom/", "pose_max_age", "map_max_age"):
            self.assertNotIn(forbidden, source)

    def test_include_is_safe_and_does_not_mix_managers(self):
        root = ET.parse(str(LAUNCH)).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")}
        self.assertEqual(arguments["enabled"], "false")
        nodes = root.findall(".//node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(
            nodes[0].attrib["type"],
            "navigation_mission_start_gate.py")
        node_types = {item.attrib["type"] for item in nodes}
        self.assertNotIn("target_search_manager_py.py", node_types)
        self.assertNotIn("navigation_mission_manager.py", node_types)

    def test_catkin_registers_script_and_test(self):
        source = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/navigation_mission_start_gate.py", source)
        self.assertIn(
            "catkin_add_nosetests(test/test_mission_start_gate.py)",
            source)


if __name__ == "__main__":
    unittest.main()
