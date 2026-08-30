#!/usr/bin/env python3

import ast
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / \
    "navigation_vcl06_assertion.py"
FORMAL_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / \
    "navigation_search_delivery_vcl06.launch"
MAVROS_CONFIG = Path(__file__).resolve().parents[2] / "patrol_control" / \
    "config" / "mavros_px4_sim.yaml"
SPEC = importlib.util.spec_from_file_location(
    "navigation_vcl06_assertion_under_test", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def decision(sequence, command, issued_sec, deadline_sec, slot=0,
             target_id=0, first_seen_ns=0, class_name="", attempt=1):
    has_target = command == MODULE.APPROACH
    return {
        "schema_version": 1,
        "mission_id": "mission-1",
        "decision_seq": sequence,
        "header_seq": sequence,
        "command": command,
        "class_profile": "r2026",
        "has_target": has_target,
        "target_id": target_id if has_target else 0,
        "target_first_seen_ns": first_seen_ns if has_target else 0,
        "target_class": class_name if has_target else "",
        "attempt": attempt if has_target else 0,
        "payload_slot": slot if has_target else 0,
        "issued_ns": int(issued_sec * 1e9),
        "deadline_ns": int(deadline_sec * 1e9),
    }


def result(event_sequence, item, status, stage, stamp_sec, terminal=False,
           payload_committed=False, retryable=False, reason="ok",
           evidence_source="executor"):
    return {
        "schema_version": 1,
        "header_seq": event_sequence,
        "mission_id": item["mission_id"],
        "executor_id": "executor-1",
        "event_seq": event_sequence,
        "decision_seq": item["decision_seq"],
        "command": item["command"],
        "status": status,
        "stage": stage,
        "terminal": terminal,
        "retryable": retryable,
        "payload_committed": payload_committed,
        "has_target": item["has_target"],
        "target_id": item["target_id"],
        "target_first_seen_ns": item["target_first_seen_ns"],
        "target_class": item["target_class"],
        "attempt": item["attempt"],
        "payload_slot": item["payload_slot"],
        "reason": reason,
        "evidence_source": evidence_source,
        "stamp_ns": int(stamp_sec * 1e9),
    }


def ready_statuses(reducer):
    reducer.observe_planner_goal_publishers([
        "/navigation/planner_bridge"])
    reducer.observe_status("field", {
        "status": "READY", "ready": True, "profile": "r2026",
        "seed": 11, "footprint_valid": True,
    })
    reducer.observe_status("anchor", {
        "status": "READY", "ready": True, "profile": "baseline",
    })
    reducer.observe_status("contact", {
        "status": "READY", "ready": True, "actual_collision_count": 0,
    })
    reducer.observe_status("bridge", {
        "adapter_faulted": False,
        "output_enabled": True,
        "planner_goal_topic": "/fastplanner/goal",
        "gate_reason": "live_planner_output_enabled",
    })
    reducer.observe_status("start_gate", {
        "status": "STARTED", "started_latched": True,
        "service_call_count": 1,
    })
    reducer.observe_status("manager", {
        "phase": "COMPLETE", "mission_failed": False,
        "committed_slots": 3, "mission_id": "mission-1",
    })


def build_passing_reducer():
    reducer = MODULE.Vcl06GateReducer()
    ready_statuses(reducer)
    reducer.observe_pose(0.0, 0.0, 2.0, "camera_init")
    approaches = [
        decision(1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent"),
        decision(2, MODULE.APPROACH, 110.0, 200.0, 2, 22, 202, "bridge"),
        decision(3, MODULE.APPROACH, 210.0, 300.0, 3, 33, 303, "panzer"),
    ]
    event_sequence = 1
    for index, item in enumerate(approaches):
        receipt = 100.0 + index * 100.0
        reducer.observe_decision(item, receipt_wall=receipt)
        reducer.observe_selected(
            item["target_class"], item["target_id"],
            item["target_first_seen_ns"])
        reducer.observe_mission_command(
            MODULE.APPROACH, item["decision_seq"], item["target_id"],
            item["target_class"], item["issued_ns"] + 1)
        reducer.observe_result(result(
            event_sequence, item, MODULE.STARTED, MODULE.CAPTURE,
            12.0 + index * 100.0), receipt_wall=receipt + 1.0)
        event_sequence += 1
        reducer.observe_result(result(
            event_sequence, item, MODULE.PROGRESS, MODULE.RELEASE,
            13.0 + index * 100.0, payload_committed=True,
            reason="release_ack_success", evidence_source="mock_ack"),
            receipt_wall=receipt + 2.0)
        event_sequence += 1
        reducer.observe_result(result(
            event_sequence, item, MODULE.SUCCEEDED, MODULE.RECOVERY,
            14.0 + index * 100.0, terminal=True),
            receipt_wall=receipt + 3.0)
        event_sequence += 1

    return_home = decision(4, MODULE.RETURN_HOME, 400.0, 470.0)
    reducer.observe_decision(return_home, receipt_wall=400.0)
    reducer.observe_result(result(
        event_sequence, return_home, MODULE.SUCCEEDED, MODULE.PLANNER,
        430.0, terminal=True), receipt_wall=430.0)
    event_sequence += 1
    land = decision(5, MODULE.LAND, 450.0, 590.0)
    reducer.observe_decision(land, receipt_wall=450.0)
    reducer.observe_result(result(
        event_sequence, land, MODULE.SUCCEEDED, MODULE.LANDING,
        500.0, terminal=True), receipt_wall=500.0)
    return reducer


class Vcl06GateReducerTest(unittest.TestCase):
    def test_complete_three_slot_chain_passes(self):
        report = build_passing_reducer().report()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["failed_checks"], [])
        self.assertEqual(report["metrics"]["release_commit_count"], 3)
        self.assertEqual(report["metrics"]["approach_command_count"], 3)
        self.assertLessEqual(report["metrics"]["mission_ros_sec"], 600.0)

    def test_result_requires_complete_decision_fence(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        reducer.observe_decision(item)
        mismatched = result(
            1, item, MODULE.PROGRESS, MODULE.RELEASE, 20.0,
            payload_committed=True, reason="release_ack_success")
        mismatched["target_first_seen_ns"] += 1
        reducer.observe_result(mismatched)
        self.assertIn("result_decision_fence_mismatch", reducer.errors)
        self.assertEqual(reducer.release_commits, set())

    def test_cross_topic_result_can_arrive_before_decision(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        event = result(1, item, MODULE.STARTED, MODULE.CAPTURE, 20.0)
        reducer.observe_result(event)
        self.assertEqual(len(reducer.unmatched_results), 1)
        self.assertNotIn("result_without_decision", reducer.errors)
        reducer.observe_decision(item)
        self.assertEqual(reducer.unmatched_results, {})
        self.assertEqual(len(reducer.results), 1)
        self.assertIn(("mission-1", 1), reducer.capture_started)

    def test_conflicting_or_out_of_order_result_is_rejected(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        reducer.observe_decision(item)
        first = result(2, item, MODULE.STARTED, MODULE.CAPTURE, 20.0)
        reducer.observe_result(first)
        conflict = dict(first)
        conflict["reason"] = "different"
        reducer.observe_result(conflict)
        reducer.observe_result(result(
            1, item, MODULE.PROGRESS, MODULE.RELEASE, 21.0,
            payload_committed=True, reason="release_ack_success"))
        self.assertIn("result_identity_conflict", reducer.errors)
        self.assertIn("result_event_sequence_not_monotonic", reducer.errors)

    def test_late_payload_commit_is_recorded_but_hard_fails(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        reducer.observe_decision(item)
        reducer.observe_result(result(
            1, item, MODULE.STARTED, MODULE.CAPTURE, 20.0))
        reducer.observe_result(result(
            2, item, MODULE.PROGRESS, MODULE.RELEASE, 100.0,
            payload_committed=True, reason="release_ack_success"))
        self.assertIn("successful_result_at_or_after_deadline",
                      reducer.errors)
        self.assertIn("late_payload_commit", reducer.errors)
        self.assertEqual(reducer.release_commits, {("mission-1", 1)})
        self.assertEqual(len(reducer.results), 2)
        self.assertEqual(reducer.report()["status"], "FAIL")

    def test_tank_selected_or_accepted_fails(self):
        reducer = MODULE.Vcl06GateReducer()
        reducer.observe_selected("tank", 5, 50)
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 5, 50, "tank")
        reducer.observe_decision(item)
        reducer.observe_result(result(
            1, item, MODULE.ACCEPTED, MODULE.DISPATCH, 20.0))
        self.assertIn("tank_selected", reducer.errors)
        self.assertIn("tank_accepted", reducer.errors)
        self.assertEqual(reducer.report()["status"], "FAIL")

    def test_collision_boundary_height_and_required_status_fail_closed(self):
        reducer = MODULE.Vcl06GateReducer()
        reducer.observe_pose(10.0, 0.0, 4.1, "camera_init")
        reducer.observe_status("contact", {
            "status": "READY", "ready": True,
            "actual_collision_count": 1,
        })
        reducer.observe_status("bridge", {
            "output_enabled": False, "adapter_faulted": False,
        })
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        for reason in ("field_boundary_violation", "height_limit_violation",
                       "actual_collision", "bridge_output_disabled"):
            self.assertIn(reason, report["errors"])

    def test_planner_goal_requires_one_expected_publisher(self):
        reducer = MODULE.Vcl06GateReducer()
        reducer.observe_planner_goal_publishers([
            "/navigation/planner_bridge", "/legacy/adapter"])
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("planner_goal_publisher_set_invalid", report["errors"])
        self.assertFalse(report["checks"][
            "single_planner_goal_publisher"])

    def test_timeout_writes_explicit_failure_reason(self):
        reducer = MODULE.Vcl06GateReducer()
        report = reducer.report(timed_out=True)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("wall_timeout", report["errors"])
        self.assertIn("required_statuses_seen", report["failed_checks"])
        for check in (
                "three_capture_started", "real_approach_commands",
                "committed_targets_were_selected"):
            self.assertFalse(report["checks"][check])

    def test_retry_approach_command_binds_by_decision_sequence(self):
        reducer = MODULE.Vcl06GateReducer()
        first = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        second = decision(
            2, MODULE.APPROACH, 20.0, 100.0, 1, 11, 101, "tent",
            attempt=2)
        reducer.observe_decision(first)
        reducer.observe_decision(second)
        reducer.observe_mission_command(
            MODULE.APPROACH, 2, 11, "tent", int(30e9))
        self.assertNotIn("mission_command_approach_ambiguous",
                         reducer.errors)
        self.assertEqual(reducer._bound_command_keys(), {("mission-1", 2)})

    def test_approach_command_validates_target_class_and_stamp(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        reducer.observe_decision(item)
        reducer.observe_mission_command(
            MODULE.APPROACH, 1, 12, "tent", int(20e9))
        reducer.observe_mission_command(
            MODULE.APPROACH, 1, 11, "bridge", int(20e9))
        reducer.observe_mission_command(
            MODULE.APPROACH, 1, 11, "tent", int(100e9))
        self.assertIn("mission_command_approach_fence_mismatch",
                      reducer.errors)
        self.assertIn("mission_command_approach_stamp_out_of_range",
                      reducer.errors)
        self.assertEqual(reducer._bound_command_keys(), set())

    def test_ros_shell_forwards_mission_command_header_sequence(self):
        calls = []

        class Recorder:
            @staticmethod
            def observe_mission_command(*args):
                calls.append(args)

        node = MODULE.NavigationVcl06AssertionNode.__new__(
            MODULE.NavigationVcl06AssertionNode)
        node._lock = MODULE.threading.RLock()
        node.reducer = Recorder()
        node._check_terminal = lambda: None
        message = SimpleNamespace(
            command=MODULE.APPROACH,
            target_id=11,
            target_class="tent",
            header=SimpleNamespace(
                seq=7, stamp=SimpleNamespace(secs=20, nsecs=30)))
        node._on_mission_command(message)
        self.assertEqual(calls, [(
            MODULE.APPROACH, 7, 11, "tent", 20_000_000_030)])

    def test_release_and_recovery_must_follow_capture_order(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(
            1, MODULE.APPROACH, 10.0, 100.0, 1, 11, 101, "tent")
        reducer.observe_decision(item)
        reducer.observe_result(result(
            1, item, MODULE.PROGRESS, MODULE.RELEASE, 20.0,
            payload_committed=True, reason="release_ack_success"))
        reducer.observe_result(result(
            2, item, MODULE.SUCCEEDED, MODULE.RECOVERY, 21.0,
            terminal=True))
        self.assertIn("release_before_capture", reducer.errors)

        second = MODULE.Vcl06GateReducer()
        second.observe_decision(item)
        second.observe_result(result(
            1, item, MODULE.STARTED, MODULE.CAPTURE, 20.0))
        second.observe_result(result(
            2, item, MODULE.SUCCEEDED, MODULE.RECOVERY, 21.0,
            terminal=True))
        self.assertIn("recovery_before_release", second.errors)

    def test_return_home_success_requires_planner_stage(self):
        reducer = MODULE.Vcl06GateReducer()
        item = decision(1, MODULE.RETURN_HOME, 10.0, 100.0)
        reducer.observe_decision(item)
        reducer.observe_result(result(
            1, item, MODULE.SUCCEEDED, MODULE.RECOVERY, 20.0,
            terminal=True))
        self.assertIn("return_home_success_stage_invalid", reducer.errors)
        self.assertEqual(reducer.return_success, set())

    def test_script_has_no_control_publisher_or_policy_engine(self):
        source = SCRIPT.read_text(encoding="utf-8")
        tree = ast.parse(source)
        publisher_calls = [
            node for node in ast.walk(tree)
            if (isinstance(node, ast.Call) and
                isinstance(node.func, ast.Attribute) and
                node.func.attr == "Publisher")]
        self.assertEqual(publisher_calls, [])
        for forbidden in (
                "coverage_search_manager", "target_search_manager_py",
                "navigation_visual_delivery_adapter", "ServiceProxy",
                "rospy.Service(", "actuator_pwm", "Servo"):
            self.assertNotIn(forbidden, source)

    def test_formal_launch_has_one_policy_and_execution_chain(self):
        root = ET.parse(str(FORMAL_LAUNCH)).getroot()
        source = FORMAL_LAUNCH.read_text(encoding="utf-8")
        includes = [item.attrib.get("file", "")
                    for item in root.findall("include")]
        self.assertEqual(sum(
            "navigation_mission_manager.launch" in item
            for item in includes), 1)
        self.assertEqual(sum(
            "navigation_planner_bridge.launch" in item
            for item in includes), 1)
        for forbidden in (
                "coverage_search_manager", "target_search_manager_py",
                "navigation_visual_delivery_adapter",
                "profile_candidate_selector"):
            self.assertNotIn(forbidden, source)
        nodes = {item.attrib.get("name"): item
                 for item in root.findall("node")}
        gate = nodes["navigation_vcl06_assertion"]
        self.assertEqual(gate.attrib.get("required"), "true")
        self.assertEqual(gate.attrib.get("if"), "$(arg start_hard_gate)")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in gate.findall("param")}
        self.assertEqual(params["planner_goal_topic"], "/fastplanner/goal")
        self.assertEqual(params["expected_planner_goal_publisher"],
                         "/navigation/planner_bridge")
        arguments = {item.attrib["name"]: item.attrib.get("default")
                     for item in root.findall("arg")}
        self.assertNotIn("mission_frame", arguments)
        self.assertEqual(arguments["class_profile"], "r2026")
        self.assertEqual(arguments["field_seed"], "11")
        self.assertEqual(arguments["standard_classes"],
                         "tent,pillbox,bridge,panzer")
        guarded = next(item for item in root.findall("include")
                       if "toudi3_visual_delivery_guarded.launch" in
                       item.attrib.get("file", ""))
        guarded_args = {item.attrib["name"]: item.attrib.get("value")
                        for item in guarded.findall("arg")}
        self.assertEqual(guarded_args["map_frame"], "camera_init")
        self.assertEqual(params["mission_frame"], "camera_init")

    def test_sitl_pose_and_setpoint_use_the_mission_frame(self):
        config = yaml.safe_load(MAVROS_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(config["local_position"]["frame_id"],
                         "camera_init")
        self.assertEqual(config["local_position"]["tf"]["frame_id"],
                         "camera_init")
        self.assertEqual(config["setpoint_position"]["tf"]["frame_id"],
                         "camera_init")


if __name__ == "__main__":
    unittest.main()
