#!/usr/bin/env python3

import ast
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / \
    "navigation_vcl06_assertion.py"
FORMAL_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / \
    "navigation_search_delivery_vcl06.launch"
GUARDED_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / \
    "toudi3_visual_delivery_guarded.launch"
MAVROS_CONFIG = Path(__file__).resolve().parents[2] / "patrol_control" / \
    "config" / "mavros_px4_sim.yaml"
RUNTIME_CONFIG = Path(__file__).resolve().parents[1] / "config" / \
    "vcl06_random_field_runtime.yaml"
RUNTIME = yaml.safe_load(RUNTIME_CONFIG.read_text(encoding="utf-8"))
CORRIDOR_ROUTE = tuple(
    tuple(point) for point in RUNTIME["mission"]["post_delivery_route"])
CORRIDOR_REVISION = RUNTIME["mission"]["post_delivery_route_revision"]
CORRIDOR_GOAL_TOLERANCE = RUNTIME["post_delivery_gate"]["goal_tolerance"]
CORRIDOR_DOORS = tuple(RUNTIME["post_delivery_gate"]["doors"])
LANDING_XY = tuple(RUNTIME["mission"]["landing_xy"])
LANDING_H_TOLERANCE = RUNTIME["post_delivery_gate"]["final_h_tolerance"]
SPEC = importlib.util.spec_from_file_location(
    "navigation_vcl06_assertion_under_test", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def decision(sequence, command, issued_sec, deadline_sec, slot=0,
             target_id=0, first_seen_ns=0, class_name="", attempt=1,
             has_goal=None, goal=(0.0, 0.0, 0.0), reason=""):
    has_target = command == MODULE.APPROACH
    if has_goal is None:
        has_goal = command in (
            MODULE.SEARCH, MODULE.APPROACH, MODULE.RESUME,
            MODULE.RETURN_HOME)
    return {
        "schema_version": 1,
        "mission_id": "mission-1",
        "decision_seq": sequence,
        "header_seq": sequence,
        "command": command,
        "class_profile": "r2026",
        "has_goal": bool(has_goal),
        "goal_frame": "camera_init",
        "goal_x": float(goal[0]),
        "goal_y": float(goal[1]),
        "goal_z": float(goal[2]),
        "reason": str(reason),
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
        "service_call_count": 3, "service_success_count": 1,
    })
    manager = {
        "phase": "COMPLETE", "mission_failed": False,
        "committed_slots": 3, "mission_id": "mission-1",
    }
    if reducer.post_delivery_route:
        manager.update({
            "post_delivery_route_revision":
                reducer.post_delivery_route_revision,
            "post_delivery_route_index":
                len(reducer.post_delivery_route),
            "post_delivery_route_size":
                len(reducer.post_delivery_route),
            "post_delivery_route_complete": True,
        })
    reducer.observe_status("manager", manager)


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


def build_corridor_reducer(crossing_names=MODULE.EXPECTED_DOOR_ORDER,
                           goal_override=None, h_evidence=True):
    reducer = MODULE.Vcl06GateReducer(
        forced_return_sec=RUNTIME["mission"]["forced_return_at"],
        post_delivery_route=CORRIDOR_ROUTE,
        post_delivery_route_revision=CORRIDOR_REVISION,
        post_delivery_goal_tolerance=CORRIDOR_GOAL_TOLERANCE,
        post_delivery_doors=CORRIDOR_DOORS,
        landing_xy=LANDING_XY,
        landing_h_tolerance=LANDING_H_TOLERANCE,
    )
    ready_statuses(reducer)
    reducer.observe_pose(0.0, 0.0, 2.0, "camera_init")
    approaches = [
        decision(1, MODULE.APPROACH, 10.0, 100.0,
                 1, 11, 101, "tent"),
        decision(2, MODULE.APPROACH, 110.0, 200.0,
                 2, 22, 202, "bridge"),
        decision(3, MODULE.APPROACH, 210.0, 300.0,
                 3, 33, 303, "panzer"),
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

    crossing_samples = {
        "Wall_15": ((-2.386703, 5.90, 1.0),
                    (-2.386703, 6.20, 1.0)),
        "Wall_20": ((-0.30, 8.053133, 1.2),
                    (0.05, 8.053133, 1.2)),
        "Wall_22": ((1.90, 8.009650, 1.2),
                    (2.30, 8.009650, 1.2)),
    }
    crossing_route_index = {"Wall_15": 2, "Wall_20": 6, "Wall_22": 9}
    goal_override = dict(goal_override or {})
    for route_index, configured_goal in enumerate(CORRIDOR_ROUTE, start=1):
        issued = 320.0 + (route_index - 1) * 12.0
        route_decision = decision(
            route_index + 3,
            MODULE.RETURN_HOME,
            issued,
            issued + 60.0,
            goal=goal_override.get(route_index, configured_goal),
            reason="post_delivery_route:%d/%d:%s:test" % (
                route_index, len(CORRIDOR_ROUTE), CORRIDOR_REVISION),
        )
        reducer.observe_decision(route_decision, receipt_wall=issued)
        for door_name, armed_index in crossing_route_index.items():
            if door_name not in crossing_names:
                continue
            if route_index == armed_index - 1:
                reducer.observe_pose(
                    *crossing_samples[door_name][0], "camera_init")
            elif route_index == armed_index:
                reducer.observe_pose(
                    *crossing_samples[door_name][1], "camera_init")
        reducer.observe_result(result(
            event_sequence, route_decision,
            MODULE.SUCCEEDED, MODULE.PLANNER,
            issued + 8.0, terminal=True), receipt_wall=issued + 8.0)
        event_sequence += 1

    land_issued = 460.0
    land = decision(
        len(CORRIDOR_ROUTE) + 4, MODULE.LAND,
        land_issued, land_issued + 90.0,
        has_goal=False, reason="post_delivery_route_complete")
    reducer.observe_decision(land, receipt_wall=land_issued)
    if h_evidence:
        reducer.observe_landing_h_mark(
            LANDING_XY[0], LANDING_XY[1], 0.75,
            "camera_init", int((land_issued + 1.0) * 1e9),
            receipt_wall=land_issued + 1.0, fresh=True)
        reducer.observe_align_mode(
            "landing", receipt_wall=land_issued + 2.0)
        reducer.observe_landed_state(
            MODULE.LANDED_STATE_ON_GROUND,
            receipt_wall=499.0)
    reducer.observe_result(result(
        event_sequence, land, MODULE.SUCCEEDED, MODULE.LANDING,
        500.0, terminal=True), receipt_wall=500.0)
    if h_evidence:
        reducer.observe_vehicle_state(False, receipt_wall=501.0)
    return reducer


class Vcl06GateReducerTest(unittest.TestCase):
    def test_complete_three_slot_chain_passes(self):
        report = build_passing_reducer().report()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["failed_checks"], [])
        self.assertEqual(report["metrics"]["release_commit_count"], 3)
        self.assertEqual(report["metrics"]["approach_command_count"], 3)
        self.assertLessEqual(report["metrics"]["mission_ros_sec"], 600.0)

    def test_complete_corridor_route_and_three_doors_pass(self):
        reducer = build_corridor_reducer()
        report = reducer.report()
        self.assertEqual(report["status"], "PASS", report)
        self.assertEqual(
            report["metrics"]["post_delivery_return_success_count"],
            len(CORRIDOR_ROUTE))
        self.assertEqual(
            [item["name"] for item in
             report["metrics"]["door_crossings"]],
            list(MODULE.EXPECTED_DOOR_ORDER))
        self.assertTrue(report["checks"]["post_delivery_return_sequence"])
        self.assertTrue(report["checks"]["post_delivery_return_goals"])
        self.assertTrue(report["checks"][
            "manager_post_delivery_route_matches"])
        self.assertTrue(report["checks"]["landing_h_mark_valid"])
        self.assertTrue(report["checks"]["landing_align_mode_seen"])
        self.assertTrue(report["checks"]["final_landed_on_ground"])
        self.assertTrue(report["checks"]["final_vehicle_disarmed"])
        self.assertEqual(reducer.forced_return_sec, 420.0)
        route_fence = next(
            item for item in report["decision_fences"]
            if item["reason"].startswith("post_delivery_route:1/"))
        self.assertTrue(route_fence["has_goal"])
        self.assertEqual(route_fence["goal_frame"], "camera_init")
        self.assertEqual(
            (route_fence["goal_x"], route_fence["goal_y"],
             route_fence["goal_z"]),
            CORRIDOR_ROUTE[0])

    def test_complete_manager_waits_for_h_landing_evidence(self):
        reducer = build_corridor_reducer(h_evidence=False)
        report = reducer.report()
        self.assertEqual(report["status"], "WAITING")
        self.assertEqual(report["errors"], [])
        for check in (
                "landing_h_mark_valid", "landing_align_mode_seen",
                "final_landed_on_ground", "final_vehicle_disarmed"):
            self.assertFalse(report["checks"][check])

        timed_out = reducer.report(timeout_reason="mission_wall_timeout")
        self.assertEqual(timed_out["status"], "FAIL")
        self.assertIn("mission_wall_timeout", timed_out["errors"])

    def test_h_landing_evidence_is_layered_and_fail_closed(self):
        reducer = build_corridor_reducer(h_evidence=False)
        land_issued_ns = int(460.0 * 1e9)

        reducer.observe_landing_h_mark(
            LANDING_XY[0], LANDING_XY[1], 0.75,
            "camera_init", land_issued_ns + 1,
            receipt_wall=461.0, fresh=False)
        reducer.observe_landing_h_mark(
            LANDING_XY[0], LANDING_XY[1], 0.75,
            "camera_init", land_issued_ns - 1,
            receipt_wall=462.0, fresh=True)
        reducer.observe_landing_h_mark(
            LANDING_XY[0] + LANDING_H_TOLERANCE + 0.01,
            LANDING_XY[1], 0.75, "camera_init",
            land_issued_ns + 2, receipt_wall=463.0, fresh=True)
        report = reducer.report()
        self.assertFalse(report["checks"]["landing_h_mark_valid"])
        self.assertEqual(
            report["metrics"]["landing_h_mark_rejections"],
            {"stale": 1, "source_before_land_decision": 1,
             "anchor_mismatch": 1})

        reducer.observe_landing_h_mark(
            LANDING_XY[0], LANDING_XY[1], 0.75,
            "camera_init", land_issued_ns + 3,
            receipt_wall=464.0, fresh=True)
        reducer.observe_align_mode("disabled", receipt_wall=465.0)
        reducer.observe_landed_state(2, receipt_wall=499.0)
        reducer.observe_vehicle_state(True, receipt_wall=501.0)
        report = reducer.report()
        self.assertTrue(report["checks"]["landing_h_mark_valid"])
        self.assertFalse(report["checks"]["landing_align_mode_seen"])
        self.assertFalse(report["checks"]["final_landed_on_ground"])
        self.assertFalse(report["checks"]["final_vehicle_disarmed"])

        reducer.observe_align_mode("landing", receipt_wall=502.0)
        reducer.observe_landed_state(
            MODULE.LANDED_STATE_ON_GROUND, receipt_wall=503.0)
        reducer.observe_vehicle_state(False, receipt_wall=504.0)
        self.assertEqual(reducer.report()["status"], "PASS")

    def test_pre_land_state_and_alignment_do_not_count(self):
        reducer = MODULE.Vcl06GateReducer(
            post_delivery_route=CORRIDOR_ROUTE,
            post_delivery_route_revision=CORRIDOR_REVISION,
            post_delivery_goal_tolerance=CORRIDOR_GOAL_TOLERANCE,
            post_delivery_doors=CORRIDOR_DOORS,
            landing_xy=LANDING_XY,
            landing_h_tolerance=LANDING_H_TOLERANCE,
        )
        reducer.observe_align_mode("landing", receipt_wall=1.0)
        reducer.observe_landed_state(
            MODULE.LANDED_STATE_ON_GROUND, receipt_wall=1.0)
        reducer.observe_vehicle_state(False, receipt_wall=1.0)
        reducer.observe_landing_h_mark(
            LANDING_XY[0], LANDING_XY[1], 0.75,
            "camera_init", 1, receipt_wall=1.0, fresh=True)
        self.assertFalse(reducer.landing_align_mode_seen)
        self.assertIsNone(reducer.latest_landed_state)
        self.assertIsNone(reducer.latest_armed)
        self.assertIsNone(reducer.valid_landing_h_mark)
        self.assertEqual(
            reducer.landing_h_mark_rejections,
            {"before_land_decision": 1})

    def test_corridor_gate_rejects_skipped_door(self):
        reducer = build_corridor_reducer(
            crossing_names=("Wall_15", "Wall_20"))
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn(
            "post_delivery_door_sequence_incomplete", report["errors"])
        self.assertFalse(report["checks"]["doors_crossed_in_order"])

    def test_corridor_gate_rejects_out_of_order_door(self):
        reducer = build_corridor_reducer(crossing_names=("Wall_20",))
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("door_crossing_out_of_order:Wall_20",
                      report["errors"])

    def test_corridor_gate_rejects_out_of_order_route_index(self):
        reducer = MODULE.Vcl06GateReducer(
            post_delivery_route=CORRIDOR_ROUTE,
            post_delivery_route_revision=CORRIDOR_REVISION,
            post_delivery_goal_tolerance=CORRIDOR_GOAL_TOLERANCE,
            post_delivery_doors=CORRIDOR_DOORS,
        )
        reducer.observe_decision(decision(
            1, MODULE.RETURN_HOME, 10.0, 70.0,
            goal=CORRIDOR_ROUTE[1],
            reason="post_delivery_route:2/%d:%s:test" % (
                len(CORRIDOR_ROUTE), CORRIDOR_REVISION)))
        self.assertIn("post_delivery_route_sequence_invalid",
                      reducer.errors)

    def test_corridor_gate_rejects_reverse_crossing(self):
        reducer = MODULE.Vcl06GateReducer(
            post_delivery_route=CORRIDOR_ROUTE,
            post_delivery_route_revision=CORRIDOR_REVISION,
            post_delivery_goal_tolerance=CORRIDOR_GOAL_TOLERANCE,
            post_delivery_doors=CORRIDOR_DOORS,
        )
        reducer.observe_decision(decision(
            1, MODULE.RETURN_HOME, 10.0, 70.0,
            goal=CORRIDOR_ROUTE[0],
            reason="post_delivery_route:1/%d:%s:test" % (
                len(CORRIDOR_ROUTE), CORRIDOR_REVISION)))
        reducer.observe_pose(-2.386703, 6.2, 1.0, "camera_init")
        reducer.observe_decision(decision(
            2, MODULE.RETURN_HOME, 20.0, 80.0,
            goal=CORRIDOR_ROUTE[1],
            reason="post_delivery_route:2/%d:%s:test" % (
                len(CORRIDOR_ROUTE), CORRIDOR_REVISION)))
        reducer.observe_pose(-2.386703, 5.9, 1.0, "camera_init")
        self.assertIn("door_direction_invalid:Wall_15", reducer.errors)

    def test_corridor_gate_rejects_crossing_outside_opening(self):
        cases = (
            ("lateral", (-3.0, 5.9, 1.0), (-3.0, 6.2, 1.0),
             "door_lateral_out_of_bounds:Wall_15"),
            ("height", (-2.386703, 5.9, 1.4),
             (-2.386703, 6.2, 1.4),
             "door_height_out_of_bounds:Wall_15"),
        )
        for label, before, after, expected_error in cases:
            with self.subTest(label=label):
                reducer = MODULE.Vcl06GateReducer(
                    post_delivery_route=CORRIDOR_ROUTE,
                    post_delivery_route_revision=CORRIDOR_REVISION,
                    post_delivery_goal_tolerance=CORRIDOR_GOAL_TOLERANCE,
                    post_delivery_doors=CORRIDOR_DOORS,
                )
                route_decision = decision(
                    1, MODULE.RETURN_HOME, 10.0, 70.0,
                    goal=CORRIDOR_ROUTE[1],
                    reason="post_delivery_route:2/%d:%s:test" % (
                        len(CORRIDOR_ROUTE), CORRIDOR_REVISION))
                reducer.observe_decision(route_decision)
                reducer.observe_pose(*before, "camera_init")
                reducer.observe_pose(*after, "camera_init")
                self.assertIn(expected_error, reducer.errors)

    def test_corridor_gate_rejects_wrong_route_goal(self):
        wrong_goal = list(CORRIDOR_ROUTE[5])
        wrong_goal[0] += CORRIDOR_GOAL_TOLERANCE + 0.01
        reducer = build_corridor_reducer(
            goal_override={6: tuple(wrong_goal)})
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("post_delivery_route_goal_mismatch",
                      report["errors"])
        self.assertFalse(report["checks"]["post_delivery_return_goals"])

    def test_corridor_gate_rejects_manager_route_mismatch(self):
        reducer = build_corridor_reducer()
        manager = dict(reducer.statuses["manager"])
        manager["post_delivery_route_index"] -= 1
        reducer.observe_status("manager", manager)
        report = reducer.report()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("manager_post_delivery_route_mismatch",
                      report["errors"])
        self.assertFalse(report["checks"][
            "manager_post_delivery_route_matches"])

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
        report = reducer.report(timeout_reason="startup_wall_timeout")
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("startup_wall_timeout", report["errors"])
        self.assertIn("required_statuses_seen", report["failed_checks"])
        for check in (
                "three_capture_started", "real_approach_commands",
                "committed_targets_were_selected"):
            self.assertFalse(report["checks"][check])

        report = reducer.report(timeout_reason="mission_wall_timeout")
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("mission_wall_timeout", report["errors"])

    def test_start_gate_counts_one_success_not_one_attempt(self):
        reducer = build_passing_reducer()
        report = reducer.report()
        self.assertTrue(report["checks"]["start_gate_started_once"])

        reducer.statuses["start_gate"]["service_success_count"] = 2
        report = reducer.report()
        self.assertFalse(report["checks"]["start_gate_started_once"])

    def test_ros_shell_uses_separate_startup_and_mission_timeouts(self):
        node = MODULE.NavigationVcl06AssertionNode.__new__(
            MODULE.NavigationVcl06AssertionNode)
        node._assertion_started_wall = 100.0
        node._mission_started_wall = None
        node._startup_wall_timeout = 180.0
        node._wall_timeout = 900.0

        self.assertEqual(node._timeout_reason(279.999), "")
        self.assertEqual(node._timeout_reason(280.0),
                         "startup_wall_timeout")

        node._latch_mission_start(250.0)
        self.assertEqual(node._timeout_reason(1149.999), "")
        self.assertEqual(node._timeout_reason(1150.0),
                         "mission_wall_timeout")
        node._latch_mission_start(500.0)
        self.assertEqual(node._mission_started_wall, 250.0)

    def test_ros_shell_latches_start_from_started_or_first_decision(self):
        class Recorder:
            def __init__(self):
                self.statuses = []
                self.decisions = []

            def observe_status(self, name, payload):
                self.statuses.append((name, payload))

            def observe_decision(self, payload, receipt_wall):
                self.decisions.append((payload, receipt_wall))

            @staticmethod
            def _error(_reason):
                raise AssertionError("unexpected JSON error")

        node = MODULE.NavigationVcl06AssertionNode.__new__(
            MODULE.NavigationVcl06AssertionNode)
        node._lock = MODULE.threading.RLock()
        node._mission_started_wall = None
        node.reducer = Recorder()
        node._check_terminal = lambda **_kwargs: None

        with mock.patch.object(MODULE.time, "monotonic",
                               return_value=42.0):
            node._status_callback("start_gate")(SimpleNamespace(
                data=json.dumps({"status": "STARTED"})))
        self.assertEqual(node._mission_started_wall, 42.0)

        node._mission_started_wall = None
        node._decision_dict = lambda _message: {"decision_seq": 1}
        with mock.patch.object(MODULE.time, "monotonic",
                               return_value=43.0):
            node._on_decision(SimpleNamespace())
        self.assertEqual(node._mission_started_wall, 43.0)
        self.assertEqual(node.reducer.decisions,
                         [({"decision_seq": 1}, 43.0)])

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

    def test_ros_shell_preserves_decision_goal_and_reason(self):
        stamp = SimpleNamespace(secs=20, nsecs=30)
        message = SimpleNamespace(
            schema_version=1,
            mission_id="mission-1",
            decision_seq=7,
            header=SimpleNamespace(seq=7, stamp=stamp),
            deadline=SimpleNamespace(secs=80, nsecs=0),
            command=MODULE.RETURN_HOME,
            class_profile="r2026",
            has_goal=True,
            goal=SimpleNamespace(
                header=SimpleNamespace(frame_id="camera_init"),
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=1.0, y=2.0, z=3.0))),
            reason="post_delivery_route:1/11:revision:test",
            has_target=False,
            target_id=0,
            target_first_seen=SimpleNamespace(secs=0, nsecs=0),
            target_class="",
            attempt=0,
            payload_slot=0,
        )
        payload = MODULE.NavigationVcl06AssertionNode._decision_dict(message)
        self.assertTrue(payload["has_goal"])
        self.assertEqual(payload["goal_frame"], "camera_init")
        self.assertEqual(
            (payload["goal_x"], payload["goal_y"], payload["goal_z"]),
            (1.0, 2.0, 3.0))
        self.assertEqual(
            payload["reason"],
            "post_delivery_route:1/11:revision:test")

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
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in root.findall("arg")}
        self.assertEqual(args["gate_wall_timeout"], "2700.0")
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
        private_loads = [
            item for item in gate.findall("rosparam")
            if item.attrib.get("command") == "load"]
        self.assertEqual(len(private_loads), 1)
        self.assertEqual(private_loads[0].attrib.get("file"),
                         "$(arg runtime_config)")
        self.assertEqual(params["startup_wall_timeout"],
                         "$(arg gate_startup_wall_timeout)")
        self.assertEqual(params["wall_timeout"],
                         "$(arg gate_wall_timeout)")
        self.assertEqual(params["planner_goal_topic"], "/fastplanner/goal")
        self.assertEqual(params["expected_planner_goal_publisher"],
                         "/navigation/planner_bridge")
        self.assertNotIn("forced_return_sec", params)
        self.assertEqual(params["landing_mark_topic"],
                         "/detect/land_mark_point")
        self.assertEqual(params["landing_mark_max_age"], "0.5")
        self.assertEqual(params["align_mode_topic"],
                         "/uav_vision/align_mode")
        self.assertEqual(params["extended_state_topic"],
                         "/mavros/extended_state")
        self.assertEqual(params["vehicle_state_topic"], "/mavros/state")
        for evidence_topic in (
                "/detect/land_mark_point", "/uav_vision/align_mode",
                "/mavros/extended_state", "/mavros/state",
                "/fastplanner/setpoint_position/local",
                "/mavros/setpoint_position/local"):
            self.assertIn(evidence_topic, source)
        arguments = {item.attrib["name"]: item.attrib.get("default")
                     for item in root.findall("arg")}
        self.assertEqual(arguments["gate_startup_wall_timeout"], "180.0")
        self.assertEqual(arguments["external_planner_max_command_z"],
                         "3.5")
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
        self.assertEqual(
            guarded_args["external_planner_max_command_z"],
            "$(arg external_planner_max_command_z)")
        self.assertEqual(params["mission_frame"], "camera_init")

        guarded_root = ET.parse(str(GUARDED_LAUNCH)).getroot()
        guarded_defaults = {
            item.attrib["name"]: item.attrib.get("default")
            for item in guarded_root.findall("arg")}
        self.assertEqual(
            guarded_defaults["external_planner_max_command_z"], "3.5")
        nested = next(
            item for item in guarded_root.findall("include")
            if "toudi3_full_competition_sim_new_vision.launch" in
            item.attrib.get("file", ""))
        nested_args = {item.attrib["name"]: item.attrib.get("value")
                       for item in nested.findall("arg")}
        self.assertEqual(
            nested_args["external_planner_max_command_z"],
            "$(arg external_planner_max_command_z)")

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
