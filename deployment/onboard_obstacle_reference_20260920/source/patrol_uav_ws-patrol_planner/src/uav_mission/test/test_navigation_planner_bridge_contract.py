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
MISSION_LAUNCH = (
    PACKAGE_ROOT / "launch" / "navigation_search_delivery_vcl06.launch")
CONFIG = PACKAGE_ROOT / "config" / "vcl06_planner_bridge.yaml"
FORMAL_RUNTIME = PACKAGE_ROOT / "config" / "vcl06_random_field_runtime.yaml"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


class NavigationPlannerBridgeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCRIPT.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.formal_runtime = yaml.safe_load(
            FORMAL_RUNTIME.read_text(encoding="utf-8"))
        cls.launch = ET.parse(str(LAUNCH)).getroot()
        cls.mission_launch = ET.parse(str(MISSION_LAUNCH)).getroot()
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
            publisher_types,
            {"PoseStamped", "NavigationResult", "String",
             "MissionCommand", "AlignmentTargetContext"})
        for forbidden in (
                "Servo", "actuator_pwm", "rospy.ServiceProxy",
                "/planning/replan", "/mavros/cmd/arming"):
            self.assertNotIn(forbidden, self.source)
        self.assertIn("ExtendedState", self.source)
        self.assertNotIn("from mavros_msgs.msg import State", self.source)

    def test_live_planner_topic_requires_explicit_acknowledgement(self):
        self.assertIn(
            'LIVE_PLANNER_GOAL_TOPIC = "/fastplanner/goal"', self.source)
        self.assertIn(
            'return False, "live_goal_output_not_acknowledged"',
            self.source)
        self.assertIn('return True, "live_planner_output_enabled"', self.source)
        self.assertIn('"live_goal_output_supported": True', self.source)
        self.assertIn("outcome.handoff", self.source)
        self.assertNotIn("DIAGNOSTIC_ONLY_INTENTS", self.source)

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
                "message.decision_seq",
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
        decision_parser = self.source[
            self.source.index("def _decision_from_message"):
            self.source.index("def _sequenced_goal_from_status")]
        self.assertNotIn("message.header.seq", decision_parser)

    def test_planner_event_sequence_is_not_coupled_to_transport_header(self):
        parser = self.source[
            self.source.index("def _planner_status_from_message"):
            self.source.index("def _odom_from_message")]
        self.assertIn("event_seq = int(message.event_seq)", parser)
        self.assertIn("event_seq=event_seq", parser)
        self.assertNotIn("message.header.seq", parser)
        self.assertNotIn("planner event header sequence mismatch", self.source)

    def test_motion_limits_do_not_duplicate_profile_policy(self):
        execution = self.config["execution"]
        self.assertFalse(execution["enabled"])
        self.assertFalse(execution["allow_live_goal_output"])
        self.assertEqual(execution["mission_frame"], "camera_init")
        self.assertEqual(execution["max_goal_z"], 2.5)
        self.assertNotIn("class_profile", execution)
        self.assertNotIn("payload_slots", execution)
        self.assertNotIn("allowed_target_classes", execution)
        self.assertEqual(execution["effective_goal_max_offset"], 0.35)
        self.assertEqual(execution["planner_accept_timeout"], 5.0)
        self.assertEqual(execution["arrival_position_tolerance"], 0.18)
        self.assertEqual(
            execution["arrival_position_tolerance"],
            self.formal_runtime["post_delivery_gate"]["goal_tolerance"],
        )
        self.assertEqual(
            execution["approach_arrival_position_tolerance"], 0.35)

    def test_planner_generation_uses_preserved_stamp_not_transport_sequence(self):
        parser = self.source[
            self.source.index("def _planner_status_from_message"):
            self.source.index("def _odom_from_message")]
        self.assertIn("transport_goal_seq = int(message.goal_seq)", parser)
        self.assertIn("resolve_goal_seq_by_stamp", parser)
        self.assertIn("requested_stamp_ns", parser)
        self.assertIn("foreign_planner_goal_stamp_ignored", self.source)

    def test_launch_defaults_to_disabled_isolated_output(self):
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.launch.findall("arg")
        }
        self.assertEqual(arguments["execution_enabled"], "false")
        self.assertEqual(arguments["allow_live_goal_output"], "false")
        self.assertEqual(
            arguments["planner_goal_topic"],
            "/navigation/fastplanner_goal",
        )
        self.assertNotEqual(
            arguments["planner_goal_topic"], "/fastplanner/goal")

    def test_ros_dependencies_are_declared(self):
        dependencies = {
            item.text for kind in (
                "depend", "build_depend", "build_export_depend",
                "exec_depend")
            for item in self.package.findall(kind)
        }
        self.assertIn("nav_msgs", dependencies)
        self.assertIn("mavros_msgs", dependencies)
        self.assertIn("plan_manage", dependencies)

    def test_all_callbacks_fail_closed(self):
        self.assertIn("threading.RLock()", self.source)
        self.assertIn("def _handle_callback_exception", self.source)
        for source in (
                "decision", "planner_status", "odom", "targets",
                "release_context", "release_result", "control_state",
                "align_mode", "landed_state", "timer"):
            self.assertIn(
                '_handle_callback_exception("%s"' % source,
                self.source,
            )

    def test_malformed_input_recovers_and_orientation_is_preserved(self):
        handler = self.source[
            self.source.index("def _handle_callback_exception"):
            self.source.index("def _on_decision")]
        self.assertIn("isinstance(error, ValueError)", handler)
        self.assertIn("ignored malformed", handler)
        self.assertLess(handler.index("return"),
                        handler.index("self._adapter_faulted = True"))
        self.assertNotIn("identity_orientation", self.source)
        for component in ("decision.goal.qx", "decision.goal.qy",
                          "decision.goal.qz", "decision.goal.qw"):
            self.assertIn(component, self.source)

    def test_target_transaction_reuses_existing_contracts(self):
        for token in (
                "TargetCandidateArray", "AlignmentTargetContext",
                "ReleaseEvidenceContext", "ReleaseResult", "MissionCommand",
                "strict_context_source", "report_target_stage"):
            self.assertIn(token, self.source)
        self.assertNotIn("TargetTransactionResult", self.source)
        self.assertNotIn("_target_event_seq", self.source)
        self.assertIn("point = candidate.map_point", self.source)
        self.assertIn("target_pose=target_pose", self.source)
        self.assertNotIn("rospy.ServiceProxy", self.source)

    def test_handoffs_require_observed_control_and_deadlines(self):
        self.assertIn('transaction.phase = "ALIGN_COMMAND_SENT"', self.source)
        self.assertIn('transaction.phase == "ALIGN_COMMAND_SENT"', self.source)
        self.assertIn('self._control_state == 2', self.source)
        self.assertIn("def _expire_handoff_if_due", self.source)
        self.assertIn('reason = "release_result_deadline_reached"',
                      self.source)
        self.assertIn('reason = "recovery_deadline_reached"', self.source)
        self.assertIn('"landing_deadline_reached"', self.source)
        self.assertIn("def _mark_alignment_started", self.source)
        self.assertIn("alignment_accepted_before_release_ack", self.source)

    def test_terminal_state_confirmation_uses_position_dwell(self):
        target = self.config["target"]
        landing = self.config["landing"]
        self.assertEqual(target["recovery_settle_radius"], 0.15)
        self.assertEqual(landing["settle_radius"], 0.15)
        self.assertNotIn("recovery_speed", target)
        self.assertNotIn("speed", landing)
        self.assertIn("PositionSettleWindow", self.source)
        self.assertNotIn("self._recovery_speed", self.source)
        self.assertNotIn("self._landing_speed", self.source)
        self.assertIn('"control_state_predates_release"', self.source)
        self.assertIn('"landed_state_predates_land_command"', self.source)
        self.assertIn('"recovery_settle"', self.source)
        self.assertIn('"landing_settle"', self.source)
        self.assertNotIn(
            "now_ns - self._control_state_receipt_ns", self.source)
        self.assertNotIn(
            "now_ns - self._align_mode_receipt_ns", self.source)
        self.assertNotIn(
            "now_ns - self._landed_state_receipt_ns", self.source)

    def test_debug_recording_includes_odom_used_by_terminal_gates(self):
        recorder = next(
            node for node in self.mission_launch.findall("node")
            if node.attrib.get("name") == "navigation_vcl06_debug_recorder")
        self.assertIn(
            "/mavros/local_position/odom", recorder.attrib["args"])
        self.assertIn("/detect/point_class", recorder.attrib["args"])

    def test_abort_reuses_planner_goal_and_hold_is_not_advertised(self):
        self.assertIn('elif command == "ABORT"', self.source)
        self.assertIn('raise ValueError("HOLD is not supported', self.source)
        self.assertNotIn('"ABORT": MissionCommand.', self.source)

    def test_land_uses_current_pose_and_explicit_landed_fact(self):
        self.assertIn("sample = self._last_odom", self.source)
        self.assertIn("x=sample.x", self.source)
        self.assertIn("y=sample.y", self.source)
        self.assertIn('z=0.0', self.source)
        self.assertIn("horizontal_error > self._landing_radius", self.source)
        self.assertIn("ExtendedState.LANDED_STATE_ON_GROUND", self.source)
        self.assertIn("self._control_state == 3", self.source)
        self.assertIn(
            '"live patrol_control execution requires camera_init frame"',
            self.source)

    def test_release_result_deduplication_is_transaction_local(self):
        self.assertIn("release_execution_id: int = 0", self.source)
        self.assertIn(
            "int(message.execution_id) != transaction.release_execution_id",
            self.source)
        self.assertNotIn("_last_release_execution_id", self.source)
        self.assertIn("self._pending_release = None", self.source)
        self.assertIn("pending_release_fence_conflict", self.source)


if __name__ == "__main__":
    unittest.main()
