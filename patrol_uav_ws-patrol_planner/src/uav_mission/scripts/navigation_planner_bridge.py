#!/usr/bin/env python3
"""Fenced ROS adapter between VCL06 decisions and Fast-Planner telemetry.

The adapter is deliberately unable to drive the live /fastplanner/goal topic.
The current planner has no acknowledged cancel/hold path, so this revision only
supports an isolated goal topic for contract and integration verification.
Payload, landing, hold and abort intents remain diagnostic-only.
"""

import json
import math
import threading
import uuid

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from plan_manage.msg import PlannerStatus
from std_msgs.msg import String

from uav_mission.msg import NavigationDecision, NavigationResult
from uav_mission.planner_execution import (
    ABORT_SAFE,
    CANCEL_PLANNER_GOAL,
    LAND_EXTERNAL,
    MotionDecision,
    MotionGoal,
    OdomSample,
    PUBLISH_PLANNER_GOAL,
    PlannerMotionConfig,
    PlannerMotionExecutor,
    PlannerStatusEvent,
    SequencedMotionGoal,
    START_TARGET_TRANSACTION,
    TargetIdentity,
)


LIVE_PLANNER_GOAL_TOPIC = "/fastplanner/goal"
MOTION_COMMANDS = frozenset((
    "SEARCH", "RESUME", "APPROACH", "RETURN_HOME",
))
DIAGNOSTIC_ONLY_INTENTS = frozenset((
    CANCEL_PLANNER_GOAL,
    START_TARGET_TRANSACTION,
    LAND_EXTERNAL,
    ABORT_SAFE,
))
COMMAND_NAMES = {
    NavigationDecision.SEARCH: "SEARCH",
    NavigationDecision.APPROACH: "APPROACH",
    NavigationDecision.ALIGN: "ALIGN",
    NavigationDecision.RESUME: "RESUME",
    NavigationDecision.RETURN_HOME: "RETURN_HOME",
    NavigationDecision.LAND: "LAND",
    NavigationDecision.HOLD: "HOLD",
    NavigationDecision.ABORT: "ABORT",
}
COMMAND_VALUES = {
    "SEARCH": NavigationResult.SEARCH,
    "APPROACH": NavigationResult.APPROACH,
    "ALIGN": NavigationResult.ALIGN,
    "RESUME": NavigationResult.RESUME,
    "RETURN_HOME": NavigationResult.RETURN_HOME,
    "LAND": NavigationResult.LAND,
    "HOLD": NavigationResult.HOLD,
    "ABORT": NavigationResult.ABORT,
}
STATUS_VALUES = {
    "ACCEPTED": NavigationResult.ACCEPTED,
    "STARTED": NavigationResult.STARTED,
    "PROGRESS": NavigationResult.PROGRESS,
    "SUCCEEDED": NavigationResult.SUCCEEDED,
    "FAILED": NavigationResult.FAILED,
    "REJECTED": NavigationResult.REJECTED,
    "CANCELLED": NavigationResult.CANCELLED,
    "TIMED_OUT": NavigationResult.TIMED_OUT,
}
STAGE_VALUES = {
    "DISPATCH": NavigationResult.DISPATCH,
    "PLANNER": NavigationResult.PLANNER,
    "CAPTURE": NavigationResult.CAPTURE,
    "ALIGNMENT": NavigationResult.ALIGNMENT,
    "RELEASE": NavigationResult.RELEASE,
    "RECOVERY": NavigationResult.RECOVERY,
    "LANDING": NavigationResult.LANDING,
}
PLANNER_STATUS_NAMES = {
    PlannerStatus.ACCEPTED: "ACCEPTED",
    PlannerStatus.PLANNING: "PLANNING",
    PlannerStatus.TRAJECTORY_READY: "TRAJECTORY_READY",
    PlannerStatus.REPLANNING: "REPLANNING",
    PlannerStatus.TRAJECTORY_FINISHED: "TRAJECTORY_FINISHED",
    PlannerStatus.FAILED_ATTEMPT: "FAILED_ATTEMPT",
    PlannerStatus.CANCELLED: "CANCELLED",
}


def _stamp_to_ns(stamp):
    secs = int(stamp.secs)
    nsecs = int(stamp.nsecs)
    if secs < 0 or nsecs < 0 or nsecs >= 1_000_000_000:
        raise ValueError("ROS stamp is not normalized")
    return secs * 1_000_000_000 + nsecs


def _ns_to_stamp(value):
    value = int(value)
    if value <= 0:
        raise ValueError("result stamp must be positive")
    return rospy.Time(
        secs=value // 1_000_000_000,
        nsecs=value % 1_000_000_000,
    )


def _seconds_to_ns(name, value, allow_zero=False):
    value = float(value)
    if not math.isfinite(value) or value < 0.0 or (value == 0.0 and not allow_zero):
        raise ValueError("%s must be finite and positive" % name)
    return int(round(value * 1_000_000_000))


def _identity_orientation(pose):
    orientation = pose.orientation
    return (
        float(orientation.x) == 0.0 and
        float(orientation.y) == 0.0 and
        float(orientation.z) == 0.0 and
        float(orientation.w) == 1.0
    )


class NavigationPlannerBridge:
    """Serialize ROS callbacks around one deterministic motion executor."""

    def __init__(self):
        self._lock = threading.RLock()
        self._adapter_faulted = False
        self._adapter_fault_reason = ""
        self._last_reason = "initializing"
        self._last_status = ""
        self._last_diagnostic_intents = []

        self._execution_requested = rospy.get_param(
            "~execution/enabled", False)
        self._allow_live_goal_output = rospy.get_param(
            "~execution/allow_live_goal_output", False)
        for name, value in (
                ("execution/enabled", self._execution_requested),
                ("execution/allow_live_goal_output",
                 self._allow_live_goal_output)):
            if not isinstance(value, bool):
                raise ValueError("%s must be boolean" % name)

        self._mission_frame = str(rospy.get_param(
            "~execution/mission_frame", "camera_init"))
        self._tick_hz = float(rospy.get_param(
            "~execution/tick_hz", 20.0))
        if not math.isfinite(self._tick_hz) or self._tick_hz <= 0.0:
            raise ValueError("execution/tick_hz must be finite and positive")

        prefix = str(rospy.get_param(
            "~execution/executor_id_prefix", "vcl06-planner-bridge"))
        if not prefix.strip():
            raise ValueError("executor_id_prefix must not be empty")
        executor_id = "%s-%s" % (prefix, uuid.uuid4().hex)
        config = PlannerMotionConfig(
            executor_id=executor_id,
            mission_frame=self._mission_frame,
            max_z_m=float(rospy.get_param(
                "~execution/max_goal_z", 4.0)),
            source_future_tolerance_ns=_seconds_to_ns(
                "source_future_tolerance",
                rospy.get_param(
                    "~execution/source_future_tolerance", 0.1),
                allow_zero=True),
            planner_accept_timeout_ns=_seconds_to_ns(
                "planner_accept_timeout",
                rospy.get_param("~execution/planner_accept_timeout", 2.0)),
            max_effective_goal_offset_m=float(rospy.get_param(
                "~execution/effective_goal_max_offset", 1.10)),
            max_planning_attempts=int(rospy.get_param(
                "~execution/max_planning_attempts", 20)),
            arrival_distance_m=float(rospy.get_param(
                "~execution/arrival_position_tolerance", 0.30)),
            arrival_speed_mps=float(rospy.get_param(
                "~execution/arrival_speed_tolerance", 0.20)),
            arrival_dwell_ns=_seconds_to_ns(
                "arrival_dwell",
                rospy.get_param("~execution/arrival_dwell", 0.50)),
            odom_max_age_ns=_seconds_to_ns(
                "odom_max_age",
                rospy.get_param("~execution/odom_max_age", 0.50)),
        )
        self._executor = PlannerMotionExecutor(config)
        self._executor_id = executor_id

        self._planner_goal_topic = rospy.resolve_name("planner_goal")
        self._output_enabled, self._gate_reason = self._evaluate_output_gate()

        self._goal_pub = None
        if self._output_enabled:
            self._goal_pub = rospy.Publisher(
                "planner_goal", PoseStamped, queue_size=1)
        self._result_pub = rospy.Publisher(
            "mission_result", NavigationResult, queue_size=20)
        self._status_pub = rospy.Publisher(
            "bridge_status", String, queue_size=1, latch=True)

        self._decision_sub = rospy.Subscriber(
            "mission_command_raw", NavigationDecision,
            self._on_decision, queue_size=1)
        self._planner_sub = rospy.Subscriber(
            "planner_status", PlannerStatus,
            self._on_planner_status, queue_size=20)
        self._odom_sub = rospy.Subscriber(
            "odom", Odometry, self._on_odom, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self._tick_hz), self._on_timer)
        self._last_reason = self._gate_reason
        self._publish_status(force=True)
        rospy.logwarn(
            "VCL06 planner bridge ready: output_enabled=%s topic=%s reason=%s",
            self._output_enabled, self._planner_goal_topic, self._gate_reason)

    def _evaluate_output_gate(self):
        if not self._execution_requested:
            return False, "execution_disabled_by_default"
        if self._planner_goal_topic == LIVE_PLANNER_GOAL_TOPIC:
            if not self._allow_live_goal_output:
                return False, "live_goal_output_not_acknowledged"
            return True, "live_planner_output_enabled"
        return True, "configured_planner_output_enabled"

    def _now_ns(self):
        value = _stamp_to_ns(rospy.Time.now())
        if value <= 0:
            raise ValueError("ROS time is not initialized")
        return value

    def _validate_pose_contract(self, stamped, seq, issued_ns):
        if int(stamped.header.seq) != int(seq):
            raise ValueError("nested goal sequence mismatch")
        if _stamp_to_ns(stamped.header.stamp) != int(issued_ns):
            raise ValueError("nested goal stamp mismatch")
        if stamped.header.frame_id != self._mission_frame:
            raise ValueError("nested goal frame mismatch")
        if not _identity_orientation(stamped.pose):
            raise ValueError("raw goal orientation must be identity")

    @staticmethod
    def _targetless_fields_are_zero(message):
        return (
            int(message.target_id) == 0 and
            _stamp_to_ns(message.target_first_seen) == 0 and
            _stamp_to_ns(message.target_observation_stamp) == 0 and
            message.target_class == "" and
            int(message.attempt) == 0 and
            int(message.payload_slot) == 0
        )

    def _decision_from_message(self, message):
        if int(message.schema_version) != NavigationDecision.SCHEMA_VERSION:
            raise ValueError("navigation decision schema mismatch")
        decision_seq = int(message.decision_seq)
        if int(message.header.seq) != decision_seq:
            raise ValueError("decision header sequence mismatch")
        issued_ns = _stamp_to_ns(message.header.stamp)
        if issued_ns <= 0:
            raise ValueError("decision source stamp must be positive")
        if message.header.frame_id != self._mission_frame:
            raise ValueError("decision frame mismatch")
        deadline_ns = _stamp_to_ns(message.deadline)
        command = COMMAND_NAMES.get(int(message.command))
        if command is None:
            raise ValueError("unknown navigation decision command")
        if command == "ALIGN":
            raise ValueError("ALIGN requires the target transaction executor")
        self._validate_pose_contract(
            message.goal, decision_seq, issued_ns)

        expects_goal = command in MOTION_COMMANDS
        expects_target = command == "APPROACH"
        if bool(message.has_goal) != expects_goal:
            raise ValueError("decision goal flag does not match command")
        if bool(message.has_target) != expects_target:
            raise ValueError("decision target flag does not match command")

        motion_goal = None
        if expects_goal:
            position = message.goal.pose.position
            motion_goal = MotionGoal(
                message.goal.header.frame_id,
                position.x, position.y, position.z)

        target = None
        if expects_target:
            target = TargetIdentity(
                target_id=int(message.target_id),
                first_seen_ns=_stamp_to_ns(message.target_first_seen),
                observation_ns=_stamp_to_ns(
                    message.target_observation_stamp),
                class_name=message.target_class,
                attempt=int(message.attempt),
                payload_slot=int(message.payload_slot),
            )
        elif not self._targetless_fields_are_zero(message):
            raise ValueError("targetless decision carries target identity")

        return MotionDecision(
            mission_id=message.mission_id,
            decision_seq=decision_seq,
            issued_at_ns=issued_ns,
            deadline_ns=deadline_ns,
            command=command,
            class_profile=message.class_profile,
            goal=motion_goal,
            target=target,
        )

    def _sequenced_goal_from_status(self, stamped, goal_seq):
        if int(stamped.header.seq) != int(goal_seq):
            raise ValueError("planner nested goal sequence mismatch")
        if stamped.header.frame_id != self._mission_frame:
            raise ValueError("planner nested goal frame mismatch")
        if _stamp_to_ns(stamped.header.stamp) <= 0:
            raise ValueError("planner nested goal stamp must be positive")
        if not _identity_orientation(stamped.pose):
            raise ValueError("planner nested goal orientation changed")
        position = stamped.pose.position
        return SequencedMotionGoal(
            goal_seq,
            MotionGoal(stamped.header.frame_id,
                       position.x, position.y, position.z),
        )

    def _planner_status_from_message(self, message):
        event_seq = int(message.event_seq)
        if int(message.header.seq) != event_seq:
            raise ValueError("planner event header sequence mismatch")
        if message.header.frame_id != self._mission_frame:
            raise ValueError("planner status frame mismatch")
        goal_seq = int(message.goal_seq)
        requested = self._sequenced_goal_from_status(
            message.requested_goal, goal_seq)
        effective = self._sequenced_goal_from_status(
            message.effective_goal, goal_seq)
        if (_stamp_to_ns(message.requested_goal.header.stamp) !=
                _stamp_to_ns(message.effective_goal.header.stamp)):
            raise ValueError("planner nested goal stamps differ")
        status = PLANNER_STATUS_NAMES.get(int(message.status))
        if status is None:
            raise ValueError("unknown planner status value")
        return PlannerStatusEvent(
            event_seq=event_seq,
            goal_seq=goal_seq,
            status=status,
            stamp_ns=_stamp_to_ns(message.header.stamp),
            requested_goal=requested,
            effective_goal=effective,
            distance_to_goal=float(message.distance_to_goal),
            planning_attempt=int(message.planning_attempt),
            reason=message.reason,
        )

    @staticmethod
    def _odom_from_message(message):
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        return OdomSample(
            stamp_ns=_stamp_to_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            x=position.x,
            y=position.y,
            z=position.z,
            vx=velocity.x,
            vy=velocity.y,
            vz=velocity.z,
        )

    def _publish_planner_goal(self, intent, decision):
        if not self._output_enabled:
            raise RuntimeError("planner output gate is closed")
        if self._goal_pub is None:
            raise RuntimeError("live planner publisher is not advertised")
        if intent.decision_seq != decision.decision_seq:
            raise RuntimeError("planner intent identity mismatch")
        if intent.goal is None:
            raise RuntimeError("planner intent has no goal")
        message = PoseStamped()
        message.header.seq = int(decision.decision_seq)
        message.header.stamp = _ns_to_stamp(decision.issued_at_ns)
        message.header.frame_id = intent.goal.frame_id
        message.pose.position.x = intent.goal.x
        message.pose.position.y = intent.goal.y
        message.pose.position.z = intent.goal.z
        message.pose.orientation.w = 1.0
        self._goal_pub.publish(message)

    def _result_message(self, event):
        message = NavigationResult()
        message.header.seq = int(event.event_seq)
        message.header.stamp = _ns_to_stamp(event.event_stamp_ns)
        message.header.frame_id = self._mission_frame
        message.schema_version = NavigationResult.SCHEMA_VERSION
        message.mission_id = event.mission_id
        message.executor_id = event.executor_id
        message.event_seq = int(event.event_seq)
        message.decision_seq = int(event.decision_seq)
        message.command = COMMAND_VALUES[event.command]
        message.status = STATUS_VALUES[event.status]
        message.stage = STAGE_VALUES[event.stage]
        message.terminal = bool(event.terminal)
        message.retryable = bool(event.retryable)
        message.payload_committed = bool(event.payload_committed)
        message.has_target = bool(event.has_target)
        message.target_id = int(event.target_id)
        if event.has_target:
            message.target_first_seen = _ns_to_stamp(
                event.target_first_seen_ns)
        message.target_class = event.target_class
        message.attempt = int(event.attempt)
        message.payload_slot = int(event.payload_slot)
        message.reason = event.reason
        message.evidence_source = event.evidence_source
        return message

    def _apply_outcome(self, outcome, submitted_decision=None):
        goal_published = False
        diagnostics = []
        for intent in outcome.intents:
            if intent.kind == PUBLISH_PLANNER_GOAL:
                if submitted_decision is None:
                    raise RuntimeError("goal intent lacks submitted decision")
                self._publish_planner_goal(intent, submitted_decision)
                goal_published = True
            elif intent.kind in DIAGNOSTIC_ONLY_INTENTS:
                diagnostics.append({
                    "kind": intent.kind,
                    "decision_seq": intent.decision_seq,
                    "reason": intent.reason,
                })
            else:
                raise RuntimeError("unknown executor intent: %s" % intent.kind)
        self._last_diagnostic_intents = diagnostics

        for event in outcome.events:
            if (event.status == "ACCEPTED" and event.stage == "DISPATCH" and
                    not goal_published):
                raise RuntimeError("dispatch acceptance precedes goal publish")
            self._result_pub.publish(self._result_message(event))
        self._last_reason = outcome.reason
        if outcome.snapshot.faulted:
            self._output_enabled = False
            self._gate_reason = outcome.snapshot.fault_reason

    def _handle_callback_exception(self, source, error):
        self._adapter_faulted = True
        self._adapter_fault_reason = "%s:%s" % (source, error)
        self._output_enabled = False
        self._gate_reason = "adapter_fault"
        self._last_reason = self._adapter_fault_reason
        rospy.logerr("VCL06 planner bridge disabled: %s",
                     self._adapter_fault_reason)
        self._publish_status(force=True)

    def _on_decision(self, message):
        with self._lock:
            if not self._output_enabled:
                self._publish_status()
                return
            try:
                now_ns = self._now_ns()
                decision = self._decision_from_message(message)
                outcome = self._executor.submit_decision(decision, now_ns)
                self._apply_outcome(
                    outcome,
                    submitted_decision=(decision if outcome.accepted else None),
                )
                self._publish_status(force=True)
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("decision", error)

    def _on_planner_status(self, message):
        with self._lock:
            if not self._output_enabled:
                self._publish_status()
                return
            try:
                now_ns = self._now_ns()
                event = self._planner_status_from_message(message)
                outcome = self._executor.apply_planner_status(event, now_ns)
                self._apply_outcome(outcome)
                self._publish_status(force=True)
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("planner_status", error)

    def _on_odom(self, message):
        with self._lock:
            if not self._output_enabled:
                return
            try:
                now_ns = self._now_ns()
                outcome = self._executor.apply_odom(
                    self._odom_from_message(message), now_ns)
                self._apply_outcome(outcome)
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("odom", error)

    def _on_timer(self, _event):
        with self._lock:
            try:
                if self._output_enabled:
                    outcome = self._executor.tick(self._now_ns())
                    self._apply_outcome(outcome)
                self._publish_status()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("timer", error)

    def _publish_status(self, force=False):
        snapshot = self._executor.snapshot()
        payload = {
            "adapter_faulted": self._adapter_faulted,
            "adapter_fault_reason": self._adapter_fault_reason,
            "execution_requested": self._execution_requested,
            "output_enabled": self._output_enabled,
            "gate_reason": self._gate_reason,
            "allow_live_goal_output": self._allow_live_goal_output,
            "live_goal_output_supported": True,
            "planner_goal_topic": self._planner_goal_topic,
            "executor_id": self._executor_id,
            "last_reason": self._last_reason,
            "diagnostic_only_intents": self._last_diagnostic_intents,
            "executor": {
                "faulted": snapshot.faulted,
                "fault_reason": snapshot.fault_reason,
                "mission_id": snapshot.mission_id,
                "last_decision_seq": snapshot.last_decision_seq,
                "active_decision_seq": snapshot.active_decision_seq,
                "active_command": snapshot.active_command,
                "active_terminal": snapshot.active_terminal,
                "active_handed_off": snapshot.active_handed_off,
                "planner_accepted": snapshot.planner_accepted,
                "trajectory_ready": snapshot.trajectory_ready,
                "trajectory_finished": snapshot.trajectory_finished,
                "dwell_start_ns": snapshot.dwell_start_ns,
                "last_planner_event_seq": snapshot.last_planner_event_seq,
                "executor_event_seq": snapshot.executor_event_seq,
                "awaiting_cancel_goal_seq":
                    snapshot.awaiting_cancel_goal_seq,
            },
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        if force or encoded != self._last_status:
            self._status_pub.publish(String(data=encoded))
            self._last_status = encoded


def main():
    rospy.init_node("navigation_planner_bridge")
    NavigationPlannerBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
