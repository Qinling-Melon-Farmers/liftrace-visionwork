#!/usr/bin/env python3
"""Single fenced executor for VCL06 planner and delivery decisions.

Live planner output remains disabled by default and requires an explicit launch
acknowledgement.  The same node forwards the existing ``MissionCommand`` and
alignment-context interfaces after an APPROACH arrives; it does not own target
selection, retry, payload-slot allocation or mission scheduling.
"""

from dataclasses import dataclass
import json
import math
import os
import sys
import threading
import uuid

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState
from nav_msgs.msg import Odometry
from plan_manage.msg import PlannerStatus
from std_msgs.msg import Int8, String

from patrol_control.msg import MissionCommand
from uav_mission.msg import NavigationDecision, NavigationResult
from uav_mission.msg import ReleaseResult
from uav_mission.planner_execution import (
    MotionDecision,
    MotionGoal,
    OdomSample,
    PlannerMotionConfig,
    PlannerMotionExecutor,
    PlannerStatusEvent,
    SequencedMotionGoal,
    TargetIdentity,
)
from uav_mission.position_settle import PositionSettleWindow
from uav_vision.msg import (
    AlignmentTargetContext, ReleaseEvidenceContext, TargetCandidateArray,
)

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from release_commitment import strict_context_source


LIVE_PLANNER_GOAL_TOPIC = "/fastplanner/goal"
WIRE_GOAL_COMMANDS = frozenset((
    "SEARCH", "RESUME", "APPROACH", "RETURN_HOME",
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
MISSION_COMMAND_VALUES = {
    "SEARCH": MissionCommand.SEARCH,
    "APPROACH": MissionCommand.APPROACH,
    "ALIGN": MissionCommand.ALIGN,
    "RESUME": MissionCommand.RESUME,
    "RETURN_HOME": MissionCommand.RETURN_HOME,
    "LAND": MissionCommand.LAND,
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


@dataclass(frozen=True)
class SemanticTargetPose:
    frame_id: str
    x: float
    y: float
    z: float
    last_seen_ns: int


@dataclass
class TargetTransaction:
    decision: MotionDecision
    phase: str = "APPROACHING"
    target_pose: object = None
    align_mode: str = ""
    strict_evidence_stamp_ns: int = 0
    release_execution_id: int = 0
    release_ack_ns: int = 0


@dataclass
class LandingTransaction:
    decision: MotionDecision
    target_pose: SemanticTargetPose
    command_sent_ns: int
    started: bool = False


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
        self._capture_max_age_ns = _seconds_to_ns(
            "target/capture_max_age",
            rospy.get_param("~target/capture_max_age", 1.0))
        self._context_max_age = float(rospy.get_param(
            "~target/context_max_age", 0.5))
        self._association_distance = float(rospy.get_param(
            "~target/max_association_distance", 0.8))
        self._recovery_height = float(rospy.get_param(
            "~target/recovery_height", 0.95))
        self._recovery_settle_radius = float(rospy.get_param(
            "~target/recovery_settle_radius", 0.15))
        self._recovery_dwell_ns = _seconds_to_ns(
            "target/recovery_dwell",
            rospy.get_param("~target/recovery_dwell", 0.5))
        self._landing_height = float(rospy.get_param(
            "~landing/height", 0.05))
        self._landing_radius = float(rospy.get_param(
            "~landing/radius", 0.25))
        self._landing_settle_radius = float(rospy.get_param(
            "~landing/settle_radius", 0.15))
        self._landing_dwell_ns = _seconds_to_ns(
            "landing/dwell", rospy.get_param("~landing/dwell", 1.0))
        for name, value in (
                ("target/context_max_age", self._context_max_age),
                ("target/max_association_distance",
                 self._association_distance),
                ("target/recovery_height", self._recovery_height),
                ("target/recovery_settle_radius",
                 self._recovery_settle_radius),
                ("landing/height", self._landing_height),
                ("landing/radius", self._landing_radius),
                ("landing/settle_radius", self._landing_settle_radius)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("%s must be finite and non-negative" % name)
        if (self._association_distance == 0.0 or
                self._recovery_settle_radius == 0.0 or
                self._landing_radius == 0.0 or
                self._landing_settle_radius == 0.0):
            raise ValueError("association and settle radii must be positive")

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
                rospy.get_param("~execution/planner_accept_timeout", 5.0)),
            max_effective_goal_offset_m=float(rospy.get_param(
                "~execution/effective_goal_max_offset", 1.10)),
            arrival_distance_m=float(rospy.get_param(
                "~execution/arrival_position_tolerance", 0.30)),
            approach_arrival_distance_m=float(rospy.get_param(
                "~execution/approach_arrival_position_tolerance", 0.35)),
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
        self._recovery_settle = PositionSettleWindow(
            self._recovery_dwell_ns,
            self._recovery_settle_radius,
            config.odom_max_age_ns,
        )
        self._landing_settle = PositionSettleWindow(
            self._landing_dwell_ns,
            self._landing_settle_radius,
            config.odom_max_age_ns,
        )
        self._executor_id = executor_id
        self._latest_candidates = ()
        self._transaction = None
        self._pending_release = None
        self._landing = None
        self._last_odom = None
        self._control_state = None
        self._control_state_receipt_ns = 0
        self._align_mode = "disabled"
        self._align_mode_receipt_ns = 0
        self._landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self._landed_state_receipt_ns = 0

        self._planner_goal_topic = rospy.resolve_name("planner_goal")
        if (self._execution_requested and self._allow_live_goal_output and
                self._planner_goal_topic == LIVE_PLANNER_GOAL_TOPIC and
                self._mission_frame != "camera_init"):
            raise ValueError(
                "live patrol_control execution requires camera_init frame")
        self._output_enabled, self._gate_reason = self._evaluate_output_gate()

        self._goal_pub = None
        if self._output_enabled:
            self._goal_pub = rospy.Publisher(
                "planner_goal", PoseStamped, queue_size=1)
        self._result_pub = rospy.Publisher(
            "mission_result", NavigationResult, queue_size=20)
        self._status_pub = rospy.Publisher(
            "bridge_status", String, queue_size=1, latch=True)
        self._mission_command_pub = rospy.Publisher(
            "mission_command", MissionCommand, queue_size=4)
        self._alignment_context_pub = rospy.Publisher(
            "alignment_target_context", AlignmentTargetContext,
            queue_size=1, latch=True)

        self._decision_sub = rospy.Subscriber(
            "mission_command_raw", NavigationDecision,
            self._on_decision, queue_size=1)
        self._planner_sub = rospy.Subscriber(
            "planner_status", PlannerStatus,
            self._on_planner_status, queue_size=20)
        self._odom_sub = rospy.Subscriber(
            "odom", Odometry, self._on_odom, queue_size=1)
        self._targets_sub = rospy.Subscriber(
            "targets", TargetCandidateArray, self._on_targets, queue_size=1)
        self._evidence_context_sub = rospy.Subscriber(
            "release_evidence_context", ReleaseEvidenceContext,
            self._on_release_evidence_context, queue_size=2)
        self._release_result_sub = rospy.Subscriber(
            "release_result", ReleaseResult,
            self._on_release_result, queue_size=4)
        self._control_state_sub = rospy.Subscriber(
            "control_state", Int8, self._on_control_state, queue_size=2)
        self._align_mode_sub = rospy.Subscriber(
            "align_mode", String, self._on_align_mode, queue_size=2)
        self._landed_state_sub = rospy.Subscriber(
            "landed_state", ExtendedState,
            self._on_landed_state, queue_size=2)
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

    def _odom_rejection_reason(self, sample, now_ns):
        if sample is None:
            return "odom_unavailable"
        if sample.frame_id != self._mission_frame:
            return "odom_frame_mismatch"
        age_ns = int(now_ns) - sample.stamp_ns
        if age_ns < -self._executor.config.source_future_tolerance_ns:
            return "odom_from_future"
        if age_ns > self._executor.config.odom_max_age_ns:
            return "odom_stale"
        return ""

    def _validate_pose_contract(self, stamped, seq, issued_ns):
        if int(stamped.header.seq) != int(seq):
            raise ValueError("nested goal sequence mismatch")
        if _stamp_to_ns(stamped.header.stamp) != int(issued_ns):
            raise ValueError("nested goal stamp mismatch")
        if stamped.header.frame_id != self._mission_frame:
            raise ValueError("nested goal frame mismatch")

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

    def _decision_from_message(self, message, now_ns):
        if int(message.schema_version) != NavigationDecision.SCHEMA_VERSION:
            raise ValueError("navigation decision schema mismatch")
        decision_seq = int(message.decision_seq)
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
        if command == "HOLD":
            raise ValueError("HOLD is not supported by the planner bridge")
        self._validate_pose_contract(
            message.goal, decision_seq, issued_ns)

        expects_goal = command in WIRE_GOAL_COMMANDS
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
                position.x, position.y, position.z,
                message.goal.pose.orientation.x,
                message.goal.pose.orientation.y,
                message.goal.pose.orientation.z,
                message.goal.pose.orientation.w)
        elif command == "ABORT":
            sample = self._last_odom
            if self._odom_rejection_reason(sample, now_ns):
                raise RuntimeError("abort_hold_odom_unavailable")
            motion_goal = MotionGoal(
                self._mission_frame, sample.x, sample.y, sample.z)

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

    def _sequenced_goal_from_status(
            self, stamped, transport_goal_seq, decision_seq):
        if int(stamped.header.seq) != int(transport_goal_seq):
            raise ValueError("planner nested goal sequence mismatch")
        if stamped.header.frame_id != self._mission_frame:
            raise ValueError("planner nested goal frame mismatch")
        if _stamp_to_ns(stamped.header.stamp) <= 0:
            raise ValueError("planner nested goal stamp must be positive")
        position = stamped.pose.position
        orientation = stamped.pose.orientation
        return SequencedMotionGoal(
            decision_seq,
            MotionGoal(stamped.header.frame_id,
                       position.x, position.y, position.z,
                       orientation.x, orientation.y,
                       orientation.z, orientation.w),
        )

    def _planner_status_from_message(self, message):
        # roscpp rewrites a top-level Header.seq with its publication counter.
        # The explicit event_seq is the planner-owned ordering/deduplication
        # contract and therefore must not be coupled to that transport field.
        event_seq = int(message.event_seq)
        if message.header.frame_id != self._mission_frame:
            raise ValueError("planner status frame mismatch")
        transport_goal_seq = int(message.goal_seq)
        requested_stamp_ns = _stamp_to_ns(
            message.requested_goal.header.stamp)
        effective_stamp_ns = _stamp_to_ns(
            message.effective_goal.header.stamp)
        if requested_stamp_ns != effective_stamp_ns:
            raise ValueError("planner nested goal stamps differ")
        # rospy/roscpp own Header.seq and may rewrite it independently of the
        # mission decision.  Resolve the echoed, preserved source stamp back
        # to the retained decision generation before applying lifecycle facts.
        goal_seq = self._executor.resolve_goal_seq_by_stamp(
            requested_stamp_ns)
        if goal_seq == 0:
            return None
        requested = self._sequenced_goal_from_status(
            message.requested_goal, transport_goal_seq, goal_seq)
        effective = self._sequenced_goal_from_status(
            message.effective_goal, transport_goal_seq, goal_seq)
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

    def _publish_planner_goal(self, decision):
        if not self._output_enabled:
            raise RuntimeError("planner output gate is closed")
        if self._goal_pub is None:
            raise RuntimeError("live planner publisher is not advertised")
        if decision.goal is None:
            raise RuntimeError("planner intent has no goal")
        message = PoseStamped()
        message.header.seq = int(decision.decision_seq)
        message.header.stamp = _ns_to_stamp(decision.issued_at_ns)
        message.header.frame_id = decision.goal.frame_id
        message.pose.position.x = decision.goal.x
        message.pose.position.y = decision.goal.y
        message.pose.position.z = decision.goal.z
        message.pose.orientation.x = decision.goal.qx
        message.pose.orientation.y = decision.goal.qy
        message.pose.orientation.z = decision.goal.qz
        message.pose.orientation.w = decision.goal.qw
        self._goal_pub.publish(message)

    def _mission_command_message(self, decision, command_name,
                                 target_pose=None):
        command = MISSION_COMMAND_VALUES.get(command_name)
        if command is None:
            raise ValueError("command has no patrol_control handoff")
        now = rospy.Time.now()
        message = MissionCommand()
        message.header.seq = int(decision.decision_seq)
        message.header.stamp = now
        message.header.frame_id = self._mission_frame
        message.command = command
        target = decision.target
        if target is not None:
            message.target_id = int(target.target_id)
            message.target_class = target.class_name
        message.goal.header.seq = int(decision.decision_seq)
        message.goal.header.stamp = now
        message.goal.header.frame_id = self._mission_frame
        message.goal.pose.orientation.w = 1.0
        if decision.goal is not None:
            message.goal.pose.position.x = decision.goal.x
            message.goal.pose.position.y = decision.goal.y
            message.goal.pose.position.z = decision.goal.z
            message.goal.pose.orientation.x = decision.goal.qx
            message.goal.pose.orientation.y = decision.goal.qy
            message.goal.pose.orientation.z = decision.goal.qz
            message.goal.pose.orientation.w = decision.goal.qw
        if target_pose is not None:
            message.goal.pose.position.x = target_pose.x
            message.goal.pose.position.y = target_pose.y
            message.goal.pose.position.z = target_pose.z
        return message

    def _publish_mission_command(self, decision, command_name,
                                 target_pose=None):
        self._mission_command_pub.publish(self._mission_command_message(
            decision, command_name, target_pose=target_pose))

    def _matching_target_pose(self, now_ns):
        transaction = self._transaction
        if transaction is None:
            return None
        target = transaction.decision.target
        if target is None:
            return None
        matches = []
        for candidate in self._latest_candidates:
            try:
                first_seen_ns = _stamp_to_ns(candidate.first_seen)
                last_seen_ns = _stamp_to_ns(candidate.last_seen)
                point = candidate.map_point
                coordinates = (
                    float(point.x), float(point.y), float(point.z))
                if (
                    int(candidate.id) != target.target_id or
                    first_seen_ns != target.first_seen_ns or
                    str(candidate.class_name) != target.class_name or
                    not bool(candidate.map_valid) or
                    not bool(candidate.association_valid) or
                    bool(str(candidate.reject_reason).strip()) or
                    int(candidate.state) < 2 or
                    str(candidate.map_frame) != self._mission_frame or
                    last_seen_ns < target.observation_ns or
                    now_ns - last_seen_ns > self._capture_max_age_ns or
                    last_seen_ns > now_ns +
                    self._executor.config.source_future_tolerance_ns or
                    not all(math.isfinite(value) for value in coordinates)
                ):
                    continue
                matches.append(SemanticTargetPose(
                    frame_id=str(candidate.map_frame),
                    x=coordinates[0], y=coordinates[1], z=coordinates[2],
                    last_seen_ns=last_seen_ns,
                ))
            except (AttributeError, TypeError, ValueError, OverflowError):
                continue
        return max(matches, key=lambda item: item.last_seen_ns) \
            if matches else None

    def _alignment_context_message(self, active, now_ns):
        transaction = self._transaction
        if transaction is None or transaction.target_pose is None:
            raise RuntimeError("alignment context has no frozen target pose")
        decision = transaction.decision
        target = decision.target
        pose = transaction.target_pose
        message = AlignmentTargetContext()
        message.header.seq = int(decision.decision_seq)
        message.header.stamp = _ns_to_stamp(now_ns)
        message.header.frame_id = self._mission_frame
        message.source = self._executor_id
        message.schema_version = AlignmentTargetContext.SCHEMA_VERSION
        message.active = bool(active)
        message.mission_id = decision.mission_id
        message.decision_seq = int(decision.decision_seq)
        message.deadline = _ns_to_stamp(decision.deadline_ns)
        message.command = AlignmentTargetContext.ALIGN
        message.class_profile = decision.class_profile
        message.align_mode = transaction.align_mode
        message.has_target = True
        message.semantic_target_id = int(target.target_id)
        message.semantic_target_first_seen = _ns_to_stamp(
            target.first_seen_ns)
        message.target_observation_stamp = _ns_to_stamp(
            target.observation_ns)
        message.semantic_target_class = target.class_name
        message.attempt = int(target.attempt)
        message.payload_slot = int(target.payload_slot)
        message.target_pose.header.seq = int(decision.decision_seq)
        message.target_pose.header.stamp = _ns_to_stamp(pose.last_seen_ns)
        message.target_pose.header.frame_id = pose.frame_id
        message.target_pose.pose.position.x = pose.x
        message.target_pose.pose.position.y = pose.y
        message.target_pose.pose.position.z = pose.z
        message.target_pose.pose.orientation.w = 1.0
        message.max_association_distance_m = self._association_distance
        return message

    def _publish_alignment_context(self, active, now_ns=None):
        now_ns = self._now_ns() if now_ns is None else int(now_ns)
        self._alignment_context_pub.publish(
            self._alignment_context_message(active, now_ns))

    def _clear_handoffs(self):
        transaction = self._transaction
        if transaction is not None and transaction.target_pose is not None:
            self._publish_alignment_context(False)
        if (transaction is not None and transaction.phase == "EXPIRED" and
                transaction.strict_evidence_stamp_ns > 0 and
                transaction.release_execution_id == 0):
            if (self._pending_release is not None and
                    self._pending_release.decision.decision_seq !=
                    transaction.decision.decision_seq):
                raise RuntimeError("pending_release_fence_conflict")
            self._pending_release = transaction
        self._transaction = None
        self._landing = None
        self._recovery_settle.reset()
        self._landing_settle.reset()

    def _report_target_stage(self, status, stage, now_ns, terminal=False,
                             retryable=False, payload_committed=False,
                             reason="", evidence_source=
                             "target_transaction_executor",
                             transaction=None):
        transaction = self._transaction if transaction is None else transaction
        if transaction is None:
            raise RuntimeError("target transaction is not active")
        outcome = self._executor.report_target_stage(
            transaction.decision.decision_seq,
            now_ns,
            status,
            stage,
            terminal=terminal,
            retryable=retryable,
            payload_committed=payload_committed,
            reason=reason,
            evidence_source=evidence_source,
        )
        if not outcome.accepted:
            raise RuntimeError(outcome.reason)
        self._apply_outcome(outcome)

    def _begin_target_capture(self, now_ns):
        transaction = self._transaction
        if transaction is None or transaction.phase != "APPROACHING":
            raise RuntimeError("APPROACH handoff has no active transaction")
        transaction.phase = "CAPTURE"
        self._try_begin_alignment(now_ns)

    def _try_begin_alignment(self, now_ns):
        transaction = self._transaction
        if transaction is None or transaction.phase != "CAPTURE":
            return
        target_pose = self._matching_target_pose(now_ns)
        if target_pose is None:
            return
        transaction.target_pose = target_pose
        transaction.align_mode = (
            "drop_cross" if transaction.decision.target.class_name ==
            "red_cross" else "drop_circle")
        transaction.phase = "ALIGN_COMMAND_SENT"
        self._publish_alignment_context(True, now_ns)
        self._publish_mission_command(
            transaction.decision, "ALIGN", target_pose=target_pose)

    def _strict_context_matches(self, context, now):
        transaction = self._transaction
        if (transaction is None or transaction.phase not in
                ("ALIGN_COMMAND_SENT", "ALIGNMENT")):
            return False
        decision = transaction.decision
        target = decision.target
        valid, _, source = strict_context_source(
            context,
            now,
            self._context_max_age,
            decision.class_profile,
            transaction.align_mode,
            target.payload_slot,
        )
        if not valid:
            return False
        return (
            str(context.context_source) == self._executor_id and
            str(context.mission_id) == decision.mission_id and
            int(context.decision_seq) == decision.decision_seq and
            _stamp_to_ns(context.deadline) == decision.deadline_ns and
            int(context.semantic_target_id) == target.target_id and
            _stamp_to_ns(context.semantic_target_first_seen) ==
            target.first_seen_ns and
            _stamp_to_ns(context.target_observation_stamp) ==
            target.observation_ns and
            str(context.semantic_target_class) == target.class_name and
            int(context.attempt) == target.attempt and
            int(context.payload_slot) == target.payload_slot and
            int(source["target_id"]) == target.target_id and
            str(source["target_class"]) == target.class_name
        )

    def _mark_alignment_started(self, transaction, now_ns,
                                reason, evidence_source):
        """Record observed alignment acceptance in deterministic order."""

        if transaction.phase != "ALIGN_COMMAND_SENT":
            return
        self._report_target_stage(
            "STARTED", "CAPTURE", now_ns,
            reason=reason,
            evidence_source=evidence_source,
            transaction=transaction,
        )
        transaction.phase = "ALIGNMENT"
        if transaction.strict_evidence_stamp_ns > 0:
            self._report_target_stage(
                "STARTED", "ALIGNMENT", now_ns,
                reason="strict_alignment_context_valid",
                evidence_source="uav_vision_release_context",
                transaction=transaction,
            )

    @staticmethod
    def _release_result_matches(transaction, message):
        if (transaction is None or
                transaction.phase not in (
                    "ALIGN_COMMAND_SENT", "ALIGNMENT", "EXPIRED") or
                transaction.strict_evidence_stamp_ns <= 0):
            return False
        target = transaction.decision.target
        result_stamp_ns = _stamp_to_ns(message.header.stamp)
        return (
            int(message.execution_id) > 0 and
            int(message.execution_id) != transaction.release_execution_id and
            result_stamp_ns >= transaction.strict_evidence_stamp_ns and
            int(message.payload_slot) == target.payload_slot and
            str(message.align_mode) == transaction.align_mode and
            int(message.target_id) == target.target_id and
            str(message.target_class) == target.class_name
        )

    def _release_result_transaction(self, message):
        for transaction in (self._transaction, self._pending_release):
            if self._release_result_matches(transaction, message):
                return transaction
        return None

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
        goal_published = outcome.planner_goal is not None
        if goal_published:
            if submitted_decision != outcome.planner_goal:
                raise RuntimeError("planner goal identity mismatch")
            self._publish_planner_goal(outcome.planner_goal)
        self._last_diagnostic_intents = ([{
            "handoff": outcome.handoff,
            "reason": outcome.reason,
        }] if outcome.handoff else [])

        for event in outcome.events:
            if (event.status == "ACCEPTED" and event.stage == "DISPATCH" and
                    not goal_published):
                raise RuntimeError("dispatch acceptance precedes goal publish")
            self._result_pub.publish(self._result_message(event))
        self._last_reason = outcome.reason

    def _start_decision_handoff(self, decision, outcome, now_ns):
        if outcome.planner_goal is not None:
            self._clear_handoffs()
            if decision.command == "APPROACH":
                self._transaction = TargetTransaction(decision=decision)
            if decision.command in MISSION_COMMAND_VALUES:
                self._publish_mission_command(decision, decision.command)
            return
        if not outcome.handoff:
            return
        self._clear_handoffs()
        if outcome.handoff == "LAND":
            sample = self._last_odom
            if self._odom_rejection_reason(sample, now_ns):
                rejected = self._executor.report_landing(
                    decision.decision_seq,
                    now_ns,
                    "REJECTED",
                    True,
                    "landing_target_odom_unavailable",
                )
                if not rejected.accepted:
                    raise RuntimeError(rejected.reason)
                self._apply_outcome(rejected)
                return
            target_pose = SemanticTargetPose(
                frame_id=self._mission_frame,
                x=sample.x,
                y=sample.y,
                z=0.0,
                last_seen_ns=sample.stamp_ns,
            )
            self._landing = LandingTransaction(
                decision=decision,
                target_pose=target_pose,
                command_sent_ns=now_ns,
            )
            self._landing_settle.reset("awaiting_control_acceptance")
            self._publish_mission_command(
                decision, "LAND", target_pose=target_pose)

    def _update_recovery(self, sample, now_ns):
        transaction = self._transaction
        if transaction is None or transaction.phase != "RECOVERY":
            return
        reason = self._odom_rejection_reason(sample, now_ns)
        if not reason and sample.z < self._recovery_height:
            reason = "recovery_height_not_reached"
        if not reason and self._control_state != 1:
            reason = "control_state_not_run"
        if (not reason and
                self._control_state_receipt_ns < transaction.release_ack_ns):
            reason = "control_state_predates_release"
        if not reason and self._align_mode != "disabled":
            reason = "alignment_still_active"
        if (not reason and
                self._align_mode_receipt_ns < transaction.release_ack_ns):
            reason = "align_mode_predates_release"
        if reason:
            self._recovery_settle.reset(reason)
            return

        settle = self._recovery_settle.update(
            sample.stamp_ns, sample.x, sample.y, sample.z)
        if not settle.ready:
            return
        self._report_target_stage(
            "SUCCEEDED", "RECOVERY", now_ns,
            terminal=True,
            reason="release_recovery_confirmed",
            evidence_source="patrol_control_recovery",
        )
        transaction.phase = "TERMINAL"

    def _update_landing(self, sample, now_ns):
        landing = self._landing
        if landing is None or not landing.started:
            return
        target = landing.target_pose
        horizontal_error = math.hypot(
            sample.x - target.x, sample.y - target.y)
        reason = self._odom_rejection_reason(sample, now_ns)
        if not reason and self._control_state != 3:
            reason = "control_state_not_landing"
        if (not reason and
                self._control_state_receipt_ns < landing.command_sent_ns):
            reason = "control_state_predates_land_command"
        if (not reason and self._landed_state !=
                ExtendedState.LANDED_STATE_ON_GROUND):
            reason = "landed_state_not_on_ground"
        if (not reason and
                self._landed_state_receipt_ns < landing.command_sent_ns):
            reason = "landed_state_predates_land_command"
        if not reason and horizontal_error > self._landing_radius:
            reason = "landing_radius_not_met"
        if not reason and sample.z > self._landing_height:
            reason = "landing_height_not_met"
        if reason:
            self._landing_settle.reset(reason)
            return

        settle = self._landing_settle.update(
            sample.stamp_ns, sample.x, sample.y, sample.z)
        if not settle.ready:
            return
        outcome = self._executor.report_landing(
            landing.decision.decision_seq,
            now_ns,
            "SUCCEEDED",
            True,
            "landed_state_and_settle_confirmed",
        )
        if not outcome.accepted:
            raise RuntimeError(outcome.reason)
        self._apply_outcome(outcome)
        self._landing = None

    def _expire_handoff_if_due(self, now_ns):
        transaction = self._transaction
        snapshot = self._executor.snapshot()
        if (transaction is not None and
                snapshot.active_handed_off and
                snapshot.active_decision_seq ==
                transaction.decision.decision_seq and
                transaction.phase not in ("TERMINAL", "EXPIRED") and
                now_ns >= transaction.decision.deadline_ns):
            if transaction.phase == "RECOVERY":
                stage = "RECOVERY"
                retryable = False
                reason = "recovery_deadline_reached"
            elif (transaction.phase in (
                    "ALIGN_COMMAND_SENT", "ALIGNMENT") and
                  transaction.strict_evidence_stamp_ns > 0):
                stage = "RELEASE"
                retryable = False
                reason = "release_result_deadline_reached"
            elif transaction.phase == "ALIGNMENT":
                stage = "ALIGNMENT"
                retryable = True
                reason = "alignment_evidence_deadline_reached"
            else:
                stage = "CAPTURE"
                retryable = True
                reason = "target_capture_deadline_reached"
            self._report_target_stage(
                "TIMED_OUT", stage, now_ns,
                terminal=True,
                retryable=retryable,
                reason=reason,
            )
            transaction.phase = "EXPIRED"
            if transaction.target_pose is not None:
                self._publish_alignment_context(False, now_ns)

        landing = self._landing
        if (landing is not None and
                now_ns >= landing.decision.deadline_ns):
            outcome = self._executor.report_landing(
                landing.decision.decision_seq,
                now_ns,
                "TIMED_OUT",
                True,
                "landing_deadline_reached",
            )
            if not outcome.accepted:
                raise RuntimeError(outcome.reason)
            self._apply_outcome(outcome)
            self._landing = None

    def _handle_callback_exception(self, source, error):
        if isinstance(error, ValueError):
            self._last_reason = "ignored_%s:%s" % (source, error)
            rospy.logwarn("VCL06 planner bridge ignored malformed %s: %s",
                          source, error)
            self._publish_status(force=True)
            return
        try:
            if (self._transaction is not None and
                    self._transaction.target_pose is not None):
                self._publish_alignment_context(False)
        except Exception:  # pylint: disable=broad-except
            pass
        self._transaction = None
        self._landing = None
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
                decision = self._decision_from_message(message, now_ns)
                outcome = self._executor.submit_decision(decision, now_ns)
                self._apply_outcome(
                    outcome,
                    submitted_decision=(decision if outcome.accepted else None),
                )
                if outcome.accepted:
                    self._start_decision_handoff(decision, outcome, now_ns)
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
                if event is None:
                    self._last_reason = "foreign_planner_goal_stamp_ignored"
                    self._publish_status(force=True)
                    return
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
                sample = self._odom_from_message(message)
                self._last_odom = sample
                outcome = self._executor.apply_odom(sample, now_ns)
                self._apply_outcome(outcome)
                if outcome.handoff == "TARGET_TRANSACTION":
                    self._begin_target_capture(now_ns)
                self._update_recovery(sample, now_ns)
                self._update_landing(sample, now_ns)
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("odom", error)

    def _on_targets(self, message):
        with self._lock:
            try:
                self._latest_candidates = tuple(message.targets)
                if self._output_enabled:
                    self._try_begin_alignment(self._now_ns())
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("targets", error)

    def _on_release_evidence_context(self, message):
        with self._lock:
            try:
                if not self._output_enabled:
                    return
                now = rospy.Time.now()
                self._expire_handoff_if_due(_stamp_to_ns(now))
                if not self._strict_context_matches(message, now):
                    return
                transaction = self._transaction
                evidence_stamp_ns = _stamp_to_ns(message.evidence.header.stamp)
                first_valid = transaction.strict_evidence_stamp_ns == 0
                transaction.strict_evidence_stamp_ns = max(
                    transaction.strict_evidence_stamp_ns,
                    evidence_stamp_ns,
                )
                if first_valid and transaction.phase == "ALIGNMENT":
                    self._report_target_stage(
                        "STARTED", "ALIGNMENT", _stamp_to_ns(now),
                        reason="strict_alignment_context_valid",
                        evidence_source="uav_vision_release_context",
                    )
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("release_context", error)

    def _on_release_result(self, message):
        with self._lock:
            try:
                if not self._output_enabled:
                    return
                now_ns = self._now_ns()
                self._expire_handoff_if_due(now_ns)
                transaction = self._release_result_transaction(message)
                if transaction is None:
                    return
                execution_id = int(message.execution_id)
                if transaction.phase == "ALIGN_COMMAND_SENT":
                    # A guarded release ACK is stronger evidence that the
                    # already-published ALIGN command was accepted than the
                    # relative callback ordering of two ROS topics.
                    self._mark_alignment_started(
                        transaction, now_ns,
                        "alignment_accepted_before_release_ack",
                        "guarded_servo_proxy",
                    )
                transaction.release_execution_id = execution_id
                source = "guarded_servo_proxy:%d" % execution_id
                if bool(message.success):
                    self._report_target_stage(
                        "PROGRESS", "RELEASE", now_ns,
                        payload_committed=True,
                        reason="release_ack_success",
                        evidence_source=source,
                        transaction=transaction,
                    )
                    if transaction is self._pending_release:
                        transaction.phase = "TERMINAL"
                        self._pending_release = None
                        return
                    self._publish_alignment_context(False, now_ns)
                    if transaction.phase == "EXPIRED":
                        transaction.phase = "TERMINAL"
                        return
                    transaction.phase = "RECOVERY"
                    transaction.release_ack_ns = now_ns
                    self._recovery_settle.reset(
                        "awaiting_post_release_state")
                else:
                    if transaction is self._pending_release:
                        transaction.phase = "TERMINAL"
                        self._pending_release = None
                        return
                    if transaction.phase == "EXPIRED":
                        transaction.phase = "TERMINAL"
                        return
                    self._report_target_stage(
                        "FAILED", "RELEASE", now_ns,
                        terminal=True,
                        reason=("release_ack_failed:%s" %
                                (message.reason or "unknown")),
                        evidence_source=source,
                    )
                    transaction.phase = "TERMINAL"
                    self._publish_alignment_context(False, now_ns)
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("release_result", error)

    def _on_control_state(self, message):
        with self._lock:
            try:
                now_ns = self._now_ns()
                self._expire_handoff_if_due(now_ns)
                self._control_state = int(message.data)
                self._control_state_receipt_ns = now_ns
                transaction = self._transaction
                if (transaction is not None and
                        transaction.phase == "ALIGN_COMMAND_SENT" and
                        self._control_state == 2):
                    self._mark_alignment_started(
                        transaction, now_ns,
                        "patrol_control_alignment_accepted",
                        "patrol_control_state",
                    )
                landing = self._landing
                if (landing is not None and not landing.started and
                        self._control_state == 3 and
                        now_ns >= landing.command_sent_ns):
                    outcome = self._executor.report_landing(
                        landing.decision.decision_seq,
                        now_ns,
                        "STARTED",
                        False,
                        "patrol_control_landing_accepted",
                    )
                    if not outcome.accepted:
                        raise RuntimeError(outcome.reason)
                    self._apply_outcome(outcome)
                    landing.started = True
                    self._landing_settle.reset("awaiting_landed_state")
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("control_state", error)

    def _on_align_mode(self, message):
        with self._lock:
            try:
                self._align_mode = str(message.data).strip()
                self._align_mode_receipt_ns = self._now_ns()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("align_mode", error)

    def _on_landed_state(self, message):
        with self._lock:
            try:
                self._landed_state = int(message.landed_state)
                self._landed_state_receipt_ns = self._now_ns()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("landed_state", error)

    def _on_timer(self, _event):
        with self._lock:
            try:
                if self._output_enabled:
                    now_ns = self._now_ns()
                    outcome = self._executor.tick(now_ns)
                    self._apply_outcome(outcome)
                    self._expire_handoff_if_due(now_ns)
                    self._try_begin_alignment(now_ns)
                    if (self._transaction is not None and
                            self._transaction.phase in (
                                "ALIGN_COMMAND_SENT", "ALIGNMENT")):
                        self._publish_alignment_context(True, now_ns)
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
            "target_transaction": {
                "decision_seq": (
                    self._transaction.decision.decision_seq
                    if self._transaction is not None else 0),
                "phase": (
                    self._transaction.phase
                    if self._transaction is not None else "IDLE"),
                "strict_context_observed": bool(
                    self._transaction is not None and
                    self._transaction.strict_evidence_stamp_ns > 0),
                "recovery_settle": {
                    "reason": self._recovery_settle.result.reason,
                    "elapsed_sec": round(
                        self._recovery_settle.result.elapsed_ns / 1e9, 3),
                    "displacement_m": round(
                        self._recovery_settle.result.displacement_m, 4),
                },
            },
            "landing_decision_seq": (
                self._landing.decision.decision_seq
                if self._landing is not None else 0),
            "landing_settle": {
                "reason": self._landing_settle.result.reason,
                "elapsed_sec": round(
                    self._landing_settle.result.elapsed_ns / 1e9, 3),
                "displacement_m": round(
                    self._landing_settle.result.displacement_m, 4),
            },
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
