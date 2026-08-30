"""Deterministic, ROS-free execution policy for navigation motion decisions.

The class in this module deliberately stops at planner motion. A ROS adapter
publishes its selected goal and exposes LAND/HOLD/ABORT/target-stage handoff
states to separate executors. This module never imports ROS and never
operates a flight mode, payload mechanism or actuator.

All clocks and source stamps are integer nanoseconds.  Decision identity is
copied into an immutable ``ExecutionEvent`` when the decision is accepted, so
later planner or odometry messages cannot rewrite transaction identity.
"""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Dict, Optional, Tuple


MOTION_COMMANDS = frozenset((
    "SEARCH", "RESUME", "APPROACH", "RETURN_HOME",
))
EXTERNAL_COMMANDS = frozenset(("LAND", "ABORT", "HOLD"))
REJECTED_COMMANDS = frozenset(("ALIGN",))
KNOWN_COMMANDS = MOTION_COMMANDS | EXTERNAL_COMMANDS | REJECTED_COMMANDS

PLANNER_STATUSES = (
    "ACCEPTED",
    "PLANNING",
    "TRAJECTORY_READY",
    "REPLANNING",
    "TRAJECTORY_FINISHED",
    "FAILED_ATTEMPT",
    "CANCELLED",
)
_PLANNER_STATUS_BY_VALUE = dict(enumerate(PLANNER_STATUSES))

def _integer(name: str, value: int, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError("%s must be an integer" % name)
    result = int(value)
    if result < minimum:
        raise ValueError("%s must be >= %d" % (name, minimum))
    return result


def _finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("%s must be numeric" % name)
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % name)
    return result


@dataclass(frozen=True)
class MotionGoal:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def __post_init__(self):
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("goal frame_id must not be empty")
        object.__setattr__(self, "x", _finite("goal x", self.x))
        object.__setattr__(self, "y", _finite("goal y", self.y))
        object.__setattr__(self, "z", _finite("goal z", self.z))
        quaternion = [_finite("goal quaternion", value) for value in
                      (self.qx, self.qy, self.qz, self.qw)]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-9:
            raise ValueError("goal orientation must be a valid quaternion")
        for name, value in zip(("qx", "qy", "qz", "qw"), quaternion):
            object.__setattr__(self, name, value / norm)


@dataclass(frozen=True)
class SequencedMotionGoal:
    goal_seq: int
    goal: MotionGoal

    def __post_init__(self):
        object.__setattr__(
            self, "goal_seq", _integer("goal sequence", self.goal_seq, 1))
        if not isinstance(self.goal, MotionGoal):
            raise ValueError("sequenced goal must contain a MotionGoal")


@dataclass(frozen=True)
class TargetIdentity:
    target_id: int
    first_seen_ns: int
    observation_ns: int
    class_name: str
    attempt: int
    payload_slot: int

    def __post_init__(self):
        object.__setattr__(
            self, "target_id", _integer("target_id", self.target_id, 0))
        object.__setattr__(self, "first_seen_ns", _integer(
            "target first_seen_ns", self.first_seen_ns, 1))
        object.__setattr__(self, "observation_ns", _integer(
            "target observation_ns", self.observation_ns, 1))
        if self.observation_ns < self.first_seen_ns:
            raise ValueError("target observation precedes first_seen")
        if not isinstance(self.class_name, str) or not self.class_name.strip():
            raise ValueError("target class_name must not be empty")
        object.__setattr__(
            self, "attempt", _integer("target attempt", self.attempt, 1))
        object.__setattr__(self, "payload_slot", _integer(
            "target payload_slot", self.payload_slot, 1))


@dataclass(frozen=True)
class MotionDecision:
    mission_id: str
    decision_seq: int
    issued_at_ns: int
    deadline_ns: int
    command: str
    class_profile: str
    goal: Optional[MotionGoal] = None
    target: Optional[TargetIdentity] = None

    def __post_init__(self):
        if not isinstance(self.mission_id, str) or not self.mission_id.strip():
            raise ValueError("mission_id must not be empty")
        object.__setattr__(self, "decision_seq", _integer(
            "decision_seq", self.decision_seq, 1))
        object.__setattr__(self, "issued_at_ns", _integer(
            "issued_at_ns", self.issued_at_ns, 1))
        object.__setattr__(self, "deadline_ns", _integer(
            "deadline_ns", self.deadline_ns, 1))
        if self.deadline_ns <= self.issued_at_ns:
            raise ValueError("decision deadline must follow issue time")
        if not isinstance(self.command, str):
            raise ValueError("command must be a string")
        object.__setattr__(self, "command", self.command.strip().upper())
        if self.command not in KNOWN_COMMANDS:
            raise ValueError("unsupported navigation command")
        if (not isinstance(self.class_profile, str) or
                not self.class_profile.strip()):
            raise ValueError("class_profile must not be empty")
        if self.command in MOTION_COMMANDS and self.goal is None:
            raise ValueError("motion command requires a goal")
        if self.command in EXTERNAL_COMMANDS and self.goal is not None:
            raise ValueError("external command must not carry a planner goal")
        if self.command == "APPROACH" and self.target is None:
            raise ValueError("APPROACH requires frozen target identity")
        if self.command not in ("APPROACH", "ALIGN") and self.target is not None:
            raise ValueError("only APPROACH may carry target identity")


@dataclass(frozen=True)
class PlannerStatusEvent:
    event_seq: int
    goal_seq: int
    status: str
    stamp_ns: int
    requested_goal: SequencedMotionGoal
    effective_goal: SequencedMotionGoal
    distance_to_goal: float
    planning_attempt: int = 0
    reason: str = ""

    def __post_init__(self):
        object.__setattr__(
            self, "event_seq", _integer("planner event_seq", self.event_seq, 1))
        object.__setattr__(
            self, "goal_seq", _integer("planner goal_seq", self.goal_seq, 1))
        object.__setattr__(
            self, "stamp_ns", _integer("planner stamp_ns", self.stamp_ns, 1))
        object.__setattr__(self, "planning_attempt", _integer(
            "planner planning_attempt", self.planning_attempt, 0))
        status = self.status
        if isinstance(status, Integral) and not isinstance(status, bool):
            status = _PLANNER_STATUS_BY_VALUE.get(int(status), "")
        if not isinstance(status, str):
            raise ValueError("planner status must be a string or known value")
        status = status.strip().upper()
        if status not in PLANNER_STATUSES:
            raise ValueError("unknown planner status")
        object.__setattr__(self, "status", status)
        if not isinstance(self.reason, str):
            raise ValueError("planner reason must be a string")
        if not isinstance(self.requested_goal, SequencedMotionGoal):
            raise ValueError("requested_goal must be sequenced")
        if not isinstance(self.effective_goal, SequencedMotionGoal):
            raise ValueError("effective_goal must be sequenced")
        if (isinstance(self.distance_to_goal, bool) or
                not isinstance(self.distance_to_goal, Real)):
            raise ValueError("distance_to_goal must be numeric")
        distance = float(self.distance_to_goal)
        if math.isinf(distance) or (math.isfinite(distance) and distance < 0.0):
            raise ValueError("distance_to_goal must be non-negative or NaN")
        object.__setattr__(self, "distance_to_goal", distance)


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    frame_id: str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float

    def __post_init__(self):
        object.__setattr__(
            self, "stamp_ns", _integer("odom stamp_ns", self.stamp_ns, 1))
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("odom frame_id must not be empty")
        for field_name in ("x", "y", "z", "vx", "vy", "vz"):
            object.__setattr__(
                self, field_name, _finite("odom %s" % field_name,
                                         getattr(self, field_name)))


@dataclass(frozen=True)
class PlannerMotionConfig:
    executor_id: str = "navigation_planner_executor"
    mission_frame: str = "camera_init"
    max_z_m: float = 4.0
    source_future_tolerance_ns: int = 100_000_000
    planner_accept_timeout_ns: int = 2_000_000_000
    max_effective_goal_offset_m: float = 1.10
    max_planning_attempts: int = 20
    arrival_distance_m: float = 0.30
    arrival_speed_mps: float = 0.20
    arrival_dwell_ns: int = 500_000_000
    odom_max_age_ns: int = 200_000_000

    def __post_init__(self):
        if not isinstance(self.executor_id, str) or not self.executor_id.strip():
            raise ValueError("executor_id must not be empty")
        if (not isinstance(self.mission_frame, str) or
                not self.mission_frame.strip()):
            raise ValueError("mission_frame must not be empty")
        max_z = _finite("max_z_m", self.max_z_m)
        if max_z <= 0.0 or max_z > 4.0:
            raise ValueError("max_z_m exceeds the competition limit")
        object.__setattr__(self, "max_z_m", max_z)
        object.__setattr__(self, "source_future_tolerance_ns", _integer(
            "source_future_tolerance_ns", self.source_future_tolerance_ns, 0))
        object.__setattr__(self, "planner_accept_timeout_ns", _integer(
            "planner_accept_timeout_ns", self.planner_accept_timeout_ns, 1))
        effective_offset = _finite(
            "max_effective_goal_offset_m", self.max_effective_goal_offset_m)
        if effective_offset <= 0.0 or effective_offset > 1.10:
            raise ValueError("effective goal offset exceeds the audited bound")
        object.__setattr__(
            self, "max_effective_goal_offset_m", effective_offset)
        object.__setattr__(self, "max_planning_attempts", _integer(
            "max_planning_attempts", self.max_planning_attempts, 1))
        distance = _finite("arrival_distance_m", self.arrival_distance_m)
        speed = _finite("arrival_speed_mps", self.arrival_speed_mps)
        if distance <= 0.0 or speed < 0.0:
            raise ValueError("arrival thresholds are invalid")
        object.__setattr__(self, "arrival_distance_m", distance)
        object.__setattr__(self, "arrival_speed_mps", speed)
        object.__setattr__(self, "arrival_dwell_ns", _integer(
            "arrival_dwell_ns", self.arrival_dwell_ns, 1))
        object.__setattr__(self, "odom_max_age_ns", _integer(
            "odom_max_age_ns", self.odom_max_age_ns, 1))


@dataclass(frozen=True)
class ExecutionEvent:
    mission_id: str
    executor_id: str
    event_seq: int
    event_stamp_ns: int
    decision_seq: int
    command: str
    status: str
    stage: str
    terminal: bool
    retryable: bool
    payload_committed: bool
    has_target: bool
    target_id: int
    target_first_seen_ns: int
    target_class: str
    attempt: int
    payload_slot: int
    reason: str
    evidence_source: str


@dataclass(frozen=True)
class ExecutorSnapshot:
    faulted: bool
    fault_reason: str
    mission_id: str
    last_decision_seq: int
    active_decision_seq: int
    active_command: str
    active_terminal: bool
    active_handed_off: bool
    planner_accepted: bool
    trajectory_ready: bool
    trajectory_finished: bool
    dwell_start_ns: int
    last_planner_event_seq: int
    executor_event_seq: int
    awaiting_cancel_goal_seq: int


@dataclass(frozen=True)
class ExecutorOutcome:
    accepted: bool
    reason: str
    events: Tuple[ExecutionEvent, ...]
    snapshot: ExecutorSnapshot
    planner_goal: Optional[MotionDecision] = None
    handoff: str = ""


@dataclass
class _GoalLifecycle:
    decision: MotionDecision
    dispatch_ns: int
    effective_goal: Optional[MotionGoal] = None
    planning_attempt: int = 0
    planner_accepted: bool = False
    trajectory_ready: bool = False
    trajectory_finished: bool = False
    finished_stamp_ns: int = 0
    dwell_start_ns: int = 0
    last_qualified_odom_ns: int = 0
    terminal: bool = False
    handed_off: bool = False
    retired: bool = False


class PlannerMotionExecutor:
    """Reduce one active motion, planner facts and odometry into results."""

    def __init__(self, config: PlannerMotionConfig = PlannerMotionConfig()):
        if not isinstance(config, PlannerMotionConfig):
            raise TypeError("config must be PlannerMotionConfig")
        self.config = config
        self._mission_id = ""
        self._last_decision: Optional[MotionDecision] = None
        self._goals: Dict[int, _GoalLifecycle] = {}
        self._last_planner_event: Optional[PlannerStatusEvent] = None
        self._last_decision_seq = 0
        self._last_planner_event_seq = 0
        self._executor_event_seq = 0
        self._last_now_ns = 0
        self._last_odom: Optional[OdomSample] = None
        self._active: Optional[_GoalLifecycle] = None
        self._awaiting_cancel_goal_seq = 0

    def snapshot(self) -> ExecutorSnapshot:
        active = self._active
        return ExecutorSnapshot(
            faulted=False,
            fault_reason="",
            mission_id=self._mission_id,
            last_decision_seq=self._last_decision_seq,
            active_decision_seq=(active.decision.decision_seq if active else 0),
            active_command=(active.decision.command if active else ""),
            active_terminal=(active.terminal if active else False),
            active_handed_off=(active.handed_off if active else False),
            planner_accepted=(active.planner_accepted if active else False),
            trajectory_ready=(active.trajectory_ready if active else False),
            trajectory_finished=(active.trajectory_finished if active else False),
            dwell_start_ns=(active.dwell_start_ns if active else 0),
            last_planner_event_seq=self._last_planner_event_seq,
            executor_event_seq=self._executor_event_seq,
            awaiting_cancel_goal_seq=self._awaiting_cancel_goal_seq,
        )

    def _outcome(self, accepted: bool, reason: str,
                 events: Tuple[ExecutionEvent, ...] = (),
                 planner_goal: Optional[MotionDecision] = None,
                 handoff: str = "") -> ExecutorOutcome:
        return ExecutorOutcome(
            accepted=accepted,
            reason=reason,
            events=tuple(events),
            snapshot=self.snapshot(),
            planner_goal=planner_goal,
            handoff=handoff,
        )

    def _result(self, state: _GoalLifecycle, now_ns: int, status: str,
                stage: str, terminal: bool, retryable: bool,
                reason: str) -> ExecutionEvent:
        self._executor_event_seq += 1
        decision = state.decision
        target = decision.target
        return ExecutionEvent(
            mission_id=decision.mission_id,
            executor_id=self.config.executor_id,
            event_seq=self._executor_event_seq,
            event_stamp_ns=now_ns,
            decision_seq=decision.decision_seq,
            command=decision.command,
            status=status,
            stage=stage,
            terminal=terminal,
            retryable=retryable,
            payload_committed=False,
            has_target=target is not None,
            target_id=(target.target_id if target else 0),
            target_first_seen_ns=(target.first_seen_ns if target else 0),
            target_class=(target.class_name if target else ""),
            attempt=(target.attempt if target else 0),
            payload_slot=(target.payload_slot if target else 0),
            reason=reason,
            evidence_source="planner_motion_executor",
        )

    def _validate_now(self, now_ns: int) -> Optional[ExecutorOutcome]:
        try:
            value = _integer("now_ns", now_ns, 1)
        except ValueError:
            return self._fail_closed("executor_clock_invalid")
        if self._last_now_ns and value < self._last_now_ns:
            return self._fail_closed("executor_clock_rollback")
        self._last_now_ns = value
        return None

    def _fail_closed(self, reason: str) -> ExecutorOutcome:
        events = ()
        active = self._active
        if (active is not None and not active.terminal and
                not active.handed_off):
            active.terminal = True
            active.retired = True
            events = (self._result(
                active,
                self._last_now_ns or active.decision.issued_at_ns,
                "FAILED",
                "PLANNER",
                True,
                False,
                reason,
            ),)
        return self._outcome(False, reason, events=events,
                             handoff="STOP_REQUIRED" if active else "")

    def _expire_if_due(self, now_ns: int) -> Optional[ExecutorOutcome]:
        active = self._active
        if (active is None or active.terminal or active.handed_off or
                active.decision.command not in MOTION_COMMANDS or
                now_ns < active.decision.deadline_ns):
            return None
        active.terminal = True
        active.retired = True
        event = self._result(
            active,
            now_ns,
            "TIMED_OUT",
            "PLANNER",
            True,
            active.decision.command in ("SEARCH", "RESUME", "APPROACH"),
            "decision_deadline_reached",
        )
        return self._outcome(True, "decision_timed_out", events=(event,),
                             handoff="CANCEL_REQUIRED")

    def _expire_acceptance_if_due(
            self, now_ns: int) -> Optional[ExecutorOutcome]:
        active = self._active
        if (active is None or active.terminal or active.handed_off or
                active.decision.command not in MOTION_COMMANDS or
                active.planner_accepted or
                now_ns < active.dispatch_ns +
                self.config.planner_accept_timeout_ns):
            return None
        active.terminal = True
        active.retired = True
        event = self._result(
            active,
            now_ns,
            "TIMED_OUT",
            "PLANNER",
            True,
            active.decision.command in ("SEARCH", "RESUME", "APPROACH"),
            "planner_accept_timeout",
        )
        return self._outcome(True, "planner_accept_timed_out", events=(event,),
                             handoff="CANCEL_REQUIRED")

    def _prepare(self, now_ns: int) -> Optional[ExecutorOutcome]:
        invalid = self._validate_now(now_ns)
        if invalid is not None:
            return invalid
        expired = self._expire_if_due(int(now_ns))
        if expired is not None:
            return expired
        return self._expire_acceptance_if_due(int(now_ns))

    def _validate_decision_contract(
            self, decision: MotionDecision) -> Optional[str]:
        if decision.command == "ALIGN":
            return "align_not_owned_by_motion_executor"
        goal = decision.goal
        if goal is not None:
            if goal.frame_id != self.config.mission_frame:
                return "decision_goal_frame_mismatch"
            if goal.z < 0.0 or goal.z > self.config.max_z_m:
                return "decision_goal_height_invalid"
        target = decision.target
        return None

    def submit_decision(self, decision: MotionDecision,
                        now_ns: int) -> ExecutorOutcome:
        """Accept one continuous decision or replay an identical decision."""

        if not isinstance(decision, MotionDecision):
            raise TypeError("decision must be MotionDecision")
        invalid_now = self._validate_now(now_ns)
        if invalid_now is not None:
            return invalid_now

        prior = self._last_decision
        if (prior is not None and
                prior.mission_id == decision.mission_id and
                prior.decision_seq == decision.decision_seq):
            if prior == decision:
                expired = self._expire_if_due(int(now_ns))
                if expired is not None:
                    return expired
                accept_expired = self._expire_acceptance_if_due(int(now_ns))
                if accept_expired is not None:
                    return accept_expired
                return self._outcome(True, "decision_idempotent")
            return self._fail_closed("decision_sequence_conflict")

        contract_error = self._validate_decision_contract(decision)
        if contract_error is not None:
            return self._fail_closed(contract_error)

        if (prior is not None and prior.mission_id == decision.mission_id and
                decision.decision_seq < prior.decision_seq):
            return self._outcome(False, "stale_decision_ignored")
        if int(now_ns) < decision.issued_at_ns:
            return self._fail_closed("decision_from_future")
        if int(now_ns) >= decision.deadline_ns:
            return self._fail_closed("decision_received_after_deadline")

        active = self._active
        replacement_requires_cancel = (
            active is not None and
            active.decision.command in MOTION_COMMANDS and
            active.decision.decision_seq != decision.decision_seq and
            not active.trajectory_finished
        )
        if replacement_requires_cancel:
            self._awaiting_cancel_goal_seq = active.decision.decision_seq
        if active is not None:
            active.retired = True

        state = _GoalLifecycle(decision=decision, dispatch_ns=int(now_ns))
        if decision.command in EXTERNAL_COMMANDS:
            state.handed_off = True
        self._active = state
        self._mission_id = decision.mission_id
        self._last_decision = decision
        self._last_decision_seq = decision.decision_seq

        if decision.command in MOTION_COMMANDS:
            self._goals[decision.decision_seq] = state
            keep = {decision.decision_seq}
            if self._awaiting_cancel_goal_seq:
                keep.add(self._awaiting_cancel_goal_seq)
            self._goals = {
                seq: goal_state for seq, goal_state in self._goals.items()
                if seq in keep
            }
            accepted_event = self._result(
                state,
                int(now_ns),
                "ACCEPTED",
                "DISPATCH",
                False,
                False,
                "planner_goal_dispatched",
            )
            return self._outcome(True, "planner_goal_intent",
                                 events=(accepted_event,),
                                 planner_goal=decision)
        return self._outcome(True, "external_command_handoff",
                             handoff=decision.command)

    def apply_planner_status(self, event: PlannerStatusEvent,
                             now_ns: int) -> ExecutorOutcome:
        """Reduce one globally sequenced planner telemetry event."""

        if not isinstance(event, PlannerStatusEvent):
            raise TypeError("event must be PlannerStatusEvent")
        prepared = self._prepare(now_ns)
        if prepared is not None:
            return prepared

        prior = self._last_planner_event
        if prior is not None and prior.event_seq == event.event_seq:
            if prior == event:
                return self._outcome(True, "planner_event_idempotent")
            return self._fail_closed("planner_event_sequence_conflict")
        if event.event_seq <= self._last_planner_event_seq:
            return self._outcome(False, "stale_planner_event_ignored")
        if (event.stamp_ns > int(now_ns) +
                self.config.source_future_tolerance_ns):
            return self._fail_closed("planner_event_from_future")

        state = self._goals.get(event.goal_seq)
        if state is None:
            return self._outcome(False, "foreign_planner_goal_ignored")
        if event.stamp_ns < state.dispatch_ns:
            return self._fail_closed("planner_event_precedes_dispatch")
        requested = event.requested_goal
        effective = event.effective_goal
        if (requested.goal_seq != event.goal_seq or
                effective.goal_seq != event.goal_seq):
            return self._fail_closed("planner_goal_sequence_mismatch")
        if requested.goal != state.decision.goal:
            return self._fail_closed("planner_requested_goal_mismatch")
        if (effective.goal.frame_id != self.config.mission_frame or
                effective.goal.frame_id != requested.goal.frame_id):
            return self._fail_closed("planner_effective_goal_frame_mismatch")
        if (effective.goal.z < 0.0 or
                effective.goal.z > self.config.max_z_m):
            return self._fail_closed("planner_effective_goal_height_invalid")
        effective_offset = math.sqrt(
            (effective.goal.x - requested.goal.x) ** 2 +
            (effective.goal.y - requested.goal.y) ** 2 +
            (effective.goal.z - requested.goal.z) ** 2
        )
        if effective_offset > self.config.max_effective_goal_offset_m:
            return self._fail_closed("planner_effective_goal_offset_exceeded")

        self._last_planner_event = event
        self._last_planner_event_seq = event.event_seq
        if state.retired:
            if (event.goal_seq == self._awaiting_cancel_goal_seq and
                    event.status == "CANCELLED"):
                self._awaiting_cancel_goal_seq = 0
                return self._outcome(True, "replacement_cancel_confirmed")
            return self._outcome(True, "retired_goal_event_ignored")
        if state is not self._active:
            return self._fail_closed("planner_event_not_active_goal")

        status = event.status
        if status == "ACCEPTED":
            if self._awaiting_cancel_goal_seq:
                return self._fail_closed(
                    "replacement_accepted_before_cancel")
            if event.planning_attempt != 0:
                return self._fail_closed("planner_attempt_invalid_for_accept")
            if state.planner_accepted or state.trajectory_finished:
                return self._fail_closed("planner_accepted_out_of_order")
            state.planner_accepted = True
            state.effective_goal = effective.goal
            return self._outcome(True, "planner_goal_accepted", events=(
                self._result(state, int(now_ns), "STARTED", "PLANNER",
                             False, False, "planner_goal_accepted"),))
        if not state.planner_accepted:
            return self._fail_closed("planner_status_before_accepted")
        if state.trajectory_finished:
            return self._fail_closed("planner_status_after_finished")
        if status in ("PLANNING", "REPLANNING"):
            expected_attempt = state.planning_attempt + 1
            if event.planning_attempt != expected_attempt:
                return self._fail_closed("planner_attempt_not_monotonic")
            if event.planning_attempt > self.config.max_planning_attempts:
                return self._fail_closed("planner_attempt_limit_exceeded")
            state.planning_attempt = event.planning_attempt
            state.trajectory_ready = False
            state.dwell_start_ns = 0
            state.last_qualified_odom_ns = 0
            state.effective_goal = effective.goal
        elif (event.planning_attempt != state.planning_attempt or
              event.planning_attempt < 1):
            return self._fail_closed("planner_attempt_inconsistent")
        else:
            state.effective_goal = effective.goal
        if status == "PLANNING":
            return self._outcome(True, "planner_attempt_started")
        if status == "REPLANNING":
            return self._outcome(True, "planner_replanning")
        if status == "TRAJECTORY_READY":
            state.trajectory_ready = True
            return self._outcome(True, "planner_trajectory_ready", events=(
                self._result(state, int(now_ns), "PROGRESS", "PLANNER",
                             False, False, "planner_trajectory_ready"),))
        if status == "FAILED_ATTEMPT":
            return self._outcome(True, "planner_attempt_failed_nonterminal",
                events=(self._result(
                    state, int(now_ns), "PROGRESS", "PLANNER", False, False,
                    "planner_attempt_failed_nonterminal"),))
        if status == "TRAJECTORY_FINISHED":
            if not state.trajectory_ready:
                return self._fail_closed("trajectory_finished_without_ready")
            state.trajectory_finished = True
            state.finished_stamp_ns = event.stamp_ns
            state.dwell_start_ns = 0
            state.last_qualified_odom_ns = 0
            return self._outcome(True, "planner_trajectory_finished")
        if status == "CANCELLED":
            return self._fail_closed("active_planner_goal_cancelled")
        return self._fail_closed("planner_status_unhandled")

    def _reset_dwell(self, state: _GoalLifecycle) -> None:
        state.dwell_start_ns = 0
        state.last_qualified_odom_ns = 0

    def _complete_arrival(self, state: _GoalLifecycle,
                          now_ns: int) -> ExecutorOutcome:
        decision = state.decision
        if decision.command == "APPROACH":
            state.handed_off = True
            event = self._result(
                state,
                now_ns,
                "PROGRESS",
                "PLANNER",
                False,
                False,
                "approach_arrival_confirmed",
            )
            return self._outcome(True, "target_transaction_handoff",
                                 events=(event,), handoff="TARGET_TRANSACTION")
        state.terminal = True
        state.retired = True
        event = self._result(
            state,
            now_ns,
            "SUCCEEDED",
            "PLANNER",
            True,
            False,
            "motion_arrival_confirmed",
        )
        return self._outcome(
            True, "motion_succeeded", events=(event,))

    def apply_odom(self, sample: OdomSample, now_ns: int) -> ExecutorOutcome:
        """Use fresh same-frame odometry to confirm distance/speed/dwell."""

        if not isinstance(sample, OdomSample):
            raise TypeError("sample must be OdomSample")
        prepared = self._prepare(now_ns)
        if prepared is not None:
            return prepared

        if (sample.stamp_ns > int(now_ns) +
                self.config.source_future_tolerance_ns):
            if self._active is not None:
                self._reset_dwell(self._active)
            return self._outcome(False, "odom_from_future")
        if self._last_odom is not None:
            if sample.stamp_ns < self._last_odom.stamp_ns:
                if self._active is not None:
                    self._reset_dwell(self._active)
                return self._outcome(False, "odom_out_of_order")
            if sample.stamp_ns == self._last_odom.stamp_ns:
                if sample == self._last_odom:
                    return self._outcome(True, "odom_idempotent")
                if self._active is not None:
                    self._reset_dwell(self._active)
                return self._outcome(False, "odom_stamp_conflict")
        self._last_odom = sample

        state = self._active
        if (state is None or state.terminal or state.handed_off or
                state.decision.command not in MOTION_COMMANDS):
            return self._outcome(True, "odom_no_active_motion")
        if (max(0, int(now_ns) - sample.stamp_ns) >
                self.config.odom_max_age_ns):
            self._reset_dwell(state)
            return self._outcome(False, "odom_stale")
        if not state.trajectory_finished:
            self._reset_dwell(state)
            return self._outcome(True, "waiting_for_trajectory_finished")
        if sample.stamp_ns < state.finished_stamp_ns:
            self._reset_dwell(state)
            return self._outcome(False, "odom_precedes_trajectory_finish")

        goal = state.effective_goal
        if goal is None:
            return self._fail_closed("planner_effective_goal_missing")
        if sample.frame_id != goal.frame_id:
            self._reset_dwell(state)
            return self._outcome(False, "odom_goal_frame_mismatch")

        distance = math.sqrt(
            (sample.x - goal.x) ** 2 +
            (sample.y - goal.y) ** 2 +
            (sample.z - goal.z) ** 2
        )
        speed = math.sqrt(sample.vx ** 2 + sample.vy ** 2 + sample.vz ** 2)
        if (distance > self.config.arrival_distance_m or
                speed > self.config.arrival_speed_mps):
            self._reset_dwell(state)
            return self._outcome(True, "arrival_threshold_not_met")

        if (state.last_qualified_odom_ns and
                sample.stamp_ns - state.last_qualified_odom_ns >
                self.config.odom_max_age_ns):
            self._reset_dwell(state)
        if state.dwell_start_ns == 0:
            state.dwell_start_ns = sample.stamp_ns
        state.last_qualified_odom_ns = sample.stamp_ns
        if (sample.stamp_ns - state.dwell_start_ns <
                self.config.arrival_dwell_ns):
            return self._outcome(True, "arrival_dwell_pending")
        return self._complete_arrival(state, int(now_ns))

    def tick(self, now_ns: int) -> ExecutorOutcome:
        """Apply exclusive deadline and freshness checks without sensor data."""

        prepared = self._prepare(now_ns)
        if prepared is not None:
            return prepared
        state = self._active
        if (state is not None and state.dwell_start_ns and
                (not state.last_qualified_odom_ns or
                 int(now_ns) - state.last_qualified_odom_ns >
                 self.config.odom_max_age_ns)):
            self._reset_dwell(state)
            return self._outcome(True, "arrival_dwell_reset_stale_odom")
        return self._outcome(True, "executor_pending")


__all__ = [
    "ExecutionEvent",
    "ExecutorOutcome",
    "ExecutorSnapshot",
    "MotionDecision",
    "MotionGoal",
    "OdomSample",
    "PlannerMotionConfig",
    "PlannerMotionExecutor",
    "PlannerStatusEvent",
    "SequencedMotionGoal",
    "TargetIdentity",
]
