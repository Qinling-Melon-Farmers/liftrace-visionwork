"""Pure navigation mission policy for candidate scheduling and results.

ROS message conversion, clocks and publishers stay in the runtime node.  The
core only consumes immutable facts and explicit timestamps, which makes every
queue and retry transition deterministic and replayable.
"""

from dataclasses import dataclass, replace
from enum import Enum
import math
from numbers import Real
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .profile_policy import CompetitionProfile


CONFIRMED_STATE = 2
MAX_FLIGHT_Z = 4.0
MAX_MISSION_DURATION = 600.0
LATEST_FORCED_RETURN = 510.0
R2026_WEIGHTS = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "red_cross": 10.0,
}
TARGET_STAGES = (
    "DISPATCH",
    "PLANNER",
    "CAPTURE",
    "ALIGNMENT",
    "RELEASE",
    "RECOVERY",
)
UNCERTAIN_RELEASE_STAGES = frozenset(("RELEASE", "RECOVERY"))


class CandidateStatus(Enum):
    PENDING = "PENDING"
    EXECUTING = "EXECUTING"
    COOLDOWN = "COOLDOWN"
    SUCCEEDED = "SUCCEEDED"
    EXHAUSTED = "EXHAUSTED"
    SKIPPED = "SKIPPED"


class MissionPhase(Enum):
    INIT = "INIT"
    SEARCH = "SEARCH"
    EXECUTING = "EXECUTING"
    POST_DELIVERY_ROUTE = "POST_DELIVERY_ROUTE"
    RETURN_HOME = "RETURN_HOME"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


class SlotStatus(Enum):
    FREE = "FREE"
    RESERVED = "RESERVED"
    COMMITTED = "COMMITTED"
    QUARANTINED = "QUARANTINED"


@dataclass(frozen=True, order=True)
class CandidateKey:
    target_id: int
    first_seen_ns: int


@dataclass(frozen=True)
class CandidateSnapshot:
    target_id: int
    class_name: str
    class_confidence: float
    geometry_confidence: float
    map_quality: float
    x: float
    y: float
    z: float
    map_frame: str
    state: int
    consecutive_observe_count: int
    map_valid: bool
    association_valid: bool
    reject_reason: str
    transform_age_sec: float
    first_seen_ns: int
    last_seen_ns: int

    @property
    def key(self) -> CandidateKey:
        return CandidateKey(int(self.target_id), int(self.first_seen_ns))


@dataclass
class CandidateEntry:
    snapshot: CandidateSnapshot
    reserved_snapshot: Optional[CandidateSnapshot] = None
    status: CandidateStatus = CandidateStatus.PENDING
    attempts: int = 0
    cooldown_until: float = 0.0
    last_result: str = ""
    payload_slot: int = 0
    reachable: Optional[bool] = None
    retry_forbidden: bool = False
    invalidated_reason: str = ""


@dataclass
class PayloadSlot:
    index: int
    status: SlotStatus = SlotStatus.FREE
    candidate_key: Optional[CandidateKey] = None


@dataclass(frozen=True)
class CandidateValidation:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class GoalSnapshot:
    frame_id: str
    x: float
    y: float
    z: float

    def __post_init__(self):
        if not self.frame_id.strip():
            raise ValueError("goal frame must not be empty")
        if not all(math.isfinite(float(value)) for value in
                   (self.x, self.y, self.z)):
            raise ValueError("goal coordinates must be finite")
        if self.z < 0.0 or self.z > MAX_FLIGHT_Z:
            raise ValueError("goal altitude is outside the competition limit")


@dataclass(frozen=True)
class CoreAction:
    command: str
    reason: str
    decision_seq: int
    profile_name: str
    issued_at: float
    deadline_at: float
    goal: Optional[GoalSnapshot] = None
    target_snapshot: Optional[CandidateSnapshot] = None
    candidate_key: Optional[CandidateKey] = None
    target_class: str = ""
    attempt: int = 0
    payload_slot: int = 0

    @property
    def has_goal(self) -> bool:
        return self.goal is not None

    @property
    def has_target(self) -> bool:
        return self.candidate_key is not None


@dataclass(frozen=True)
class ResultEvent:
    mission_id: str
    executor_id: str
    event_seq: int
    event_stamp_ns: int
    decision_seq: int
    command: str
    has_target: bool
    target_id: int
    target_first_seen_ns: int
    target_class: str
    attempt: int
    payload_slot: int
    status: str
    stage: str
    terminal: bool
    retryable: bool
    payload_committed: bool
    reason: str
    evidence_source: str

    @property
    def candidate_key(self) -> CandidateKey:
        return CandidateKey(int(self.target_id), int(self.target_first_seen_ns))

    @property
    def event_time(self) -> float:
        return float(self.event_stamp_ns) / 1.0e9


@dataclass(frozen=True)
class MissionConfig:
    mission_frame: str = "camera_init"
    candidate_max_age: float = 0.5
    transform_max_age: float = 0.5
    min_streak: int = 3
    max_target_z: float = 4.0
    max_attempts: int = 2
    retry_cooldown: float = 20.0
    mission_timeout: float = 600.0
    forced_return_at: float = 510.0
    return_land_reserve: float = 90.0
    delivery_reserve_per_slot: float = 60.0
    path_factor: float = 1.5
    nominal_speed: float = 1.0
    decision_guard: float = 15.0
    approach_altitude: float = 1.2
    return_altitude: float = 2.2
    target_action_timeout: float = 90.0
    motion_action_timeout: float = 60.0
    landing_action_timeout: float = 90.0
    result_future_tolerance: float = 0.1
    home_xy: Tuple[float, float] = (0.0, 0.0)
    post_delivery_route: Tuple[GoalSnapshot, ...] = ()
    post_delivery_route_revision: str = "direct-home-v1"
    landing_xy: Tuple[float, float] = (0.0, 0.0)
    landing_anchor_tolerance: float = 0.15

    def __post_init__(self):
        positive = {
            "candidate_max_age": self.candidate_max_age,
            "transform_max_age": self.transform_max_age,
            "max_target_z": self.max_target_z,
            "max_attempts": self.max_attempts,
            "retry_cooldown": self.retry_cooldown,
            "mission_timeout": self.mission_timeout,
            "forced_return_at": self.forced_return_at,
            "return_land_reserve": self.return_land_reserve,
            "delivery_reserve_per_slot": self.delivery_reserve_per_slot,
            "path_factor": self.path_factor,
            "nominal_speed": self.nominal_speed,
            "approach_altitude": self.approach_altitude,
            "return_altitude": self.return_altitude,
            "target_action_timeout": self.target_action_timeout,
            "motion_action_timeout": self.motion_action_timeout,
            "landing_action_timeout": self.landing_action_timeout,
            "landing_anchor_tolerance": self.landing_anchor_tolerance,
        }
        for name, value in positive.items():
            if (isinstance(value, bool) or not isinstance(value, Real) or
                    not math.isfinite(float(value)) or float(value) <= 0.0):
                raise ValueError("%s must be finite and positive" % name)
        if (isinstance(self.min_streak, bool) or
                not isinstance(self.min_streak, int)):
            raise ValueError("min_streak must be an integer")
        if self.min_streak <= 0:
            raise ValueError("min_streak must be positive")
        if (isinstance(self.max_attempts, bool) or
                not isinstance(self.max_attempts, int)):
            raise ValueError("max_attempts must be an integer")
        if (isinstance(self.decision_guard, bool) or
                not isinstance(self.decision_guard, Real) or
                not math.isfinite(float(self.decision_guard)) or
                self.decision_guard < 0.0):
            raise ValueError("decision_guard must be finite and non-negative")
        if (isinstance(self.result_future_tolerance, bool) or
                not isinstance(self.result_future_tolerance, Real) or
                not math.isfinite(float(self.result_future_tolerance)) or
                self.result_future_tolerance < 0.0):
            raise ValueError(
                "result_future_tolerance must be finite and non-negative")
        if self.forced_return_at >= self.mission_timeout:
            raise ValueError("forced_return_at must precede mission_timeout")
        if self.mission_timeout > MAX_MISSION_DURATION:
            raise ValueError("mission_timeout exceeds the competition limit")
        if self.forced_return_at > LATEST_FORCED_RETURN:
            raise ValueError("forced_return_at exceeds the frozen deadline")
        if self.max_target_z > MAX_FLIGHT_Z:
            raise ValueError("max_target_z exceeds the competition limit")
        if (self.approach_altitude > MAX_FLIGHT_Z or
                self.return_altitude > MAX_FLIGHT_Z):
            raise ValueError("configured flight altitude exceeds the limit")
        if (not isinstance(self.mission_frame, str) or
                not self.mission_frame.strip()):
            raise ValueError("mission_frame must not be empty")
        if (self.mission_timeout - self.forced_return_at <
                self.return_land_reserve):
            raise ValueError(
                "forced return must preserve the return/landing reserve")
        if (not isinstance(self.home_xy, (tuple, list)) or
                len(self.home_xy) != 2 or
                not all(math.isfinite(float(value)) for value in self.home_xy)):
            raise ValueError("home_xy must contain two finite coordinates")
        if (not isinstance(self.landing_xy, (tuple, list)) or
                len(self.landing_xy) != 2 or
                not all(math.isfinite(float(value))
                        for value in self.landing_xy)):
            raise ValueError("landing_xy must contain two finite coordinates")
        if (not isinstance(self.post_delivery_route, (tuple, list)) or
                not all(isinstance(goal, GoalSnapshot)
                        for goal in self.post_delivery_route)):
            raise ValueError(
                "post_delivery_route must contain GoalSnapshot values")
        route = tuple(self.post_delivery_route)
        if any(goal.frame_id != self.mission_frame for goal in route):
            raise ValueError(
                "post_delivery_route frame must match mission_frame")
        if (not isinstance(self.post_delivery_route_revision, str) or
                not self.post_delivery_route_revision.strip()):
            raise ValueError(
                "post_delivery_route_revision must not be empty")
        object.__setattr__(
            self, "home_xy", tuple(float(value) for value in self.home_xy))
        object.__setattr__(
            self, "landing_xy",
            tuple(float(value) for value in self.landing_xy))
        object.__setattr__(self, "post_delivery_route", route)
        if route:
            final_goal = route[-1]
            final_error = math.hypot(
                final_goal.x - self.landing_xy[0],
                final_goal.y - self.landing_xy[1],
            )
            if final_error > self.landing_anchor_tolerance:
                raise ValueError(
                    "post_delivery_route must end at the landing anchor")


def validate_candidate(candidate: CandidateSnapshot, now: float,
                       profile: CompetitionProfile,
                       config: MissionConfig) -> CandidateValidation:
    """Apply the formal fail-closed candidate admission contract."""

    if not profile.allows(candidate.class_name):
        return CandidateValidation(False, "profile_excluded")
    if int(candidate.state) != CONFIRMED_STATE:
        return CandidateValidation(False, "state_not_confirmed")
    if int(candidate.consecutive_observe_count) < config.min_streak:
        return CandidateValidation(False, "streak_too_short")
    if not candidate.map_valid:
        return CandidateValidation(False, "map_invalid")
    if candidate.map_frame != config.mission_frame:
        return CandidateValidation(False, "map_frame_mismatch")
    if not candidate.association_valid:
        return CandidateValidation(False, "association_invalid")
    if candidate.reject_reason:
        return CandidateValidation(False, "candidate_rejected")
    numeric = (
        candidate.class_confidence,
        candidate.geometry_confidence,
        candidate.map_quality,
        candidate.x,
        candidate.y,
        candidate.z,
        candidate.transform_age_sec,
        now,
    )
    if not all(math.isfinite(float(value)) for value in numeric):
        return CandidateValidation(False, "non_finite_value")
    if candidate.first_seen_ns <= 0 or candidate.last_seen_ns <= 0:
        return CandidateValidation(False, "invalid_source_stamp")
    if candidate.first_seen_ns > candidate.last_seen_ns:
        return CandidateValidation(False, "source_stamp_order_invalid")
    age = float(now) - float(candidate.last_seen_ns) / 1.0e9
    if age < 0.0 or age > config.candidate_max_age:
        return CandidateValidation(False, "candidate_stale")
    if (candidate.transform_age_sec < 0.0 or
            candidate.transform_age_sec > config.transform_max_age):
        return CandidateValidation(False, "transform_stale")
    if candidate.map_quality < 0.0:
        return CandidateValidation(False, "map_quality_invalid")
    confidence_values = (
        candidate.class_confidence,
        candidate.geometry_confidence,
        candidate.map_quality,
    )
    if any(value < 0.0 or value > 1.0 for value in confidence_values):
        return CandidateValidation(False, "quality_out_of_range")
    if candidate.z > config.max_target_z:
        return CandidateValidation(False, "target_above_height_limit")
    return CandidateValidation(True, "accepted")


class CandidateQueue:
    """Persistent, result-driven candidate queue for one mission session."""

    def __init__(self, profile: CompetitionProfile, config: MissionConfig):
        self.profile = profile
        self.config = config
        self.entries: Dict[CandidateKey, CandidateEntry] = {}
        self.delivered_classes = set()

    def ingest(self, candidate: CandidateSnapshot,
               now: float) -> CandidateValidation:
        validation = validate_candidate(candidate, now, self.profile,
                                        self.config)
        if not validation.accepted:
            existing = self.entries.get(candidate.key)
            is_newer = (
                existing is not None and
                candidate.last_seen_ns > existing.snapshot.last_seen_ns)
            is_terminal_lifecycle = (
                int(candidate.state) in (3, 4) or
                bool(candidate.reject_reason))
            if is_newer and is_terminal_lifecycle:
                self.invalidate(
                    candidate.key,
                    candidate.reject_reason or validation.reason,
                )
            return validation
        key = candidate.key
        existing = self.entries.get(key)
        if existing is None:
            self.entries[key] = CandidateEntry(snapshot=candidate)
            return validation
        if candidate.last_seen_ns <= existing.snapshot.last_seen_ns:
            return CandidateValidation(False, "observation_not_newer")
        if existing.status in (CandidateStatus.SUCCEEDED,
                               CandidateStatus.EXHAUSTED,
                               CandidateStatus.SKIPPED):
            return CandidateValidation(False, "candidate_terminal")
        existing.snapshot = candidate
        return validation

    def refresh_cooldowns(self, now: float) -> None:
        for entry in self.entries.values():
            if (entry.status == CandidateStatus.COOLDOWN and
                    now >= entry.cooldown_until and
                    entry.attempts < self.config.max_attempts):
                entry.status = CandidateStatus.PENDING
                entry.payload_slot = 0
                entry.reserved_snapshot = None

    def _rank_key(self, entry: CandidateEntry,
                  current_xy: Tuple[float, float]):
        snapshot = entry.snapshot
        distance = math.hypot(
            snapshot.x - current_xy[0], snapshot.y - current_xy[1])
        reachability = 0 if entry.reachable is True else (
            1 if entry.reachable is None else 2)
        return (
            -self.profile.weight(snapshot.class_name),
            reachability,
            -snapshot.map_quality,
            -snapshot.class_confidence,
            distance,
            snapshot.first_seen_ns,
            snapshot.target_id,
        )

    def ranked(self, now: float, current_xy: Tuple[float, float],
               allowed_classes: Optional[Iterable[str]] = None
               ) -> List[CandidateEntry]:
        self.refresh_cooldowns(now)
        allowed = set(allowed_classes) if allowed_classes is not None else None
        pending = [
            entry for entry in self.entries.values()
            if entry.status == CandidateStatus.PENDING
            and entry.snapshot.class_name not in self.delivered_classes
            and (allowed is None or entry.snapshot.class_name in allowed)
            and entry.reachable is not False
        ]
        # Only the best candidate for each formal class may compete for a slot.
        best_by_class: Dict[str, CandidateEntry] = {}
        for entry in pending:
            class_name = entry.snapshot.class_name
            current = best_by_class.get(class_name)
            if (current is None or
                    self._rank_key(entry, current_xy) <
                    self._rank_key(current, current_xy)):
                best_by_class[class_name] = entry
        return sorted(
            best_by_class.values(),
            key=lambda item: self._rank_key(item, current_xy),
        )

    def reserve(self, key: CandidateKey, payload_slot: int) -> CandidateEntry:
        entry = self.entries[key]
        if entry.status != CandidateStatus.PENDING:
            raise ValueError("candidate is not pending")
        entry.status = CandidateStatus.EXECUTING
        entry.attempts += 1
        entry.payload_slot = int(payload_slot)
        entry.reserved_snapshot = entry.snapshot
        return entry

    def fail(self, key: CandidateKey, retryable: bool, now: float,
             reason: str) -> CandidateEntry:
        entry = self.entries[key]
        if entry.status != CandidateStatus.EXECUTING:
            raise ValueError("candidate is not executing")
        entry.last_result = reason
        entry.payload_slot = 0
        entry.reserved_snapshot = None
        if (retryable and not entry.retry_forbidden and
                entry.attempts < self.config.max_attempts):
            entry.status = CandidateStatus.COOLDOWN
            entry.cooldown_until = now + self.config.retry_cooldown
        else:
            entry.status = CandidateStatus.EXHAUSTED
        return entry

    def invalidate(self, key: CandidateKey, reason: str) -> CandidateEntry:
        entry = self.entries[key]
        entry.invalidated_reason = reason or "candidate_invalidated"
        entry.retry_forbidden = True
        if entry.status in (CandidateStatus.PENDING,
                            CandidateStatus.COOLDOWN):
            entry.status = CandidateStatus.SKIPPED
            entry.payload_slot = 0
            entry.reserved_snapshot = None
        return entry

    def commit(self, key: CandidateKey, payload_slot: int,
               target_class: str, reason: str) -> CandidateEntry:
        entry = self.entries[key]
        if entry.status != CandidateStatus.EXECUTING:
            raise ValueError("candidate is not executing")
        if entry.payload_slot != int(payload_slot):
            raise ValueError("payload slot mismatch")
        if (entry.reserved_snapshot is None or
                entry.reserved_snapshot.class_name != target_class):
            raise ValueError("reserved target class mismatch")
        entry.status = CandidateStatus.SUCCEEDED
        entry.last_result = reason
        self.delivered_classes.add(target_class)
        return entry

    def quarantine(self, key: CandidateKey, reason: str) -> CandidateEntry:
        entry = self.entries[key]
        if entry.status != CandidateStatus.EXECUTING:
            raise ValueError("candidate is not executing")
        entry.status = CandidateStatus.EXHAUSTED
        entry.last_result = reason
        # Keep the reserved snapshot and slot identity so a delayed positive
        # release ACK can still be reconciled without ever reusing the slot.
        return entry

    def commit_quarantined(self, key: CandidateKey, payload_slot: int,
                           target_class: str,
                           reason: str) -> CandidateEntry:
        entry = self.entries[key]
        if entry.status != CandidateStatus.EXHAUSTED:
            raise ValueError("candidate is not quarantined")
        if entry.payload_slot != int(payload_slot):
            raise ValueError("payload slot mismatch")
        if (entry.reserved_snapshot is None or
                entry.reserved_snapshot.class_name != target_class):
            raise ValueError("reserved target class mismatch")
        entry.status = CandidateStatus.SUCCEEDED
        entry.last_result = reason
        self.delivered_classes.add(target_class)
        return entry

    def set_reachability(self, key: CandidateKey, reachable: Optional[bool],
                         reason: str = "") -> CandidateEntry:
        entry = self.entries[key]
        if entry.status in (CandidateStatus.SUCCEEDED,
                            CandidateStatus.EXHAUSTED,
                            CandidateStatus.SKIPPED):
            raise ValueError("candidate is terminal")
        if reachable not in (True, False, None):
            raise ValueError("reachable must be true, false or unknown")
        entry.reachable = reachable
        if reason:
            entry.last_result = reason
        return entry


class MissionCore:
    """Deterministic target scheduling and result reducer."""

    def __init__(self, profile: CompetitionProfile,
                 config: Optional[MissionConfig] = None):
        if profile.name == "r2026":
            if dict(profile.weights) != R2026_WEIGHTS:
                raise ValueError("r2026 weights do not match the frozen profile")
            if profile.interrupt_top_k != 3:
                raise ValueError("r2026 interrupt_top_k must equal 3")
            if profile.required_deliveries != 3:
                raise ValueError("r2026 requires exactly three deliveries")
        self.profile = profile
        self.config = config or MissionConfig()
        self.queue = CandidateQueue(profile, self.config)
        self.phase = MissionPhase.INIT
        self.mission_id = ""
        self.started_at = 0.0
        self.decision_seq = 0
        self.active_action: Optional[CoreAction] = None
        self.active_committed = False
        self.active_release_started = False
        self.quarantined_actions: Dict[int, CoreAction] = {}
        self.executor_id: Optional[str] = None
        self.last_event_seq: Dict[str, int] = {}
        self.slots = [PayloadSlot(index=index) for index in
                      range(1, profile.required_deliveries + 1)]
        self.mission_failed = False
        self.post_delivery_route_index = 0

    def start(self, mission_id: str, now: float) -> None:
        if self.phase != MissionPhase.INIT:
            raise RuntimeError("mission has already started")
        if not mission_id.strip():
            raise ValueError("mission_id must not be empty")
        if not math.isfinite(float(now)) or float(now) < 0.0:
            raise ValueError("mission start time must be finite and non-negative")
        self.mission_id = mission_id
        self.started_at = float(now)
        self.phase = MissionPhase.SEARCH

    @property
    def committed_slots(self) -> int:
        return sum(slot.status == SlotStatus.COMMITTED for slot in self.slots)

    @property
    def remaining_slots(self) -> int:
        return len(self.slots) - self.committed_slots

    def ingest(self, candidates: Sequence[CandidateSnapshot],
               now: float) -> List[CandidateValidation]:
        if self.phase not in (MissionPhase.SEARCH, MissionPhase.EXECUTING):
            return [
                CandidateValidation(False, "mission_not_accepting_candidates")
                for _ in candidates
            ]
        return [self.queue.ingest(candidate, now) for candidate in candidates]

    def _next_free_slot(self) -> Optional[PayloadSlot]:
        return next((slot for slot in self.slots
                     if slot.status == SlotStatus.FREE), None)

    def _elapsed(self, now: float) -> float:
        return max(0.0, float(now) - self.started_at)

    def _new_action(self, command: str, reason: str, now: float,
                    entry: Optional[CandidateEntry] = None,
                    slot: Optional[PayloadSlot] = None,
                    goal: Optional[GoalSnapshot] = None,
                    timeout: Optional[float] = None) -> CoreAction:
        if not math.isfinite(float(now)) or float(now) < self.started_at:
            raise ValueError("action clock is invalid")
        frozen_target = None
        if entry is not None:
            frozen_target = entry.reserved_snapshot
            if frozen_target is None:
                raise ValueError("target action requires a reserved snapshot")
        timeout_sec = (self.config.motion_action_timeout if timeout is None
                       else float(timeout))
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("action timeout must be finite and positive")
        deadline = min(
            float(now) + timeout_sec,
            self.started_at + self.config.mission_timeout,
        )
        if deadline <= float(now):
            if command not in ("RETURN_HOME", "LAND", "HOLD", "ABORT"):
                raise RuntimeError("mission deadline forbids a new action")
            # Safety completion remains actionable after the competition
            # timer, but is marked as a mission failure and gets a fresh,
            # finite lease instead of an already-expired deadline.
            deadline = float(now) + timeout_sec
            self.mission_failed = True
        self.decision_seq += 1
        action = CoreAction(
            command=command,
            reason=reason,
            decision_seq=self.decision_seq,
            profile_name=self.profile.name,
            issued_at=float(now),
            deadline_at=deadline,
            goal=goal,
            target_snapshot=frozen_target,
            candidate_key=(frozen_target.key if frozen_target else None),
            target_class=(frozen_target.class_name if frozen_target else ""),
            attempt=(entry.attempts if entry else 0),
            payload_slot=(slot.index if slot else 0),
        )
        self.active_action = action
        return action

    def _return_action(self, reason: str, now: float) -> CoreAction:
        self.phase = MissionPhase.RETURN_HOME
        self.post_delivery_route_index = 0
        goal = GoalSnapshot(
            self.config.mission_frame,
            self.config.home_xy[0],
            self.config.home_xy[1],
            self.config.return_altitude,
        )
        return self._new_action(
            "RETURN_HOME", reason, now, goal=goal,
            timeout=self.config.motion_action_timeout)

    def _post_delivery_route_action(
            self, reason: str, now: float, start: bool = False
            ) -> CoreAction:
        route = self.config.post_delivery_route
        if not route:
            return self._return_action(reason, now)
        if start:
            self.post_delivery_route_index = 0
        if not 0 <= self.post_delivery_route_index < len(route):
            raise RuntimeError("post-delivery route cursor is out of range")
        self.phase = MissionPhase.POST_DELIVERY_ROUTE
        index = self.post_delivery_route_index
        action_reason = "post_delivery_route:%d/%d:%s:%s" % (
            index + 1,
            len(route),
            self.config.post_delivery_route_revision,
            reason,
        )
        return self._new_action(
            "RETURN_HOME", action_reason, now, goal=route[index],
            timeout=self.config.motion_action_timeout)

    def dispatch_search_motion(self, command: str, goal: GoalSnapshot,
                               reason: str, now: float) -> CoreAction:
        if command not in ("SEARCH", "RESUME"):
            raise ValueError("search motion command must be SEARCH or RESUME")
        if self.phase != MissionPhase.SEARCH:
            raise RuntimeError("mission is not accepting search motion")
        if self.active_action is not None:
            raise RuntimeError("another navigation decision is still active")
        if goal.frame_id != self.config.mission_frame:
            raise ValueError("search goal frame does not match mission frame")
        if self._elapsed(now) >= self.config.forced_return_at:
            raise RuntimeError("forced return is due; search motion is forbidden")
        action = self._new_action(
            command, reason, now, goal=goal,
            timeout=self.config.motion_action_timeout)
        search_deadline = self.started_at + self.config.forced_return_at
        if action.deadline_at > search_deadline:
            action = replace(action, deadline_at=search_deadline)
            self.active_action = action
        return action

    def _delivery_work_eta(self, entries: Sequence[CandidateEntry],
                           current_xy: Tuple[float, float]) -> float:
        point = current_xy
        travel_distance = 0.0
        for entry in entries:
            target = (entry.snapshot.x, entry.snapshot.y)
            travel_distance += math.hypot(
                target[0] - point[0], target[1] - point[1])
            point = target
        travel_sec = (self.config.path_factor * travel_distance /
                      self.config.nominal_speed)
        return (travel_sec +
                len(entries) * self.config.delivery_reserve_per_slot +
                self.config.decision_guard)

    def _delivery_eta(self, entries: Sequence[CandidateEntry],
                      current_xy: Tuple[float, float]) -> float:
        point = current_xy if not entries else (
            entries[-1].snapshot.x, entries[-1].snapshot.y)
        return_distance = math.hypot(
            self.config.home_xy[0] - point[0],
            self.config.home_xy[1] - point[1],
        )
        return (self._delivery_work_eta(entries, current_xy) +
                self.config.path_factor * return_distance /
                self.config.nominal_speed +
                self.config.return_land_reserve)

    def should_stop_search(self, now: float,
                           current_xy: Tuple[float, float]) -> bool:
        ranked = self.queue.ranked(now, current_xy)
        if not ranked:
            return False
        planned = ranked[:self.remaining_slots]
        elapsed = self._elapsed(now)
        mission_remaining = self.config.mission_timeout - elapsed
        delivery_remaining = self.config.forced_return_at - elapsed
        return (
            mission_remaining <=
            self._delivery_eta(planned, current_xy) +
            self.config.decision_guard or
            delivery_remaining <=
            self._delivery_work_eta(planned, current_xy) +
            self.config.decision_guard
        )

    def _candidate_fits(self, entry: CandidateEntry, now: float,
                        current_xy: Tuple[float, float]) -> bool:
        elapsed = self._elapsed(now)
        return (
            elapsed + self._delivery_work_eta([entry], current_xy) <=
            self.config.forced_return_at and
            elapsed + self._delivery_eta([entry], current_xy) <=
            self.config.mission_timeout
        )

    def choose(self, now: float, current_xy: Tuple[float, float],
               route_complete: bool = False) -> Optional[CoreAction]:
        if (not math.isfinite(float(now)) or len(current_xy) != 2 or
                not all(math.isfinite(float(value)) for value in current_xy)):
            raise ValueError("choose requires a finite clock and position")
        if float(now) < self.started_at:
            raise ValueError("mission clock moved before its start")
        if self.phase != MissionPhase.SEARCH:
            return None
        if self.committed_slots >= self.profile.required_deliveries:
            return self._return_action("required_deliveries_complete", now)
        if self._elapsed(now) >= self.config.forced_return_at:
            return self._return_action("forced_return_deadline", now)

        interrupt = self.queue.ranked(
            now, current_xy, self.profile.interrupt_classes)
        fallback = self.queue.ranked(now, current_xy)
        interrupt_entry = next(
            (item for item in interrupt
             if self._candidate_fits(item, now, current_xy)),
            None,
        )
        fallback_entry = next(
            (item for item in fallback
             if self._candidate_fits(item, now, current_xy)),
            None,
        )
        entry = None
        reason = ""
        if interrupt_entry is not None:
            entry = interrupt_entry
            reason = "high_weight_search_interrupt"
        elif route_complete or self.should_stop_search(now, current_xy):
            if fallback_entry is not None:
                entry = fallback_entry
                reason = ("coverage_complete_fallback" if route_complete
                          else "time_budget_fallback")
            else:
                return self._return_action(
                    "coverage_complete_insufficient_candidates" if
                    route_complete else "time_budget_no_feasible_candidate",
                    now)
        if entry is None:
            return None

        slot = self._next_free_slot()
        if slot is None:
            return self._return_action("no_free_payload_slot", now)
        self.queue.reserve(entry.snapshot.key, slot.index)
        slot.status = SlotStatus.RESERVED
        slot.candidate_key = entry.snapshot.key
        self.phase = MissionPhase.EXECUTING
        self.active_committed = False
        self.active_release_started = False
        target = entry.reserved_snapshot
        goal = GoalSnapshot(
            self.config.mission_frame,
            target.x,
            target.y,
            self.config.approach_altitude,
        )
        action = self._new_action(
            "APPROACH", reason, now, entry, slot, goal,
            self.config.target_action_timeout)
        hard_target_deadline = (
            self.started_at + self.config.forced_return_at -
            self.config.decision_guard)
        if action.deadline_at > hard_target_deadline:
            action = replace(action, deadline_at=hard_target_deadline)
            self.active_action = action
        return action

    def _validate_result_common(self, event: ResultEvent) -> str:
        if event.mission_id != self.mission_id:
            return "mission_mismatch"
        if not event.executor_id.strip():
            return "executor_missing"
        if self.executor_id is not None and self.executor_id != event.executor_id:
            return "executor_changed"
        if event.event_seq <= 0:
            return "event_seq_invalid"
        if event.event_stamp_ns <= 0:
            return "event_stamp_invalid"
        previous_seq = self.last_event_seq.get(event.executor_id, -1)
        if event.event_seq <= previous_seq:
            return "event_duplicate_or_out_of_order"
        return "accepted"

    @staticmethod
    def _validate_result_against_action(event: ResultEvent,
                                        action: CoreAction) -> str:
        if event.event_time < action.issued_at:
            return "result_precedes_decision"
        if event.decision_seq != action.decision_seq:
            return "decision_mismatch"
        if event.command != action.command:
            return "command_mismatch"
        if event.has_target != action.has_target:
            return "result_target_flag_mismatch"
        if action.has_target:
            if event.candidate_key != action.candidate_key:
                return "target_mismatch"
            if event.target_class != action.target_class:
                return "class_mismatch"
            if event.attempt != action.attempt:
                return "attempt_mismatch"
            if event.payload_slot != action.payload_slot:
                return "payload_slot_mismatch"
        elif event.attempt != 0 or event.payload_slot != 0:
            return "targetless_result_fields_invalid"
        return "accepted"

    def _validate_result_identity(self, event: ResultEvent) -> str:
        common = self._validate_result_common(event)
        if common != "accepted":
            return common
        if self.active_action is None:
            return "no_active_decision"
        action = self.active_action
        return self._validate_result_against_action(event, action)

    def _record_result_sequence(self, event: ResultEvent) -> None:
        if self.executor_id is None:
            self.executor_id = event.executor_id
        self.last_event_seq[event.executor_id] = event.event_seq

    def _observe_target_stage(self, event: ResultEvent) -> str:
        """Latch physical release uncertainty; later stage rollback cannot clear it."""

        if event.stage not in TARGET_STAGES:
            return "target_stage_invalid"
        if event.stage in UNCERTAIN_RELEASE_STAGES:
            self.active_release_started = True
        return "accepted"

    @staticmethod
    def _terminal_status_valid(event: ResultEvent) -> bool:
        return event.status in (
            "SUCCEEDED", "FAILED", "REJECTED", "CANCELLED", "TIMED_OUT")

    @staticmethod
    def _is_payload_commit_event(event: ResultEvent) -> bool:
        return (
            event.payload_committed and
            event.status == "PROGRESS" and
            event.stage == "RELEASE" and
            not event.terminal and
            not event.retryable and
            bool(event.evidence_source.strip()) and
            event.reason == "release_ack_success"
        )

    def _result_semantics_error(self, action: CoreAction,
                                event: ResultEvent) -> str:
        """Validate result shape before an on-time or expired reduction."""

        if action.has_target:
            if event.payload_committed:
                return ("accepted" if self._is_payload_commit_event(event)
                        else "invalid_payload_commit_event")
            if not event.terminal:
                if event.retryable:
                    return "nonterminal_result_must_not_retry"
                if event.status not in ("ACCEPTED", "STARTED", "PROGRESS"):
                    return "nonterminal_status_invalid"
                return "accepted"
            if not self._terminal_status_valid(event):
                return "terminal_status_invalid"
            if event.status == "SUCCEEDED" and event.retryable:
                return "successful_result_must_not_retry"
            if not self.active_committed and event.status == "SUCCEEDED":
                return "success_without_payload_commit"
            if self.active_committed:
                if event.stage != "RECOVERY":
                    return "committed_terminal_stage_invalid"
                if event.retryable:
                    return "committed_result_must_not_retry"
            return "accepted"

        if event.payload_committed:
            return "targetless_payload_commit"
        if not event.terminal:
            if event.retryable:
                return "nonterminal_result_must_not_retry"
            if event.status not in ("ACCEPTED", "STARTED", "PROGRESS"):
                return "nonterminal_status_invalid"
            return "accepted"
        if not self._terminal_status_valid(event):
            return "terminal_status_invalid"
        if event.status == "SUCCEEDED" and event.retryable:
            return "successful_result_must_not_retry"
        if (action.command in ("SEARCH", "RESUME", "RETURN_HOME") and
                event.stage != "PLANNER"):
            return "motion_terminal_stage_invalid"
        if action.command == "LAND" and event.stage != "LANDING":
            return "landing_terminal_stage_invalid"
        return "accepted"

    def _abort_action(self, reason: str, now: float) -> CoreAction:
        self.phase = MissionPhase.ABORTED
        self.mission_failed = True
        return self._new_action(
            "ABORT", reason, now, timeout=self.config.motion_action_timeout)

    def abort(self, reason: str, now: float) -> CoreAction:
        """Fail closed while preserving any already committed payload slot."""

        if not reason.strip():
            raise ValueError("abort reason must not be empty")
        if not math.isfinite(float(now)) or float(now) < self.started_at:
            raise ValueError("abort clock is invalid")
        if self.phase == MissionPhase.COMPLETE:
            raise RuntimeError("completed mission cannot be aborted")
        action = self.active_action
        self.active_action = None
        if action is not None and action.has_target and not self.active_committed:
            entry = self.queue.entries[action.candidate_key]
            if entry.status == CandidateStatus.EXECUTING:
                self.queue.quarantine(
                    action.candidate_key, "mission_aborted_uncertain")
            slot = self.slots[action.payload_slot - 1]
            slot.status = SlotStatus.QUARANTINED
            self.quarantined_actions[action.decision_seq] = action
        return self._abort_action(reason, now)

    def _apply_motion_result(self, action: CoreAction, event: ResultEvent,
                             now: float
                             ) -> Tuple[bool, str, Optional[CoreAction]]:
        if event.payload_committed:
            return False, "targetless_payload_commit", None
        if not event.terminal:
            if event.retryable:
                return False, "nonterminal_result_must_not_retry", None
            if event.status not in ("ACCEPTED", "STARTED", "PROGRESS"):
                return False, "nonterminal_status_invalid", None
            self._record_result_sequence(event)
            return True, "progress_recorded", None
        if not self._terminal_status_valid(event):
            return False, "terminal_status_invalid", None
        if event.status == "SUCCEEDED" and event.retryable:
            return False, "successful_result_must_not_retry", None
        if action.command in ("SEARCH", "RESUME", "RETURN_HOME"):
            if event.stage != "PLANNER":
                return False, "motion_terminal_stage_invalid", None
        elif action.command == "LAND" and event.stage != "LANDING":
            return False, "landing_terminal_stage_invalid", None

        self._record_result_sequence(event)
        self.active_action = None
        if action.command in ("SEARCH", "RESUME"):
            self.phase = MissionPhase.SEARCH
            if event.status == "SUCCEEDED":
                return True, "search_motion_complete", None
            return True, "search_motion_failed", None
        if action.command == "RETURN_HOME":
            if event.status != "SUCCEEDED":
                return True, "return_home_failed", self._abort_action(
                    "return_home_failed", now)
            if self.phase == MissionPhase.POST_DELIVERY_ROUTE:
                self.post_delivery_route_index += 1
                if (self.post_delivery_route_index <
                        len(self.config.post_delivery_route)):
                    return (True, "post_delivery_route_segment_complete",
                            self._post_delivery_route_action(
                                "segment_complete", now))
                self.phase = MissionPhase.LAND
                return True, "post_delivery_route_complete", self._new_action(
                    "LAND", "post_delivery_route_complete", now,
                    timeout=self.config.landing_action_timeout)
            self.phase = MissionPhase.LAND
            return True, "return_home_complete", self._new_action(
                "LAND", "return_home_complete", now,
                timeout=self.config.landing_action_timeout)
        if action.command == "LAND":
            if event.status != "SUCCEEDED":
                return True, "landing_failed", self._abort_action(
                    "landing_failed", now)
            self.phase = MissionPhase.COMPLETE
            return True, "mission_complete", None
        if action.command in ("HOLD", "ABORT"):
            self.phase = MissionPhase.ABORTED
            return True, "abort_acknowledged", None
        return False, "targetless_command_unsupported", None

    def _apply_target_result(self, action: CoreAction, event: ResultEvent,
                             now: float
                             ) -> Tuple[bool, str, Optional[CoreAction]]:
        key = action.candidate_key
        slot = self.slots[action.payload_slot - 1]

        if event.payload_committed:
            if not self._is_payload_commit_event(event):
                return False, "invalid_payload_commit_event", None
            self._record_result_sequence(event)
            if not self.active_committed:
                self.queue.commit(
                    key, slot.index, action.target_class, event.reason)
                slot.status = SlotStatus.COMMITTED
                self.active_committed = True
            return True, "payload_committed", None

        if not event.terminal:
            if event.retryable:
                return False, "nonterminal_result_must_not_retry", None
            if event.status not in ("ACCEPTED", "STARTED", "PROGRESS"):
                return False, "nonterminal_status_invalid", None
            self._record_result_sequence(event)
            return True, "progress_recorded", None

        if not self._terminal_status_valid(event):
            return False, "terminal_status_invalid", None

        if self.active_committed:
            if event.stage != "RECOVERY":
                return False, "committed_terminal_stage_invalid", None
            if event.retryable:
                return False, "committed_result_must_not_retry", None
            self._record_result_sequence(event)
            self.active_action = None
            self.active_release_started = False
            if event.status != "SUCCEEDED":
                self.mission_failed = True
                return True, "committed_recovery_failed", self._return_action(
                    "committed_recovery_failed", now)
            if self.committed_slots >= self.profile.required_deliveries:
                return (True, "required_deliveries_complete",
                        self._post_delivery_route_action(
                            "required_deliveries_complete", now, start=True))
            self.phase = MissionPhase.SEARCH
            return True, "delivery_complete", None

        if event.status == "SUCCEEDED":
            return False, "success_without_payload_commit", None
        self._record_result_sequence(event)
        if self.active_release_started:
            self.queue.quarantine(key, event.reason or
                                  "release_state_uncertain")
            slot.status = SlotStatus.QUARANTINED
            self.quarantined_actions[action.decision_seq] = action
            self.active_action = None
            self.mission_failed = True
            return True, "candidate_release_state_uncertain", self._return_action(
                "candidate_release_state_uncertain", now)
        self.queue.fail(key, event.retryable, now, event.reason)
        slot.status = SlotStatus.FREE
        slot.candidate_key = None
        self.active_action = None
        self.active_release_started = False
        self.phase = MissionPhase.SEARCH
        return True, "candidate_failed", None

    def _apply_quarantined_result(
            self, action: CoreAction, event: ResultEvent
            ) -> Tuple[bool, str, Optional[CoreAction]]:
        """Reconcile delayed evidence without making the slot reusable."""

        slot = self.slots[action.payload_slot - 1]
        if event.payload_committed:
            if not self._is_payload_commit_event(event):
                return False, "invalid_payload_commit_event", None
            self.queue.commit_quarantined(
                action.candidate_key,
                action.payload_slot,
                action.target_class,
                event.reason,
            )
            slot.status = SlotStatus.COMMITTED
            self._record_result_sequence(event)
            self.quarantined_actions.pop(action.decision_seq, None)
            self.mission_failed = True
            return True, "late_payload_committed", None
        if not event.terminal:
            if event.retryable:
                return False, "nonterminal_result_must_not_retry", None
            if event.status not in ("ACCEPTED", "STARTED", "PROGRESS"):
                return False, "nonterminal_status_invalid", None
            self._record_result_sequence(event)
            return True, "quarantined_progress_recorded", None
        if not self._terminal_status_valid(event):
            return False, "terminal_status_invalid", None
        if event.status == "SUCCEEDED":
            return False, "success_without_payload_commit", None
        self._record_result_sequence(event)
        # A negative terminal result cannot prove the payload is still
        # present.  Keep the tombstone so a later guarded release ACK can
        # still reconcile the irreversible physical fact.
        return True, "quarantined_terminal_recorded", None

    def _expire_action(self, action: CoreAction, now: float
                       ) -> Tuple[bool, str, Optional[CoreAction]]:
        """Reduce an already-identified lease expiry exactly once."""

        self.active_action = None
        if action.has_target:
            if self.active_committed:
                self.mission_failed = True
                return True, "committed_recovery_timed_out", self._return_action(
                    "committed_recovery_timed_out", now)
            self.queue.quarantine(
                action.candidate_key, "decision_timeout_uncertain")
            slot = self.slots[action.payload_slot - 1]
            slot.status = SlotStatus.QUARANTINED
            self.quarantined_actions[action.decision_seq] = action
            self.mission_failed = True
            return True, "target_action_timed_out_uncertain", self._return_action(
                "target_action_timed_out_uncertain", now)
        if action.command in ("SEARCH", "RESUME"):
            if self._elapsed(now) >= self.config.forced_return_at:
                return True, "forced_return_deadline", self._return_action(
                    "forced_return_deadline", now)
            self.phase = MissionPhase.SEARCH
            return True, "search_motion_timed_out", None
        if action.command in ("RETURN_HOME", "LAND"):
            return True, "safety_motion_timed_out", self._abort_action(
                "safety_motion_timed_out", now)
        self.phase = MissionPhase.ABORTED
        return True, "abort_timed_out", None

    def _apply_expired_result(
            self, action: CoreAction, event: ResultEvent, now: float
            ) -> Tuple[bool, str, Optional[CoreAction]]:
        """Make lease expiry independent of timer/result callback order."""

        semantics = self._result_semantics_error(action, event)
        if semantics != "accepted":
            return False, semantics, None
        self._record_result_sequence(event)
        if action.has_target and event.payload_committed:
            slot = self.slots[action.payload_slot - 1]
            if not self.active_committed:
                self.queue.commit(
                    action.candidate_key,
                    action.payload_slot,
                    action.target_class,
                    event.reason,
                )
                slot.status = SlotStatus.COMMITTED
                self.active_committed = True
            self.active_action = None
            self.mission_failed = True
            return True, "late_payload_committed", self._return_action(
                "target_decision_lease_expired_after_commit", now)
        return self._expire_action(action, now)

    def apply_result(self, event: ResultEvent,
                     now: float) -> Tuple[bool, str, Optional[CoreAction]]:
        if not math.isfinite(float(now)):
            raise ValueError("result clock must be finite")
        if float(now) < self.started_at:
            raise ValueError("mission clock moved before its start")
        if event.event_time > float(now) + self.config.result_future_tolerance:
            return False, "event_stamp_in_future", None
        quarantined = self.quarantined_actions.get(event.decision_seq)
        if quarantined is not None:
            reason = self._validate_result_common(event)
            if reason == "accepted":
                reason = self._validate_result_against_action(
                    event, quarantined)
            if reason == "accepted" and event.stage not in TARGET_STAGES:
                reason = "target_stage_invalid"
            if reason != "accepted":
                return False, reason, None
            return self._apply_quarantined_result(quarantined, event)
        reason = self._validate_result_identity(event)
        if reason != "accepted":
            return False, reason, None
        action = self.active_action
        if action.has_target:
            reason = self._observe_target_stage(event)
            if reason != "accepted":
                return False, reason, None
        if (float(now) >= action.deadline_at or
                event.event_time >= action.deadline_at):
            return self._apply_expired_result(action, event, now)
        if action.has_target:
            return self._apply_target_result(action, event, now)
        return self._apply_motion_result(action, event, now)

    def expire_active(self, now: float
                      ) -> Tuple[bool, str, Optional[CoreAction]]:
        """Apply a local decision deadline without inventing executor facts."""

        if not math.isfinite(float(now)) or float(now) < self.started_at:
            raise ValueError("timeout clock is invalid")
        action = self.active_action
        if action is None:
            return False, "no_active_decision", None
        if (float(now) < action.deadline_at and
                self._elapsed(now) < self.config.mission_timeout):
            return False, "decision_not_expired", None
        return self._expire_action(action, now)
