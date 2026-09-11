"""Thread-safe, ROS-free orchestration for the navigation mission.

The runtime owns the transaction boundary between MissionCore and the nominal
CoverageRoute. It does not publish planner goals or operate a payload
mechanism; callers publish only the CoreAction returned in a RuntimeOutcome.
"""

from dataclasses import dataclass
import math
from numbers import Real
from threading import RLock
from typing import Optional, Sequence, Tuple

from .coverage_route import CoverageRoute, RouteOutcome
from .local_motion import LocalMotionConfig, validate_advice
from .mission_core import (
    CandidateSnapshot,
    CandidateValidation,
    CoreAction,
    GoalSnapshot,
    MissionCore,
    MissionPhase,
    ResultEvent,
)


@dataclass(frozen=True)
class RuntimeSnapshot:
    """Immutable status suitable for conversion by a ROS wrapper."""

    mission_id: str
    phase: MissionPhase
    route_revision: str
    route_index: int
    route_size: int
    route_complete: bool
    route_active_decision_seq: int
    post_delivery_route_revision: str
    post_delivery_route_index: int
    post_delivery_route_size: int
    post_delivery_route_complete: bool
    active_decision_seq: int
    active_command: str
    active_deadline_at: float
    committed_slots: int
    required_deliveries: int
    mission_failed: bool


@dataclass(frozen=True)
class RuntimeOutcome:
    """One atomic runtime operation and, at most, one action to publish."""

    accepted: bool
    reason: str
    action: Optional[CoreAction]
    snapshot: RuntimeSnapshot
    route_outcome: Optional[RouteOutcome] = None
    candidate_validations: Tuple[CandidateValidation, ...] = ()


class MissionRuntime:
    """Serialize mission facts, decisions and coverage cursor transitions."""

    def __init__(self, core: MissionCore, route: CoverageRoute,
                 local_config: Optional[LocalMotionConfig] = None):
        if not isinstance(core, MissionCore):
            raise TypeError("core must be a MissionCore")
        if not isinstance(route, CoverageRoute):
            raise TypeError("route must be a CoverageRoute")
        if core.phase != MissionPhase.INIT:
            raise ValueError("runtime requires a fresh mission core")
        if route.active is not None or route.current_index != 0:
            raise ValueError("runtime requires a fresh coverage route")
        self.core = core
        self.route = route
        self._lock = RLock()
        self._started = False
        self._current_xy: Optional[Tuple[float, float]] = None
        self._resume_pending = False
        self._last_now: Optional[float] = None
        self.local_config = local_config or LocalMotionConfig()
        if self.local_config.enabled:
            self.core.enable_research_issue_order()
        self._local_active_seq = 0
        self._local_nominal_deadline = None
        self._local_resume_deadline = None
        self._local_goal = None
        self._local_used = set()
        self._local_reason = 'ready' if self.local_config.enabled else 'disabled'
        self._local_last_outcome = None

    def _clear_local(self):
        self._local_active_seq = 0
        self._local_nominal_deadline = None
        self._local_resume_deadline = None
        self._local_goal = None

    def local_status(self):
        with self._lock:
            return dict(enabled=self.local_config.enabled,active_seq=self._local_active_seq,
                used=len(self._local_used),max_per_mission=self.local_config.max_per_mission,
                remaining_for_waypoint=int(self.route.current_index not in self._local_used and
                    len(self._local_used)<self.local_config.max_per_mission and not self.route.is_complete),
                original_deadline=self._local_nominal_deadline,last_reason=self._local_reason,
                last_outcome=self._local_last_outcome)

    def adopt_local_advice(self, data, memory, now, current_xyz, pose_stamp):
        """Optional task-owned handover; advice never publishes goals itself."""
        with self._lock:
            self._require_started()
            def reject(reason):
                self._local_reason=reason
                return self._outcome(False,reason)
            if not self.local_config.enabled:return reject('local_motion_disabled')
            now,failed=self._operation_time(now)
            if failed is not None:return failed
            if self._local_active_seq:return reject('local_entry_already_active')
            if self.route.current_index in self._local_used:return reject('local_waypoint_budget_used')
            if len(self._local_used)>=self.local_config.max_per_mission:return reject('local_mission_budget_used')
            active=self.core.active_action
            if active is None or self.core.phase!=MissionPhase.SEARCH or not self._route_binding_matches(active):
                return reject('local_search_binding_missing')
            point=self.route.current_waypoint
            try:
                entry=validate_advice(data,memory,active,(point.x,point.y,point.z),self.core.mission_id,
                                      current_xyz,pose_stamp,now,self.local_config)
            except (ValueError,TypeError,KeyError,AttributeError) as error:
                return reject(str(error))
            # Existing target/return/deadline scheduling retains priority.
            priority=self.tick(now,tuple(current_xyz[:2]))
            if priority.action is not None or not priority.accepted:return priority
            if self.core.active_action is not active:return reject('local_action_changed')
            try:
                replacement=self.core.replace_search_motion(active.decision_seq,entry.goal,
                    'research_local_entry:%d'%entry.request_id,now,self.local_config.timeout)
                retired=self.route.interrupt(active.decision_seq)
                if not retired.accepted:raise RuntimeError(retired.reason)
                self.route.bind(replacement.decision_seq,replacement.command)
            except Exception:
                return self._fail_closed('local_entry_transaction_failed',now)
            self._local_active_seq=replacement.decision_seq
            self._local_nominal_deadline=active.deadline_at
            self._local_goal=entry.goal
            self._local_used.add(self.route.current_index)
            self._local_reason='local_entry_adopted'
            return self._outcome(True,self._local_reason,action=replacement,route_outcome=retired)

    @staticmethod
    def _normalize_xy(current_xy) -> Tuple[float, float]:
        if (not isinstance(current_xy, (tuple, list)) or
                len(current_xy) != 2):
            raise ValueError("current_xy must contain two coordinates")
        if any(isinstance(value, bool) or not isinstance(value, Real)
               for value in current_xy):
            raise ValueError("current_xy coordinates must be numeric")
        result = tuple(float(value) for value in current_xy)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("current_xy coordinates must be finite")
        return result

    def _set_current_xy(self, current_xy) -> Tuple[float, float]:
        self._current_xy = self._normalize_xy(current_xy)
        return self._current_xy

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("mission runtime has not started")

    def _validate_now(self, now: float) -> float:
        if (isinstance(now, bool) or not isinstance(now, Real) or
                not math.isfinite(float(now))):
            raise ValueError("runtime clock must be finite")
        value = float(now)
        if (self._started and
                (value < self.core.started_at or
                 (self._last_now is not None and value < self._last_now))):
            raise RuntimeError("runtime clock moved backwards")
        self._last_now = value
        return value

    def _operation_time(
            self, now: float
            ) -> Tuple[float, Optional[RuntimeOutcome]]:
        """Accept a monotonic clock or atomically retire the mission."""

        try:
            return self._validate_now(now), None
        except RuntimeError:
            safe_now = (self._last_now if self._last_now is not None else
                        self.core.started_at)
            return safe_now, self._fail_closed(
                "runtime_clock_rollback", safe_now)
        except ValueError:
            safe_now = (self._last_now if self._last_now is not None else
                        self.core.started_at)
            return safe_now, self._fail_closed(
                "runtime_clock_invalid", safe_now)

    def _snapshot(self) -> RuntimeSnapshot:
        active = self.core.active_action
        route_active = self.route.active
        return RuntimeSnapshot(
            mission_id=self.core.mission_id,
            phase=self.core.phase,
            route_revision=self.route.route_revision,
            route_index=self.route.current_index,
            route_size=len(self.route.waypoints),
            route_complete=self.route.is_complete,
            route_active_decision_seq=(
                route_active.decision_seq if route_active else 0),
            post_delivery_route_revision=(
                self.core.config.post_delivery_route_revision),
            post_delivery_route_index=(
                self.core.post_delivery_route_index),
            post_delivery_route_size=len(
                self.core.config.post_delivery_route),
            post_delivery_route_complete=(
                not self.core.config.post_delivery_route or
                self.core.post_delivery_route_index >=
                len(self.core.config.post_delivery_route)),
            active_decision_seq=(active.decision_seq if active else 0),
            active_command=(active.command if active else ""),
            active_deadline_at=(active.deadline_at if active else 0.0),
            committed_slots=self.core.committed_slots,
            required_deliveries=self.core.profile.required_deliveries,
            mission_failed=self.core.mission_failed,
        )

    def snapshot(self) -> RuntimeSnapshot:
        """Return one coherent, read-only status under the runtime lock."""

        with self._lock:
            return self._snapshot()

    def _outcome(self, accepted: bool, reason: str,
                 action: Optional[CoreAction] = None,
                 route_outcome: Optional[RouteOutcome] = None,
                 candidate_validations: Sequence[CandidateValidation] = (
                 )) -> RuntimeOutcome:
        return RuntimeOutcome(
            accepted=bool(accepted),
            reason=str(reason),
            action=action,
            snapshot=self._snapshot(),
            route_outcome=route_outcome,
            candidate_validations=tuple(candidate_validations),
        )

    def _fail_closed(self, reason: str, now: float,
                     route_outcome: Optional[RouteOutcome] = None
                     ) -> RuntimeOutcome:
        self._clear_local()
        if self.route.active is not None:
            cleanup = self.route.interrupt(self.route.active.decision_seq)
            if route_outcome is None:
                route_outcome = cleanup
        if self.core.phase == MissionPhase.ABORTED:
            return self._outcome(
                False, reason, route_outcome=route_outcome)
        if self.core.phase == MissionPhase.COMPLETE:
            return self._outcome(
                False, reason, route_outcome=route_outcome)
        action = self.core.abort(reason, now)
        return self._outcome(
            False, reason, action=action, route_outcome=route_outcome)

    def _route_binding_matches(self, action: CoreAction) -> bool:
        route_active = self.route.active
        return (
            route_active is not None and
            route_active.decision_seq == action.decision_seq and
            route_active.command == action.command and
            route_active.waypoint_index == self.route.current_index
        )

    def _dispatch_route(self, command: str, reason: str, now: float,
                        route_outcome: Optional[RouteOutcome] = None
                        ) -> RuntimeOutcome:
        if self.route.is_complete:
            return self._fail_closed(
                "route_dispatch_after_completion", now, route_outcome)
        if self.route.active is not None:
            return self._fail_closed(
                "route_dispatch_while_active", now, route_outcome)
        point = self.route.current_waypoint
        goal = GoalSnapshot(
            self.core.config.mission_frame, point.x, point.y, point.z)
        try:
            deadline=self._local_resume_deadline
            self._local_resume_deadline=None
            action = self.core.dispatch_search_motion(
                command, goal, reason, now,deadline_at=deadline)
            self.route.bind(action.decision_seq, command)
        except Exception:
            return self._fail_closed(
                "route_dispatch_transaction_failed", now, route_outcome)
        return self._outcome(
            True, reason, action=action, route_outcome=route_outcome)

    def _schedule_from_search(
            self, now: float, prefer_resume: bool,
            route_outcome: Optional[RouteOutcome] = None
            ) -> RuntimeOutcome:
        if self.core.phase != MissionPhase.SEARCH:
            return self._fail_closed(
                "scheduler_phase_mismatch", now, route_outcome)
        if self.core.active_action is not None:
            return self._fail_closed(
                "scheduler_active_decision_mismatch", now, route_outcome)
        if self.route.active is not None:
            return self._fail_closed(
                "scheduler_route_decision_mismatch", now, route_outcome)
        try:
            action = self.core.choose(
                now, self._current_xy, route_complete=self.route.is_complete)
        except Exception:
            return self._fail_closed(
                "mission_selection_failed", now, route_outcome)
        if action is not None:
            self._local_resume_deadline=None
            if action.command not in ("APPROACH", "RETURN_HOME"):
                return self._fail_closed(
                    "mission_selection_command_invalid", now, route_outcome)
            return self._outcome(
                True, action.reason, action=action,
                route_outcome=route_outcome)
        if self.route.is_complete:
            return self._fail_closed(
                "coverage_completion_without_decision", now, route_outcome)
        command = "RESUME" if prefer_resume else "SEARCH"
        reason = ("resume_interrupted_waypoint" if prefer_resume else
                  "coverage_waypoint")
        return self._dispatch_route(
            command, reason, now, route_outcome=route_outcome)

    def _consider_search_replacement(self, now: float) -> RuntimeOutcome:
        search_action = self.core.active_action
        if (search_action is None or
                search_action.command not in ("SEARCH", "RESUME")):
            return self._fail_closed("search_action_missing", now)
        if not self._route_binding_matches(search_action):
            return self._fail_closed("search_route_binding_mismatch", now)

        # MissionCore.choose() only replaces SEARCH with a feasible target or
        # a safety return. The runtime lock makes that replacement and route
        # retirement one externally atomic transaction.
        try:
            replacement = self.core.choose(
                now, self._current_xy, route_complete=False)
        except Exception:
            return self._fail_closed("search_replacement_failed", now)
        if replacement is None:
            return self._outcome(True, "search_continues")
        if replacement.command not in ("APPROACH", "RETURN_HOME"):
            return self._fail_closed(
                "search_replacement_command_invalid", now)
        route_outcome = self.route.interrupt(search_action.decision_seq)
        if not route_outcome.accepted:
            return self._fail_closed(
                "route_interrupt_failed", now, route_outcome)
        self._resume_pending = replacement.command == "APPROACH"
        self._clear_local()
        return self._outcome(
            True, replacement.reason, action=replacement,
            route_outcome=route_outcome)

    def _finish_route(self, action: CoreAction, succeeded: bool,
                      now: float) -> Tuple[Optional[RouteOutcome],
                                           Optional[RuntimeOutcome]]:
        if action.decision_seq == self._local_active_seq:
            deadline=self._local_nominal_deadline
            error=math.hypot(self._current_xy[0]-self._local_goal.x,self._current_xy[1]-self._local_goal.y)
            reason=('local_entry_complete' if succeeded and error<=self.local_config.entry_arrival_xy else
                    'local_entry_adjusted_or_missed' if succeeded else 'local_entry_failed_or_timed_out')
            self._local_last_outcome=dict(reason=reason,decision_seq=action.decision_seq,
                original_deadline=deadline,finished_at=now,entry_error_xy=error)
            self._local_reason=reason
            self._clear_local()
            if now>=deadline:
                route_outcome=self.route.finish(action.decision_seq,False)
            else:
                retired=self.route.interrupt(action.decision_seq)
                if not retired.accepted:return retired,self._fail_closed('local_retirement_failed',now,retired)
                route_outcome=RouteOutcome(True,reason,complete=self.route.is_complete)
                self._local_resume_deadline=deadline
        else:
            route_outcome = self.route.finish(action.decision_seq, succeeded)
        if not route_outcome.accepted:
            return route_outcome, self._fail_closed(
                "route_result_reduction_failed", now, route_outcome)
        self._resume_pending = not route_outcome.advanced
        return route_outcome, None

    def start(self, mission_id: str, now: float,
              current_xy) -> RuntimeOutcome:
        """Start once and return the first nominal SEARCH action."""

        with self._lock:
            if self._started:
                raise RuntimeError("mission runtime has already started")
            now = self._validate_now(now)
            self._set_current_xy(current_xy)
            self.core.start(mission_id, now)
            self._started = True
            return self._dispatch_route("SEARCH", "coverage_start", now)

    def start_post_delivery_validation(
            self, mission_id: str, now: float, current_xy
            ) -> RuntimeOutcome:
        """Start an isolated corridor-to-LAND integration stage.

        Coverage and payload accounting are intentionally left untouched; a
        scope-aware assertion must be used for this stage so its result can
        never be reported as a complete three-delivery mission.
        """

        with self._lock:
            if self._started:
                raise RuntimeError("mission runtime has already started")
            now = self._validate_now(now)
            self._set_current_xy(current_xy)
            action = self.core.start_post_delivery_validation(
                mission_id, now)
            self._started = True
            return self._outcome(
                True, "post_delivery_validation_started", action=action)

    def ingest(self, candidates: Sequence[CandidateSnapshot],
               now: float) -> RuntimeOutcome:
        """Atomically ingest candidates; tick performs scheduling."""

        with self._lock:
            self._require_started()
            now, failed = self._operation_time(now)
            if failed is not None:
                return failed
            validations = self.core.ingest(tuple(candidates), now)
            return self._outcome(
                True, "candidates_ingested",
                candidate_validations=validations)

    def tick(self, now: float, current_xy) -> RuntimeOutcome:
        """Advance leases or select one safe next action."""

        with self._lock:
            self._require_started()
            now, failed = self._operation_time(now)
            if failed is not None:
                return failed
            self._set_current_xy(current_xy)
            active = self.core.active_action
            if active is not None:
                handled, reason, next_action = self.core.expire_active(now)
                if handled:
                    route_outcome = None
                    if active.command in ("SEARCH", "RESUME"):
                        if next_action is not None:
                            route_outcome = self.route.interrupt(
                                active.decision_seq)
                            if not route_outcome.accepted:
                                return self._fail_closed(
                                    "route_interrupt_failed",
                                    now,
                                    route_outcome,
                                )
                            self._resume_pending = False
                            self._clear_local()
                        else:
                            route_outcome, reduction_failed = (
                                self._finish_route(active, False, now))
                            if reduction_failed is not None:
                                return reduction_failed
                    if next_action is not None:
                        return self._outcome(
                            True, reason, action=next_action,
                            route_outcome=route_outcome)
                    if self.core.phase == MissionPhase.SEARCH:
                        return self._schedule_from_search(
                            now, self._resume_pending, route_outcome)
                    return self._outcome(
                        True, reason, route_outcome=route_outcome)
                if reason != "decision_not_expired":
                    return self._fail_closed("lease_reduction_failed", now)
                if active.command in ("SEARCH", "RESUME"):
                    return self._consider_search_replacement(now)
                return self._outcome(True, "decision_pending")

            if self.core.phase == MissionPhase.SEARCH:
                return self._schedule_from_search(
                    now, self._resume_pending)
            if self.core.phase == MissionPhase.COMPLETE:
                return self._outcome(True, "mission_complete")
            if self.core.phase == MissionPhase.ABORTED:
                return self._outcome(True, "mission_aborted")
            return self._fail_closed(
                "mission_has_no_active_decision", now)

    def apply_result(self, event: ResultEvent, now: float,
                     current_xy) -> RuntimeOutcome:
        """Reduce one executor fact without stale facts moving the route."""

        with self._lock:
            self._require_started()
            now, failed = self._operation_time(now)
            if failed is not None:
                return failed
            self._set_current_xy(current_xy)
            previous = self.core.active_action
            accepted, reason, next_action = self.core.apply_result(event, now)
            if not accepted:
                if reason == "executor_changed":
                    return self._fail_closed(reason, now)
                return self._outcome(False, reason)
            matched_active = (
                previous is not None and
                previous.decision_seq == event.decision_seq)
            if not matched_active:
                # A guarded late fact can reconcile a quarantined target, but
                # it must never move the current coverage cursor.
                return self._outcome(
                    True, reason, action=next_action)
            action_ended = (
                self.core.active_action is None or
                self.core.active_action.decision_seq != previous.decision_seq)
            route_outcome = None
            if (previous.command in ("SEARCH", "RESUME") and action_ended):
                if next_action is not None:
                    route_outcome = self.route.interrupt(
                        previous.decision_seq)
                    if not route_outcome.accepted:
                        return self._fail_closed(
                            "route_interrupt_failed",
                            now,
                            route_outcome,
                        )
                    self._resume_pending = False
                    self._clear_local()
                else:
                    route_outcome, reduction_failed = self._finish_route(
                        previous,
                        reason == "search_motion_complete",
                        now,
                    )
                    if reduction_failed is not None:
                        return reduction_failed
            if next_action is not None:
                return self._outcome(
                    True, reason, action=next_action,
                    route_outcome=route_outcome)
            if action_ended and self.core.phase == MissionPhase.SEARCH:
                return self._schedule_from_search(
                    now, self._resume_pending, route_outcome)
            return self._outcome(
                True, reason, route_outcome=route_outcome)

    def abort(self, reason: str, now: float) -> RuntimeOutcome:
        """Abort and retire a bound search decision when possible."""

        with self._lock:
            self._require_started()
            now, failed = self._operation_time(now)
            if failed is not None:
                return failed
            if self.core.phase == MissionPhase.ABORTED:
                return self._outcome(True, "mission_already_aborted")
            if self.core.phase == MissionPhase.COMPLETE:
                return self._outcome(False, "mission_not_abortable")
            route_outcome = None
            active = self.core.active_action
            if (active is not None and
                    active.command in ("SEARCH", "RESUME")):
                route_outcome = self.route.interrupt(active.decision_seq)
                if not route_outcome.accepted:
                    return self._fail_closed(
                        "abort_route_retirement_failed", now, route_outcome)
            action = self.core.abort(reason, now)
            self._clear_local()
            return self._outcome(
                True, reason, action=action, route_outcome=route_outcome)
