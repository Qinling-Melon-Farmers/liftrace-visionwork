"""Deterministic cursor for an interruptible coverage route."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .search_types import Waypoint


@dataclass(frozen=True)
class RouteDispatch:
    decision_seq: int
    command: str
    waypoint_index: int
    route_revision: str
    nominal_waypoint: Waypoint


@dataclass(frozen=True)
class RouteOutcome:
    accepted: bool
    reason: str
    advanced: bool = False
    skipped: bool = False
    complete: bool = False


class CoverageRoute:
    """Own the nominal waypoint cursor independently of planner adjustments.

    A target interruption only clears the active search dispatch.  It never
    advances the cursor.  Only a matching successful terminal result advances
    normally; repeated matching failures can skip one unreachable waypoint
    after the configured bound.
    """

    def __init__(self, waypoints: Sequence[Waypoint], route_revision: str,
                 max_failures_per_waypoint: int = 2):
        if not route_revision.strip():
            raise ValueError("route revision must not be empty")
        if (int(max_failures_per_waypoint) != max_failures_per_waypoint or
                max_failures_per_waypoint <= 0):
            raise ValueError("max failures per waypoint must be positive")
        frozen = tuple(waypoints)
        if not frozen:
            raise ValueError("coverage route must contain at least one waypoint")
        for point in frozen:
            if not all(math.isfinite(float(value)) for value in point.as_tuple()):
                raise ValueError("coverage waypoint coordinates must be finite")
            if point.z < 0.0 or point.z > 4.0:
                raise ValueError(
                    "coverage waypoint altitude is outside the competition limit")
        self._waypoints: Tuple[Waypoint, ...] = frozen
        self.route_revision = route_revision
        self.max_failures_per_waypoint = int(max_failures_per_waypoint)
        self.current_index = 0
        self.active: Optional[RouteDispatch] = None
        self.last_interrupted: Optional[RouteDispatch] = None
        self.failure_count = 0
        self.skipped_indices = []
        self._terminal_decisions = set()
        self._retired_decisions = set()
        self._last_decision_seq = 0

    @property
    def waypoints(self) -> Tuple[Waypoint, ...]:
        return self._waypoints

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self.is_complete:
            return None
        return self._waypoints[self.current_index]

    @property
    def is_complete(self) -> bool:
        return self.current_index >= len(self._waypoints)

    @property
    def coverage_ratio(self) -> float:
        return min(float(self.current_index) / len(self._waypoints), 1.0)

    def bind(self, decision_seq: int, command: str) -> RouteDispatch:
        if command not in ("SEARCH", "RESUME"):
            raise ValueError("route command must be SEARCH or RESUME")
        if int(decision_seq) <= 0:
            raise ValueError("decision sequence must be positive")
        if int(decision_seq) <= self._last_decision_seq:
            raise ValueError("decision sequence must increase monotonically")
        if self.is_complete:
            raise RuntimeError("coverage route is complete")
        if self.active is not None:
            raise RuntimeError("a route decision is already active")
        dispatch = RouteDispatch(
            decision_seq=int(decision_seq),
            command=command,
            waypoint_index=self.current_index,
            route_revision=self.route_revision,
            nominal_waypoint=self.current_waypoint,
        )
        self.active = dispatch
        self._last_decision_seq = dispatch.decision_seq
        return dispatch

    def interrupt(self, decision_seq: int) -> RouteOutcome:
        if self.active is None:
            return RouteOutcome(False, "route_has_no_active_decision")
        if int(decision_seq) != self.active.decision_seq:
            return RouteOutcome(False, "route_decision_mismatch")
        self.last_interrupted = self.active
        self._retired_decisions.add(self.active.decision_seq)
        self.active = None
        return RouteOutcome(True, "route_interrupted", complete=self.is_complete)

    def finish(self, decision_seq: int, succeeded: bool) -> RouteOutcome:
        decision_seq = int(decision_seq)
        if decision_seq in self._terminal_decisions:
            return RouteOutcome(False, "route_result_duplicate")
        if decision_seq in self._retired_decisions:
            return RouteOutcome(False, "route_result_retired")
        if self.active is None:
            return RouteOutcome(False, "route_has_no_active_decision")
        if decision_seq != self.active.decision_seq:
            return RouteOutcome(False, "route_decision_mismatch")

        self._terminal_decisions.add(decision_seq)
        self.active = None
        if succeeded:
            self.current_index += 1
            self.failure_count = 0
            return RouteOutcome(
                True,
                "route_waypoint_complete",
                advanced=True,
                complete=self.is_complete,
            )

        self.failure_count += 1
        if self.failure_count < self.max_failures_per_waypoint:
            return RouteOutcome(
                True,
                "route_waypoint_retry",
                complete=self.is_complete,
            )
        skipped_index = self.current_index
        self.skipped_indices.append(skipped_index)
        self.current_index += 1
        self.failure_count = 0
        return RouteOutcome(
            True,
            "route_waypoint_skipped",
            advanced=True,
            skipped=True,
            complete=self.is_complete,
        )
