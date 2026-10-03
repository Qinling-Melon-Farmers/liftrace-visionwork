import math
import os
from dataclasses import replace

from uav_mission.mission_core import MissionCore, MissionPhase
from uav_mission.profile_policy import CompetitionProfile
from uav_mission.coverage_route import CoverageRoute, RouteOutcome


class StrictRoute(CoverageRoute):
    def finish(self, decision_seq, succeeded):
        if (self.active is not None and self.active.decision_seq == decision_seq
                and not succeeded and self.failure_count + 1 >= self.max_failures_per_waypoint):
            return RouteOutcome(False, "waypoint_failure_budget_exhausted")
        return super().finish(decision_seq, succeeded)


class SingleDropCore(MissionCore):
    def __init__(self, config):
        super().__init__(CompetitionProfile("red_cross_4x5", {"red_cross": 10.0}, 1, 1), config)
        self.attempt_started = False
        self.delivery_outcome = "not_attempted"

    def _new_action(self, *args, **kwargs):
        action = replace(super()._new_action(*args, **kwargs), profile_name="r2026")
        self.active_action = action
        return action

    def choose(self, now, current_xy, route_complete=False):
        if self.phase != MissionPhase.SEARCH:
            return None
        if route_complete:
            return super()._return_action("test_route_complete", now)
        if self._elapsed(now) >= self._search_cutoff():
            return self._abort_action("test_mission_timeout", now)
        if self.attempt_started:
            return None
        action = super().choose(now, current_xy, False)
        if action is not None and action.command == "APPROACH":
            self.attempt_started = True
            self.delivery_outcome = "in_progress"
        return action

    def _post_delivery_route_action(self, reason, now, start=False):
        if reason == "required_deliveries_complete":
            self.delivery_outcome = "acknowledged_and_recovered"
            self.phase = MissionPhase.SEARCH
            self.active_action = None
            return None
        return super()._post_delivery_route_action(reason, now, start)

    def _return_action(self, reason, now):
        recoverable = {
            "committed_recovery_failed", "candidate_release_state_uncertain",
            "committed_recovery_timed_out", "target_action_timed_out_uncertain",
        }
        if self.attempt_started and reason in recoverable:
            self.delivery_outcome = reason
            self.phase = MissionPhase.SEARCH
            self.active_action = None
            self.active_release_started = False
            return None
        return super()._return_action(reason, now)

    def _apply_target_result(self, action, event, now):
        outcome = super()._apply_target_result(action, event, now)
        if outcome[0] and outcome[1] == "candidate_failed":
            self.delivery_outcome = "failed_before_release"
        return outcome

    def ingest(self, candidates, now):
        bounded = tuple(candidate for candidate in candidates
                        if -1.4 <= candidate.x <= 1.4 and 0.6 <= candidate.y <= 4.4)
        return super().ingest(bounded, now)


def bounded_z(value, limit=0.25):
    if not math.isfinite(value):
        raise ValueError("nonfinite setpoint height")
    return min(value, limit)


class OnceRelease:
    def __init__(self, receipt_path):
        self.receipt_path = receipt_path

    def consume(self, logical_slot):
        if logical_slot != 1:
            return False
        try:
            descriptor = os.open(self.receipt_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            return False
        with os.fdopen(descriptor, "w") as receipt:
            receipt.write("logical_slot=1 physical_req=2 consumed_before_call\n")
            receipt.flush()
            os.fsync(receipt.fileno())
        return True


def landing_ready(now, state, state_at, odom, endpoint):
    if state is None or odom is None:
        return False
    if not state.connected or not state.armed or state.mode != "OFFBOARD":
        return False
    if not 0 <= now - state_at <= 0.5:
        return False
    if odom.header.frame_id != "camera_init" or not -0.05 <= now - odom.header.stamp.to_sec() <= 0.3:
        return False
    position = odom.pose.pose.position
    velocity = odom.twist.twist.linear
    values = (position.x, position.y, position.z, velocity.x, velocity.y, velocity.z)
    return (all(math.isfinite(value) for value in values)
            and math.hypot(position.x - endpoint[0], position.y - endpoint[1]) <= 0.15
            and abs(position.z - 0.25) <= 0.08
            and math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) <= 0.12)
