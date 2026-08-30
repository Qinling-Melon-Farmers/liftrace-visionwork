#!/usr/bin/env python3
"""Read-only hard Gate for the typed VCL06 navigation execution chain.

The reducer below has no ROS dependency.  The ROS shell only translates
observations into immutable dictionaries, writes one report atomically and
terminates.  It never publishes commands and owns no mission policy.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
import sys
import tempfile
import threading
import time

try:  # Keep the reducer importable by system-Python unit tests.
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from patrol_control.msg import MissionCommand
    from uav_mission.msg import NavigationDecision, NavigationResult
    from uav_vision.msg import TargetCandidate
except ImportError:  # pragma: no cover - exercised by pure import environments.
    rosgraph = None
    rospy = None
    PoseStamped = String = MissionCommand = None
    NavigationDecision = NavigationResult = TargetCandidate = None


SCHEMA_VERSION = 1

SEARCH = 0
APPROACH = 1
ALIGN = 2
RESUME = 3
RETURN_HOME = 4
LAND = 5
HOLD = 6
ABORT = 7

ACCEPTED = 0
STARTED = 1
PROGRESS = 2
SUCCEEDED = 3
FAILED = 4
REJECTED = 5
CANCELLED = 6
TIMED_OUT = 7

DISPATCH = 0
PLANNER = 1
CAPTURE = 2
ALIGNMENT = 3
RELEASE = 4
RECOVERY = 5
LANDING = 6

COMMANDS = frozenset((SEARCH, APPROACH, ALIGN, RESUME, RETURN_HOME,
                      LAND, HOLD, ABORT))
SUCCESS_STATUSES = frozenset((ACCEPTED, STARTED, PROGRESS, SUCCEEDED))
def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _positive_int(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _int_or(value, default):
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


@dataclass(frozen=True)
class DecisionFence:
    mission_id: str
    decision_seq: int
    command: int
    class_profile: str
    has_target: bool
    target_id: int
    target_first_seen_ns: int
    target_class: str
    attempt: int
    payload_slot: int
    issued_ns: int
    deadline_ns: int

    @property
    def key(self):
        return self.mission_id, self.decision_seq

    @property
    def target_instance(self):
        return self.target_id, self.target_first_seen_ns


@dataclass(frozen=True)
class ResultEvent:
    mission_id: str
    executor_id: str
    event_seq: int
    decision_seq: int
    command: int
    status: int
    stage: int
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
    stamp_ns: int

    @property
    def identity(self):
        return self.mission_id, self.executor_id, self.event_seq


@dataclass(frozen=True)
class ApproachCommand:
    decision_seq: int
    target_id: int
    target_class: str
    stamp_ns: int


class Vcl06GateReducer:
    """Accumulate observations without making navigation decisions."""

    REQUIRED_STATUSES = (
        "manager", "start_gate", "field", "anchor", "contact", "bridge")

    def __init__(self, profile="r2026", nav_feature_profile="baseline",
                 mission_frame="camera_init", field_bounds=None,
                 max_height=4.0, max_mission_sec=600.0,
                 forced_return_sec=510.0,
                 expected_goal_publisher="/navigation/planner_bridge"):
        self.profile = str(profile)
        self.nav_feature_profile = str(nav_feature_profile)
        self.mission_frame = str(mission_frame)
        self.field_bounds = dict(field_bounds or {
            "min_x": -3.992, "max_x": 4.008,
            "min_y": -1.132, "max_y": 8.718,
        })
        self.max_height = float(max_height)
        self.max_mission_sec = float(max_mission_sec)
        self.forced_return_sec = float(forced_return_sec)
        self.expected_goal_publisher = str(expected_goal_publisher)

        self.decisions = {}
        self.results = {}
        self.unmatched_results = {}
        self.last_executor_sequence = {}
        self.release_commits = set()
        self.recovery_success = set()
        self.capture_started = set()
        self.return_success = set()
        self.land_success = set()
        self.approach_commands = []
        self.approach_command_bindings = {}
        self.selected_instances = set()
        self.selected_classes = []
        self.accepted_classes = []
        self.statuses = {}
        self.planner_goal_publishers = ()
        self.errors = []
        self.pose_samples = 0
        self.boundary_violations = 0
        self.height_violations = 0
        self.max_observed_height = None
        self.first_decision_receipt_wall = None
        self.land_success_receipt_wall = None

    def _error(self, reason):
        reason = str(reason)
        if reason not in self.errors:
            self.errors.append(reason)

    @staticmethod
    def _decision_from_dict(value):
        required = (
            "schema_version", "mission_id", "decision_seq", "header_seq",
            "command", "class_profile", "has_target", "target_id",
            "target_first_seen_ns", "target_class", "attempt",
            "payload_slot", "issued_ns", "deadline_ns")
        if not isinstance(value, dict) or any(name not in value for name in required):
            raise ValueError("decision_fields_missing")
        if value["schema_version"] != SCHEMA_VERSION:
            raise ValueError("decision_schema_invalid")
        if not value["mission_id"]:
            raise ValueError("decision_mission_id_empty")
        if not _positive_int(value["decision_seq"]):
            raise ValueError("decision_sequence_invalid")
        if value["header_seq"] != value["decision_seq"]:
            raise ValueError("decision_header_sequence_mismatch")
        if value["command"] not in COMMANDS:
            raise ValueError("decision_command_invalid")
        if not _positive_int(value["issued_ns"]):
            raise ValueError("decision_stamp_invalid")
        if (not _positive_int(value["deadline_ns"]) or
                value["deadline_ns"] <= value["issued_ns"]):
            raise ValueError("decision_deadline_invalid")
        has_target = bool(value["has_target"])
        if has_target != (value["command"] == APPROACH):
            raise ValueError("decision_target_flag_invalid")
        if has_target:
            if (not _positive_int(value["target_first_seen_ns"]) or
                    not value["target_class"] or
                    not _positive_int(value["attempt"]) or
                    value["payload_slot"] not in (1, 2, 3)):
                raise ValueError("decision_target_fence_invalid")
        elif any((value["target_id"], value["target_first_seen_ns"],
                  bool(value["target_class"]), value["attempt"],
                  value["payload_slot"])):
            raise ValueError("targetless_decision_has_identity")
        return DecisionFence(
            mission_id=str(value["mission_id"]),
            decision_seq=int(value["decision_seq"]),
            command=int(value["command"]),
            class_profile=str(value["class_profile"]),
            has_target=has_target,
            target_id=int(value["target_id"]),
            target_first_seen_ns=int(value["target_first_seen_ns"]),
            target_class=str(value["target_class"]),
            attempt=int(value["attempt"]),
            payload_slot=int(value["payload_slot"]),
            issued_ns=int(value["issued_ns"]),
            deadline_ns=int(value["deadline_ns"]),
        )

    @staticmethod
    def _result_from_dict(value):
        required = (
            "schema_version", "header_seq", "mission_id", "executor_id",
            "event_seq", "decision_seq", "command", "status", "stage",
            "terminal", "retryable", "payload_committed", "has_target",
            "target_id", "target_first_seen_ns", "target_class", "attempt",
            "payload_slot", "reason", "evidence_source", "stamp_ns")
        if not isinstance(value, dict) or any(name not in value for name in required):
            raise ValueError("result_fields_missing")
        if value["schema_version"] != SCHEMA_VERSION:
            raise ValueError("result_schema_invalid")
        if not value["mission_id"] or not value["executor_id"]:
            raise ValueError("result_source_identity_empty")
        if not _positive_int(value["event_seq"]):
            raise ValueError("result_event_sequence_invalid")
        if value["header_seq"] != value["event_seq"]:
            raise ValueError("result_header_sequence_mismatch")
        if not _positive_int(value["decision_seq"]):
            raise ValueError("result_decision_sequence_invalid")
        if (value["command"] not in COMMANDS or
                value["status"] not in range(ACCEPTED, TIMED_OUT + 1) or
                value["stage"] not in range(DISPATCH, LANDING + 1) or
                not _positive_int(value["stamp_ns"])):
            raise ValueError("result_contract_invalid")
        return ResultEvent(
            mission_id=str(value["mission_id"]),
            executor_id=str(value["executor_id"]),
            event_seq=int(value["event_seq"]),
            decision_seq=int(value["decision_seq"]),
            command=int(value["command"]),
            status=int(value["status"]),
            stage=int(value["stage"]),
            terminal=bool(value["terminal"]),
            retryable=bool(value["retryable"]),
            payload_committed=bool(value["payload_committed"]),
            has_target=bool(value["has_target"]),
            target_id=int(value["target_id"]),
            target_first_seen_ns=int(value["target_first_seen_ns"]),
            target_class=str(value["target_class"]),
            attempt=int(value["attempt"]),
            payload_slot=int(value["payload_slot"]),
            reason=str(value["reason"]),
            evidence_source=str(value["evidence_source"]),
            stamp_ns=int(value["stamp_ns"]),
        )

    @staticmethod
    def _result_matches_decision(event, decision):
        return (
            event.mission_id == decision.mission_id and
            event.decision_seq == decision.decision_seq and
            event.command == decision.command and
            event.has_target == decision.has_target and
            event.target_id == decision.target_id and
            event.target_first_seen_ns == decision.target_first_seen_ns and
            event.target_class == decision.target_class and
            event.attempt == decision.attempt and
            event.payload_slot == decision.payload_slot)

    def observe_decision(self, value, receipt_wall=None):
        try:
            decision = self._decision_from_dict(value)
        except ValueError as error:
            self._error(error)
            return
        if decision.class_profile != self.profile:
            self._error("decision_profile_mismatch")
        previous = self.decisions.get(decision.key)
        if previous is not None and previous != decision:
            self._error("decision_identity_conflict")
            return
        self.decisions[decision.key] = decision
        if self.first_decision_receipt_wall is None:
            self.first_decision_receipt_wall = (
                float(receipt_wall) if receipt_wall is not None else None)
        self._bind_approach_commands()
        for identity, pending in list(self.unmatched_results.items()):
            event, pending_receipt_wall = pending
            if (event.mission_id, event.decision_seq) == decision.key:
                del self.unmatched_results[identity]
                self._accept_result(event, pending_receipt_wall)

    def observe_result(self, value, receipt_wall=None):
        try:
            event = self._result_from_dict(value)
        except ValueError as error:
            self._error(error)
            return
        previous = self.results.get(event.identity)
        if previous is None and event.identity in self.unmatched_results:
            previous = self.unmatched_results[event.identity][0]
        if previous is not None:
            if previous != event:
                self._error("result_identity_conflict")
            return
        decision = self.decisions.get((event.mission_id, event.decision_seq))
        if decision is None:
            self.unmatched_results[event.identity] = (event, receipt_wall)
            return
        self._accept_result(event, receipt_wall)

    def _accept_result(self, event, receipt_wall):
        last_sequence = self.last_executor_sequence.get(event.executor_id, 0)
        if event.event_seq <= last_sequence:
            self._error("result_event_sequence_not_monotonic")
            return
        decision = self.decisions.get((event.mission_id, event.decision_seq))
        if decision is None:
            self._error("internal_result_without_decision")
            return
        if not self._result_matches_decision(event, decision):
            self._error("result_decision_fence_mismatch")
            return
        late_success = (event.status in SUCCESS_STATUSES and
                        event.stamp_ns >= decision.deadline_ns)
        if late_success:
            self._error("successful_result_at_or_after_deadline")
        self.results[event.identity] = event
        self.last_executor_sequence[event.executor_id] = event.event_seq

        if event.target_class == "tank" and event.status in SUCCESS_STATUSES:
            self.accepted_classes.append(event.target_class)
            self._error("tank_accepted")
        elif event.status in SUCCESS_STATUSES and event.has_target:
            self.accepted_classes.append(event.target_class)

        key = decision.key
        if (event.command == APPROACH and event.status == STARTED and
                event.stage == CAPTURE and not event.terminal):
            self.capture_started.add(key)
        if (event.command == APPROACH and event.status == PROGRESS and
                event.stage == RELEASE and
                event.payload_committed and not event.retryable and
                not event.terminal and
                event.reason == "release_ack_success" and
                bool(event.evidence_source)):
            if key not in self.capture_started:
                self._error("release_before_capture")
            if key in self.release_commits:
                self._error("duplicate_release_commit")
            self.release_commits.add(key)
            if late_success:
                self._error("late_payload_commit")
        if (event.status == SUCCEEDED and event.stage == RECOVERY and
                event.terminal and event.command == APPROACH):
            if key not in self.release_commits:
                self._error("recovery_before_release")
            if key in self.recovery_success:
                self._error("duplicate_recovery_success")
            self.recovery_success.add(key)
        if (event.status == SUCCEEDED and event.terminal and
                event.command == RETURN_HOME):
            if event.stage != PLANNER:
                self._error("return_home_success_stage_invalid")
            else:
                if key in self.return_success:
                    self._error("duplicate_return_home_success")
                self.return_success.add(key)
        if (event.status == SUCCEEDED and event.stage == LANDING and
                event.terminal and event.command == LAND):
            if key in self.land_success:
                self._error("duplicate_land_success")
            self.land_success.add(key)
            if receipt_wall is not None:
                self.land_success_receipt_wall = float(receipt_wall)

    def observe_mission_command(self, command, decision_seq, target_id,
                                target_class, stamp_ns):
        if int(command) != APPROACH:
            return
        item = ApproachCommand(
            int(decision_seq), int(target_id), str(target_class),
            int(stamp_ns))
        if not _positive_int(item.decision_seq):
            self._error("mission_command_approach_sequence_invalid")
            return
        if not item.target_class:
            self._error("mission_command_approach_class_empty")
            return
        if not _positive_int(item.stamp_ns):
            self._error("mission_command_approach_stamp_invalid")
            return
        self.approach_commands.append(item)
        self._bind_approach_commands()

    def _bind_approach_commands(self):
        for index, command in enumerate(self.approach_commands):
            if index in self.approach_command_bindings:
                continue
            sequence_candidates = [
                decision for decision in self.decisions.values()
                if (decision.command == APPROACH and
                    decision.decision_seq == command.decision_seq)
            ]
            if len(sequence_candidates) > 1:
                self._error("mission_command_approach_ambiguous")
            elif len(sequence_candidates) == 1:
                decision = sequence_candidates[0]
                if (decision.target_id != command.target_id or
                        decision.target_class != command.target_class):
                    self._error("mission_command_approach_fence_mismatch")
                elif not (decision.issued_ns <= command.stamp_ns <
                          decision.deadline_ns):
                    self._error("mission_command_approach_stamp_out_of_range")
                else:
                    self.approach_command_bindings[index] = decision.key

    def observe_selected(self, class_name, target_id=0,
                         target_first_seen_ns=0):
        class_name = str(class_name)
        self.selected_classes.append(class_name)
        self.selected_instances.add(
            (int(target_id), int(target_first_seen_ns)))
        if class_name == "tank":
            self._error("tank_selected")

    def observe_status(self, name, payload):
        if name not in self.REQUIRED_STATUSES:
            self._error("unknown_status_component:%s" % name)
            return
        if not isinstance(payload, dict):
            self._error("invalid_%s_status" % name)
            return
        self.statuses[name] = dict(payload)
        status = str(payload.get("status", ""))
        if status in ("FAIL", "ERROR"):
            self._error("%s_status_%s" % (name, status.lower()))
        if name == "manager":
            if payload.get("mission_failed") is True or status == "ABORTED":
                self._error("manager_failed")
            if str(payload.get("phase", "")) == "ABORTED":
                self._error("manager_aborted")
        elif name == "start_gate" and status == "DISABLED":
            self._error("start_gate_disabled")
        elif name == "contact":
            count = payload.get("actual_collision_count")
            if isinstance(count, int) and count > 0:
                self._error("actual_collision")
        elif name == "bridge":
            if payload.get("adapter_faulted") is True:
                self._error("bridge_faulted")
            if payload.get("output_enabled") is False:
                self._error("bridge_output_disabled")

    def observe_planner_goal_publishers(self, publishers):
        names = tuple(sorted(set(str(item) for item in publishers)))
        self.planner_goal_publishers = names
        if names and names != (self.expected_goal_publisher,):
            self._error("planner_goal_publisher_set_invalid")

    def observe_pose(self, x, y, z, frame_id):
        values = (x, y, z)
        if not all(_finite(value) for value in values):
            self._error("pose_non_finite")
            return
        if str(frame_id) != self.mission_frame:
            self._error("pose_frame_mismatch")
            return
        x, y, z = (float(value) for value in values)
        self.pose_samples += 1
        self.max_observed_height = (
            z if self.max_observed_height is None else
            max(self.max_observed_height, z))
        if (x < self.field_bounds["min_x"] or
                x > self.field_bounds["max_x"] or
                y < self.field_bounds["min_y"] or
                y > self.field_bounds["max_y"]):
            self.boundary_violations += 1
            self._error("field_boundary_violation")
        if z > self.max_height:
            self.height_violations += 1
            self._error("height_limit_violation")

    def _bound_command_keys(self):
        return set(self.approach_command_bindings.values())

    @staticmethod
    def _ready_status(payload, profile):
        return bool(payload and payload.get("ready") and
                    payload.get("status") == "READY" and
                    payload.get("profile") == profile)

    def _checks(self):
        committed = [self.decisions[key] for key in self.release_commits
                     if key in self.decisions]
        committed_keys = set(self.release_commits)
        target_instances = {item.target_instance for item in committed}
        slots = sorted(item.payload_slot for item in committed)
        bound_commands = self._bound_command_keys()
        return_decisions = [self.decisions[key] for key in self.return_success]
        land_decisions = [self.decisions[key] for key in self.land_success]
        mission_ids = {item.mission_id for item in self.decisions.values()}
        committed_mission_ids = {item.mission_id for item in committed}
        three_release_commits = (
            len(committed_keys) == 3 and len(target_instances) == 3 and
            slots == [1, 2, 3] and len(committed_mission_ids) == 1)
        first_issued = min(
            (item.issued_ns for item in self.decisions.values()), default=None)
        return_issued = min(
            (item.issued_ns for item in return_decisions), default=None)
        land_stamp = min((event.stamp_ns for event in self.results.values()
                          if ((event.mission_id, event.decision_seq) in
                              self.land_success)), default=None)
        mission_ros_sec = None
        if first_issued is not None and land_stamp is not None:
            mission_ros_sec = (land_stamp - first_issued) / 1e9
        mission_wall_sec = None
        if (self.first_decision_receipt_wall is not None and
                self.land_success_receipt_wall is not None):
            mission_wall_sec = (self.land_success_receipt_wall -
                                self.first_decision_receipt_wall)

        manager = self.statuses.get("manager") or {}
        start_gate = self.statuses.get("start_gate") or {}
        field = self.statuses.get("field") or {}
        anchor = self.statuses.get("anchor") or {}
        contact = self.statuses.get("contact") or {}
        bridge = self.statuses.get("bridge") or {}
        checks = {
            "required_statuses_seen": all(
                name in self.statuses for name in self.REQUIRED_STATUSES),
            "single_mission": len(mission_ids) == 1,
            "manager_complete": (
                manager.get("phase") == "COMPLETE" and
                manager.get("mission_failed") is False and
                _int_or(manager.get("committed_slots"), -1) == 3),
            "manager_identity_matches": (
                len(mission_ids) == 1 and
                manager.get("mission_id") in mission_ids),
            "start_gate_started_once": (
                start_gate.get("status") == "STARTED" and
                start_gate.get("started_latched") is True and
                _int_or(start_gate.get("service_call_count"), -1) == 1),
            "field_ready": (
                self._ready_status(field, self.profile) and
                field.get("footprint_valid") is True and
                _int_or(field.get("seed"), 0) != 0),
            "anchor_ready": self._ready_status(
                anchor, self.nav_feature_profile),
            "contact_ready_zero": (
                contact.get("ready") is True and
                contact.get("status") == "READY" and
                _int_or(contact.get("actual_collision_count"), -1) == 0),
            "bridge_live_and_healthy": (
                bridge.get("adapter_faulted") is False and
                bridge.get("output_enabled") is True and
                bridge.get("planner_goal_topic") == "/fastplanner/goal" and
                bridge.get("gate_reason") == "live_planner_output_enabled"),
            "single_planner_goal_publisher": (
                self.planner_goal_publishers ==
                (self.expected_goal_publisher,)),
            "three_release_commits": three_release_commits,
            "three_recovery_successes": (
                len(self.recovery_success) == 3 and
                committed_keys == self.recovery_success),
            "three_capture_started": (
                three_release_commits and
                committed_keys.issubset(self.capture_started)),
            "real_approach_commands": (
                three_release_commits and
                committed_keys.issubset(bound_commands)),
            "committed_targets_were_selected": (
                three_release_commits and
                target_instances.issubset(self.selected_instances)),
            "return_home_success": len(self.return_success) == 1,
            "land_success": len(self.land_success) == 1,
            "return_before_land": (
                len(return_decisions) == 1 and len(land_decisions) == 1 and
                return_decisions[0].decision_seq <
                land_decisions[0].decision_seq),
            "return_after_deliveries": (
                len(return_decisions) == 1 and bool(committed) and
                return_decisions[0].mission_id in committed_mission_ids and
                return_decisions[0].decision_seq >
                max(item.decision_seq for item in committed)),
            "forced_return_within_limit": (
                first_issued is not None and return_issued is not None and
                0.0 <= (return_issued - first_issued) / 1e9 <=
                self.forced_return_sec),
            "mission_ros_within_limit": (
                mission_ros_sec is not None and
                0.0 <= mission_ros_sec <= self.max_mission_sec),
            "mission_wall_within_limit": (
                mission_wall_sec is not None and
                0.0 <= mission_wall_sec <= self.max_mission_sec),
            "tank_selected_zero": "tank" not in self.selected_classes,
            "tank_accepted_zero": "tank" not in self.accepted_classes,
            "pose_seen": self.pose_samples > 0,
            "zero_boundary_violations": self.boundary_violations == 0,
            "zero_height_violations": self.height_violations == 0,
            "zero_collisions": (
                _int_or(contact.get("actual_collision_count"), -1) == 0),
            "unmatched_results_zero": not self.unmatched_results,
            "contract_errors_zero": not self.errors,
        }
        metrics = {
            "decision_count": len(self.decisions),
            "result_count": len(self.results),
            "unmatched_result_count": len(self.unmatched_results),
            "release_commit_count": len(self.release_commits),
            "recovery_success_count": len(self.recovery_success),
            "capture_started_count": len(self.capture_started),
            "approach_command_count": len(bound_commands),
            "selected_count": len(self.selected_classes),
            "planner_goal_publishers": list(self.planner_goal_publishers),
            "pose_samples": self.pose_samples,
            "boundary_violations": self.boundary_violations,
            "height_violations": self.height_violations,
            "max_observed_height": self.max_observed_height,
            "mission_ros_sec": mission_ros_sec,
            "mission_wall_sec": mission_wall_sec,
        }
        return checks, metrics

    def report(self, timed_out=False):
        checks, metrics = self._checks()
        errors = list(self.errors)
        if timed_out and "wall_timeout" not in errors:
            errors.append("wall_timeout")
        hard_failure = bool(errors)
        complete = all(checks.values())
        status = "FAIL" if hard_failure or timed_out else (
            "PASS" if complete else "WAITING")
        failed_checks = sorted(name for name, value in checks.items()
                               if not value)
        return {
            "status": status,
            "reason": (errors[0] if errors else
                       ("all_checks_passed" if complete else
                        "waiting_for_required_evidence")),
            "checks": checks,
            "failed_checks": failed_checks,
            "errors": errors,
            "metrics": metrics,
            "profile": self.profile,
            "nav_feature_profile": self.nav_feature_profile,
            "mission_frame": self.mission_frame,
            "decision_fences": [
                asdict(item) for item in sorted(
                    self.decisions.values(),
                    key=lambda value: (value.mission_id,
                                       value.decision_seq))],
        }


def _stamp_ns(stamp):
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


class NavigationVcl06AssertionNode:
    """ROS subscriber shell around :class:`Vcl06GateReducer`."""

    def __init__(self):
        if rospy is None:
            raise RuntimeError("ROS imports are unavailable")
        self._lock = threading.RLock()
        self._finished = False
        self.exit_code = 1
        self._started_wall = time.monotonic()
        self._wall_timeout = float(rospy.get_param("~wall_timeout", 650.0))
        self._planner_goal_topic = rospy.get_param(
            "~planner_goal_topic", "/fastplanner/goal")
        self._expected_goal_publisher = rospy.get_param(
            "~expected_planner_goal_publisher",
            "/navigation/planner_bridge")
        self._master = rosgraph.Master(rospy.get_name())
        run_dir = os.environ.get("SIM_RUN_DIR", "/tmp")
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(run_dir, "gate_status.json"))
        self.reducer = Vcl06GateReducer(
            profile=rospy.get_param("~class_profile", "r2026"),
            nav_feature_profile=rospy.get_param(
                "~nav_feature_profile", "baseline"),
            mission_frame=rospy.get_param("~mission_frame", "camera_init"),
            field_bounds={
                "min_x": float(rospy.get_param("~field/min_x", -3.992)),
                "max_x": float(rospy.get_param("~field/max_x", 4.008)),
                "min_y": float(rospy.get_param("~field/min_y", -1.132)),
                "max_y": float(rospy.get_param("~field/max_y", 8.718)),
            },
            max_height=float(rospy.get_param("~max_height", 4.0)),
            max_mission_sec=float(rospy.get_param(
                "~max_mission_sec", 600.0)),
            forced_return_sec=float(rospy.get_param(
                "~forced_return_sec", 510.0)),
            expected_goal_publisher=self._expected_goal_publisher,
        )

        topics = {
            "decision": rospy.get_param(
                "~decision_topic", "/navigation/mission_command_raw"),
            "result": rospy.get_param(
                "~result_topic", "/navigation/mission_result"),
            "mission_command": rospy.get_param(
                "~mission_command_topic", "/mission/command"),
            "manager": rospy.get_param(
                "~manager_status_topic", "/navigation/mission_status"),
            "start_gate": rospy.get_param(
                "~start_gate_status_topic",
                "/navigation/mission_start_gate_status"),
            "field": rospy.get_param(
                "~field_status_topic", "/mission/random_field_status"),
            "anchor": rospy.get_param(
                "~anchor_status_topic", "/mission/planner_anchor_status"),
            "contact": rospy.get_param(
                "~contact_status_topic", "/mission/gazebo_contact_status"),
            "bridge": rospy.get_param(
                "~bridge_status_topic", "/navigation/planner_bridge_status"),
            "pose": rospy.get_param(
                "~pose_topic", "/mavros/local_position/pose"),
            "selected": rospy.get_param(
                "~selected_topic", "/uav_vision/selected_target"),
        }
        rospy.Subscriber(topics["decision"], NavigationDecision,
                         self._on_decision, queue_size=20)
        rospy.Subscriber(topics["result"], NavigationResult,
                         self._on_result, queue_size=100)
        rospy.Subscriber(topics["mission_command"], MissionCommand,
                         self._on_mission_command, queue_size=20)
        for name in Vcl06GateReducer.REQUIRED_STATUSES:
            rospy.Subscriber(topics[name], String, self._status_callback(name),
                             queue_size=20)
        rospy.Subscriber(topics["pose"], PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber(topics["selected"], TargetCandidate,
                         self._on_selected, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)

    @staticmethod
    def _decision_dict(message):
        return {
            "schema_version": int(message.schema_version),
            "mission_id": message.mission_id,
            "decision_seq": int(message.decision_seq),
            "header_seq": int(message.header.seq),
            "command": int(message.command),
            "class_profile": message.class_profile,
            "has_target": bool(message.has_target),
            "target_id": int(message.target_id),
            "target_first_seen_ns": _stamp_ns(message.target_first_seen),
            "target_class": message.target_class,
            "attempt": int(message.attempt),
            "payload_slot": int(message.payload_slot),
            "issued_ns": _stamp_ns(message.header.stamp),
            "deadline_ns": _stamp_ns(message.deadline),
        }

    @staticmethod
    def _result_dict(message):
        return {
            "schema_version": int(message.schema_version),
            "header_seq": int(message.header.seq),
            "mission_id": message.mission_id,
            "executor_id": message.executor_id,
            "event_seq": int(message.event_seq),
            "decision_seq": int(message.decision_seq),
            "command": int(message.command),
            "status": int(message.status),
            "stage": int(message.stage),
            "terminal": bool(message.terminal),
            "retryable": bool(message.retryable),
            "payload_committed": bool(message.payload_committed),
            "has_target": bool(message.has_target),
            "target_id": int(message.target_id),
            "target_first_seen_ns": _stamp_ns(message.target_first_seen),
            "target_class": message.target_class,
            "attempt": int(message.attempt),
            "payload_slot": int(message.payload_slot),
            "reason": message.reason,
            "evidence_source": message.evidence_source,
            "stamp_ns": _stamp_ns(message.header.stamp),
        }

    def _on_decision(self, message):
        with self._lock:
            self.reducer.observe_decision(
                self._decision_dict(message), time.monotonic())
            self._check_terminal()

    def _on_result(self, message):
        with self._lock:
            self.reducer.observe_result(
                self._result_dict(message), time.monotonic())
            self._check_terminal()

    def _on_mission_command(self, message):
        with self._lock:
            self.reducer.observe_mission_command(
                int(message.command), int(message.header.seq),
                int(message.target_id), message.target_class,
                _stamp_ns(message.header.stamp))
            self._check_terminal()

    def _status_callback(self, name):
        def callback(message):
            with self._lock:
                try:
                    payload = json.loads(message.data)
                except (TypeError, ValueError):
                    self.reducer._error("invalid_%s_json" % name)
                else:
                    self.reducer.observe_status(name, payload)
                self._check_terminal()
        return callback

    def _on_pose(self, message):
        with self._lock:
            position = message.pose.position
            self.reducer.observe_pose(
                position.x, position.y, position.z,
                message.header.frame_id)
            self._check_terminal()

    def _on_selected(self, message):
        with self._lock:
            self.reducer.observe_selected(
                message.class_name, int(message.id),
                _stamp_ns(message.first_seen))
            self._check_terminal()

    def _on_timer(self, _event):
        with self._lock:
            try:
                publishers, _, _ = self._master.getSystemState()
                nodes = next((names for topic, names in publishers
                              if topic == self._planner_goal_topic), ())
                self.reducer.observe_planner_goal_publishers(nodes)
            except rosgraph.MasterError:
                pass
            timed_out = (time.monotonic() - self._started_wall >=
                         self._wall_timeout)
            self._check_terminal(timed_out=timed_out)

    def _write_report(self, report):
        directory = os.path.dirname(os.path.abspath(self._report_path))
        os.makedirs(directory, exist_ok=True)
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=".gate_status.", suffix=".tmp", dir=directory)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(report, handle, ensure_ascii=False,
                          indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self._report_path)
        finally:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)

    def _check_terminal(self, timed_out=False):
        if self._finished:
            return
        report = self.reducer.report(timed_out=timed_out)
        if report["status"] == "WAITING":
            return
        self._finished = True
        self.exit_code = 0 if report["status"] == "PASS" else 1
        try:
            self._write_report(report)
        except Exception as error:  # pylint: disable=broad-except
            self.exit_code = 1
            rospy.logerr("VCL06 Gate report write failed: %s", error)
        rospy.loginfo("VCL06 Gate terminal status=%s reason=%s",
                      report["status"], report["reason"])
        rospy.signal_shutdown("VCL06 Gate %s" % report["status"])


def main():
    if rospy is None:
        raise RuntimeError("ROS imports are unavailable")
    rospy.init_node("navigation_vcl06_assertion")
    node = NavigationVcl06AssertionNode()
    rospy.spin()
    return node.exit_code


if __name__ == "__main__":
    sys.exit(main())
