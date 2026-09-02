#!/usr/bin/env python3
"""ROS1 runtime shell for the navigation-owned VCL06 mission contract."""

import json
import math
from numbers import Real
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from uav_vision.msg import TargetCandidateArray

from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import (
    CandidateSnapshot,
    GoalSnapshot,
    MissionConfig,
    MissionCore,
    MissionPhase,
    ResultEvent,
)
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.msg import NavigationDecision, NavigationResult
from uav_mission.profile_policy import load_profile
from uav_mission.search_policy import SearchPolicy


COMMAND_NAMES = {
    NavigationResult.SEARCH: "SEARCH",
    NavigationResult.APPROACH: "APPROACH",
    NavigationResult.ALIGN: "ALIGN",
    NavigationResult.RESUME: "RESUME",
    NavigationResult.RETURN_HOME: "RETURN_HOME",
    NavigationResult.LAND: "LAND",
    NavigationResult.HOLD: "HOLD",
    NavigationResult.ABORT: "ABORT",
}
COMMAND_VALUES = {
    "SEARCH": NavigationDecision.SEARCH,
    "APPROACH": NavigationDecision.APPROACH,
    "ALIGN": NavigationDecision.ALIGN,
    "RESUME": NavigationDecision.RESUME,
    "RETURN_HOME": NavigationDecision.RETURN_HOME,
    "LAND": NavigationDecision.LAND,
    "HOLD": NavigationDecision.HOLD,
    "ABORT": NavigationDecision.ABORT,
}
STATUS_NAMES = {
    NavigationResult.ACCEPTED: "ACCEPTED",
    NavigationResult.STARTED: "STARTED",
    NavigationResult.PROGRESS: "PROGRESS",
    NavigationResult.SUCCEEDED: "SUCCEEDED",
    NavigationResult.FAILED: "FAILED",
    NavigationResult.REJECTED: "REJECTED",
    NavigationResult.CANCELLED: "CANCELLED",
    NavigationResult.TIMED_OUT: "TIMED_OUT",
}
STAGE_NAMES = {
    NavigationResult.DISPATCH: "DISPATCH",
    NavigationResult.PLANNER: "PLANNER",
    NavigationResult.CAPTURE: "CAPTURE",
    NavigationResult.ALIGNMENT: "ALIGNMENT",
    NavigationResult.RELEASE: "RELEASE",
    NavigationResult.RECOVERY: "RECOVERY",
    NavigationResult.LANDING: "LANDING",
}


def _stamp_to_ns(stamp):
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


def _ns_to_stamp(value):
    value = int(value)
    return rospy.Time(
        secs=value // 1_000_000_000,
        nsecs=value % 1_000_000_000,
    )


def _phase_name(phase):
    return phase.value if hasattr(phase, "value") else str(phase)


class NavigationMissionManager:
    """Serialize all ROS callbacks around one deterministic runtime."""

    def __init__(self):
        self._lock = threading.RLock()
        self._runtime = None
        self._pose = None
        self._map = None
        self._mission_counter = 0
        self._last_reason = "waiting_for_manual_start"
        self._last_status = ""

        self._profile_path = rospy.get_param("~profile/path")
        self._profile_name = rospy.get_param("~profile/name", "r2026")
        self._pose_max_age = float(
            rospy.get_param("~readiness/pose_max_age", 0.5))
        self._map_max_age = float(
            rospy.get_param("~readiness/map_max_age", 2.0))
        self._stamp_future_tolerance = float(
            rospy.get_param(
                "~readiness/stamp_future_tolerance", 0.05))
        self._require_map = rospy.get_param("~readiness/require_map", True)
        self._tick_hz = float(rospy.get_param("~runtime/tick_hz", 10.0))
        self._mission_id_prefix = str(
            rospy.get_param("~runtime/mission_id_prefix", "vcl06"))
        self._validate_shell_parameters()

        self._decision_pub = rospy.Publisher(
            "mission_command_raw", NavigationDecision,
            queue_size=1, latch=True)
        self._status_pub = rospy.Publisher(
            "mission_status", String, queue_size=1, latch=True)

        self._candidate_sub = rospy.Subscriber(
            "candidates", TargetCandidateArray,
            self._on_candidates, queue_size=1)
        self._pose_sub = rospy.Subscriber(
            "pose", PoseStamped, self._on_pose, queue_size=1)
        self._map_sub = rospy.Subscriber(
            "map", PointCloud2, self._on_map, queue_size=1)
        self._result_sub = rospy.Subscriber(
            "mission_result", NavigationResult,
            self._on_result, queue_size=20)

        self._start_service = rospy.Service(
            "start_mission", Trigger, self._on_start)
        self._abort_service = rospy.Service(
            "abort_mission", Trigger, self._on_abort)
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self._tick_hz), self._on_timer)
        self._publish_status(force=True)
        rospy.loginfo(
            "VCL06 mission manager ready; manual start required, profile=%s",
            self._profile_name,
        )

    def _validate_shell_parameters(self):
        numeric = {
            "pose_max_age": self._pose_max_age,
            "map_max_age": self._map_max_age,
            "tick_hz": self._tick_hz,
        }
        for name, value in numeric.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be finite and positive" % name)
        if (not math.isfinite(self._stamp_future_tolerance) or
                self._stamp_future_tolerance < 0.0 or
                self._stamp_future_tolerance > 0.1):
            raise ValueError(
                "stamp_future_tolerance must be finite and within [0, 0.1]")
        if not isinstance(self._require_map, bool):
            raise ValueError("readiness/require_map must be boolean")
        if self._profile_name == "r2026" and not self._require_map:
            raise ValueError("r2026 requires map readiness")
        if not self._mission_id_prefix.strip():
            raise ValueError("runtime/mission_id_prefix must not be empty")

    def _mission_config(self):
        mission_frame = rospy.get_param("~mission/frame", "camera_init")
        route_values = rospy.get_param(
            "~mission/post_delivery_route", [])
        if not isinstance(route_values, (list, tuple)):
            raise ValueError("mission/post_delivery_route must be a list")
        post_delivery_route = []
        for index, point in enumerate(route_values):
            if (not isinstance(point, (list, tuple)) or len(point) != 3 or
                    any(isinstance(value, bool) or not isinstance(value, Real)
                        for value in point)):
                raise ValueError(
                    "mission/post_delivery_route[%d] must be [x,y,z]" %
                    index)
            post_delivery_route.append(GoalSnapshot(
                mission_frame, *(float(value) for value in point)))
        return MissionConfig(
            mission_frame=mission_frame,
            candidate_max_age=rospy.get_param(
                "~mission/candidate_max_age", 0.5),
            transform_max_age=rospy.get_param(
                "~mission/transform_max_age", 0.5),
            min_streak=rospy.get_param("~mission/min_streak", 3),
            max_target_z=rospy.get_param("~mission/max_target_z", 4.0),
            max_attempts=rospy.get_param("~mission/max_attempts", 2),
            retry_cooldown=rospy.get_param(
                "~mission/retry_cooldown", 20.0),
            mission_timeout=rospy.get_param("~mission/timeout", 600.0),
            forced_return_at=rospy.get_param(
                "~mission/forced_return_at", 510.0),
            return_land_reserve=rospy.get_param(
                "~mission/return_land_reserve", 90.0),
            delivery_reserve_per_slot=rospy.get_param(
                "~mission/delivery_reserve_per_slot", 60.0),
            path_factor=rospy.get_param("~mission/path_factor", 1.5),
            nominal_speed=rospy.get_param("~mission/nominal_speed", 1.0),
            decision_guard=rospy.get_param("~mission/decision_guard", 15.0),
            approach_altitude=rospy.get_param(
                "~mission/approach_altitude", 1.2),
            return_altitude=rospy.get_param(
                "~mission/return_altitude", 2.2),
            target_action_timeout=rospy.get_param(
                "~mission/target_action_timeout", 90.0),
            motion_action_timeout=rospy.get_param(
                "~mission/motion_action_timeout", 60.0),
            landing_action_timeout=rospy.get_param(
                "~mission/landing_action_timeout", 90.0),
            result_future_tolerance=rospy.get_param(
                "~mission/result_future_tolerance", 0.1),
            home_xy=rospy.get_param("~mission/home_xy", [0.0, 0.0]),
            post_delivery_route=tuple(post_delivery_route),
            post_delivery_route_revision=rospy.get_param(
                "~mission/post_delivery_route_revision", "direct-home-v1"),
            landing_xy=rospy.get_param(
                "~mission/landing_xy", [0.0, 0.0]),
            landing_anchor_tolerance=rospy.get_param(
                "~mission/landing_anchor_tolerance", 0.15),
        )

    def _new_runtime(self):
        config = self._mission_config()
        profile = load_profile(self._profile_path, self._profile_name)
        search = SearchPolicy(
            min_x=rospy.get_param("~search/min_x", -3.6),
            max_x=rospy.get_param("~search/max_x", 2.6),
            min_y=rospy.get_param("~search/min_y", -2.0),
            max_y=rospy.get_param("~search/max_y", 6.0),
            lane_spacing=rospy.get_param("~search/lane_spacing", 1.2),
            altitude=rospy.get_param("~search/altitude", 2.2),
        )
        route = CoverageRoute(
            search.waypoints,
            str(rospy.get_param(
                "~search/route_revision", "toudi4-copy-r1")),
            rospy.get_param("~search/max_failures_per_waypoint", 2),
        )
        return MissionRuntime(MissionCore(profile, config), route)

    @staticmethod
    def _age_state(stamp, now, max_age, future_tolerance):
        stamp_sec = stamp.to_sec()
        if stamp_sec <= 0.0:
            return "stale"
        age = now - stamp_sec
        if not math.isfinite(age):
            return "stale"
        if age < -future_tolerance:
            return "future"
        if age > max_age:
            return "stale"
        return "fresh"

    def _readiness(self, now):
        if not math.isfinite(now) or now <= 0.0:
            return False, "ros_clock_invalid"
        config = (self._runtime.core.config if self._runtime is not None
                  else self._mission_config())
        if self._pose is None:
            return False, "pose_missing"
        if self._pose.header.frame_id != config.mission_frame:
            return False, "pose_frame_mismatch"
        pose_age_state = self._age_state(
            self._pose.header.stamp,
            now,
            self._pose_max_age,
            self._stamp_future_tolerance,
        )
        if pose_age_state == "future":
            return False, "pose_stamp_in_future"
        if pose_age_state != "fresh":
            return False, "pose_stale"
        position = self._pose.pose.position
        if not all(math.isfinite(value) for value in
                   (position.x, position.y, position.z)):
            return False, "pose_non_finite"
        if position.z < 0.0 or position.z > 4.0:
            return False, "pose_altitude_out_of_bounds"
        if not self._require_map:
            return True, "ready"
        if self._map is None:
            return False, "map_missing"
        if self._map.header.frame_id != config.mission_frame:
            return False, "map_frame_mismatch"
        map_age_state = self._age_state(
            self._map.header.stamp,
            now,
            self._map_max_age,
            self._stamp_future_tolerance,
        )
        if map_age_state == "future":
            return False, "map_stamp_in_future"
        if map_age_state != "fresh":
            return False, "map_stale"
        width = int(self._map.width)
        height = int(self._map.height)
        point_step = int(self._map.point_step)
        row_step = int(self._map.row_step)
        if width <= 0 or height <= 0:
            return False, "map_empty"
        if point_step <= 0 or row_step < width * point_step:
            return False, "map_layout_invalid"
        if len(self._map.data) < row_step * height:
            return False, "map_data_truncated"
        return True, "ready"

    def _current_xy(self):
        position = self._pose.pose.position
        return float(position.x), float(position.y)

    def _on_pose(self, message):
        with self._lock:
            self._pose = message

    def _on_map(self, message):
        with self._lock:
            self._map = message

    def _handle_callback_exception(self, source, error):
        """Keep subscriber/timer threads alive and fail closed once."""

        reason = "%s_exception:%s" % (source, type(error).__name__)
        self._last_reason = reason
        rospy.logerr("VCL06 %s callback failed: %s", source, error)
        if (self._runtime is not None and
                self._runtime.core.phase not in (
                    MissionPhase.COMPLETE, MissionPhase.ABORTED)):
            try:
                outcome = self._runtime.abort(
                    reason, rospy.Time.now().to_sec())
                self._last_reason = outcome.reason
                self._publish_action(outcome.action)
            except Exception as abort_error:  # pylint: disable=broad-except
                self._last_reason = "%s:abort_failed:%s" % (
                    reason, type(abort_error).__name__)
                rospy.logerr(
                    "VCL06 fail-closed abort failed: %s", abort_error)
        try:
            self._publish_status(force=True)
        except Exception as status_error:  # pylint: disable=broad-except
            rospy.logerr(
                "VCL06 fail-closed status publish failed: %s",
                status_error,
            )

    @staticmethod
    def _candidate_snapshot(message):
        return CandidateSnapshot(
            target_id=int(message.id),
            class_name=str(message.class_name),
            class_confidence=float(message.class_confidence),
            geometry_confidence=float(message.geometry_confidence),
            map_quality=float(message.map_quality),
            x=float(message.map_point.x),
            y=float(message.map_point.y),
            z=float(message.map_point.z),
            map_frame=str(message.map_frame),
            state=int(message.state),
            consecutive_observe_count=int(message.consecutive_observe_count),
            map_valid=bool(message.map_valid),
            association_valid=bool(message.association_valid),
            reject_reason=str(message.reject_reason),
            transform_age_sec=float(message.transform_age_sec),
            first_seen_ns=_stamp_to_ns(message.first_seen),
            last_seen_ns=_stamp_to_ns(message.last_seen),
        )

    def _on_candidates(self, message):
        with self._lock:
            if self._runtime is None:
                return
            try:
                now = rospy.Time.now().to_sec()
                snapshots = tuple(
                    self._candidate_snapshot(item)
                    for item in message.targets)
                outcome = self._runtime.ingest(snapshots, now)
                rejected = sum(
                    not item.accepted
                    for item in outcome.candidate_validations)
                if rejected:
                    rospy.logdebug(
                        "VCL06 rejected %d/%d candidate snapshots",
                        rejected, len(snapshots))
                self._last_reason = outcome.reason
                self._publish_action(outcome.action)
                self._publish_status()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("candidates", error)

    @staticmethod
    def _result_event(message):
        if message.schema_version != NavigationResult.SCHEMA_VERSION:
            raise ValueError("result_schema_version_mismatch")
        try:
            command = COMMAND_NAMES[message.command]
            status = STATUS_NAMES[message.status]
            stage = STAGE_NAMES[message.stage]
        except KeyError as exc:
            raise ValueError("result_enum_invalid") from exc
        return ResultEvent(
            mission_id=str(message.mission_id),
            executor_id=str(message.executor_id),
            event_seq=int(message.event_seq),
            event_stamp_ns=_stamp_to_ns(message.header.stamp),
            decision_seq=int(message.decision_seq),
            command=command,
            has_target=bool(message.has_target),
            target_id=int(message.target_id),
            target_first_seen_ns=_stamp_to_ns(message.target_first_seen),
            target_class=str(message.target_class),
            attempt=int(message.attempt),
            payload_slot=int(message.payload_slot),
            status=status,
            stage=stage,
            terminal=bool(message.terminal),
            retryable=bool(message.retryable),
            payload_committed=bool(message.payload_committed),
            reason=str(message.reason),
            evidence_source=str(message.evidence_source),
        )

    def _on_result(self, message):
        with self._lock:
            if self._runtime is None or self._pose is None:
                return
            try:
                try:
                    event = self._result_event(message)
                except ValueError as exc:
                    self._last_reason = str(exc)
                    rospy.logwarn("VCL06 result rejected: %s", exc)
                    self._publish_status()
                    return
                now = rospy.Time.now().to_sec()
                outcome = self._runtime.apply_result(
                    event, now, self._current_xy())
                self._last_reason = outcome.reason
                self._publish_action(outcome.action)
                self._publish_status()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("result", error)

    def _on_start(self, _request):
        with self._lock:
            now_stamp = rospy.Time.now()
            now = now_stamp.to_sec()
            if self._runtime is not None:
                core = self._runtime.core
                if (core.active_action is not None or
                        core.phase not in (
                            MissionPhase.COMPLETE, MissionPhase.ABORTED)):
                    return TriggerResponse(
                        success=False, message="mission_already_active")
            try:
                runtime = self._new_runtime()
                self._runtime = runtime
                ready, reason = self._readiness(now)
                if not ready:
                    self._runtime = None
                    self._last_reason = reason
                    self._publish_status(force=True)
                    return TriggerResponse(success=False, message=reason)
                self._mission_counter += 1
                mission_id = "%s-%d-%09d-%d" % (
                    self._mission_id_prefix,
                    now_stamp.secs,
                    now_stamp.nsecs,
                    self._mission_counter,
                )
                outcome = runtime.start(mission_id, now, self._current_xy())
            except Exception as exc:  # pylint: disable=broad-except
                self._runtime = None
                self._last_reason = "start_rejected:%s" % exc
                rospy.logerr("VCL06 mission start rejected: %s", exc)
                self._publish_status(force=True)
                return TriggerResponse(
                    success=False, message=self._last_reason)
            self._last_reason = outcome.reason
            self._publish_action(outcome.action)
            self._publish_status(force=True)
            if not outcome.accepted:
                return TriggerResponse(
                    success=False, message=outcome.reason)
            return TriggerResponse(success=True, message=mission_id)

    def _on_abort(self, _request):
        with self._lock:
            if self._runtime is None:
                return TriggerResponse(
                    success=False, message="mission_not_started")
            if self._runtime.core.phase in (
                    MissionPhase.COMPLETE, MissionPhase.ABORTED):
                return TriggerResponse(
                    success=False, message="mission_not_abortable")
            now = rospy.Time.now().to_sec()
            try:
                outcome = self._runtime.abort("manual_abort_requested", now)
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logerr("VCL06 manual abort rejected: %s", exc)
                return TriggerResponse(success=False, message=str(exc))
            self._last_reason = outcome.reason
            self._publish_action(outcome.action)
            self._publish_status(force=True)
            return TriggerResponse(success=True, message=outcome.reason)

    def _on_timer(self, _event):
        with self._lock:
            try:
                if self._runtime is None or self._pose is None:
                    self._publish_status()
                    return
                now = rospy.Time.now().to_sec()
                core = self._runtime.core
                if core.phase not in (
                        MissionPhase.COMPLETE, MissionPhase.ABORTED):
                    ready, reason = self._readiness(now)
                    if not ready:
                        outcome = self._runtime.abort(reason, now)
                        self._last_reason = outcome.reason
                        self._publish_action(outcome.action)
                        self._publish_status(force=True)
                        return
                outcome = self._runtime.tick(now, self._current_xy())
                self._last_reason = outcome.reason
                self._publish_action(outcome.action)
                self._publish_status()
            except Exception as error:  # pylint: disable=broad-except
                self._handle_callback_exception("timer", error)

    def _publish_action(self, action):
        if action is None:
            return
        if action.command not in COMMAND_VALUES:
            raise ValueError("unsupported core command: %s" % action.command)
        message = NavigationDecision()
        message.header.seq = int(action.decision_seq)
        message.header.stamp = rospy.Time.from_sec(action.issued_at)
        message.header.frame_id = (
            action.goal.frame_id if action.goal is not None
            else self._runtime.core.config.mission_frame)
        message.schema_version = NavigationDecision.SCHEMA_VERSION
        message.mission_id = self._runtime.core.mission_id
        message.decision_seq = int(action.decision_seq)
        message.deadline = rospy.Time.from_sec(action.deadline_at)
        message.command = COMMAND_VALUES[action.command]
        message.class_profile = action.profile_name
        message.has_goal = action.has_goal
        message.has_target = action.has_target
        if action.target_snapshot is not None:
            target = action.target_snapshot
            message.target_id = int(target.target_id)
            message.target_first_seen = _ns_to_stamp(target.first_seen_ns)
            message.target_observation_stamp = _ns_to_stamp(
                target.last_seen_ns)
            message.target_class = action.target_class
            message.attempt = int(action.attempt)
            message.payload_slot = int(action.payload_slot)
        message.goal.header.seq = int(action.decision_seq)
        message.goal.header.stamp = message.header.stamp
        message.goal.header.frame_id = message.header.frame_id
        message.goal.pose.orientation.w = 1.0
        if action.goal is not None:
            message.goal.pose.position.x = action.goal.x
            message.goal.pose.position.y = action.goal.y
            message.goal.pose.position.z = action.goal.z
        message.reason = action.reason
        self._decision_pub.publish(message)
        rospy.loginfo(
            "VCL06 raw decision seq=%d command=%s deadline=%.3f reason=%s",
            action.decision_seq,
            action.command,
            action.deadline_at,
            action.reason,
        )

    def _publish_status(self, force=False):
        payload = {
            "last_reason": self._last_reason,
            "manual_start_required": True,
            "profile": self._profile_name,
        }
        if self._runtime is None:
            payload.update({"mission_id": "", "phase": "IDLE"})
        else:
            core = self._runtime.core
            snapshot = self._runtime.snapshot()
            payload.update({
                "mission_id": core.mission_id,
                "phase": _phase_name(snapshot.phase),
                "active_command": snapshot.active_command,
                "active_decision_seq": snapshot.active_decision_seq,
                "route_index": snapshot.route_index,
                "route_complete": snapshot.route_complete,
                "route_active_decision_seq":
                    snapshot.route_active_decision_seq,
                "post_delivery_route_revision":
                    snapshot.post_delivery_route_revision,
                "post_delivery_route_index":
                    snapshot.post_delivery_route_index,
                "post_delivery_route_size":
                    snapshot.post_delivery_route_size,
                "post_delivery_route_complete":
                    snapshot.post_delivery_route_complete,
                "committed_slots": snapshot.committed_slots,
                "mission_failed": snapshot.mission_failed,
                "slot_status": [slot.status.value for slot in core.slots],
            })
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        if force or encoded != self._last_status:
            self._status_pub.publish(String(data=encoded))
            self._last_status = encoded


def main():
    rospy.init_node("navigation_mission_manager")
    NavigationMissionManager()
    rospy.spin()


if __name__ == "__main__":
    main()
