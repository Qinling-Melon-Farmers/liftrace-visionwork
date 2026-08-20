#!/usr/bin/env python3
"""评测专用搜索导航外挂：全覆盖基线或蓝环粗发现后缩短覆盖并下视复核。"""

import json
import math
import os
import sys
import time

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8, String, UInt8

from patrol_control.msg import MissionCommand
from uav_mission.msg import ReleaseResult
from uav_vision.msg import TargetCandidateArray

from uav_vision_eval.aux_search_policy import (
    AuxCandidateBook,
    SOURCE_AUX_CV,
    STATUS_CONFIRMED,
    STATUS_REJECTED,
    fresh_spatial_match,
    handoff_gate_status,
)


UAV_MISSION_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("uav_mission"), "scripts")
if UAV_MISSION_SCRIPTS not in sys.path:
    sys.path.insert(0, UAV_MISSION_SCRIPTS)

from coverage_policy import (  # noqa: E402
    GoalRetryPolicy, generate_serpentine, select_serpentine_entry)


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


class AuxGuidedSearchManager:
    def __init__(self):
        rospy.init_node("aux_guided_search_manager")
        self._strategy = rospy.get_param("~strategy", "baseline")
        if self._strategy not in ("baseline", "guided"):
            raise ValueError("strategy must be baseline or guided")
        self._frame = rospy.get_param("~mission_frame", "camera_init")
        self._bounds = (
            float(rospy.get_param("~field/min_x", -5.0)),
            float(rospy.get_param("~field/max_x", 5.0)),
            float(rospy.get_param("~field/min_y", -5.0)),
            float(rospy.get_param("~field/max_y", 5.0)),
        )
        self._search_bounds = (
            float(rospy.get_param("~search_region/min_x", self._bounds[0])),
            float(rospy.get_param("~search_region/max_x", self._bounds[1])),
            float(rospy.get_param("~search_region/min_y", self._bounds[2])),
            float(rospy.get_param("~search_region/max_y", self._bounds[3])),
        )
        self._margin = float(rospy.get_param("~coverage/safety_margin", 0.5))
        self._spacing = float(rospy.get_param("~coverage/spacing", 1.0))
        self._height = float(rospy.get_param("~coverage/height", 2.0))
        self._dwell_time = float(rospy.get_param("~coverage/dwell_time", 0.0))
        self._takeoff_height = float(
            rospy.get_param("~navigation/takeoff_height", 1.7))
        self._arrival_radius = float(
            rospy.get_param("~navigation/arrival_radius", 0.40))
        self._progress_epsilon = float(
            rospy.get_param("~navigation/progress_epsilon", 0.10))
        self._map_ready_timeout = float(
            rospy.get_param("~navigation/map_ready_timeout", 60.0))
        self._map_max_age = float(
            rospy.get_param("~navigation/map_max_age", 2.0))
        self._aux_match_distance = float(
            rospy.get_param("~guided/aux_match_distance_m", 0.80))
        self._aux_source = rospy.get_param(
            "~guided/aux_source", SOURCE_AUX_CV)
        self._aux_min_quality = float(
            rospy.get_param("~guided/aux_min_map_quality", 0.10))
        self._aux_trigger_count = int(
            rospy.get_param("~guided/aux_trigger_count", 5))
        self._aux_min_approach_count = int(
            rospy.get_param("~guided/aux_min_approach_count", 2))
        self._verify_dwell = float(
            rospy.get_param("~guided/downward_verify_dwell_sec", 2.0))
        self._handoff_match_distance = float(rospy.get_param(
            "~guided/handoff_match_distance_m", 1.0))
        self._handoff_max_age = float(rospy.get_param(
            "~guided/handoff_max_age_sec", 0.75))
        self._handoff_preverify_grace = float(rospy.get_param(
            "~guided/handoff_preverify_grace_sec", 0.20))
        self._sparse_rows = {
            int(value) for value in rospy.get_param(
                "~guided/sparse_row_indices", [2, 5])}
        self._required_count = int(rospy.get_param("~required_target_count", 5))
        self._mission_timeout = float(rospy.get_param("~mission_timeout", 600.0))
        self._wall_timeout = float(rospy.get_param("~wall_timeout", 1200.0))
        self._wall_started = time.monotonic()
        self._report_path = os.path.abspath(rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json")))

        self._canonical_route = generate_serpentine(
            *self._search_bounds, self._margin, self._spacing, self._height)
        self._full_route = []
        self._active_route = []
        self._route_index = 0
        self._route_phase = "not_started"
        self._state = "WAIT_TAKEOFF"
        self._pose = None
        self._last_path_pose = None
        self._control_state = None
        self._last_map_stamp = None
        self._map_updates = 0
        self._takeoff_ready_since = None
        self._map_wait_started_at = None
        self._mission_started_at = None
        self._search_completed_at = None
        self._active_goal = None
        self._best_goal_distance = None
        self._goal_policy = GoalRetryPolicy(
            retry_interval=5.0, unreachable_timeout=25.0, max_retries=3)
        self._dwell_started_at = None
        self._candidate_book = AuxCandidateBook(self._aux_match_distance)
        self._aux_route = []
        self._aux_route_index = 0
        self._active_aux_candidate = None
        self._downward_confirmed = {}
        self._visited = []
        self._skipped = []
        self._goal_publish_count = 0
        self._command_sequence = []
        self._path_length = 0.0
        self._raw_calls = []
        self._release_results = []
        self._fallback_used = False
        self._fallback_count = 0
        self._goal_progress_timeout_count = 0
        self._aux_triggered = False
        self._terminal = False

        self._goal_pub = rospy.Publisher(
            "/fastplanner/goal", PoseStamped, queue_size=1)
        self._command_pub = rospy.Publisher(
            "/mission/command", MissionCommand, queue_size=4)
        self._status_pub = rospy.Publisher(
            "/mission/guided_search_status", String, queue_size=1, latch=True)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber("/detect/point_class", Int8,
                         self._on_control_state, queue_size=2)
        rospy.Subscriber("/sdf_map/occupancy_inflate", PointCloud2,
                         self._on_map, queue_size=1)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_downward_targets, queue_size=2)
        rospy.Subscriber("/uav_vision/aux/blue_targets", TargetCandidateArray,
                         self._on_aux_targets, queue_size=2)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=4)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release, queue_size=4)
        self._write("RUNNING", "waiting_for_takeoff")

    def _now(self):
        return rospy.Time.now().to_sec()

    def _mission_elapsed(self):
        if self._mission_started_at is None:
            return None
        return max(0.0, self._now() - self._mission_started_at)

    def _on_pose(self, message):
        self._pose = message
        if self._mission_started_at is None:
            return
        position = message.pose.position
        current = (float(position.x), float(position.y), float(position.z))
        if self._last_path_pose is not None:
            delta = math.sqrt(sum(
                (current[index] - self._last_path_pose[index]) ** 2
                for index in range(3)))
            if delta <= 2.0:
                self._path_length += delta
        self._last_path_pose = current

    def _on_control_state(self, message):
        self._control_state = int(message.data)

    def _on_map(self, message):
        stamp = message.header.stamp.to_sec()
        self._last_map_stamp = stamp if stamp > 0.0 else self._now()
        self._map_updates += 1

    def _on_raw_call(self, message):
        self._raw_calls.append(int(message.data))

    def _on_release(self, message):
        self._release_results.append({
            "slot": int(message.payload_slot),
            "success": bool(message.success),
            "reason": message.reason,
        })

    def _valid_downward(self, target):
        return (
            target.class_name in STANDARD_CLASSES and target.state >= 2 and
            target.map_valid and target.map_frame == self._frame and
            target.association_valid and not target.reject_reason and
            target.class_confidence >= 0.45)

    def _on_downward_targets(self, message):
        if self._mission_started_at is None:
            return
        for target in message.targets:
            if not self._valid_downward(target):
                continue
            class_name = target.class_name
            last_seen = target.last_seen.to_sec()
            if last_seen <= 0.0:
                last_seen = message.header.stamp.to_sec()
            if last_seen <= 0.0:
                last_seen = self._now()
            previous = self._downward_confirmed.get(class_name)
            if previous is not None and \
                    last_seen < previous["last_seen_sec"]:
                continue
            self._downward_confirmed[class_name] = {
                "id": int(target.id),
                "class_name": class_name,
                "x": float(target.map_point.x),
                "y": float(target.map_point.y),
                "map_point": [
                    float(target.map_point.x), float(target.map_point.y)],
                "confidence": float(target.class_confidence),
                "first_elapsed_sec": (
                    self._mission_elapsed() if previous is None else
                    previous["first_elapsed_sec"]),
                "last_elapsed_sec": self._mission_elapsed(),
                "last_seen_sec": last_seen,
            }
            if previous is None:
                self._write("RUNNING", "downward_confirmed_%s" % class_name)

    def _valid_aux(self, target):
        return (
            self._strategy == "guided" and target.class_name == "circle" and
            target.state >= 2 and target.map_valid and
            target.map_frame == self._frame and
            target.map_quality >= self._aux_min_quality)

    def _on_aux_targets(self, message):
        if self._mission_started_at is None:
            return
        for target in message.targets:
            if not self._valid_aux(target):
                continue
            x = float(target.map_point.x)
            y = float(target.map_point.y)
            if not (self._search_bounds[0] <= x <= self._search_bounds[1] and
                    self._search_bounds[2] <= y <= self._search_bounds[3]):
                continue
            stamp = target.last_seen.to_sec()
            if stamp <= 0.0:
                stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = self._now()
            self._candidate_book.observe(
                source_id=target.id,
                x=x,
                y=y,
                confidence=target.class_confidence,
                map_quality=target.map_quality,
                stamp_sec=stamp,
                source=self._aux_source,
                class_hint=target.class_name)

    def _map_ready(self):
        return (
            self._last_map_stamp is not None and self._map_updates > 0 and
            self._now() - self._last_map_stamp <= self._map_max_age)

    def _goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self._frame
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.w = 1.0
        return goal

    def _distance(self, goal):
        if self._pose is None or goal is None:
            return float("inf")
        position = self._pose.pose.position
        target = goal.pose.position
        return math.sqrt(
            (position.x - target.x) ** 2 +
            (position.y - target.y) ** 2 +
            (position.z - target.z) ** 2)

    def _publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self._goal_pub.publish(goal)
        self._goal_publish_count += 1

    def _publish_command(self, command, goal, target_id=0,
                         target_class=""):
        message = MissionCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._frame
        message.command = command
        message.target_id = target_id
        message.target_class = target_class
        message.goal = goal
        self._command_pub.publish(message)
        self._command_sequence.append(int(command))

    def _start_goal(self, goal, command, target_id=0, target_class=""):
        self._active_goal = goal
        self._best_goal_distance = self._distance(goal)
        self._publish_goal(goal)
        self._publish_command(command, goal, target_id, target_class)
        self._goal_policy.start(self._now())

    def _tick_goal(self):
        distance = self._distance(self._active_goal)
        if distance <= self._arrival_radius:
            return "arrived"
        if (math.isfinite(distance) and
                (self._best_goal_distance is None or
                 distance <= self._best_goal_distance - self._progress_epsilon)):
            self._best_goal_distance = distance
            self._goal_policy.note_progress(self._now())
        decision = self._goal_policy.decision(self._now())
        if decision == "retry":
            self._publish_goal(self._active_goal)
        return decision

    def _select_routes(self):
        position = self._pose.pose.position
        self._full_route = select_serpentine_entry(
            self._canonical_route, position.x, position.y)
        if self._strategy == "baseline":
            self._active_route = list(self._full_route)
            self._route_phase = "full_coverage"
        else:
            self._active_route = [
                point for point in self._full_route
                if point.row in self._sparse_rows]
            self._route_phase = "sparse_scan"
        self._route_index = 0

    def _start_next_route_goal(self):
        if self._route_index >= len(self._active_route):
            if self._strategy == "baseline":
                self._complete_search("full_coverage_complete")
            elif self._route_phase == "sparse_scan":
                self._start_aux_approach_or_fallback("sparse_scan_complete")
            else:
                self._complete_search("fallback_coverage_complete")
            return
        point = self._active_route[self._route_index]
        goal = self._goal(point.x, point.y, point.z)
        self._start_goal(goal, MissionCommand.SEARCH)
        self._state = "SCAN"
        self._write("RUNNING", "%s_goal_%d" % (
            self._route_phase, point.index))

    def _build_aux_route(self):
        if self._pose is None:
            return self._candidate_book.visit_order(0.0, 0.0)
        return self._candidate_book.visit_order(
            self._pose.pose.position.x, self._pose.pose.position.y)

    def _start_aux_approach_or_fallback(self, reason):
        if len(self._downward_confirmed) >= self._required_count:
            self._complete_search("all_targets_downward_confirmed")
            return
        if len(self._candidate_book.records) >= self._aux_min_approach_count:
            self._aux_triggered = True
            self._aux_route = self._build_aux_route()
            self._aux_route_index = 0
            self._state = "AUX_APPROACH"
            self._start_next_aux_goal(reason)
            return
        self._start_fallback("insufficient_aux_candidates")

    def _start_next_aux_goal(self, reason="next_aux_candidate"):
        if len(self._downward_confirmed) >= self._required_count:
            self._complete_search("all_targets_downward_confirmed")
            return
        if self._aux_route_index >= len(self._aux_route):
            self._active_aux_candidate = None
            self._start_fallback("aux_candidates_exhausted")
            return
        candidate = self._aux_route[self._aux_route_index]
        if candidate.status in (STATUS_CONFIRMED, STATUS_REJECTED):
            self._aux_route_index += 1
            self._start_next_aux_goal("skip_terminal_aux_candidate")
            return
        self._active_aux_candidate = candidate
        candidate.start_approach(self._now())
        goal = self._goal(candidate.x, candidate.y, self._height)
        self._start_goal(
            goal, MissionCommand.APPROACH,
            candidate.id, candidate.class_hint)
        self._state = "AUX_APPROACH"
        self._write("RUNNING", reason)

    def _start_fallback(self, reason):
        self._fallback_used = True
        self._fallback_count += 1
        self._active_aux_candidate = None
        visited_xy = [(item["x"], item["y"]) for item in self._visited]
        self._active_route = [
            point for point in self._full_route
            if not any(math.hypot(point.x - x, point.y - y) <= 0.25
                       for x, y in visited_xy)]
        self._route_index = 0
        self._route_phase = "fallback_coverage"
        self._write("RUNNING", reason)
        self._start_next_route_goal()

    def _complete_search(self, reason):
        if self._search_completed_at is None:
            self._search_completed_at = self._now()
        self._state = "RETURN_HOME"
        goal = self._goal(0.0, 0.0, self._takeoff_height)
        self._start_goal(goal, MissionCommand.RETURN_HOME)
        self._write("RUNNING", "return_after_%s" % reason)

    def _payload(self, status, reason):
        search_elapsed = (
            None if self._search_completed_at is None or
            self._mission_started_at is None else
            max(0.0, self._search_completed_at - self._mission_started_at))
        search_passed = (
            self._search_completed_at is not None and
            len(self._downward_confirmed) >= self._required_count)
        returned_home = (
            status == "PASS" and reason == "search_complete_returned_home")
        candidate_state_counts = self._candidate_book.state_counts()
        handoff_terminal = (
            candidate_state_counts["CONFIRMED"] +
            candidate_state_counts["REJECTED"])
        return {
            "gate": "oblique_guided_search_%s" % self._strategy,
            "status": status,
            "reason": reason,
            "strategy": self._strategy,
            "state": self._state,
            "mission_elapsed_sec": self._mission_elapsed(),
            "search_elapsed_sec": search_elapsed,
            "wall_elapsed_sec": time.monotonic() - self._wall_started,
            "path_length_m": self._path_length,
            "full_route_total": len(self._full_route),
            "active_route_total": len(self._active_route),
            "route_phase": self._route_phase,
            "route_index": self._route_index,
            "visited": list(self._visited),
            "skipped": list(self._skipped),
            "aux_candidate_count": len(self._candidate_book.records),
            "aux_candidate_state_counts": candidate_state_counts,
            "aux_handoff_success_rate": (
                None if handoff_terminal == 0 else
                candidate_state_counts["CONFIRMED"] /
                float(handoff_terminal)),
            "aux_candidates": [
                candidate.to_dict()
                for candidate in self._candidate_book.records],
            "aux_triggered": self._aux_triggered,
            "aux_approach_total": len(self._aux_route),
            "aux_approach_index": self._aux_route_index,
            "active_aux_candidate_id": (
                None if self._active_aux_candidate is None else
                self._active_aux_candidate.id),
            "fallback_used": self._fallback_used,
            "fallback_count": self._fallback_count,
            "goal_progress_timeout_count": self._goal_progress_timeout_count,
            "downward_confirmed_count": len(self._downward_confirmed),
            "downward_confirmed": dict(sorted(
                self._downward_confirmed.items())),
            "goal_publish_count": self._goal_publish_count,
            "command_sequence": list(self._command_sequence),
            "map_updates": self._map_updates,
            "map_ready": self._map_ready(),
            "raw_servo_calls": list(self._raw_calls),
            "release_results": list(self._release_results),
            "subgates": {
                "aux_handoff": handoff_gate_status(
                    self._candidate_book.records, self._aux_triggered),
                "search_complete": (
                    "PASS" if search_passed else
                    "FAIL" if status == "FAIL" else "PENDING"),
                "return_home": (
                    "PASS" if returned_home else
                    "FAIL" if search_passed and status == "FAIL" else
                    "NOT_REACHED" if status == "FAIL" else "PENDING"),
                "release_safety": (
                    "PASS" if not self._raw_calls and
                    not self._release_results else "FAIL"),
            },
        }

    def _write(self, status, reason):
        payload = self._payload(status, reason)
        encoded = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(String(data=encoded))
        os.makedirs(os.path.dirname(self._report_path) or ".", exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self._report_path)

    def _finish(self, status, reason):
        self._terminal = True
        self._state = "COMPLETE"
        self._write(status, reason)
        rospy.loginfo("[AuxGuidedSearch] %s reason=%s", status, reason)
        return 0 if status == "PASS" else 1

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._terminal:
            if self._raw_calls or self._release_results:
                return self._finish("FAIL", "release_occurred_in_search_gate")
            if time.monotonic() - self._wall_started >= self._wall_timeout:
                return self._finish("FAIL", "wall_timeout")
            if (self._mission_started_at is not None and
                    self._mission_elapsed() >= self._mission_timeout):
                return self._finish("FAIL", "mission_timeout")

            now = self._now()
            if self._state == "WAIT_TAKEOFF":
                ready = (self._pose is not None and self._control_state == 1 and
                         self._pose.pose.position.z >= self._takeoff_height)
                if ready and self._takeoff_ready_since is None:
                    self._takeoff_ready_since = time.monotonic()
                    self._map_wait_started_at = self._takeoff_ready_since
                elif (ready and self._map_ready() and
                      time.monotonic() - self._takeoff_ready_since >= 2.0):
                    self._mission_started_at = now
                    self._last_path_pose = None
                    self._select_routes()
                    self._start_next_route_goal()
                elif (ready and self._map_wait_started_at is not None and
                      time.monotonic() - self._map_wait_started_at >=
                      self._map_ready_timeout):
                    return self._finish("FAIL", "occupancy_map_not_ready")
                elif not ready:
                    self._takeoff_ready_since = None
                    self._map_wait_started_at = None

            elif self._state == "SCAN":
                if (self._strategy == "guided" and
                        self._route_phase == "sparse_scan" and
                        len(self._candidate_book.records) >=
                        self._aux_trigger_count):
                    self._start_aux_approach_or_fallback(
                        "aux_trigger_count_reached")
                else:
                    result = self._tick_goal()
                    if result == "arrived":
                        self._state = "SCAN_DWELL"
                        self._dwell_started_at = now
                    elif result == "timeout":
                        point = self._active_route[self._route_index]
                        self._goal_progress_timeout_count += 1
                        self._skipped.append({
                            "phase": self._route_phase,
                            "index": int(point.index),
                            "reason": "goal_progress_timeout_25s",
                        })
                        self._route_index += 1
                        self._start_next_route_goal()

            elif self._state == "SCAN_DWELL":
                if now - self._dwell_started_at >= self._dwell_time:
                    point = self._active_route[self._route_index]
                    self._visited.append({
                        "phase": self._route_phase,
                        "index": int(point.index),
                        "x": float(point.x), "y": float(point.y),
                    })
                    self._route_index += 1
                    self._start_next_route_goal()

            elif self._state == "AUX_APPROACH":
                result = self._tick_goal()
                if result == "arrived":
                    if self._active_aux_candidate is None or not \
                            self._active_aux_candidate.start_verify(now):
                        return self._finish(
                            "FAIL", "invalid_aux_candidate_transition")
                    self._state = "AUX_VERIFY"
                    self._dwell_started_at = now
                    self._write("RUNNING", "downward_verify")
                elif result == "timeout":
                    self._goal_progress_timeout_count += 1
                    if self._active_aux_candidate is not None:
                        self._active_aux_candidate.reject(
                            now, "goal_progress_timeout_25s")
                    self._skipped.append({
                        "phase": "aux_approach",
                        "index": self._aux_route_index,
                        "candidate_id": (
                            None if self._active_aux_candidate is None else
                            self._active_aux_candidate.id),
                        "reason": "goal_progress_timeout_25s",
                    })
                    self._aux_route_index += 1
                    self._start_next_aux_goal("aux_approach_timeout")

            elif self._state == "AUX_VERIFY":
                match = fresh_spatial_match(
                    self._active_aux_candidate,
                    self._downward_confirmed.values(),
                    now_sec=now,
                    max_distance_m=self._handoff_match_distance,
                    max_age_sec=self._handoff_max_age,
                    preverify_grace_sec=self._handoff_preverify_grace)
                if match is not None:
                    distance, target = match
                    self._active_aux_candidate.confirm(
                        now, target["id"], target["class_name"], distance)
                    self._write(
                        "RUNNING", "downward_handoff_confirmed_%s" %
                        target["class_name"])
                    if len(self._downward_confirmed) >= self._required_count:
                        self._complete_search("all_targets_downward_confirmed")
                    else:
                        self._aux_route_index += 1
                        self._start_next_aux_goal()
                elif now - self._dwell_started_at >= self._verify_dwell:
                    if self._active_aux_candidate is not None:
                        self._active_aux_candidate.reject(
                            now, "downward_verify_timeout")
                    self._aux_route_index += 1
                    self._start_next_aux_goal()

            elif self._state == "RETURN_HOME":
                result = self._tick_goal()
                if result == "arrived":
                    found_all = len(self._downward_confirmed) >= self._required_count
                    baseline_complete = (
                        self._strategy != "baseline" or
                        self._route_phase == "full_coverage" and
                        self._route_index >= len(self._active_route))
                    if found_all and baseline_complete:
                        return self._finish(
                            "PASS", "search_complete_returned_home")
                    return self._finish(
                        "FAIL", "returned_home_missing_downward_targets")
                if result == "timeout":
                    self._goal_progress_timeout_count += 1
                    return self._finish(
                        "FAIL", "return_home_progress_timeout_25s")

            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(AuxGuidedSearchManager().run())
