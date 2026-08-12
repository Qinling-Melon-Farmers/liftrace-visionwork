#!/usr/bin/env python3
"""Obstacle-aware coverage and visual-candidate mission manager."""

import json
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Int8, String

from coverage_policy import (
    CandidateData,
    CandidateQueue,
    GoalRetryPolicy,
    RULE_WEIGHTS,
    generate_serpentine,
    resolve_safe_waypoint,
    select_serpentine_entry,
)
from patrol_control.msg import MissionCommand
from uav_mission.msg import ReleaseResult
from uav_vision.msg import TargetCandidate, TargetCandidateArray


class CoverageSearchManager:
    def __init__(self):
        rospy.init_node("coverage_search_manager")
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
        self._tracking_reserve = float(
            rospy.get_param("~coverage/tracking_reserve", 0.0))
        self._effective_endpoint_margin = self._margin + self._tracking_reserve
        self._endpoint_max_adjustment = float(
            rospy.get_param("~navigation/endpoint_max_adjustment", 1.0))
        self._spacing = float(rospy.get_param("~coverage/spacing", 1.0))
        self._height = float(rospy.get_param("~coverage/height", 2.0))
        self._dwell_time = float(
            rospy.get_param("~coverage/dwell_time", 0.0))
        self._arrival_radius = float(
            rospy.get_param("~navigation/arrival_radius", 0.40))
        self._progress_epsilon = float(
            rospy.get_param("~navigation/progress_epsilon", 0.10))
        self._takeoff_height = float(
            rospy.get_param("~navigation/takeoff_height", 1.7))
        self._clearance = float(
            rospy.get_param("~navigation/endpoint_clearance", 0.35))
        self._collision_clearance = float(
            rospy.get_param("~navigation/collision_clearance", 0.10))
        self._execute_candidates = bool(
            rospy.get_param("~execute_candidates", False))
        self._navigation_only = bool(
            rospy.get_param("~navigation_only", True))
        self._collect_before_delivery = bool(
            rospy.get_param("~collect_before_delivery", True))
        self._final_land = bool(rospy.get_param("~final_land", False))
        self._required_deliveries = int(
            rospy.get_param("~required_deliveries", 3))
        self._require_north = bool(rospy.get_param("~require_north", True))
        self._recovery_height = float(
            rospy.get_param("~navigation/recovery_height", 0.95))
        self._landing_height = float(
            rospy.get_param("~navigation/landing_height", 0.15))
        self._landing_timeout = float(
            rospy.get_param("~navigation/landing_timeout", 90.0))
        self._semantic_match_distance = float(
            rospy.get_param("~candidate/semantic_match_distance", 0.75))
        self._release_attribution_distance = float(
            rospy.get_param("~candidate/release_attribution_distance", 0.75))
        self._alignment_timeout = float(
            rospy.get_param("~candidate/alignment_timeout", 90.0))
        self._search_timeout = float(rospy.get_param("~search_timeout", 540.0))
        self._mission_timeout = float(
            rospy.get_param("~mission_timeout", 600.0))
        self._map_ready_timeout = float(
            rospy.get_param("~navigation/map_ready_timeout", 60.0))
        self._map_max_age = float(
            rospy.get_param("~navigation/map_max_age", 2.0))
        self._wall_deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 1800.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "coverage_status.json"))
        self._gate_name = rospy.get_param("~gate_name", "coverage_navigation")

        self._canonical_route = generate_serpentine(
            *self._search_bounds, self._margin, self._spacing, self._height)
        self._route = list(self._canonical_route)
        self._route_selected = False
        self._route_entry = None
        self._map_ready_at_start = False
        self._state = "WAIT_TAKEOFF"
        self._pose = None
        self._control_state = None
        self._ready_since = None
        self._map_wait_started_at = None
        self._mission_started_at = None
        self._coverage_index = 0
        self._dwell_started_at = None
        self._active_goal = None
        self._active_nominal = None
        self._goal_policy = GoalRetryPolicy()
        self._best_goal_distance = None
        self._goal_attempts = {}
        self._visited = []
        self._skipped = []
        self._adjusted = []
        self._north_visited = False
        self._return_reason = ""
        self._return_pass = False
        self._mission_success = False
        self._candidate_queue = CandidateQueue()
        self._current_candidate = None
        self._selected_target = None
        self._release_result = None
        self._delivered = []
        self._failed_targets = []
        self._selection_sequence = []
        self._release_results = []
        self._discovered = {}
        self._resume_index = 0
        self._align_started_at = None
        self._recovery_started_at = None
        self._landing_started_at = None

        self._occupied = []
        self._occupancy_updates = 0
        self._last_occupancy_stamp = None
        self._last_cloud_process = -1.0
        self._last_clearance_check = -1.0
        self._minimum_clearance = None
        self._collision_count = 0
        self._collision_active = False
        self._collision_events = []
        self._boundary_violations = 0
        self._max_abs_x = 0.0
        self._max_abs_y = 0.0
        self._command_sequence = []
        self._goal_publish_count = 0

        self._goal_pub = rospy.Publisher(
            "/fastplanner/goal", PoseStamped, queue_size=1)
        self._command_pub = rospy.Publisher(
            "/mission/command", MissionCommand, queue_size=4)
        self._status_pub = rospy.Publisher(
            "/mission/coverage_status", String, queue_size=1, latch=True)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber("/detect/point_class", Int8,
                         self._on_control_state, queue_size=2)
        rospy.Subscriber("/sdf_map/occupancy_inflate", PointCloud2,
                         self._on_occupancy, queue_size=1)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=2)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate,
                         self._on_selected_target, queue_size=2)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release_result, queue_size=4)
        self._publish_status("RUNNING", "waiting_for_takeoff")

    def _now(self):
        return rospy.Time.now().to_sec()

    def _on_pose(self, msg):
        self._pose = msg
        position = msg.pose.position
        self._max_abs_x = max(self._max_abs_x, abs(position.x))
        self._max_abs_y = max(self._max_abs_y, abs(position.y))
        if self._mission_started_at is not None:
            min_x, max_x, min_y, max_y = self._bounds
            if not (min_x <= position.x <= max_x and
                    min_y <= position.y <= max_y):
                self._boundary_violations += 1
            self._update_clearance()

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)

    def _on_occupancy(self, msg):
        now = self._now()
        if now > 0.0 and now - self._last_cloud_process < 0.5:
            return
        self._last_cloud_process = now
        points = []
        for index, point in enumerate(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % 2:
                continue
            x, y, z = (float(point[0]), float(point[1]), float(point[2]))
            if (self._bounds[0] - 1.0 <= x <= self._bounds[1] + 1.0 and
                    self._bounds[2] - 1.0 <= y <= self._bounds[3] + 1.0 and
                    -0.2 <= z <= self._height + 1.0):
                points.append((x, y, z))
            if len(points) >= 12000:
                break
        self._occupied = points
        self._occupancy_updates += 1
        self._last_occupancy_stamp = now

    def _update_clearance(self):
        now = self._now()
        if not self._occupied or now - self._last_clearance_check < 0.25:
            return
        self._last_clearance_check = now
        position = self._pose.pose.position
        nearest_point = min(self._occupied, key=lambda point:
                            (position.x - point[0]) ** 2 +
                            (position.y - point[1]) ** 2 +
                            (position.z - point[2]) ** 2)
        nearest = math.sqrt(
            (position.x - nearest_point[0]) ** 2 +
            (position.y - nearest_point[1]) ** 2 +
            (position.z - nearest_point[2]) ** 2)
        if self._minimum_clearance is None or nearest < self._minimum_clearance:
            self._minimum_clearance = nearest
        if nearest < self._collision_clearance and not self._collision_active:
            self._collision_active = True
            self._collision_count += 1
            self._collision_events.append({
                "coverage_index": self._coverage_index,
                "clearance": nearest,
                "pose": [position.x, position.y, position.z],
                "occupied_point": list(nearest_point),
            })
        elif nearest >= self._collision_clearance + 0.05:
            self._collision_active = False

    def _on_targets(self, msg):
        now = self._now()
        candidates = []
        for target in msg.targets:
            candidates.append(CandidateData(
                target_id=int(target.id),
                class_name=target.class_name,
                confidence=float(target.class_confidence),
                first_seen=target.first_seen.to_sec(),
                last_seen=target.last_seen.to_sec(),
                state=int(target.state),
                map_valid=bool(target.map_valid),
                map_frame=target.map_frame,
                association_valid=bool(target.association_valid),
                reject_reason=target.reject_reason,
                x=float(target.map_point.x),
                y=float(target.map_point.y),
            ))
        active_ids = {
            candidate.target_id for candidate in candidates
            if candidate.class_name in RULE_WEIGHTS}
        self._candidate_queue.retain(active_ids)
        for target_id in list(self._discovered):
            if target_id not in active_ids:
                self._discovered.pop(target_id, None)
        self._candidate_queue.update(candidates, now, self._frame, 0.5)
        for candidate in self._candidate_queue.pending:
            self._discovered[candidate.target_id] = {
                "id": candidate.target_id,
                "class": candidate.class_name,
                "weight": RULE_WEIGHTS[candidate.class_name],
                "confidence": candidate.confidence,
                "first_seen": candidate.first_seen,
                "map": [candidate.x, candidate.y],
            }

    def _on_selected_target(self, msg):
        self._selected_target = msg

    def _on_release_result(self, msg):
        if self._current_candidate is None or self._state != "ALIGN":
            return
        self._release_result = msg

    def _pose_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = self._frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.w = 1.0
        return goal

    def _publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self._goal_pub.publish(goal)
        self._goal_publish_count += 1

    def _publish_command(self, command, goal, target=None):
        message = MissionCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._frame
        message.command = command
        if target is not None:
            message.target_id = target.target_id
            message.target_class = target.class_name
        message.goal = goal
        self._command_pub.publish(message)
        self._command_sequence.append(int(command))

    def _distance(self, goal):
        if self._pose is None or goal is None:
            return float("inf")
        position = self._pose.pose.position
        target = goal.pose.position
        return math.sqrt(
            (position.x - target.x) ** 2 +
            (position.y - target.y) ** 2 +
            (position.z - target.z) ** 2)

    def _start_goal(self, goal, command, target=None):
        self._active_goal = goal
        self._best_goal_distance = self._distance(goal)
        self._publish_goal(goal)
        self._publish_command(command, goal, target)
        self._goal_policy.start(self._now())

    def _start_coverage_point(self):
        while self._coverage_index < len(self._route):
            nominal = self._route[self._coverage_index]
            resolved = resolve_safe_waypoint(
                (nominal.x, nominal.y, nominal.z), self._occupied,
                self._search_bounds, self._effective_endpoint_margin,
                self._clearance, 0.55, 0.25, self._endpoint_max_adjustment)
            if resolved is None:
                self._skipped.append({
                    "index": nominal.index,
                    "row": nominal.row,
                    "reason": "known_occupied_no_clear_endpoint",
                })
                self._coverage_index += 1
                continue
            if resolved != (nominal.x, nominal.y, nominal.z):
                self._adjusted.append({
                    "index": nominal.index,
                    "nominal": [nominal.x, nominal.y, nominal.z],
                    "resolved": list(resolved),
                })
            self._active_nominal = nominal
            goal = self._pose_goal(*resolved)
            self._start_goal(goal, MissionCommand.SEARCH)
            self._state = "SEARCH"
            self._goal_attempts[str(nominal.index)] = 1
            self._publish_status("RUNNING", "coverage_goal_%d" % nominal.index)
            return
        if self._execute_candidates:
            self._start_next_candidate_or_return()
        else:
            self._begin_return("coverage_complete", navigation_pass=True)

    def _map_ready(self):
        if self._last_occupancy_stamp is None or self._occupancy_updates == 0:
            return False
        return self._now() - self._last_occupancy_stamp <= self._map_max_age

    def _select_route(self):
        position = self._pose.pose.position
        self._route = select_serpentine_entry(
            self._canonical_route, position.x, position.y)
        self._route_selected = True
        self._map_ready_at_start = self._map_ready()
        first = self._route[0]
        self._route_entry = [first.x, first.y, first.z]

    def _begin_return(self, reason, navigation_pass=False,
                      mission_success=False):
        self._return_reason = reason
        self._return_pass = bool(navigation_pass)
        self._mission_success = bool(mission_success)
        goal = self._pose_goal(0.0, 0.0, self._takeoff_height)
        self._start_goal(goal, MissionCommand.RETURN_HOME)
        self._state = "RETURN_HOME"
        self._publish_status("RUNNING", "return_home_%s" % reason)

    @staticmethod
    def _candidate_record(candidate):
        return {
            "id": candidate.target_id,
            "class": candidate.class_name,
            "weight": RULE_WEIGHTS[candidate.class_name],
            "confidence": candidate.confidence,
            "map": [candidate.x, candidate.y],
        }

    def _release_matches_candidate(self):
        if (self._release_result is None or self._current_candidate is None or
                self._pose is None):
            return False, "missing_release_identity_context"
        expected_slot = len(self._delivered) + 1
        if int(self._release_result.payload_slot) != expected_slot:
            return False, "unexpected_release_slot"
        # 任务候选是标准图案语义 ID；近地精对准和安全释放证据是该靶外圈的
        # circle ID。两者不应强行要求同 ID/同类别。ACK 归因使用顺序槽、
        # drop_circle 证据，以及飞机仍在已锁定语义地图点邻域三重约束。
        if (self._release_result.align_mode != "drop_circle" or
                self._release_result.target_class != "circle"):
            return False, "release_evidence_not_circle"
        position = self._pose.pose.position
        map_distance = math.hypot(
            position.x - self._current_candidate.x,
            position.y - self._current_candidate.y)
        if map_distance > self._release_attribution_distance:
            return False, "release_outside_candidate_neighborhood"
        return True, "release_identity_matched"

    def _finish_candidate(self, success, reason):
        candidate = self._current_candidate
        self._candidate_queue.mark_terminal(candidate.target_id)
        record = self._candidate_record(candidate)
        record["reason"] = reason
        if success:
            record["slot"] = int(self._release_result.payload_slot)
            self._delivered.append(record)
        else:
            self._failed_targets.append(record)
        self._publish_command(
            MissionCommand.RESUME, self._active_goal, candidate)
        self._current_candidate = None
        self._release_result = None
        if len(self._delivered) >= self._required_deliveries:
            self._begin_return(
                "three_deliveries_complete", mission_success=True)
        elif self._coverage_index >= len(self._route):
            self._start_next_candidate_or_return()
        else:
            self._coverage_index = self._resume_index
            self._start_coverage_point()

    def _start_next_candidate_or_return(self):
        if len(self._delivered) >= self._required_deliveries:
            self._begin_return(
                "three_deliveries_complete", mission_success=True)
            return
        if self._maybe_start_candidate(force=True):
            return
        self._begin_return(
            "coverage_complete_insufficient_candidates",
            mission_success=False)

    def _maybe_start_candidate(self, force=False):
        if (not self._execute_candidates or
                len(self._delivered) >= self._required_deliveries or
                (self._collect_before_delivery and not force)):
            return False
        candidate = self._candidate_queue.pop()
        if candidate is None:
            return False
        self._current_candidate = candidate
        self._resume_index = self._coverage_index
        self._selection_sequence.append(self._candidate_record(candidate))
        goal = self._pose_goal(candidate.x, candidate.y, self._height)
        self._start_goal(goal, MissionCommand.APPROACH, candidate)
        self._state = "CANDIDATE_APPROACH"
        self._publish_status("RUNNING", "candidate_%d" % candidate.target_id)
        return True

    def _tick_goal(self):
        distance = self._distance(self._active_goal)
        if distance <= self._arrival_radius:
            return "arrived"
        if (math.isfinite(distance) and
                (self._best_goal_distance is None or
                 distance <= self._best_goal_distance -
                 self._progress_epsilon)):
            self._best_goal_distance = distance
            self._goal_policy.note_progress(self._now())
        decision = self._goal_policy.decision(self._now())
        if decision == "retry":
            self._publish_goal(self._active_goal)
            if self._state == "SEARCH" and self._active_nominal is not None:
                key = str(self._active_nominal.index)
                self._goal_attempts[key] = self._goal_policy.retries + 1
        return decision

    def _publish_status(self, status, reason):
        payload = self._report_payload(status, reason)
        encoded = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(String(data=encoded))
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def _report_payload(self, status, reason):
        occupancy_age = None
        if self._last_occupancy_stamp is not None:
            occupancy_age = max(0.0, self._now() - self._last_occupancy_stamp)
        return {
            "gate": self._gate_name,
            "status": status,
            "reason": reason,
            "state": self._state,
            "mission_elapsed": (
                None if self._mission_started_at is None else
                max(0.0, self._now() - self._mission_started_at)),
            "arena_bounds": list(self._bounds),
            "search_region": list(self._search_bounds),
            "route_total": len(self._route),
            "safety_margin": self._margin,
            "tracking_reserve": self._tracking_reserve,
            "dwell_time": self._dwell_time,
            "effective_endpoint_margin": self._effective_endpoint_margin,
            "route_selected": self._route_selected,
            "route_entry": self._route_entry,
            "coverage_index": self._coverage_index,
            "visited": list(self._visited),
            "skipped": list(self._skipped),
            "adjusted": list(self._adjusted),
            "goal_attempts": dict(self._goal_attempts),
            "north_visited": self._north_visited,
            "require_north": self._require_north,
            "delivered": list(self._delivered),
            "failed_targets": list(self._failed_targets),
            "selection_sequence": list(self._selection_sequence),
            "release_results": list(self._release_results),
            "discovered": sorted(
                self._discovered.values(),
                key=lambda item: (-item["weight"], item["id"])),
            "pending_candidates": [
                self._candidate_record(candidate)
                for candidate in self._candidate_queue.pending],
            "terminal_target_ids": sorted(self._candidate_queue.terminal_ids),
            "goal_publish_count": self._goal_publish_count,
            "command_sequence": list(self._command_sequence),
            "occupancy_updates": self._occupancy_updates,
            "occupancy_age": occupancy_age,
            "map_ready": self._map_ready(),
            "map_ready_at_start": self._map_ready_at_start,
            "minimum_clearance": self._minimum_clearance,
            "collision_count": self._collision_count,
            "collision_events": list(self._collision_events),
            "boundary_violations": self._boundary_violations,
            "max_abs_x": self._max_abs_x,
            "max_abs_y": self._max_abs_y,
            "navigation_only": self._navigation_only,
            "execute_candidates": self._execute_candidates,
            "collect_before_delivery": self._collect_before_delivery,
            "final_land": self._final_land,
            "required_deliveries": self._required_deliveries,
            "alignment_timeout": self._alignment_timeout,
            "release_attribution_distance":
                self._release_attribution_distance,
        }

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if time.monotonic() >= self._wall_deadline:
                self._publish_status("FAIL", "wall_timeout")
                return 1

            now = self._now()
            if (self._mission_started_at is not None and
                    now - self._mission_started_at >= self._mission_timeout and
                    self._state not in ("WAIT_TAKEOFF", "COMPLETE")):
                self._state = "COMPLETE"
                self._publish_status("FAIL", "mission_timeout")
                return 1
            if (self._mission_started_at is not None and
                    now - self._mission_started_at >= self._search_timeout and
                    self._state in ("SEARCH", "CANDIDATE_APPROACH")):
                self._begin_return("search_timeout", navigation_pass=False)

            if self._state == "WAIT_TAKEOFF":
                ready = (self._pose is not None and self._control_state == 1 and
                         self._pose.pose.position.z >= self._takeoff_height)
                if ready and self._ready_since is None:
                    self._ready_since = time.monotonic()
                    self._map_wait_started_at = self._ready_since
                elif (ready and self._map_ready() and
                      time.monotonic() - self._ready_since >= 2.0):
                    self._select_route()
                    self._mission_started_at = now
                    self._start_coverage_point()
                elif (ready and self._map_wait_started_at is not None and
                      time.monotonic() - self._map_wait_started_at >=
                      self._map_ready_timeout):
                    self._publish_status("FAIL", "occupancy_map_not_ready")
                    return 1
                elif not ready:
                    self._ready_since = None
                    self._map_wait_started_at = None

            elif self._state == "SEARCH":
                if self._maybe_start_candidate():
                    pass
                else:
                    result = self._tick_goal()
                    if result == "arrived":
                        self._state = "COVERAGE_DWELL"
                        self._dwell_started_at = now
                        self._publish_status(
                            "RUNNING", "coverage_dwell_%d" %
                            self._active_nominal.index)
                    elif result == "timeout":
                        nominal = self._active_nominal
                        self._skipped.append({
                            "index": nominal.index,
                            "row": nominal.row,
                            "reason": "planner_unreachable_20s",
                        })
                        self._coverage_index += 1
                        self._start_coverage_point()

            elif self._state == "COVERAGE_DWELL":
                if now - self._dwell_started_at >= self._dwell_time:
                    nominal = self._active_nominal
                    self._visited.append(nominal.index)
                    if (nominal.y >= self._search_bounds[3] -
                            self._margin - 1.0):
                        self._north_visited = True
                    self._coverage_index += 1
                    self._start_coverage_point()

            elif self._state == "CANDIDATE_APPROACH":
                result = self._tick_goal()
                if result == "arrived":
                    self._publish_command(
                        MissionCommand.ALIGN, self._active_goal,
                        self._current_candidate)
                    self._state = "ALIGN"
                    self._align_started_at = now
                    self._publish_status("RUNNING", "candidate_align")
                elif result == "timeout":
                    self._finish_candidate(False, "approach_unreachable_20s")

            elif self._state == "ALIGN":
                if self._release_result is not None:
                    result_record = {
                        "slot": int(self._release_result.payload_slot),
                        "success": bool(self._release_result.success),
                        "target_id": int(self._release_result.target_id),
                        "target_class": self._release_result.target_class,
                        "reason": self._release_result.reason,
                    }
                    self._release_results.append(result_record)
                    if self._release_result.success:
                        matched, match_reason = \
                            self._release_matches_candidate()
                        if not matched:
                            self._state = "COMPLETE"
                            self._publish_status("FAIL", match_reason)
                            return 1
                        self._state = "RELEASE_RECOVERY"
                        self._recovery_started_at = now
                        self._publish_status(
                            "RUNNING", "guarded_ack_waiting_for_climb")
                    else:
                        self._finish_candidate(
                            False, "release_denied_%s" %
                            self._release_result.reason)
                elif now - self._align_started_at >= self._alignment_timeout:
                    self._finish_candidate(
                        False, "alignment_timeout_%.0fs" %
                        self._alignment_timeout)

            elif self._state == "RELEASE_RECOVERY":
                recovered = (
                    self._control_state == 1 and self._pose is not None and
                    self._pose.pose.position.z >= self._recovery_height)
                if recovered:
                    self._finish_candidate(True, "guarded_ack_and_climb")
                elif now - self._recovery_started_at >= 60.0:
                    self._state = "COMPLETE"
                    self._publish_status("FAIL", "release_recovery_timeout")
                    return 1

            elif self._state == "RETURN_HOME":
                result = self._tick_goal()
                if result == "arrived":
                    processed = len(self._visited) + len(self._skipped)
                    enough_visited = len(self._visited) >= math.ceil(
                        0.8 * len(self._route))
                    navigation_passed = (
                        self._return_pass and processed == len(self._route) and
                        enough_visited and
                        (self._north_visited or not self._require_north))
                    mission_passed = (
                        self._mission_success and
                        len(self._delivered) == self._required_deliveries and
                        len({item["id"] for item in self._delivered}) ==
                        self._required_deliveries)
                    passed = (
                        (navigation_passed if self._navigation_only else
                         mission_passed) and
                        self._collision_count == 0 and
                        self._boundary_violations == 0)
                    if passed and self._final_land:
                        land_goal = self._pose_goal(0.0, 0.0, 0.0)
                        self._publish_command(MissionCommand.LAND, land_goal)
                        self._state = "LAND"
                        self._landing_started_at = now
                        self._publish_status("RUNNING", "landing_commanded")
                    else:
                        status = "PASS" if passed else "FAIL"
                        self._state = "COMPLETE"
                        self._publish_status(status, self._return_reason)
                        return 0 if passed else 1
                if result == "timeout":
                    self._state = "COMPLETE"
                    self._publish_status("FAIL", "return_home_unreachable")
                    return 1

            elif self._state == "LAND":
                if (self._pose is not None and
                        self._pose.pose.position.z <= self._landing_height):
                    self._state = "COMPLETE"
                    self._publish_status("PASS", "three_deliveries_landed")
                    return 0
                if now - self._landing_started_at >= self._landing_timeout:
                    self._state = "COMPLETE"
                    self._publish_status("FAIL", "landing_timeout")
                    return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(CoverageSearchManager().run())
