#!/usr/bin/env python3
"""在不修改导航组搜索节点的前提下接入视觉投递闭环。

导航组 ``target_search_manager_py`` 的目标被重映射到
``/navigation/goal_raw``。本适配器只负责起飞门控、把原始目标转发给
Fast-Planner、在抵近后触发既有 ALIGN/释放安全链，以及三投后的返航降落。
"""

from dataclasses import dataclass
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

from coverage_policy import build_command_event, profile_allowed_classes
from patrol_control.msg import MissionCommand
from uav_mission.msg import ReleaseResult
from uav_mission.search_policy import SearchPolicy
from uav_vision.msg import TargetCandidate, TargetCandidateArray


@dataclass(frozen=True)
class Candidate:
    target_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    last_seen: float


class NavigationVisualDeliveryAdapter:
    def __init__(self):
        rospy.init_node("navigation_visual_delivery_adapter")
        self._frame = rospy.get_param("~mission_frame", "camera_init")
        self._class_profile = rospy.get_param("~class_profile", "full")
        self._allowed_classes = profile_allowed_classes(self._class_profile)
        self._selected_max_age = float(
            rospy.get_param("~target/selected_max_age", 0.5))
        self._policy = SearchPolicy(
            rospy.get_param("~search/min_x", -3.106588),
            rospy.get_param("~search/max_x", 3.093412),
            rospy.get_param("~search/min_y", -0.227310),
            rospy.get_param("~search/max_y", 7.772690),
            rospy.get_param("~search/lane_spacing", 1.2),
            rospy.get_param("~search/altitude", 2.2),
        )
        self._approach_altitude = float(
            rospy.get_param("~target/approach_altitude", 1.2))
        self._arrival = float(rospy.get_param("~arrival/threshold", 0.3))
        self._takeoff_height = float(
            rospy.get_param("~navigation/takeoff_height", 1.7))
        self._recovery_height = float(
            rospy.get_param("~navigation/recovery_height", 0.95))
        self._landing_height = float(
            rospy.get_param("~navigation/landing_height", 0.15))
        self._capture_timeout = float(
            rospy.get_param("~target/capture_timeout", 20.0))
        self._capture_age = float(
            rospy.get_param("~target/capture_fresh_age", 1.0))
        self._capture_radius = float(
            rospy.get_param("~target/capture_radius", 0.8))
        self._align_timeout = float(
            rospy.get_param("~target/alignment_timeout", 90.0))
        self._required = int(rospy.get_param("~target/required_deliveries", 3))
        self._mission_timeout = float(rospy.get_param("~mission_timeout", 600.0))
        self._navigation_stall_timeout = float(
            rospy.get_param("~navigation/stall_timeout", 45.0))
        self._return_stall_timeout = float(
            rospy.get_param("~navigation/return_stall_timeout", 60.0))
        self._progress_epsilon = float(
            rospy.get_param("~navigation/progress_epsilon", 0.1))
        self._wall_deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 1800.0))
        self._field = (
            float(rospy.get_param("~field/min_x", -3.992)),
            float(rospy.get_param("~field/max_x", 4.008)),
            float(rospy.get_param("~field/min_y", -1.132)),
            float(rospy.get_param("~field/max_y", 8.718)),
        )
        self._max_height_limit = float(
            rospy.get_param("~field/max_z", 4.0))
        self._collision_clearance = float(
            rospy.get_param("~navigation/collision_clearance", 0.10))
        self._require_field_ready = bool(
            rospy.get_param("~require_field_ready", False))
        self._require_anchor_ready = bool(
            rospy.get_param("~require_anchor_ready", False))
        self._nav_feature_profile = rospy.get_param(
            "~nav_feature_profile", "baseline")
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"),
                "target_search_status.json"))

        self._state = "WAIT_TAKEOFF"
        self._pose = None
        self._control_state = None
        self._map_stamp = None
        self._ready_since = None
        self._started_at = None
        self._raw_goal = None
        self._active_goal = None
        self._buffered_search_goal = None
        self._raw_goal_received_at = None
        self._pending_candidate_goal = None
        self._active_route_index = None
        self._visited = []
        self._latest_selected = None
        self._selected_history = {}
        self._current_candidate = None
        self._capture_evidence = []
        self._capture_started_at = None
        self._align_started_at = None
        self._release_result = None
        self._delivered = []
        self._failed = []
        self._discovered = {}
        self._release_results = []
        self._command_sequence = []
        self._goal_publish_count = 0
        self._boundary_violations = 0
        self._max_abs_x = 0.0
        self._max_abs_y = 0.0
        self._max_altitude = float("-inf")
        self._occupied = []
        self._last_cloud_process = -1.0
        self._last_clearance_check = -1.0
        self._minimum_clearance = None
        self._collision_count = 0
        self._collision_active = False
        self._collision_events = []
        self._return_success = False
        self._return_reason = ""
        self._phase_started_at = None
        self._best_goal_distance = float("inf")
        self._last_progress_at = None
        self._field_ready = not self._require_field_ready
        self._field_status = None
        self._field_failed = False
        self._anchor_ready = not self._require_anchor_ready
        self._anchor_status = None
        self._anchor_failed = False
        self._event_sequence = 0
        self._last_event_state = None
        self._status_events = []
        self._command_events = []
        self._selection_events = []

        self._goal_pub = rospy.Publisher(
            "/fastplanner/goal", PoseStamped, queue_size=1)
        self._command_pub = rospy.Publisher(
            "/mission/command", MissionCommand, queue_size=4)
        self._status_pub = rospy.Publisher(
            "/mission/target_search_status", String, queue_size=1, latch=True)
        rospy.Subscriber("/navigation/goal_raw", PoseStamped,
                         self._on_raw_goal, queue_size=4)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber("/detect/point_class", Int8,
                         self._on_control_state, queue_size=2)
        rospy.Subscriber("/sdf_map/occupancy_inflate", PointCloud2,
                         self._on_map, queue_size=1)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate,
                         self._on_selected, queue_size=2)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=2)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_release, queue_size=4)
        if self._require_field_ready:
            rospy.Subscriber(
                rospy.get_param(
                    "~field_status_topic", "/mission/random_field_status"),
                String, self._on_field_status, queue_size=1)
        if self._require_anchor_ready:
            rospy.Subscriber(
                rospy.get_param(
                    "~anchor_status_topic", "/mission/planner_anchor_status"),
                String, self._on_anchor_status, queue_size=1)
        self._publish_status("RUNNING", "waiting_for_takeoff")

    def _now(self):
        return rospy.Time.now().to_sec()

    def _on_pose(self, msg):
        self._pose = msg
        p = msg.pose.position
        self._max_abs_x = max(self._max_abs_x, abs(p.x))
        self._max_abs_y = max(self._max_abs_y, abs(p.y))
        self._max_altitude = max(self._max_altitude, float(p.z))
        if self._started_at is not None:
            if not (self._field[0] <= p.x <= self._field[1] and
                    self._field[2] <= p.y <= self._field[3]):
                self._boundary_violations += 1
            self._update_clearance()

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)

    def _on_map(self, msg):
        stamp = msg.header.stamp.to_sec()
        self._map_stamp = stamp if stamp > 0.0 else self._now()
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
            if (self._field[0] - 1.0 <= x <= self._field[1] + 1.0 and
                    self._field[2] - 1.0 <= y <= self._field[3] + 1.0 and
                    -0.2 <= z <= self._max_height_limit + 1.0):
                points.append((x, y, z))
            if len(points) >= 12000:
                break
        self._occupied = points

    def _update_clearance(self):
        now = self._now()
        if (not self._occupied or self._pose is None or
                now - self._last_clearance_check < 0.25):
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
                "clearance": nearest,
                "pose": [position.x, position.y, position.z],
                "occupied_point": list(nearest_point),
            })
        elif nearest >= self._collision_clearance + 0.05:
            self._collision_active = False

    def _on_field_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self._field_ready = False
            self._field_status = {"status": "INVALID_JSON"}
            return
        self._field_status = payload
        same_profile = payload.get("profile") == self._class_profile
        self._field_ready = bool(
            payload.get("ready") and payload.get("status") == "READY" and
            same_profile)
        self._field_failed = payload.get("status") == "FAIL"

    def _on_anchor_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self._anchor_ready = False
            self._anchor_status = {"status": "INVALID_JSON"}
            return
        self._anchor_status = payload
        same_profile = payload.get("profile") == self._nav_feature_profile
        self._anchor_ready = bool(
            payload.get("ready") and payload.get("status") == "READY" and
            same_profile)
        self._anchor_failed = payload.get("status") == "FAIL"

    @staticmethod
    def _candidate(msg):
        return Candidate(int(msg.id), msg.class_name,
                         float(msg.class_confidence),
                         float(msg.map_point.x), float(msg.map_point.y),
                         float(msg.map_point.z), msg.last_seen.to_sec())

    def _selected_live(self, msg):
        now = self._now()
        point = msg.map_point
        return (
            msg.class_name in self._allowed_classes and
            int(msg.state) == 2 and
            int(msg.consecutive_observe_count) >= 3 and
            msg.map_valid and msg.map_frame == self._frame and
            msg.association_valid and not msg.reject_reason and
            msg.last_seen.to_sec() > 0.0 and
            max(0.0, now - msg.last_seen.to_sec()) <= self._selected_max_age and
            all(math.isfinite(value) for value in
                (point.x, point.y, point.z)) and
            self._field[0] <= point.x <= self._field[1] and
            self._field[2] <= point.y <= self._field[3] and
            point.z <= self._max_height_limit
        )

    def _matching_selected(self, goal):
        now = self._now()
        for target_id in list(self._selected_history):
            message, received_at = self._selected_history[target_id]
            if now - received_at > self._selected_max_age:
                self._selected_history.pop(target_id, None)
                continue
        candidates = [item[0] for item in self._selected_history.values()
                      if self._selected_live(item[0])]
        if not candidates:
            return None
        selected = min(candidates, key=lambda item: math.hypot(
            goal.pose.position.x - item.map_point.x,
            goal.pose.position.y - item.map_point.y))
        distance = math.hypot(
            goal.pose.position.x - selected.map_point.x,
            goal.pose.position.y - selected.map_point.y)
        return selected if distance <= 0.6 else None

    def _on_selected(self, msg):
        if not self._selected_live(msg):
            return
        self._latest_selected = msg
        self._selected_history[int(msg.id)] = (msg, self._now())
        candidate = self._candidate(msg)
        self._discovered[candidate.target_id] = self._candidate_record(candidate)
        self._selection_events.append({
            "stamp": self._now(), "id": candidate.target_id,
            "class": candidate.class_name, "source": "profile_selector"})
        if (self._state == "SEARCH" and self._operational_ready() and
                self._pending_candidate_goal is not None):
            selected = self._matching_selected(self._pending_candidate_goal)
            if selected is not None:
                goal = self._pending_candidate_goal
                self._pending_candidate_goal = None
                self._start_approach(goal, selected)

    def _on_targets(self, msg):
        evidence = []
        for target in msg.targets:
            if target.class_name not in ("circle", "red_cross"):
                continue
            if (target.state != 2 or
                    target.consecutive_observe_count < 3 or
                    not target.map_valid or
                    target.map_frame != self._frame or
                    not target.association_valid or target.reject_reason):
                continue
            if (target.class_name == "red_cross" and
                    (not target.center_refined or
                     target.center_source != "red_cross_geometry")):
                continue
            evidence.append((target.class_name, float(target.map_point.x),
                             float(target.map_point.y),
                             target.last_seen.to_sec()))
        self._capture_evidence = evidence

    def _on_release(self, msg):
        if self._state == "ALIGN":
            self._release_result = msg

    def _route_index(self, goal):
        for index, waypoint in enumerate(self._policy.waypoints):
            if (abs(goal.pose.position.x - waypoint.x) < 1e-4 and
                    abs(goal.pose.position.y - waypoint.y) < 1e-4 and
                    abs(goal.pose.position.z - waypoint.z) < 1e-4):
                return index
        return None

    def _on_raw_goal(self, msg):
        self._raw_goal = msg
        self._raw_goal_received_at = self._now()
        route_index = self._route_index(msg)
        if route_index is not None:
            # 导航组只会在确认当前航点到达后发布下一航点。用新索引到达
            # 这一事件记录上一航点，避免订阅回调顺序造成覆盖统计漏记。
            if (self._active_route_index is not None and
                    route_index != self._active_route_index and
                    self._active_route_index not in self._visited):
                self._visited.append(self._active_route_index)
            self._active_route_index = route_index
            self._buffered_search_goal = msg
            if self._state in ("WAIT_TAKEOFF", "APPROACH", "CAPTURE", "ALIGN",
                               "RECOVERY", "RETURN_HOME", "LAND", "COMPLETE"):
                return
            self._forward_search(msg, MissionCommand.SEARCH)
            return

        # A raw PoseStamped has no target ID. Cache it until SEARCH is active,
        # then bind it spatially to a still-live profile-selected candidate.
        self._pending_candidate_goal = msg
        if self._state != "SEARCH" or not self._operational_ready():
            rospy.loginfo("[NavAdapter] candidate raw goal cached until ready")
            return
        selected = self._matching_selected(msg)
        if selected is None:
            rospy.logwarn("[NavAdapter] waiting for live candidate matching raw goal")
            return
        self._pending_candidate_goal = None
        self._start_approach(msg, selected)

    def _start_approach(self, msg, selected):
        if self._state != "SEARCH":
            rospy.logerr("[NavAdapter] rejected APPROACH transition from %s",
                         self._state)
            self._pending_candidate_goal = msg
            return
        previous_state = self._state
        self._current_candidate = self._candidate(selected)
        self._active_goal = msg
        self._state = "APPROACH"
        self._publish_goal(msg)
        self._publish_command(MissionCommand.APPROACH, msg,
                              self._current_candidate,
                              from_state=previous_state)
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "candidate_approach")

    def _forward_search(self, goal, command):
        previous_state = self._state
        self._active_goal = goal
        self._state = "SEARCH"
        self._publish_goal(goal)
        self._publish_command(command, goal, from_state=previous_state)
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "search_goal_%d" % self._active_route_index)

    def _publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self._goal_pub.publish(goal)
        self._goal_publish_count += 1

    def _publish_command(self, command, goal, candidate=None, from_state=None):
        msg = MissionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame
        msg.command = command
        if candidate is not None:
            msg.target_id = candidate.target_id
            msg.target_class = candidate.class_name
        msg.goal = goal
        self._command_pub.publish(msg)
        self._command_sequence.append(int(command))
        self._command_events.append(build_command_event(
            len(self._command_events) + 1, self._now(), int(command),
            from_state or self._state, self._state,
            None if candidate is None else candidate.target_id,
            "" if candidate is None else candidate.class_name))

    def _distance(self, goal=None):
        if self._pose is None or (goal or self._active_goal) is None:
            return float("inf")
        target = (goal or self._active_goal).pose.position
        current = self._pose.pose.position
        return math.sqrt((current.x - target.x) ** 2 +
                         (current.y - target.y) ** 2 +
                         (current.z - target.z) ** 2)

    def _reset_goal_progress(self):
        self._best_goal_distance = self._distance()
        self._last_progress_at = self._now()

    def _goal_stalled(self, timeout):
        """仅在一段时间内没有实质接近目标时判定导航停滞。"""
        distance = self._distance()
        now = self._now()
        if distance + self._progress_epsilon < self._best_goal_distance:
            self._best_goal_distance = distance
            self._last_progress_at = now
        return (self._last_progress_at is not None and
                now - self._last_progress_at >= timeout and
                distance > self._arrival)

    def _capture_ok(self):
        if self._current_candidate is None:
            return False
        expected = ("red_cross" if self._current_candidate.class_name ==
                    "red_cross" else "circle")
        now = self._now()
        for class_name, x, y, stamp in self._capture_evidence:
            if (class_name == expected and now - stamp <= self._capture_age and
                    math.hypot(x - self._current_candidate.x,
                               y - self._current_candidate.y) <= self._capture_radius):
                return True
        return False

    def _release_matches(self):
        result = self._release_result
        candidate = self._current_candidate
        if result is None or candidate is None or self._pose is None:
            return False
        if int(result.payload_slot) != len(self._delivered) + 1:
            return False
        pair_ok = ((result.align_mode == "drop_circle" and
                    result.target_class == "circle") or
                   (result.align_mode == "drop_cross" and
                    result.target_class == "red_cross"))
        p = self._pose.pose.position
        return (pair_ok and math.hypot(p.x - candidate.x,
                                      p.y - candidate.y) <= 0.8)

    def _resume_search(self, success, reason):
        record = self._candidate_record(self._current_candidate)
        record["reason"] = reason
        if success:
            record["slot"] = int(self._release_result.payload_slot)
            self._delivered.append(record)
        else:
            self._failed.append(record)
        self._current_candidate = None
        self._release_result = None
        if len(self._delivered) >= self._required:
            self._begin_return(True, "three_deliveries_complete")
        elif self._buffered_search_goal is not None:
            goal = self._buffered_search_goal
            self._buffered_search_goal = None
            self._forward_search(goal, MissionCommand.RESUME)
        else:
            self._state = "RECOVERY"

    def _begin_return(self, success, reason):
        self._return_success = success
        self._return_reason = reason
        goal = PoseStamped()
        goal.header.frame_id = self._frame
        goal.pose.position.z = self._takeoff_height
        goal.pose.orientation.w = 1.0
        previous_state = self._state
        self._active_goal = goal
        self._state = "RETURN_HOME"
        self._publish_goal(goal)
        self._publish_command(
            MissionCommand.RETURN_HOME, goal, from_state=previous_state)
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "return_home_%s" % reason)

    @staticmethod
    def _candidate_record(candidate):
        return {"id": candidate.target_id, "class": candidate.class_name,
                "confidence": candidate.confidence,
                "map": [candidate.x, candidate.y, candidate.z],
                "last_seen": candidate.last_seen}

    def _map_ready(self):
        return self._map_stamp is not None and self._now() - self._map_stamp <= 2.0

    def _operational_ready(self):
        return (
            self._field_ready and self._anchor_ready and
            self._pose is not None and self._control_state == 1 and
            self._pose.pose.position.z >= self._takeoff_height and
            self._map_ready()
        )

    def _payload(self, status, reason):
        return {
            "gate": "navigation_search_visual_delivery",
            "status": status, "reason": reason, "state": self._state,
            "mission_elapsed": (None if self._started_at is None else
                                self._now() - self._started_at),
            "class_profile": self._class_profile,
            "allowed_classes": list(self._allowed_classes),
            "nav_feature_profile": self._nav_feature_profile,
            "field_ready": self._field_ready,
            "field_status": self._field_status,
            "anchor_ready": self._anchor_ready,
            "anchor_status": self._anchor_status,
            "route_source": "liftrace-controlwork@5144aa8/unmodified",
            "map_experiment_source": (
                "liftrace-controlwork@a68925d15293e5510e2b4351c6b3d9bc5aa136ab"),
            "route_total": len(self._policy.waypoints),
            "coverage_index": self._active_route_index,
            "visited": list(self._visited),
            "discovered": list(self._discovered.values()),
            "delivered": list(self._delivered),
            "failed_targets": list(self._failed),
            "release_results": list(self._release_results),
            "command_sequence": list(self._command_sequence),
            "command_events": list(self._command_events),
            "selection_events": list(self._selection_events),
            "status_events": list(self._status_events),
            "event_sequence": self._event_sequence,
            "goal_publish_count": self._goal_publish_count,
            "required_deliveries": self._required,
            "boundary_violations": self._boundary_violations,
            "max_abs_x": self._max_abs_x, "max_abs_y": self._max_abs_y,
            "max_altitude": (None if self._max_altitude == float("-inf")
                             else self._max_altitude),
            "max_height_limit": self._max_height_limit,
            "minimum_clearance": self._minimum_clearance,
            "collision_count": self._collision_count,
            "collision_events": list(self._collision_events),
            "current_candidate": (
                None if self._current_candidate is None
                else self._candidate_record(self._current_candidate)),
            "latest_profile_selected": (
                None if self._latest_selected is None
                else self._candidate_record(
                    self._candidate(self._latest_selected))),
            "target_id_binding": "spatial_inference",
            "temporary_coverage_manager_active": False,
            "navigation_source_modified": False,
        }

    def _publish_status(self, status, reason):
        previous_state = self._last_event_state
        self._event_sequence += 1
        self._status_events.append({
            "sequence": self._event_sequence,
            "stamp": self._now(),
            "event": reason,
            "previous_state": previous_state,
            "state": self._state,
            "candidate": (None if self._current_candidate is None
                          else self._candidate_record(
                              self._current_candidate)),
            "profile": self._class_profile,
        })
        self._last_event_state = self._state
        payload = self._payload(status, reason)
        os.makedirs(os.path.dirname(self._report_path) or ".", exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)
        # Publish terminal/nonterminal state only after its durable snapshot is
        # visible, so a required Gate cannot race the artifact check.
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _finish(self, success, reason):
        self._state = "COMPLETE"
        self._publish_status("PASS" if success else "FAIL", reason)
        return 0 if success else 1

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = self._now()
            if time.monotonic() >= self._wall_deadline:
                return self._finish(False, "wall_timeout")
            if self._field_failed:
                return self._finish(False, "random_field_failed")
            if self._anchor_failed:
                return self._finish(False, "planner_anchor_failed")
            if self._started_at is not None and now - self._started_at >= self._mission_timeout:
                return self._finish(False, "mission_timeout")

            if self._state == "WAIT_TAKEOFF":
                ready = (self._operational_ready() and
                         self._buffered_search_goal is not None)
                if ready and self._ready_since is None:
                    self._ready_since = time.monotonic()
                elif ready and time.monotonic() - self._ready_since >= 2.0:
                    self._started_at = now
                    self._forward_search(
                        self._buffered_search_goal, MissionCommand.SEARCH)
                elif not ready:
                    self._ready_since = None

            elif self._state == "SEARCH":
                if (self._pending_candidate_goal is not None and
                        self._operational_ready()):
                    selected = self._matching_selected(
                        self._pending_candidate_goal)
                    if selected is not None:
                        goal = self._pending_candidate_goal
                        self._pending_candidate_goal = None
                        self._start_approach(goal, selected)
                        rate.sleep()
                        continue
                if self._distance() <= self._arrival and self._active_route_index not in self._visited:
                    self._visited.append(self._active_route_index)
                if (self._active_route_index == len(self._policy.waypoints) - 1 and
                        self._distance() <= self._arrival and
                        now - self._raw_goal_received_at >= 5.0):
                    self._begin_return(False, "coverage_complete_insufficient_deliveries")
                elif self._goal_stalled(self._navigation_stall_timeout):
                    return self._finish(False, "navigation_goal_unreachable")

            elif self._state == "APPROACH":
                if self._distance() <= self._arrival:
                    self._state = "CAPTURE"
                    self._capture_started_at = now
                    self._publish_status("RUNNING", "candidate_capture")
                elif self._goal_stalled(self._navigation_stall_timeout):
                    self._resume_search(False, "approach_unreachable")

            elif self._state == "CAPTURE":
                if self._capture_ok():
                    previous_state = self._state
                    self._state = "ALIGN"
                    self._publish_command(MissionCommand.ALIGN, self._active_goal,
                                          self._current_candidate,
                                          from_state=previous_state)
                    self._align_started_at = now
                    self._publish_status("RUNNING", "candidate_align")
                elif now - self._capture_started_at >= self._capture_timeout:
                    self._resume_search(False, "capture_timeout")

            elif self._state == "ALIGN":
                if self._release_result is not None:
                    result = self._release_result
                    self._release_results.append({
                        "slot": int(result.payload_slot),
                        "success": bool(result.success),
                        "target_class": result.target_class,
                        "align_mode": result.align_mode,
                        "reason": result.reason})
                    if result.success and self._release_matches():
                        self._state = "RECOVERY"
                        self._phase_started_at = now
                    else:
                        self._resume_search(False, "release_rejected")
                elif now - self._align_started_at >= self._align_timeout:
                    self._resume_search(False, "alignment_timeout")

            elif self._state == "RECOVERY":
                if (self._release_result is not None and self._pose is not None and
                        self._control_state == 1 and
                        self._pose.pose.position.z >= self._recovery_height):
                    self._resume_search(True, "guarded_ack_and_climb")
                elif now - self._phase_started_at >= 60.0:
                    return self._finish(False, "release_recovery_timeout")

            elif self._state == "RETURN_HOME":
                if self._distance() <= self._arrival:
                    goal = PoseStamped()
                    goal.header.frame_id = self._frame
                    goal.pose.orientation.w = 1.0
                    previous_state = self._state
                    self._state = "LAND"
                    self._publish_command(
                        MissionCommand.LAND, goal, from_state=previous_state)
                    self._phase_started_at = now
                elif self._goal_stalled(self._return_stall_timeout):
                    return self._finish(False, "return_home_unreachable")

            elif self._state == "LAND":
                if self._pose is not None and self._pose.pose.position.z <= self._landing_height:
                    return self._finish(self._return_success,
                                        "three_deliveries_landed" if self._return_success
                                        else self._return_reason)
                if now - self._phase_started_at >= 90.0:
                    return self._finish(False, "landing_timeout")
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(NavigationVisualDeliveryAdapter().run())
