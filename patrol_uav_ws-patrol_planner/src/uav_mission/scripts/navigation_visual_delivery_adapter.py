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

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8, String

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


class NavigationVisualDeliveryAdapter:
    def __init__(self):
        rospy.init_node("navigation_visual_delivery_adapter")
        self._frame = rospy.get_param("~mission_frame", "camera_init")
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
        self._return_success = False
        self._return_reason = ""
        self._phase_started_at = None
        self._best_goal_distance = float("inf")
        self._last_progress_at = None

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
        self._publish_status("RUNNING", "waiting_for_takeoff")

    def _now(self):
        return rospy.Time.now().to_sec()

    def _on_pose(self, msg):
        self._pose = msg
        p = msg.pose.position
        self._max_abs_x = max(self._max_abs_x, abs(p.x))
        self._max_abs_y = max(self._max_abs_y, abs(p.y))
        if self._started_at is not None:
            if not (self._field[0] <= p.x <= self._field[1] and
                    self._field[2] <= p.y <= self._field[3]):
                self._boundary_violations += 1

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)

    def _on_map(self, msg):
        stamp = msg.header.stamp.to_sec()
        self._map_stamp = stamp if stamp > 0.0 else self._now()

    @staticmethod
    def _candidate(msg):
        return Candidate(int(msg.id), msg.class_name,
                         float(msg.class_confidence),
                         float(msg.map_point.x), float(msg.map_point.y))

    def _on_selected(self, msg):
        self._latest_selected = msg
        if msg.map_valid and msg.class_name:
            candidate = self._candidate(msg)
            self._discovered[candidate.target_id] = self._candidate_record(candidate)
            if (self._pending_candidate_goal is not None and
                    math.hypot(
                        self._pending_candidate_goal.pose.position.x -
                        msg.map_point.x,
                        self._pending_candidate_goal.pose.position.y -
                        msg.map_point.y) <= 0.6):
                goal = self._pending_candidate_goal
                self._pending_candidate_goal = None
                self._start_approach(goal, msg)

    def _on_targets(self, msg):
        evidence = []
        for target in msg.targets:
            if target.class_name not in ("circle", "red_cross"):
                continue
            if (target.state < 2 or not target.map_valid or
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
            if self._state in ("WAIT_TAKEOFF", "CAPTURE", "ALIGN", "RECOVERY",
                               "RETURN_HOME", "LAND", "COMPLETE"):
                return
            self._forward_search(msg, MissionCommand.SEARCH)
            return

        selected = self._latest_selected
        if (selected is None or not selected.map_valid or
                math.hypot(msg.pose.position.x - selected.map_point.x,
                           msg.pose.position.y - selected.map_point.y) > 0.6):
            self._pending_candidate_goal = msg
            rospy.logwarn("[NavAdapter] waiting for candidate matching raw goal")
            return
        self._start_approach(msg, selected)

    def _start_approach(self, msg, selected):
        self._current_candidate = self._candidate(selected)
        self._active_goal = msg
        self._publish_goal(msg)
        self._publish_command(MissionCommand.APPROACH, msg,
                              self._current_candidate)
        self._state = "APPROACH"
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "candidate_approach")

    def _forward_search(self, goal, command):
        self._active_goal = goal
        self._publish_goal(goal)
        self._publish_command(command, goal)
        self._state = "SEARCH"
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "search_goal_%d" % self._active_route_index)

    def _publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self._goal_pub.publish(goal)
        self._goal_publish_count += 1

    def _publish_command(self, command, goal, candidate=None):
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
        self._active_goal = goal
        self._publish_goal(goal)
        self._publish_command(MissionCommand.RETURN_HOME, goal)
        self._state = "RETURN_HOME"
        self._phase_started_at = self._now()
        self._reset_goal_progress()
        self._publish_status("RUNNING", "return_home_%s" % reason)

    @staticmethod
    def _candidate_record(candidate):
        return {"id": candidate.target_id, "class": candidate.class_name,
                "confidence": candidate.confidence,
                "map": [candidate.x, candidate.y]}

    def _map_ready(self):
        return self._map_stamp is not None and self._now() - self._map_stamp <= 2.0

    def _payload(self, status, reason):
        return {
            "gate": "navigation_search_visual_delivery",
            "status": status, "reason": reason, "state": self._state,
            "mission_elapsed": (None if self._started_at is None else
                                self._now() - self._started_at),
            "route_source": "liftrace-controlwork@5144aa8/unmodified",
            "route_total": len(self._policy.waypoints),
            "coverage_index": self._active_route_index,
            "visited": list(self._visited),
            "discovered": list(self._discovered.values()),
            "delivered": list(self._delivered),
            "failed_targets": list(self._failed),
            "release_results": list(self._release_results),
            "command_sequence": list(self._command_sequence),
            "goal_publish_count": self._goal_publish_count,
            "required_deliveries": self._required,
            "boundary_violations": self._boundary_violations,
            "max_abs_x": self._max_abs_x, "max_abs_y": self._max_abs_y,
            "temporary_coverage_manager_active": False,
            "navigation_source_modified": False,
        }

    def _publish_status(self, status, reason):
        payload = self._payload(status, reason)
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        os.makedirs(os.path.dirname(self._report_path) or ".", exist_ok=True)
        with open(self._report_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")

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
            if self._started_at is not None and now - self._started_at >= self._mission_timeout:
                return self._finish(False, "mission_timeout")

            if self._state == "WAIT_TAKEOFF":
                ready = (self._pose is not None and self._control_state == 1 and
                         self._pose.pose.position.z >= self._takeoff_height and
                         self._map_ready() and self._raw_goal is not None)
                if ready and self._ready_since is None:
                    self._ready_since = time.monotonic()
                elif ready and time.monotonic() - self._ready_since >= 2.0:
                    self._started_at = now
                    self._forward_search(self._raw_goal, MissionCommand.SEARCH)
                elif not ready:
                    self._ready_since = None

            elif self._state == "SEARCH":
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
                    self._publish_command(MissionCommand.ALIGN, self._active_goal,
                                          self._current_candidate)
                    self._state = "ALIGN"
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
                    self._publish_command(MissionCommand.LAND, goal)
                    self._state = "LAND"
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
