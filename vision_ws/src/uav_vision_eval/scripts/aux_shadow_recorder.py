#!/usr/bin/env python3
"""比较覆盖航线上斜下辅助链与下视权威链的首次发现和模拟交接。"""

import json
import math
import os
import statistics
import threading
import time

import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from uav_vision.msg import TargetDetectionArray


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}
FORBIDDEN_TOPICS = {
    "/fastplanner/goal", "/Servo", "/legacy/Servo_raw",
    "/mission/release_permission", "/uav_vision/selected_target",
    "/uav_vision/drop_ready", "/uav_vision/release_evidence",
    "/detect/waypoint_mark_point", "/detect/land_mark_point",
}
AUX_PREFIXES = ("/aux_", "/oblique_aux_")


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


class Recorder:
    def __init__(self):
        catalog_path = rospy.get_param("~target_catalog")
        with open(catalog_path, "r", encoding="utf-8") as stream:
            catalog = yaml.safe_load(stream)
        self._truth = {
            item["class_name"]: item["fallback_center_world"]
            for item in catalog["targets"]
            if item["class_name"] in STANDARD_CLASSES or
            item["class_name"] == "red_cross"}
        self._aux_topic = rospy.get_param(
            "~aux_topic", "/uav_vision/aux/detections_mapped")
        self._down_topic = rospy.get_param(
            "~downward_topic", "/uav_vision/detections_mapped")
        self._pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        self._status_topic = rospy.get_param(
            "~coverage_status_topic", "/mission/coverage_status")
        self._world_offset = [float(value) for value in rospy.get_param(
            "~camera_init_world_offset", [-0.493412, -1.772690, 0.0])]
        self._min_aux_confidence = float(
            rospy.get_param("~min_aux_confidence", 0.35))
        self._min_down_confidence = float(
            rospy.get_param("~min_down_confidence", 0.45))
        self._handoff_timeout = float(rospy.get_param("~handoff_timeout_sec", 5.0))
        self._handoff_distance = float(
            rospy.get_param("~handoff_distance_m", 1.0))
        self._wall_timeout = float(rospy.get_param("~wall_timeout_sec", 700.0))
        output_dir = os.path.abspath(rospy.get_param(
            "~output_dir", os.environ.get("SIM_RUN_DIR", "/tmp")))
        os.makedirs(output_dir, exist_ok=True)
        self._summary_path = os.path.join(output_dir, "aux_shadow_summary.json")
        self._gate_path = os.path.join(output_dir, "gate_status.json")
        self._pose = None
        self._aux_first = {}
        self._down_first = {}
        self._started = time.monotonic()
        self._terminal_status = None
        self._terminal_at = None
        self._finished = False
        rospy.Subscriber(self._pose_topic, PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber(self._aux_topic, TargetDetectionArray,
                         self._on_aux, queue_size=2)
        rospy.Subscriber(self._down_topic, TargetDetectionArray,
                         self._on_down, queue_size=2)
        rospy.Subscriber(self._status_topic, String,
                         self._on_status, queue_size=2)
        self._thread = threading.Thread(target=self._wall_loop, daemon=True)
        self._thread.start()

    def _on_pose(self, message):
        self._pose = message.pose.position

    def _pose_world(self):
        if self._pose is None:
            return None
        return [self._pose.x + self._world_offset[0],
                self._pose.y + self._world_offset[1],
                self._pose.z + self._world_offset[2]]

    @staticmethod
    def _stamp(message):
        value = message.header.stamp.to_sec()
        return value if value > 0.0 else rospy.Time.now().to_sec()

    def _map_world(self, detection):
        if detection.map_frame == "world":
            return [detection.map_point.x, detection.map_point.y,
                    detection.map_point.z]
        if detection.map_frame == "camera_init":
            return [detection.map_point.x + self._world_offset[0],
                    detection.map_point.y + self._world_offset[1],
                    detection.map_point.z + self._world_offset[2]]
        return None

    def _record(self, storage, message, min_confidence):
        pose_world = self._pose_world()
        for detection in message.detections:
            class_name = detection.class_name
            if (class_name not in self._truth or class_name in storage or
                    not detection.map_valid or
                    detection.class_confidence < min_confidence):
                continue
            point = self._map_world(detection)
            if point is None:
                continue
            truth = self._truth[class_name]
            storage[class_name] = {
                "stamp": self._stamp(message),
                "map_point_world": point,
                "map_error_xy_m": math.hypot(
                    point[0] - truth[0], point[1] - truth[1]),
                "aircraft_position_world": pose_world,
                "aircraft_target_distance_m": (
                    None if pose_world is None else
                    math.hypot(pose_world[0] - truth[0],
                               pose_world[1] - truth[1])),
                "class_confidence": float(detection.class_confidence),
                "map_quality": float(detection.map_quality),
            }

    def _on_aux(self, message):
        self._record(self._aux_first, message, self._min_aux_confidence)

    def _on_down(self, message):
        self._record(self._down_first, message, self._min_down_confidence)

    def _on_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if payload.get("status") in ("PASS", "FAIL"):
            self._terminal_status = payload
            self._terminal_at = time.monotonic()

    @staticmethod
    def _aux_forbidden_publishers():
        publishers, _subscribers, _services = rosgraph.Master(
            "/aux_shadow_recorder").getSystemState()
        violations = []
        for topic, nodes in publishers:
            if topic not in FORBIDDEN_TOPICS:
                continue
            for node in nodes:
                if node.startswith(AUX_PREFIXES):
                    violations.append({"topic": topic, "node": node})
        return violations

    def _finalize(self, reason):
        if self._finished:
            return
        self._finished = True
        pairs = []
        for class_name in sorted(set(self._aux_first) & set(self._down_first)):
            auxiliary = self._aux_first[class_name]
            downward = self._down_first[class_name]
            lead_time = downward["stamp"] - auxiliary["stamp"]
            aux_distance = auxiliary["aircraft_target_distance_m"]
            down_distance = downward["aircraft_target_distance_m"]
            lead_distance = (
                None if aux_distance is None or down_distance is None else
                aux_distance - down_distance)
            handoff_distance = math.hypot(
                auxiliary["map_point_world"][0] - downward["map_point_world"][0],
                auxiliary["map_point_world"][1] - downward["map_point_world"][1])
            handoff_ok = (
                0.0 <= lead_time <= self._handoff_timeout and
                handoff_distance <= self._handoff_distance)
            pairs.append({
                "class_name": class_name,
                "lead_time_sec": lead_time,
                "lead_distance_m": lead_distance,
                "handoff_distance_m": handoff_distance,
                "handoff_ok": handoff_ok,
                "auxiliary": auxiliary,
                "downward": downward,
            })
        earlier = [item for item in pairs if item["lead_time_sec"] > 0.0]
        lead_times = [item["lead_time_sec"] for item in earlier]
        lead_distances = [item["lead_distance_m"] for item in earlier
                          if item["lead_distance_m"] is not None]
        coarse_errors = [item["map_error_xy_m"]
                         for item in self._aux_first.values()]
        handoff_rate = (
            sum(item["handoff_ok"] for item in pairs) / float(len(pairs))
            if pairs else 0.0)
        median_time = statistics.median(lead_times) if lead_times else None
        median_distance = (
            statistics.median(lead_distances) if lead_distances else None)
        p90_error = percentile(coarse_errors, 0.90)
        violations = self._aux_forbidden_publishers()
        route_pass = (self._terminal_status is not None and
                      self._terminal_status.get("status") == "PASS")
        checks = {
            "route_completed": route_pass,
            "two_targets_earlier": len(earlier) >= 2,
            "lead_gain": ((median_distance is not None and median_distance >= 1.5) or
                          (median_time is not None and median_time >= 1.0)),
            "coarse_map_p90": p90_error is not None and p90_error <= 0.80,
            "handoff_success": handoff_rate >= 0.80,
            "no_aux_control_publishers": not violations,
        }
        status = "PASS" if all(checks.values()) else "FAIL"
        payload = {
            "gate": "oblique_aux_coverage_shadow",
            "status": status,
            "reason": reason,
            "checks": checks,
            "metrics": {
                "auxiliary_distinct_targets": len(self._aux_first),
                "downward_distinct_targets": len(self._down_first),
                "paired_targets": len(pairs),
                "earlier_distinct_targets": len(earlier),
                "median_lead_time_sec": median_time,
                "median_lead_distance_m": median_distance,
                "p90_aux_map_error_xy_m": p90_error,
                "handoff_success_rate": handoff_rate,
            },
            "pairs": pairs,
            "auxiliary_first": self._aux_first,
            "downward_first": self._down_first,
            "coverage_terminal": self._terminal_status,
            "forbidden_aux_publishers": violations,
        }
        for path in (self._summary_path, self._gate_path):
            temporary = path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2,
                          sort_keys=True)
                stream.write("\n")
            os.replace(temporary, path)
        rospy.loginfo("Oblique auxiliary coverage shadow %s", status)
        rospy.signal_shutdown("shadow evaluation complete")

    def _wall_loop(self):
        while not rospy.is_shutdown() and not self._finished:
            if (self._terminal_at is not None and
                    time.monotonic() - self._terminal_at >= 2.0):
                self._finalize("coverage_terminal")
                return
            if time.monotonic() - self._started >= self._wall_timeout:
                self._finalize("wall_timeout")
                return
            time.sleep(0.20)


if __name__ == "__main__":
    rospy.init_node("aux_shadow_recorder")
    Recorder()
    rospy.spin()
