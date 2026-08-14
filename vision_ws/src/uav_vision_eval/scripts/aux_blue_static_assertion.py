#!/usr/bin/env python3
"""固定斜视场景下验证 OpenCV 蓝环粗发现和地面投影。"""

import json
import math
import os
import threading
import time

import rosgraph
import rospy
import yaml

from uav_vision.msg import TargetDetectionArray


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}
FORBIDDEN_TOPICS = {
    "/fastplanner/goal", "/Servo", "/legacy/Servo_raw",
    "/mission/release_permission", "/uav_vision/selected_target",
    "/uav_vision/drop_ready", "/uav_vision/release_evidence",
}


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + \
        ordered[upper] * (position - lower)


class Assertion:
    def __init__(self):
        catalog_path = rospy.get_param("~target_catalog")
        with open(catalog_path, "r", encoding="utf-8") as stream:
            catalog = yaml.safe_load(stream)
        self._truth = {
            item["target_id"]: item["fallback_center_world"]
            for item in catalog["targets"]
            if item["class_name"] in STANDARD_CLASSES
        }
        self._topic = rospy.get_param(
            "~topic", "/uav_vision/aux/blue_detections_mapped")
        self._wall_timeout = float(rospy.get_param("~wall_timeout_sec", 25.0))
        self._report_path = os.path.abspath(rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json")))
        self._max_error = float(rospy.get_param("~max_map_error_m", 0.80))
        self._min_observations = int(rospy.get_param("~min_observations", 2))
        self._started = time.monotonic()
        self._errors = []
        self._matches = {}
        self._invalid = 0
        self._finished = False
        rospy.Subscriber(self._topic, TargetDetectionArray,
                         self._on_detections, queue_size=4)
        threading.Thread(target=self._wall_loop, daemon=True).start()

    def _on_detections(self, message):
        for detection in message.detections:
            if detection.class_name != "circle" or not detection.map_valid:
                self._invalid += 1
                continue
            point = detection.map_point
            distances = [
                (math.hypot(point.x - truth[0], point.y - truth[1]), target_id)
                for target_id, truth in self._truth.items()
            ]
            error, target_id = min(distances)
            self._errors.append(error)
            if error <= self._max_error:
                self._matches[target_id] = self._matches.get(target_id, 0) + 1

    @staticmethod
    def _forbidden_publishers():
        publishers = rosgraph.Master(
            "/aux_blue_static_assertion").getSystemState()[0]
        violations = []
        for topic, nodes in publishers:
            if topic not in FORBIDDEN_TOPICS:
                continue
            for node in nodes:
                if node.startswith("/aux_blue"):
                    violations.append({"topic": topic, "node": node})
        return violations

    def _finalize(self):
        if self._finished:
            return
        self._finished = True
        p90 = percentile(self._errors, 0.90)
        good = sum(self._matches.values())
        precision = good / float(len(self._errors)) if self._errors else 0.0
        violations = self._forbidden_publishers()
        checks = {
            "enough_observations": len(self._errors) >= self._min_observations,
            "one_standard_target_located": bool(self._matches),
            "precision": precision >= 0.90,
            "map_error_p90": p90 is not None and p90 <= self._max_error,
            "no_aux_control_publishers": not violations,
        }
        payload = {
            "gate": "oblique_aux_blue_static",
            "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "observations": len(self._errors),
            "invalid_observations": self._invalid,
            "matched_targets": dict(sorted(self._matches.items())),
            "precision": precision,
            "p90_map_error_xy_m": p90,
            "forbidden_aux_publishers": violations,
        }
        os.makedirs(os.path.dirname(self._report_path) or ".", exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self._report_path)
        rospy.loginfo("[AuxBlueStatic] %s", payload["status"])
        rospy.signal_shutdown("blue static assertion complete")

    def _wall_loop(self):
        while not rospy.is_shutdown() and not self._finished:
            if time.monotonic() - self._started >= self._wall_timeout:
                self._finalize()
                return
            time.sleep(0.20)


if __name__ == "__main__":
    rospy.init_node("aux_blue_static_assertion")
    Assertion()
    rospy.spin()
