#!/usr/bin/env python3
"""Fail-closed ROS shell that starts the VCL06 manager exactly once."""

import json
import math
import os
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

from uav_mission.mission_start_gate import (
    MissionStartGate,
    StartGateConfig,
)


class NavigationMissionStartGateNode:
    """Bridge latched readiness JSON into one guarded Trigger invocation."""

    def __init__(self):
        self._lock = threading.RLock()
        self._status_topic = rospy.get_param(
            "~status_topic", "/navigation/mission_start_gate_status")
        self._start_service_name = rospy.get_param(
            "~start_service", "/navigation/start_mission")
        self._control_ready_topic = rospy.get_param(
            "~control_ready_topic", "/mission/control_ready")
        self._service_probe_timeout = float(
            rospy.get_param("~service_probe_timeout", 0.05))
        self._tick_hz = float(rospy.get_param("~tick_hz", 5.0))
        if (not math.isfinite(self._service_probe_timeout) or
                self._service_probe_timeout <= 0.0):
            raise ValueError("service_probe_timeout must be positive")
        if not math.isfinite(self._tick_hz) or self._tick_hz <= 0.0:
            raise ValueError("tick_hz must be positive")

        config = StartGateConfig(
            expected_seed=int(rospy.get_param("~field_seed", 11)),
            expected_truth_path=rospy.get_param(
                "~expected_truth_path",
                os.path.join(
                    os.environ.get("SIM_RUN_DIR", "/tmp"),
                    "random_field_truth.yaml")),
            profile=rospy.get_param("~class_profile", "r2026"),
            nav_feature_profile=rospy.get_param(
                "~nav_feature_profile", "baseline"),
            retry_initial_sec=float(
                rospy.get_param("~retry_initial_sec", 0.5)),
            retry_max_sec=float(
                rospy.get_param("~retry_max_sec", 5.0)),
        )
        self._gate = MissionStartGate(
            config, enabled=rospy.get_param("~enabled", False))
        self._runtime_error = ""
        self._last_encoded = ""

        self._status_pub = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)
        self._start_service = rospy.ServiceProxy(
            self._start_service_name, Trigger)
        rospy.Subscriber(
            rospy.get_param(
                "~field_status_topic", "/mission/random_field_status"),
            String, self._on_field, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                "~anchor_status_topic", "/mission/planner_anchor_status"),
            String, self._on_anchor, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                "~manager_status_topic", "/navigation/mission_status"),
            String, self._on_manager, queue_size=1)
        rospy.Subscriber(
            self._control_ready_topic,
            Bool, self._on_control_ready, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self._tick_hz), self._on_timer)
        self._refresh_truth()
        self._publish(force=True)
        rospy.loginfo(
            "VCL06 mission start gate ready; enabled=%s seed=%d profile=%s",
            self._gate.enabled,
            config.expected_seed,
            config.profile,
        )

    @staticmethod
    def _decode(message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("JSON root must be an object")
            return payload
        except (TypeError, ValueError) as error:
            return {"_decode_error": "%s:%s" % (
                type(error).__name__, str(error))}

    def _read_truth(self):
        path = self._gate.config.expected_truth_path
        if not os.path.isfile(path):
            return None, False
        try:
            if os.path.getsize(path) <= 0:
                return None, False
            with open(path, encoding="utf-8") as handle:
                payload = yaml.safe_load(handle)
            if not isinstance(payload, dict):
                raise ValueError("truth YAML root must be a mapping")
            return payload, True
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            return {"_decode_error": "%s:%s" % (
                type(error).__name__, str(error))}, True

    def _refresh_truth(self):
        truth, durable = self._read_truth()
        self._gate.update_field(
            self._gate.field, truth=truth, truth_durable=durable)

    def _on_field(self, message):
        with self._lock:
            self._gate.field = self._decode(message)
            self._refresh_truth()
            self._publish(force=True)

    def _on_anchor(self, message):
        with self._lock:
            self._gate.update_anchor(self._decode(message))
            self._publish(force=True)

    def _on_manager(self, message):
        with self._lock:
            self._gate.update_manager(self._decode(message))
            self._publish(force=True)

    def _on_control_ready(self, message):
        with self._lock:
            self._gate.update_control_ready(message.data)
            self._publish(force=True)

    def _publish(self, force=False):
        now = time.monotonic()
        payload = self._gate.status(now)
        payload.update({
            "start_service": self._start_service_name,
            "status_topic": self._status_topic,
            "control_ready_topic": self._control_ready_topic,
            "runtime_error": self._runtime_error,
            "updated_wall_time": time.time(),
        })
        if self._runtime_error:
            payload["status"] = "ERROR"
            payload["reason"] = "internal_error_fail_closed"
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        if force or encoded != self._last_encoded:
            self._status_pub.publish(String(data=encoded))
            self._last_encoded = encoded

    def _probe_service(self):
        try:
            rospy.wait_for_service(
                self._start_service_name,
                timeout=self._service_probe_timeout)
            return True, ""
        except rospy.ROSException as error:
            return False, "service_unavailable:%s" % error

    def _on_timer(self, _event):
        try:
            with self._lock:
                self._refresh_truth()
                now = time.monotonic()
                should_probe = bool(
                    not self._runtime_error and
                    self._gate.may_probe_service(now))
                self._publish()
            if not should_probe:
                return

            available, message = self._probe_service()
            if not available:
                with self._lock:
                    self._gate.record_service_unavailable(
                        time.monotonic(), message)
                    self._publish(force=True)
                return

            with self._lock:
                now = time.monotonic()
                self._refresh_truth()
                if not self._gate.begin_service_call(now):
                    self._publish(force=True)
                    return
                self._publish(force=True)

            try:
                response = self._start_service()
                success = bool(response.success)
                message = str(response.message)
            except rospy.ServiceException as error:
                success = False
                message = "service_call_failed:%s" % error

            with self._lock:
                self._gate.complete_service_call(
                    time.monotonic(), success, message)
                self._publish(force=True)
                if success:
                    rospy.loginfo(
                        "VCL06 mission start accepted and permanently "
                        "latched: %s", message)
                else:
                    rospy.logwarn(
                        "VCL06 mission start rejected; bounded retry: %s",
                        message)
        except Exception as error:  # pylint: disable=broad-except
            with self._lock:
                self._runtime_error = "%s:%s" % (
                    type(error).__name__, str(error))
                rospy.logerr(
                    "VCL06 mission start gate failed closed: %s", error)
                self._publish(force=True)


def main():
    rospy.init_node("navigation_mission_start_gate")
    NavigationMissionStartGateNode()
    rospy.spin()


if __name__ == "__main__":
    main()
