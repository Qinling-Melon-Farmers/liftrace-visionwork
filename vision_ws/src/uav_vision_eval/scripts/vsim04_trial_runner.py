#!/usr/bin/env python3
"""Drive all V-SIM-04 camera trials in one fail-closed Gazebo session."""

import json
import math
import os
import sys
import threading
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String
from std_srvs.srv import Empty

from uav_vision_eval.vsim04_metrics import (
    call_with_monotonic_deadline,
    handshake_timeout_is_safe,
    load_trial_matrix,
    select_trial_matrix,
)


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class VSim04TrialRunner:
    def __init__(self):
        rospy.init_node("vsim04_trial_runner")
        self._matrix_path = os.path.abspath(rospy.get_param("~matrix_file"))
        self._matrix = select_trial_matrix(
            load_trial_matrix(self._matrix_path),
            rospy.get_param("~trial_selector", ""))
        self._evaluation_scope = self._matrix["evaluation_scope"]
        runner = self._matrix.get("runner", {})
        self._camera_model = rospy.get_param(
            "~camera_model", runner.get("camera_model", "vision_eval_camera"))
        self._reset_service_name = rospy.get_param(
            "~reset_service", runner.get(
                "reset_service", "/uav_vision/reset_memory"))
        self._event_topic = rospy.get_param(
            "~trial_event_topic", runner.get(
                "trial_event_topic", "/uav_vision_eval/vsim04/trial_event"))
        self._status_topic = rospy.get_param(
            "~status_topic", "/uav_vision_eval/vsim04/status")
        self._offscreen_offset = float(runner.get(
            "offscreen_offset_m", 3.5))
        self._pretrial_settle = float(runner.get(
            "pretrial_settle_sec", 0.5))
        self._posttrial_settle = float(runner.get(
            "posttrial_settle_sec", 0.8))
        self._arena_limit = float(rospy.get_param("~arena_limit_m", 4.8))
        self._rpy = [float(value) for value in runner.get(
            "camera_rpy", [0.0, math.pi / 2.0, 0.0])]
        self._rpy = [float(value) for value in rospy.get_param(
            "~camera_rpy", self._rpy)]
        self._clock_stall_timeout = float(rospy.get_param(
            "~clock_stall_timeout_sec", 2.0))
        self._service_call_timeout = float(rospy.get_param(
            "~service_call_timeout_sec", 5.0))
        self._ros_wait_wall_factor = float(rospy.get_param(
            "~ros_wait_wall_factor", 10.0))
        self._ros_wait_wall_padding = float(rospy.get_param(
            "~ros_wait_wall_padding_sec", 5.0))
        self._recorder_drain_timeout = float(rospy.get_param(
            "~recorder_output_drain_timeout_sec", 10.0))
        self._recorder_quiet_sec = float(rospy.get_param(
            "~recorder_output_quiet_sec", 0.25))
        self._recorder_status_period = float(rospy.get_param(
            "~recorder_status_period_sec", 0.25))
        self._handshake_write_margin = float(rospy.get_param(
            "~handshake_write_margin_sec", 5.0))
        self._trial_handshake_timeout = float(rospy.get_param(
            "~trial_handshake_timeout_sec", 16.0))
        if not self._matrix["trials"]:
            raise ValueError("V-SIM-04 runner requires at least one trial")
        if (len(self._rpy) != 3 or
                not all(math.isfinite(value) for value in self._rpy)):
            raise ValueError("camera_rpy must contain three finite values")
        if (not math.isfinite(self._clock_stall_timeout) or
                self._clock_stall_timeout <= 0.0):
            raise ValueError("clock_stall_timeout_sec must be positive")
        for name, value in (
                ("service_call_timeout_sec", self._service_call_timeout),
                ("ros_wait_wall_factor", self._ros_wait_wall_factor),
                ("ros_wait_wall_padding_sec", self._ros_wait_wall_padding),
                ("recorder_output_drain_timeout_sec",
                 self._recorder_drain_timeout),
                ("recorder_output_quiet_sec", self._recorder_quiet_sec),
                ("recorder_status_period_sec",
                 self._recorder_status_period),
                ("handshake_write_margin_sec",
                 self._handshake_write_margin),
                ("trial_handshake_timeout_sec",
                 self._trial_handshake_timeout)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be positive".format(name))
        handshake_floor = (
            self._recorder_drain_timeout + self._recorder_quiet_sec +
            self._recorder_status_period + self._handshake_write_margin)
        if not handshake_timeout_is_safe(
                self._trial_handshake_timeout,
                self._recorder_drain_timeout, self._recorder_quiet_sec,
                self._recorder_status_period, self._handshake_write_margin):
            raise ValueError(
                "trial_handshake_timeout_sec must exceed recorder drain, "
                "quiet, status period, and write margin ({:.2f}s)".format(
                    handshake_floor))
        self._event_seq = 0
        self._publisher = rospy.Publisher(
            self._event_topic, String, queue_size=40, latch=True)
        self._set_state_service_name = rospy.get_param(
            "~set_model_state_service", "/gazebo/set_model_state")
        self._set_state = rospy.ServiceProxy(
            self._set_state_service_name, SetModelState)
        self._reset = rospy.ServiceProxy(self._reset_service_name, Empty)
        self._status_condition = threading.Condition()
        self._recorder_status = None
        rospy.Subscriber(
            self._status_topic, String, self._on_status, queue_size=10)

    def _on_status(self, message):
        try:
            status = json.loads(message.data)
            if status.get("evaluation_id") != "V-SIM-04":
                return
            with self._status_condition:
                self._recorder_status = status
                self._status_condition.notify_all()
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "invalid V-SIM-04 recorder status")

    def _publish_event(self, event, trial=None, **details):
        self._event_seq += 1
        payload = {
            "event": event,
            "event_seq": self._event_seq,
            "stamp": rospy.Time.now().to_sec(),
            "monotonic_sec": time.monotonic(),
        }
        if trial is not None:
            payload["trial_id"] = trial["trial_id"]
            payload["trial"] = trial
        payload.update(details)
        self._publisher.publish(String(data=json.dumps(
            payload, sort_keys=True)))
        return payload

    def _call_service_with_deadline(self, name, proxy, *args):
        return call_with_monotonic_deadline(
            lambda: proxy(*args), self._service_call_timeout,
            "{}_service".format(name), rospy.is_shutdown)

    def _set_camera(self, x, y, z):
        state = ModelState()
        state.model_name = self._camera_model
        state.reference_frame = "world"
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        quaternion = _quaternion_from_rpy(*self._rpy)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        response = self._call_service_with_deadline(
            "set_model_state", self._set_state, state)
        if not response.success:
            raise RuntimeError("set_model_state failed: " +
                               response.status_message)

    def _anchor(self, trial):
        values = self._matrix["target_anchors"][trial["class_name"]]["xyz"]
        return tuple(float(value) for value in values)

    def _offscreen(self, trial):
        x, y, z = self._anchor(trial)
        camera_x = min(self._arena_limit, x + self._offscreen_offset)
        return camera_x, y, z + trial["height_m"]

    def _sleep_until_ros(self, target_time, wall_deadline=None):
        """Wait for ROS time with a monotonic watchdog for stalled /clock."""
        last_ros = rospy.Time.now()
        started_wall = time.monotonic()
        last_progress = started_wall
        if wall_deadline is None:
            expected_remaining = max(
                0.0, (target_time - last_ros).to_sec())
            wall_deadline = (
                started_wall +
                expected_remaining * self._ros_wait_wall_factor +
                self._ros_wait_wall_padding)
        while not rospy.is_shutdown():
            now_wall = time.monotonic()
            if now_wall >= wall_deadline:
                raise RuntimeError(
                    "ROS-time wait exceeded total wall-clock budget")
            now_ros = rospy.Time.now()
            if now_ros >= target_time:
                return
            if now_ros > last_ros:
                last_ros = now_ros
                last_progress = time.monotonic()
            elif now_ros < last_ros:
                raise RuntimeError("ROS time moved backwards during trial")
            elif now_wall - last_progress > self._clock_stall_timeout:
                raise RuntimeError(
                    "/clock stalled for {:.2f}s".format(
                        self._clock_stall_timeout))
            time.sleep(0.02)
        raise RuntimeError("ROS shutdown while waiting for simulation time")

    def _sleep_ros_duration(self, duration_sec):
        duration_sec = float(duration_sec)
        if duration_sec <= 0.0:
            return
        self._sleep_until_ros(
            rospy.Time.now() + rospy.Duration(duration_sec))

    def _run_static(self, trial):
        x, y, z = self._anchor(trial)
        self._set_camera(x, y, z + trial["height_m"])
        start = rospy.Time.now()
        expected = float(self._matrix["static"].get(
            "center_dwell_sec", 2.0))
        self._sleep_until_ros(start + rospy.Duration(expected))
        actual = max(0.0, (rospy.Time.now() - start).to_sec())
        return {
            "mode": "static_dwell",
            "expected_duration_sec": expected,
            "actual_duration_sec": actual,
            "distance_m": 0.0,
            "expected_speed_mps": None,
            "actual_speed_mps": None,
        }

    def _run_dynamic(self, trial):
        x, y, z = self._anchor(trial)
        config = self._matrix["dynamic"]
        half_length = float(config.get("path_half_length_m", 3.5))
        update_rate = float(config.get("update_rate_hz", 20.0))
        speed = float(trial["speed_mps"])
        start_x = max(-self._arena_limit, x - half_length)
        finish_x = min(self._arena_limit, x + half_length)
        distance = max(0.0, finish_x - start_x)
        expected_duration = distance / speed
        steps = max(1, int(math.ceil(expected_duration * update_rate)))
        self._set_camera(start_x, y, z + trial["height_m"])
        start_time = rospy.Time.now()
        wall_deadline = (
            time.monotonic() +
            expected_duration * self._ros_wait_wall_factor +
            self._ros_wait_wall_padding)
        for index in range(1, steps + 1):
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown during dynamic trial")
            fraction = index / float(steps)
            target_time = start_time + rospy.Duration(
                expected_duration * fraction)
            self._sleep_until_ros(target_time, wall_deadline)
            self._set_camera(
                start_x + fraction * distance, y,
                z + trial["height_m"])
        if time.monotonic() >= wall_deadline:
            raise RuntimeError(
                "dynamic trajectory exceeded total wall-clock budget")
        actual_duration = max(
            0.0, (rospy.Time.now() - start_time).to_sec())
        return {
            "mode": "absolute_ros_time_linear",
            "expected_duration_sec": expected_duration,
            "actual_duration_sec": actual_duration,
            "distance_m": distance,
            "expected_speed_mps": speed,
            "actual_speed_mps": (
                distance / actual_duration if actual_duration > 0.0 else None),
            "update_rate_hz": update_rate,
            "steps": steps,
            "start_x": start_x,
            "finish_x": finish_x,
        }

    def _wait_for_event_subscriber(self, deadline):
        while self._publisher.get_num_connections() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("trial recorder did not subscribe to events")
            time.sleep(0.05)

    def _wait_for_recorder_ready(self, deadline):
        with self._status_condition:
            while True:
                status = self._recorder_status
                if status is not None:
                    if status.get("state") == "FAIL":
                        raise RuntimeError(
                            "recorder preflight failed: {}".format(
                                status.get("error", "unknown")))
                    if status.get("state") == "READY" and status.get("ready"):
                        return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    missing = ((status or {}).get("missing", [
                        "recorder_status"]))
                    raise RuntimeError(
                        "recorder preflight timeout; missing={}".format(
                            ",".join(missing)))
                self._status_condition.wait(min(remaining, 0.25))

    def _wait_for_trial_status(self, trial_id, completed_count, timeout):
        deadline = time.monotonic() + timeout
        with self._status_condition:
            while True:
                status = self._recorder_status
                if status is not None:
                    if status.get("state") == "FAIL":
                        raise RuntimeError(
                            "recorder failed during trial handshake: {}".format(
                                status.get("error", "unknown")))
                    if completed_count is None:
                        if (status.get("state") == "RUNNING" and
                                status.get("active_trial") == trial_id):
                            return
                    elif (int(status.get("completed_trial_count", 0)) >=
                          int(completed_count) and
                          status.get("state") == "READY"):
                        return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        "recorder trial handshake timeout: {}".format(trial_id))
                self._status_condition.wait(min(remaining, 0.25))

    def _wait_for_recorder_final(self, timeout):
        deadline = time.monotonic() + timeout
        with self._status_condition:
            while True:
                status = self._recorder_status
                if status is not None and status.get("state") == "FINALIZED":
                    expected_status = (
                        "MEASURED" if self._evaluation_scope == "full" else
                        "DIAGNOSTIC")
                    if (status.get("success") and
                            status.get("summary_status") == expected_status and
                            int(status.get("completed_trial_count", 0)) ==
                            int(status.get("expected_trial_count", -1))):
                        return
                    raise RuntimeError(
                        "recorder terminal validation failed: {}".format(
                            status.get("error", "INVALID")))
                if status is not None and status.get("state") == "FAIL":
                    raise RuntimeError(
                        "recorder failed before finalization: {}".format(
                            status.get("error", "unknown")))
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("recorder finalization timeout")
                self._status_condition.wait(min(remaining, 0.25))

    def run(self):
        timeout = float(rospy.get_param("~startup_timeout_sec", 30.0))
        rospy.wait_for_service(self._set_state_service_name, timeout=timeout)
        rospy.wait_for_service(self._reset_service_name, timeout=timeout)
        deadline = time.monotonic() + timeout
        self._wait_for_event_subscriber(deadline)

        first_trial = self._matrix["trials"][0]
        last_error = None
        while time.monotonic() < deadline:
            try:
                self._set_camera(*self._offscreen(first_trial))
                last_error = None
                break
            except TimeoutError:
                raise
            except Exception as error:
                last_error = error
                time.sleep(0.1)
        if last_error is not None:
            raise RuntimeError(
                "evaluation camera did not become available: {}".format(
                    last_error))

        self._wait_for_recorder_ready(deadline)
        self._sleep_ros_duration(float(rospy.get_param(
            "~startup_settle_sec", 0.5)))

        for trial_index, trial in enumerate(self._matrix["trials"], 1):
            self._set_camera(*self._offscreen(trial))
            self._sleep_ros_duration(self._pretrial_settle)
            self._call_service_with_deadline(
                "reset_memory", self._reset)
            self._publish_event("trial_start", trial)
            self._wait_for_trial_status(
                trial["trial_id"], None, self._trial_handshake_timeout)
            if trial["kind"] == "static":
                trajectory = self._run_static(trial)
            else:
                trajectory = self._run_dynamic(trial)
            self._set_camera(*self._offscreen(trial))
            self._sleep_ros_duration(self._posttrial_settle)
            self._publish_event(
                "trial_end", trial, trajectory=trajectory)
            self._wait_for_trial_status(
                trial["trial_id"], trial_index,
                self._trial_handshake_timeout)

        self._publish_event(
            "run_complete", trial_count=len(self._matrix["trials"]))
        self._wait_for_recorder_final(float(rospy.get_param(
            "~finalization_timeout_sec", 30.0)))
        rospy.loginfo(
            "V-SIM-04 trial runner complete and validated (%s, %d trials)",
            self._evaluation_scope, len(self._matrix["trials"]))

    def abort(self, error):
        try:
            with self._status_condition:
                status = self._recorder_status
                if status is not None and status.get("finalized"):
                    return
            payload = self._publish_event("run_abort", error=str(error))
            deadline = time.monotonic() + float(rospy.get_param(
                "~abort_ack_timeout_sec", 5.0))
            with self._status_condition:
                while True:
                    status = self._recorder_status
                    if status is not None:
                        if status.get("finalized"):
                            return
                        if (status.get("state") == "FAIL" and
                                int(status.get("abort_event_seq", -1)) ==
                                int(payload["event_seq"])):
                            return
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0 or rospy.is_shutdown():
                        rospy.logerr(
                            "V-SIM-04 recorder did not ACK run_abort seq=%d",
                            payload["event_seq"])
                        return
                    self._status_condition.wait(min(remaining, 0.25))
        except Exception as publish_error:
            rospy.logerr("failed to publish V-SIM-04 abort: %s", publish_error)


def main():
    runner = None
    try:
        runner = VSim04TrialRunner()
        runner.run()
    except Exception as error:
        rospy.logerr("V-SIM-04 trial runner failed: %s", error)
        if runner is not None:
            runner.abort(error)
        return 8
    return 0


if __name__ == "__main__":
    sys.exit(main())
