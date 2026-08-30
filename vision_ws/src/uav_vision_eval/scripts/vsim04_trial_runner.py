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

from uav_vision_eval.failure_capture import validate_capture_status
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
            rospy.get_param("~trial_selector", ""),
            rospy.get_param("~trial_slice", ""))
        self._evaluation_scope = self._matrix["evaluation_scope"]
        self._selected_trial_ids = [
            trial["trial_id"] for trial in self._matrix["trials"]]
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
        self._require_capture_ready = bool(rospy.get_param(
            "~require_capture_ready", False))
        self._capture_status_topic = rospy.get_param(
            "~capture_status_topic",
            "/uav_vision_eval/vsim04/failure_capture/status")
        self._capture_control_topic = rospy.get_param(
            "~capture_control_topic",
            "/uav_vision_eval/vsim04/failure_capture/control")
        self._capture_sampling_start_lead = float(rospy.get_param(
            "~capture_sampling_start_lead_sec", 1.0))
        if self._require_capture_ready:
            capture_topics = [
                rospy.resolve_name(value) for value in (
                    self._event_topic, self._status_topic,
                    self._capture_status_topic,
                    self._capture_control_topic)]
            if len(capture_topics) != len(set(capture_topics)):
                raise ValueError(
                    "recorder/capture event and status topics must be distinct")
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
                 self._trial_handshake_timeout),
                ("capture_sampling_start_lead_sec",
                 self._capture_sampling_start_lead)):
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
        self._capture_event_seq = 0
        self._publisher = rospy.Publisher(
            self._event_topic, String, queue_size=40, latch=True)
        self._capture_control_publisher = rospy.Publisher(
            self._capture_control_topic, String, queue_size=4, latch=True)
        self._set_state_service_name = rospy.get_param(
            "~set_model_state_service", "/gazebo/set_model_state")
        self._set_state = rospy.ServiceProxy(
            self._set_state_service_name, SetModelState)
        self._reset = rospy.ServiceProxy(self._reset_service_name, Empty)
        self._status_condition = threading.Condition()
        self._recorder_status = None
        self._capture_status_condition = threading.Condition()
        self._capture_status = None
        rospy.Subscriber(
            self._status_topic, String, self._on_status, queue_size=10)
        if self._require_capture_ready:
            rospy.Subscriber(
                self._capture_status_topic, String,
                self._on_capture_status, queue_size=10)

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

    def _on_capture_status(self, message):
        try:
            status = validate_capture_status(
                json.loads(message.data), self._selected_trial_ids)
            with self._capture_status_condition:
                self._capture_status = status
                self._capture_status_condition.notify_all()
        except (TypeError, ValueError):
            rospy.logwarn_throttle(
                5.0, "invalid V-SIM-04 failure capture status")

    def _wait_for_capture_status(self, predicate, deadline, description):
        if not self._require_capture_ready:
            return
        with self._capture_status_condition:
            while True:
                status = self._capture_status
                if status is not None:
                    if status.get("state") == "FAIL":
                        raise RuntimeError(
                            "failure capture failed: {}".format(
                                status.get("error", "unknown")))
                    if predicate(status):
                        return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or rospy.is_shutdown():
                    raise RuntimeError(
                        "failure capture handshake timeout: " + description)
                self._capture_status_condition.wait(min(remaining, 0.25))

    def _wait_for_capture_ready(self, deadline):
        self._wait_for_capture_status(
            lambda status: (
                status.get("state") == "READY" and status.get("ready") is True
                and int(status.get("completed_trial_count", -1)) == 0),
            deadline, "startup ready")

    def _wait_for_capture_trial_start(self, trial_id, event_seq):
        self._wait_for_capture_status(
            lambda status: (
                status.get("state") == "RUNNING" and
                status.get("active_trial") == trial_id and
                status.get("active_event") == "trial_start" and
                int(status.get("active_event_seq", -1)) == int(event_seq)),
            time.monotonic() + self._trial_handshake_timeout,
            "trial_start {}".format(trial_id))

    def _wait_for_capture_sampling_start(self, trial_id, event_seq):
        self._wait_for_capture_status(
            lambda status: (
                status.get("state") == "RUNNING" and
                status.get("active_trial") == trial_id and
                status.get("active_event") == "sampling_start" and
                int(status.get("active_event_seq", -1)) == int(event_seq)),
            time.monotonic() + self._trial_handshake_timeout,
            "sampling_start {}".format(trial_id))

    def _wait_for_capture_trial_end(self, completed_count):
        self._wait_for_capture_status(
            lambda status: (
                status.get("state") == "READY" and
                not status.get("active_trial") and
                int(status.get("completed_trial_count", -1)) >=
                int(completed_count)),
            time.monotonic() + self._trial_handshake_timeout,
            "trial_end {}".format(completed_count))

    def _wait_for_capture_final(self, timeout):
        self._wait_for_capture_status(
            lambda status: (
                status.get("state") == "FINALIZED" and
                status.get("run_complete") is True and
                int(status.get("completed_trial_count", -1)) ==
                len(self._selected_trial_ids)),
            time.monotonic() + float(timeout), "run_complete")

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

    def _arm_capture_sampling(self, trial, trajectory):
        if not self._require_capture_ready:
            return None
        sampling_start = rospy.Time.now() + rospy.Duration(
            self._capture_sampling_start_lead)
        self._capture_event_seq += 1
        event = {
            "event": "sampling_start",
            "event_seq": self._capture_event_seq,
            "stamp": rospy.Time.now().to_sec(),
            "monotonic_sec": time.monotonic(),
            "trial_id": trial["trial_id"],
            "sampling_start_stamp_sec": sampling_start.to_sec(),
            "sampling_expected_duration_sec":
                trajectory["expected_duration_sec"],
            "sampling_target_center_offset_sec": trajectory.get(
                "target_center_offset_sec"),
        }
        self._capture_control_publisher.publish(String(data=json.dumps(
            event, sort_keys=True)))
        self._wait_for_capture_sampling_start(
            trial["trial_id"], event["event_seq"])
        if rospy.Time.now() >= sampling_start:
            raise RuntimeError(
                "failure capture sampling_start ACK missed source-time lead")
        return sampling_start

    def _call_service_with_deadline(self, name, proxy, *args):
        def invoke():
            try:
                return proxy(*args)
            except rospy.ServiceException as first_error:
                # The runner only uses this helper for idempotent camera-pose
                # and memory-reset calls. A single ROS transport reconnect
                # must not discard an otherwise valid long operating surface.
                if rospy.is_shutdown():
                    raise
                rospy.logwarn(
                    "V-SIM-04 %s service transport retry: %s",
                    name, first_error)
                time.sleep(0.05)
                return proxy(*args)

        try:
            return call_with_monotonic_deadline(
                invoke, self._service_call_timeout,
                "{}_service".format(name), rospy.is_shutdown)
        except Exception as error:
            raise RuntimeError(
                "{} service call failed: {}".format(name, error)) from error

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

    def _run_static(self, trial, sampling_start=None):
        x, y, z = self._anchor(trial)
        if sampling_start is not None:
            self._sleep_until_ros(sampling_start)
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

    def _dynamic_trajectory_plan(self, trial):
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
        target_center_offset = (x - start_x) / speed
        if (distance <= 0.0 or expected_duration <= 0.0 or
                target_center_offset <= 0.0 or
                target_center_offset >= expected_duration):
            raise RuntimeError("dynamic trajectory geometry is invalid")
        return {
            "mode": "absolute_ros_time_linear",
            "expected_duration_sec": expected_duration,
            "distance_m": distance,
            "expected_speed_mps": speed,
            "update_rate_hz": update_rate,
            "steps": steps,
            "start_x": start_x,
            "start_y": y,
            "finish_x": finish_x,
            "finish_y": y,
            "target_center_offset_sec": target_center_offset,
            "camera_z": z + trial["height_m"],
        }

    def _run_dynamic(self, trial, trajectory, sampling_start=None):
        update_rate = trajectory["update_rate_hz"]
        if sampling_start is None:
            # SetModelState is intentionally discontinuous. Keep one command
            # period outside the motion window so stamped LinkStates can
            # settle at the planned start before telemetry begins.
            self._sleep_ros_duration(1.0 / update_rate)
            start_time = rospy.Time.now()
        else:
            start_time = sampling_start
            self._sleep_until_ros(sampling_start)
        expected_duration = trajectory["expected_duration_sec"]
        speed = trajectory["expected_speed_mps"]
        distance = trajectory["distance_m"]
        steps = trajectory["steps"]
        start_x = trajectory["start_x"]
        finish_x = trajectory["finish_x"]
        y = trajectory["start_y"]
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
                trajectory["camera_z"])
        if time.monotonic() >= wall_deadline:
            raise RuntimeError(
                "dynamic trajectory exceeded total wall-clock budget")
        end_time = rospy.Time.now()
        actual_duration = max(
            0.0, (end_time - start_time).to_sec())
        result = dict(trajectory)
        result.pop("camera_z")
        result.update({
            "motion_start_source_stamp": start_time.to_sec(),
            "motion_end_source_stamp": end_time.to_sec(),
            "actual_duration_sec": actual_duration,
            "actual_speed_mps": (
                distance / actual_duration if actual_duration > 0.0 else None),
        })
        return result

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
        self._wait_for_capture_ready(deadline)
        self._sleep_ros_duration(float(rospy.get_param(
            "~startup_settle_sec", 0.5)))

        for trial_index, trial in enumerate(self._matrix["trials"], 1):
            self._set_camera(*self._offscreen(trial))
            self._sleep_ros_duration(self._pretrial_settle)
            self._call_service_with_deadline(
                "reset_memory", self._reset)
            start_event = self._publish_event("trial_start", trial)
            self._wait_for_trial_status(
                trial["trial_id"], None, self._trial_handshake_timeout)
            self._wait_for_capture_trial_start(
                trial["trial_id"], start_event["event_seq"])
            if trial["kind"] == "static":
                sampling_contract = {
                    "expected_duration_sec": float(
                        self._matrix["static"].get(
                            "center_dwell_sec", 2.0)),
                }
                sampling_start = self._arm_capture_sampling(
                    trial, sampling_contract)
                trajectory = self._run_static(trial, sampling_start)
            else:
                sampling_contract = self._dynamic_trajectory_plan(trial)
                self._set_camera(
                    sampling_contract["start_x"],
                    sampling_contract["start_y"],
                    sampling_contract["camera_z"])
                sampling_start = self._arm_capture_sampling(
                    trial, sampling_contract)
                trajectory = self._run_dynamic(
                    trial, sampling_contract, sampling_start)
            self._set_camera(*self._offscreen(trial))
            self._sleep_ros_duration(self._posttrial_settle)
            self._publish_event(
                "trial_end", trial, trajectory=trajectory)
            self._wait_for_trial_status(
                trial["trial_id"], trial_index,
                self._trial_handshake_timeout)
            self._wait_for_capture_trial_end(trial_index)

        self._publish_event(
            "run_complete", trial_count=len(self._matrix["trials"]))
        self._wait_for_recorder_final(float(rospy.get_param(
            "~finalization_timeout_sec", 30.0)))
        self._wait_for_capture_final(float(rospy.get_param(
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
                        abort_event_seq = status.get("abort_event_seq")
                        if (status.get("state") == "FAIL" and
                                abort_event_seq is not None):
                            try:
                                if (int(abort_event_seq) ==
                                        int(payload["event_seq"])):
                                    return
                            except (TypeError, ValueError, OverflowError):
                                # A malformed or not-yet-populated ACK must not
                                # hide the original preflight/trial failure.
                                pass
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
