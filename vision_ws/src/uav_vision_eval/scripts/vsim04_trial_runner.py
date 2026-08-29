#!/usr/bin/env python3
"""Drive all V-SIM-04 camera trials in one Gazebo session."""

import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String
from std_srvs.srv import Empty

from uav_vision_eval.vsim04_metrics import load_trial_matrix


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
        self._matrix = load_trial_matrix(self._matrix_path)
        runner = self._matrix.get("runner", {})
        self._camera_model = rospy.get_param(
            "~camera_model", runner.get("camera_model", "vision_eval_camera"))
        self._reset_service_name = rospy.get_param(
            "~reset_service", runner.get(
                "reset_service", "/uav_vision/reset_memory"))
        self._event_topic = rospy.get_param(
            "~trial_event_topic", runner.get(
                "trial_event_topic", "/uav_vision_eval/vsim04/trial_event"))
        self._offscreen_offset = float(runner.get(
            "offscreen_offset_m", 3.5))
        self._pretrial_settle = float(runner.get(
            "pretrial_settle_sec", 0.5))
        self._posttrial_settle = float(runner.get(
            "posttrial_settle_sec", 0.8))
        self._arena_limit = float(rospy.get_param("~arena_limit_m", 4.8))
        self._rpy = [float(value) for value in runner.get(
            "camera_rpy", [0.0, math.pi / 2.0, 0.0])]
        self._event_seq = 0
        self._publisher = rospy.Publisher(
            self._event_topic, String, queue_size=20, latch=True)
        self._set_state_service_name = rospy.get_param(
            "~set_model_state_service", "/gazebo/set_model_state")
        self._set_state = rospy.ServiceProxy(
            self._set_state_service_name, SetModelState)
        self._reset = rospy.ServiceProxy(self._reset_service_name, Empty)

    def _publish_event(self, event, trial=None, **details):
        self._event_seq += 1
        payload = {
            "event": event,
            "event_seq": self._event_seq,
            "stamp": rospy.Time.now().to_sec(),
        }
        if trial is not None:
            payload["trial_id"] = trial["trial_id"]
            payload["trial"] = trial
        payload.update(details)
        self._publisher.publish(String(data=json.dumps(
            payload, sort_keys=True)))

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
        response = self._set_state(state)
        if not response.success:
            raise RuntimeError("set_model_state failed: " + response.status_message)

    def _anchor(self, trial):
        values = self._matrix["target_anchors"][trial["class_name"]]["xyz"]
        return tuple(float(value) for value in values)

    def _offscreen(self, trial):
        x, y, z = self._anchor(trial)
        camera_x = min(self._arena_limit, x + self._offscreen_offset)
        return camera_x, y, z + trial["height_m"]

    def _run_static(self, trial):
        x, y, z = self._anchor(trial)
        self._set_camera(x, y, z + trial["height_m"])
        rospy.sleep(float(self._matrix["static"].get(
            "center_dwell_sec", 2.0)))

    def _run_dynamic(self, trial):
        x, y, z = self._anchor(trial)
        config = self._matrix["dynamic"]
        half_length = float(config.get("path_half_length_m", 3.5))
        update_rate = float(config.get("update_rate_hz", 20.0))
        speed = float(trial["speed_mps"])
        start = max(-self._arena_limit, x - half_length)
        finish = min(self._arena_limit, x + half_length)
        distance = max(0.0, finish - start)
        steps = max(1, int(math.ceil(distance / speed * update_rate)))
        for index in range(steps + 1):
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown during dynamic trial")
            fraction = index / float(steps)
            self._set_camera(
                start + fraction * distance, y, z + trial["height_m"])
            rospy.sleep(1.0 / update_rate)

    def run(self):
        timeout = float(rospy.get_param("~startup_timeout_sec", 30.0))
        rospy.wait_for_service(self._set_state_service_name, timeout=timeout)
        rospy.wait_for_service(self._reset_service_name, timeout=timeout)
        deadline = time.monotonic() + timeout
        while self._publisher.get_num_connections() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("trial recorder did not subscribe")
            time.sleep(0.05)

        first_trial = self._matrix["trials"][0]
        last_error = None
        while time.monotonic() < deadline:
            try:
                self._set_camera(*self._offscreen(first_trial))
                last_error = None
                break
            except Exception as error:
                last_error = error
                time.sleep(0.1)
        if last_error is not None:
            raise RuntimeError(
                "evaluation camera did not become available: {}".format(
                    last_error))
        rospy.sleep(float(rospy.get_param("~startup_settle_sec", 2.0)))

        for trial in self._matrix["trials"]:
            self._set_camera(*self._offscreen(trial))
            rospy.sleep(self._pretrial_settle)
            self._reset()
            self._publish_event("trial_start", trial)
            if trial["kind"] == "static":
                self._run_static(trial)
            else:
                self._run_dynamic(trial)
            self._set_camera(*self._offscreen(trial))
            rospy.sleep(self._posttrial_settle)
            self._publish_event("trial_end", trial)
            rospy.sleep(0.1)
        self._publish_event("run_complete", trial_count=len(
            self._matrix["trials"]))
        rospy.sleep(1.0)
        rospy.loginfo("V-SIM-04 trial runner complete (%d trials)",
                      len(self._matrix["trials"]))


def main():
    try:
        VSim04TrialRunner().run()
    except Exception as error:
        rospy.logerr("V-SIM-04 trial runner failed: %s", error)
        return 8
    return 0


if __name__ == "__main__":
    sys.exit(main())
