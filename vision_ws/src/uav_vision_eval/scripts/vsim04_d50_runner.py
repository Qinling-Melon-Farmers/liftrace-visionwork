#!/usr/bin/env python3
"""Run D50 single-target trials with source-time pose/quaternion samples."""

import importlib.util
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState

from uav_vision_eval.vsim04_d_matrix import generate_d50_pose_samples


def _load_base_runner_class():
    # Catkin's devel-space relay executes source with its real __file__. Loading
    # the sibling source explicitly works in source, devel and install spaces;
    # importing the relay as a normal module would hide its private exec scope.
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                        "vsim04_trial_runner.py")
    spec = importlib.util.spec_from_file_location(
        "uav_vision_eval_vsim04_trial_runner", path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load established V-SIM-04 runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VSim04TrialRunner


VSim04TrialRunner = _load_base_runner_class()


class VSim04D50Runner(VSim04TrialRunner):
    """Thin trajectory adapter over the established fail-closed runner."""

    def __init__(self):
        super().__init__()
        if self._matrix.get("matrix_kind") != \
                "vsim04_d50_trajectory_association":
            raise ValueError("D50 runner requires the D50 matrix")
        not_ready = [
            "{}={}".format(
                trial["trial_id"], trial.get("d50_not_run_reason", "unknown"))
            for trial in self._matrix["trials"]
            if trial.get("d50_runtime_status") != "READY_FOR_SINGLE_SMOKE"]
        if not_ready:
            raise ValueError(
                "D50 selection contains NOT_RUN trials: {}".format(
                    ",".join(not_ready)))

    def _set_camera_pose_sample(self, sample):
        state = ModelState()
        state.model_name = self._camera_model
        state.reference_frame = "world"
        state.pose.position.x = float(sample["position_x_m"])
        state.pose.position.y = float(sample["position_y_m"])
        state.pose.position.z = float(sample["position_z_m"])
        state.pose.orientation.x = float(sample["orientation_x"])
        state.pose.orientation.y = float(sample["orientation_y"])
        state.pose.orientation.z = float(sample["orientation_z"])
        state.pose.orientation.w = float(sample["orientation_w"])
        response = self._call_service_with_deadline(
            "set_model_state", self._set_state, state)
        if not response.success:
            raise RuntimeError(
                "set_model_state D50 pose failed: " +
                response.status_message)

    def _dynamic_trajectory_plan(self, trial):
        generated = generate_d50_pose_samples(self._matrix, trial)
        samples = generated["samples"]
        first, last = samples[0], samples[-1]
        duration = float(generated["expected_duration_sec"])
        if len(samples) < 3 or duration <= 0.0:
            raise RuntimeError("D50 generated trajectory is invalid")
        outside = [
            sample for sample in samples
            if (abs(float(sample["position_x_m"])) > self._arena_limit or
                abs(float(sample["position_y_m"])) > self._arena_limit)]
        if outside:
            raise RuntimeError(
                "D50 camera trajectory exceeds arena limit and remains "
                "NOT_RUN: {}".format(trial["trial_id"]))
        return {
            "mode": "d50_source_time_pose_sequence",
            "expected_duration_sec": duration,
            "distance_m": float(generated["path_distance_m"]),
            "expected_speed_mps": float(trial["speed_mps"]),
            "update_rate_hz": float(generated["sample_rate_hz"]),
            "steps": len(samples) - 1,
            "start_x": float(first["position_x_m"]),
            "start_y": float(first["position_y_m"]),
            "finish_x": float(last["position_x_m"]),
            "finish_y": float(last["position_y_m"]),
            "camera_z": float(first["position_z_m"]),
            "target_center_offset_sec": duration / 2.0,
            "pose_samples": samples,
            "planned_sample_count": len(samples),
            "design_kind": trial["design_kind"],
            "relative_angle_deg": float(trial["relative_angle_deg"]),
            "relative_angle_measurement": "optical_image_basis",
            "motion_profile": trial["motion_profile"],
            "framing": trial["framing"],
            "visibility_profile": trial["visibility_profile"],
            "expected_primary_target_id":
                trial["expected_primary_target_id"],
        }

    def _set_trajectory_start(self, trial, trajectory):
        self._set_camera_pose_sample(trajectory["pose_samples"][0])

    def _run_dynamic(self, trial, trajectory, sampling_start=None):
        samples = trajectory["pose_samples"]
        update_rate = float(trajectory["update_rate_hz"])
        expected_duration = float(trajectory["expected_duration_sec"])

        # _set_trajectory_start established the complete quaternion before the
        # recorder/capture handshake, so no trial frame sees parked ZYX RPY.
        if sampling_start is None:
            self._sleep_ros_duration(1.0 / update_rate)
            start_time = rospy.Time.now()
        else:
            start_time = sampling_start
            self._sleep_until_ros(sampling_start)

        wall_deadline = (
            time.monotonic() +
            expected_duration * self._ros_wait_wall_factor +
            self._ros_wait_wall_padding)
        for sample in samples[1:]:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown during D50 trial")
            target_time = start_time + rospy.Duration(
                float(sample["source_time_offset_sec"]))
            self._sleep_until_ros(target_time, wall_deadline)
            self._set_camera_pose_sample(sample)
        if time.monotonic() >= wall_deadline:
            raise RuntimeError("D50 trajectory exceeded wall-clock budget")

        end_time = rospy.Time.now()
        actual_duration = max(0.0, (end_time - start_time).to_sec())
        result = {
            key: value for key, value in trajectory.items()
            if key not in {"pose_samples", "camera_z"}
        }
        result.update({
            "motion_start_source_stamp": start_time.to_sec(),
            "motion_end_source_stamp": end_time.to_sec(),
            "actual_duration_sec": actual_duration,
            "actual_speed_mps": (
                float(trajectory["distance_m"]) / actual_duration
                if actual_duration > 0.0 else None),
        })
        return result


def main():
    runner = None
    try:
        runner = VSim04D50Runner()
        runner.run()
    except Exception as error:
        rospy.logerr("V-SIM-04 D50 runner failed: %s", error)
        if runner is not None:
            runner.abort(error)
        return 8
    return 0


if __name__ == "__main__":
    sys.exit(main())
