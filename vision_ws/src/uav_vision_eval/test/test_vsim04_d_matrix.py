#!/usr/bin/env python3

import json
import math
import os
import tempfile
import unittest

import yaml

from uav_vision_eval.vsim04_d_matrix import (
    audit_multitarget_associations,
    camera_basis_for_relative_angle,
    generate_d50_pose_samples,
    load_d50_matrix,
    quaternion_from_camera_basis,
    relative_image_angle_deg,
    write_d50_dry_run,
)
from uav_vision_eval.vsim04_metrics import (
    load_trial_matrix,
    select_trial_matrix,
)


MATRIX_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "config",
    "vsim04_trajectory_d50_matrix.yaml"))


def _circular_error_degrees(actual, expected):
    return abs((float(actual) - float(expected) + 180.0) % 360.0 - 180.0)


class VSim04DMatrixTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = load_d50_matrix(MATRIX_PATH)

    def test_d50_counts_and_pairwise_coverage(self):
        trials = self.matrix["trials"]
        self.assertEqual(50, len(trials))
        self.assertEqual(
            40, sum(trial["kind"] == "single_pairwise"
                    for trial in trials))
        self.assertEqual(
            10, sum(trial["kind"] == "multi_directed"
                    for trial in trials))
        self.assertTrue(self.matrix["pairwise_coverage"]["complete"])
        self.assertEqual(
            {"tent": 8, "pillbox": 8, "bridge": 8,
             "panzer": 8, "red_cross": 8},
            {class_name: sum(
                trial["kind"] == "single_pairwise" and
                trial["class_name"] == class_name for trial in trials)
             for class_name in
             ("tent", "pillbox", "bridge", "panzer", "red_cross")})

    def test_runtime_adapter_reuses_dynamic_recorder_diagnostically(self):
        runtime = load_trial_matrix(MATRIX_PATH)
        self.assertTrue(runtime["diagnostic_only"])
        self.assertEqual(50, runtime["formal_expected_trial_count"])
        self.assertTrue(all(trial["kind"] == "dynamic"
                            for trial in runtime["trials"]))
        self.assertEqual(
            40, sum(trial["design_kind"] == "single_pairwise"
                    for trial in runtime["trials"]))
        clipped = next(
            trial for trial in runtime["trials"]
            if trial["framing"] == "partial")
        self.assertEqual("full", clipped["visibility_profile"])
        self.assertEqual("NOT_RUN", clipped["d50_runtime_status"])
        self.assertEqual(
            "clipped_target_observation_window_not_wired",
            clipped["d50_not_run_reason"])
        full_selection = select_trial_matrix(runtime, "", "")
        self.assertEqual("diagnostic", full_selection["evaluation_scope"])
        single = select_trial_matrix(runtime, "d_single_01", "")
        self.assertEqual("diagnostic", single["evaluation_scope"])
        self.assertEqual(["d_single_01"], single["trial_selector"])
        single40 = select_trial_matrix(runtime, "", "single40")
        self.assertEqual(40, len(single40["trials"]))
        self.assertTrue(all(
            trial["design_kind"] == "single_pairwise"
            for trial in single40["trials"]))
        arena_rejected = next(
            trial for trial in runtime["trials"]
            if trial["trial_id"] == "d_single_39")
        self.assertEqual("NOT_RUN", arena_rejected["d50_runtime_status"])
        self.assertEqual(
            "camera_trajectory_exceeds_arena_limit",
            arena_rejected["d50_not_run_reason"])
        supported = select_trial_matrix(
            runtime, "", "single_smoke_supported")
        self.assertNotIn(
            "d_single_39",
            [trial["trial_id"] for trial in supported["trials"]])
        for rejected_id in ("d_single_12", "d_single_16", "d_single_32"):
            rejected = next(
                trial for trial in runtime["trials"]
                if trial["trial_id"] == rejected_id)
            self.assertEqual("NOT_RUN", rejected["d50_runtime_status"])
            self.assertEqual(
                "fully_in_frame_enter_leave_preflight_failed",
                rejected["d50_not_run_reason"])
            self.assertFalse(
                rejected["d50_visibility_preflight"]["preflight_pass"])
        self.assertTrue(supported["trials"])
        for trial in supported["trials"]:
            self.assertTrue(
                trial["d50_visibility_preflight"]["preflight_pass"])
            trajectory = generate_d50_pose_samples(runtime, trial)
            self.assertTrue(all(
                abs(sample["position_x_m"]) <= 4.8 and
                abs(sample["position_y_m"]) <= 4.8
                for sample in trajectory["samples"]))

    def test_relative_angle_uses_optical_image_basis(self):
        intrinsics = self.matrix["camera"]["intrinsics"]
        expected_focal = intrinsics["width"] / (
            2.0 * math.tan(intrinsics["horizontal_fov_rad"] / 2.0))
        self.assertAlmostEqual(expected_focal, intrinsics["fx"], places=9)
        self.assertAlmostEqual(expected_focal, intrinsics["fy"], places=9)
        for expected in (0.0, 45.0, 90.0, 135.0):
            basis = camera_basis_for_relative_angle(expected)
            quaternion = quaternion_from_camera_basis(basis)
            actual = relative_image_angle_deg((1.0, 0.0, 0.0), quaternion)
            self.assertLess(_circular_error_degrees(actual, expected), 1.0e-8)
            self.assertAlmostEqual(
                1.0, math.sqrt(sum(value * value for value in quaternion)),
                places=10)
            self.assertAlmostEqual(-1.0,
                                   basis["optical_axis_world"][2], places=10)

    def test_matrix_rejects_focal_length_that_disagrees_with_hfov(self):
        with open(MATRIX_PATH, "r", encoding="utf-8") as stream:
            invalid = yaml.safe_load(stream)
        invalid["camera"]["intrinsics"]["fx"] += 1.0
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "invalid_d50.yaml")
            with open(path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(invalid, stream, sort_keys=False)
            with self.assertRaisesRegex(ValueError, "horizontal_fov"):
                load_d50_matrix(path)

    def test_constant_accel_and_turn_pose_samples(self):
        by_profile = {}
        for trial in self.matrix["trials"]:
            if trial["kind"] == "single_pairwise" and \
                    trial["framing"] == "center" and \
                    trial["motion_profile"] not in by_profile:
                by_profile[trial["motion_profile"]] = trial
        self.assertEqual({"constant", "accel_decel", "turn"},
                         set(by_profile))
        generated = {
            name: generate_d50_pose_samples(self.matrix, trial)
            for name, trial in by_profile.items()}
        for trajectory in generated.values():
            self.assertAlmostEqual(1.0, trajectory["mean_speed_mps"], places=3)
            self.assertEqual(81, len(trajectory["samples"]))
            for sample in trajectory["samples"]:
                norm = math.sqrt(sum(sample[field] ** 2 for field in (
                    "orientation_x", "orientation_y",
                    "orientation_z", "orientation_w")))
                self.assertAlmostEqual(1.0, norm, places=9)
        constant_speeds = [
            sample["linear_speed_mps"]
            for sample in generated["constant"]["samples"][1:]]
        self.assertLess(max(abs(value - 1.0) for value in constant_speeds),
                        1.0e-9)
        accel_speeds = [
            sample["linear_speed_mps"]
            for sample in generated["accel_decel"]["samples"][1:]]
        self.assertLess(accel_speeds[0], 0.1)
        self.assertGreater(max(accel_speeds), 1.45)
        self.assertLess(accel_speeds[-1], 0.1)
        turn_angles = [
            sample["relative_image_angle_deg"]
            for sample in generated["turn"]["samples"]]
        turn_span = _circular_error_degrees(turn_angles[-1], turn_angles[0])
        self.assertAlmostEqual(90.0, turn_span, places=6)
        midpoint = generated["turn"]["samples"][40]
        self.assertLess(_circular_error_degrees(
            midpoint["relative_image_angle_deg"],
            by_profile["turn"]["relative_angle_deg"]), 1.0e-8)

    def test_default_single_smoke_camera_samples_stay_inside_arena(self):
        trial = next(
            trial for trial in self.matrix["trials"]
            if trial["trial_id"] == "d_single_01")
        trajectory = generate_d50_pose_samples(self.matrix, trial)
        maximum = max(
            max(abs(sample["position_x_m"]),
                abs(sample["position_y_m"]))
            for sample in trajectory["samples"])
        self.assertLessEqual(maximum, 4.8)

    def test_framing_offset_is_defined_in_image_basis(self):
        trial = next(
            trial for trial in self.matrix["trials"]
            if trial["kind"] == "single_pairwise" and
            trial["framing"] == "partial" and
            trial["motion_profile"] == "constant")
        trajectory = generate_d50_pose_samples(self.matrix, trial)
        midpoint = trajectory["samples"][40]
        target = self.matrix["target_anchors"][trial["class_name"]]["xyz"]
        camera = (midpoint["position_x_m"], midpoint["position_y_m"],
                  midpoint["position_z_m"])
        target_ray = tuple(target[index] - camera[index] for index in range(3))
        right = (midpoint["image_right_x"], midpoint["image_right_y"],
                 midpoint["image_right_z"])
        height = trial["height_m"]
        intrinsics = self.matrix["camera"]["intrinsics"]
        measured_half_frame_u = (
            sum(a * b for a, b in zip(target_ray, right)) /
            height * intrinsics["fx"] / (intrinsics["width"] * 0.5))
        expected = self.matrix["camera"][
            "framing_offsets_half_frame"]["partial"][0]
        self.assertAlmostEqual(expected, measured_half_frame_u, places=9)
        self.assertGreater(measured_half_frame_u, 1.0)

    def test_multitarget_audit_clean_and_failure_modes(self):
        clean_trial = next(
            trial for trial in self.matrix["trials"]
            if trial["trial_id"] == "d_multi_43_red_cross_priority")
        clean = []
        for frame in range(1, 7):
            clean.extend((
                {"frame_seq": frame, "truth_target_id": "red_cross_1",
                 "associated_truth_target_id": "red_cross_1",
                 "stable_id": "stable-red", "confirmed": True,
                 "visible": True, "selected": True},
                {"frame_seq": frame, "truth_target_id": "panzer_2",
                 "associated_truth_target_id": "panzer_2",
                 "stable_id": "stable-panzer", "confirmed": True,
                 "visible": True, "selected": False},
            ))
        clean_result = audit_multitarget_associations(clean_trial, clean)
        self.assertTrue(clean_result["passed"])
        self.assertEqual(0, clean_result["max_priority_starvation_streak_frames"])

        failure_trial = next(
            trial for trial in self.matrix["trials"]
            if trial["trial_id"] == "d_multi_49_overlap_with_h")
        broken = [
            {"frame_seq": 1, "truth_target_id": "red_cross_1",
             "associated_truth_target_id": "pillbox_2",
             "stable_id": "stable-shared", "visible": True,
             "confirmed": True, "selected": False},
            {"frame_seq": 1, "truth_target_id": "pillbox_2",
             "associated_truth_target_id": "pillbox_2",
             "stable_id": "stable-shared", "visible": True,
             "confirmed": True, "selected": True},
            {"frame_seq": 2, "truth_target_id": "red_cross_1",
             "associated_truth_target_id": "red_cross_1",
             "stable_id": "stable-red-second", "visible": True,
             "confirmed": True, "selected": False},
            {"frame_seq": 2, "truth_target_id": "landing_h_1",
             "associated_truth_target_id": "landing_h_1",
             "stable_id": "", "visible": True,
             "confirmed": True, "selected": True},
        ]
        failure = audit_multitarget_associations(failure_trial, broken)
        self.assertFalse(failure["passed"])
        self.assertGreater(failure["duplicate_stable_id_excess"], 0)
        self.assertGreater(failure["merged_truth_target_excess"], 0)
        self.assertGreater(failure["wrong_association_count"], 0)
        self.assertGreater(failure["empty_stable_id_count"], 0)
        self.assertGreater(failure["landing_h_selected_count"], 0)
        self.assertFalse(failure["search_h_gate_pass"])
        self.assertTrue(any(
            value.startswith("insufficient_frame_coverage")
            for value in failure["violations"]))
        with self.assertRaisesRegex(ValueError, "missing fields"):
            audit_multitarget_associations(
                failure_trial, [{"frame_seq": 1}])

    def test_dry_run_artifacts_are_explicitly_not_gazebo_results(self):
        with tempfile.TemporaryDirectory() as output_dir:
            summary = write_d50_dry_run(MATRIX_PATH, output_dir)
            self.assertEqual("DRY_RUN", summary["status"])
            self.assertEqual("NOT_RUN", summary["gazebo_execution_status"])
            self.assertEqual(50, summary["trial_count"])
            self.assertEqual(4, summary["multi_target_with_landing_h_count"])
            self.assertIn(
                "d_single_01", summary["runtime_readiness"][
                    "supported_single_smoke_trial_ids"])
            self.assertIn(
                "d_single_39", summary["runtime_readiness"][
                    "not_run_by_reason"][
                        "camera_trajectory_exceeds_arena_limit"])
            for name in (
                    "d50_manifest.json", "d50_trials.csv",
                    "d50_trajectory_samples.csv", "d50_coverage.json",
                    "d50_association_contracts.json", "summary.json"):
                self.assertTrue(os.path.isfile(os.path.join(output_dir, name)))
            with open(os.path.join(output_dir, "summary.json"), "r",
                      encoding="utf-8") as stream:
                persisted = json.load(stream)
            self.assertEqual("NOT_RUN", persisted["gazebo_execution_status"])
            with open(os.path.join(
                    output_dir, "d50_association_contracts.json"), "r",
                    encoding="utf-8") as stream:
                association_contract = json.load(stream)
            self.assertEqual("search", association_contract["mission_phase"])
            self.assertEqual(10, len(association_contract["trials"]))


if __name__ == "__main__":
    unittest.main()
