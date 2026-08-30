#!/usr/bin/env python3
"""Pure deterministic assertion for the V-SIM-04 matrix and artifact schema."""

import csv
import json
import math
import os
import shutil
import sys
import tempfile
import time
import traceback

from uav_vision_eval.vsim04_metrics import (
    REQUIRED_ARTIFACTS,
    FRAME_FIELDS,
    PERFORMANCE_FIELDS,
    annotate_motion_frames,
    candidate_audit_observation,
    candidate_audit_summary,
    classify_failure_stage,
    completed_sources_cover,
    call_with_monotonic_deadline,
    correlate_admission_events,
    decorate_performance_rows,
    detector_diagnostic_errors,
    evaluate_performance_verdict,
    handshake_timeout_is_safe,
    dry_run_artifacts,
    load_trial_matrix,
    planned_trial_result,
    quaternion_yaw,
    select_trial_matrix,
    summarize_trial_results,
    watermarks_cover_source_stamp,
    write_artifacts,
)
from uav_vision_eval.stamped_pose_buffer import StampedPoseBuffer

from geometry_msgs.msg import PoseStamped
import rospy


def main():
    assert {
        "camera_pose_valid", "camera_pose_source_stamp",
        "camera_pose_age_sec", "camera_position_x_m",
        "camera_position_y_m", "camera_position_z_m", "camera_yaw_rad",
        "camera_pose_invalid_reason", "motion_delta_valid",
        "actual_linear_speed_mps", "actual_yaw_rate_radps",
        "motion_invalid_reason", "path_lateral_offset_m",
        "path_lateral_offset_normalized", "path_lateral_invalid_reason",
    }.issubset(set(FRAME_FIELDS))
    assert PERFORMANCE_FIELDS.index("actual_speed_mps") < \
        PERFORMANCE_FIELDS.index("class_group_completed_trials")
    assert quaternion_yaw(0.0, 0.0, 0.0, 1.0) == 0.0
    assert quaternion_yaw(0.0, 0.0, 0.0, 0.0) is None
    half_yaw = 0.5 * 1.2
    assert abs(quaternion_yaw(
        0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)) - 1.2) < 1.0e-9

    history = StampedPoseBuffer(max_length=3)
    for stamp_sec, x_value in ((1.0, 1.0), (2.0, 2.0), (4.0, 4.0)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(stamp_sec)
        pose.header.frame_id = "world"
        pose.pose.position.x = x_value
        assert history.add(pose)
    selected, age_sec = history.at_or_before(rospy.Time.from_sec(3.0))
    assert selected is not None and selected.pose.position.x == 2.0
    assert abs(age_sec - 1.0) < 1.0e-9
    missing, missing_age = history.at_or_before(rospy.Time.from_sec(0.5))
    assert missing is None and missing_age is None
    reset_pose = PoseStamped()
    reset_pose.header.stamp = rospy.Time.from_sec(0.25)
    reset_pose.header.frame_id = "world"
    reset_pose.pose.position.x = 0.25
    assert history.add(reset_pose)
    selected, _age_sec = history.at_or_before(rospy.Time.from_sec(3.0))
    assert selected is not None and selected.pose.position.x == 0.25

    def motion_row(stamp, x_value=None, y_value=None, z_value=None,
                   yaw_value=None, pose_stamp=None):
        valid = all(value is not None for value in (
            x_value, y_value, z_value, yaw_value))
        if valid and pose_stamp is None:
            pose_stamp = stamp
        return {
            "stamp": stamp,
            "camera_pose_valid": valid,
            "camera_pose_source_stamp": pose_stamp if valid else "",
            "camera_position_x_m": "" if x_value is None else x_value,
            "camera_position_y_m": "" if y_value is None else y_value,
            "camera_position_z_m": "" if z_value is None else z_value,
            "camera_yaw_rad": "" if yaw_value is None else yaw_value,
        }

    reset_prestart = motion_row(1.1, 10.0, 0.1, 2.0, 3.0, 1.0)
    start_row = motion_row(2.1, 0.0, 0.1, 2.0, 3.0, 2.0)
    normal_row = motion_row(3.1, 1.0, 0.1, 2.0, -3.0, 3.0)
    missing_row = motion_row(3.5)
    zero_row = motion_row(4.1, 1.0, 0.1, 2.0, -3.0, 4.0)
    duplicate_row = motion_row(4.2, 1.0, 0.1, 2.0, -3.0, 4.0)
    finish_row = motion_row(5.1, 2.0, 0.1, 2.0, -2.8, 5.0)
    postfinish_row = motion_row(6.1, 10.0, 0.1, 2.0, -2.8, 6.0)
    # Deliberately model cross-topic callback order, not source-time order.
    motion_rows = [
        finish_row, reset_prestart, zero_row, missing_row, start_row,
        postfinish_row, normal_row, duplicate_row,
    ]
    motion = annotate_motion_frames(motion_rows, "dynamic", {
        "start_x": 0.0, "start_y": 0.0,
        "finish_x": 10.0, "finish_y": 0.0,
        "expected_speed_mps": 1.0, "update_rate_hz": 10.0,
        "steps": 10, "motion_start_source_stamp": 2.0,
        "motion_end_source_stamp": 5.0,
    })
    assert reset_prestart["actual_linear_speed_mps"] == ""
    assert reset_prestart["motion_invalid_reason"] == \
        "trajectory_reset_prestart"
    assert reset_prestart["path_lateral_offset_normalized"] == ""
    assert start_row["motion_invalid_reason"] == "first_valid_pose"
    assert abs(start_row["path_lateral_offset_normalized"] - 0.01) < 1.0e-9
    assert abs(normal_row["actual_linear_speed_mps"] - 1.0) < 1.0e-9
    expected_yaw_rate = 2.0 * math.pi - 6.0
    assert abs(normal_row["actual_yaw_rate_radps"] -
               expected_yaw_rate) < 1.0e-9
    assert not missing_row["motion_delta_valid"]
    assert missing_row["motion_invalid_reason"] == \
        "camera_pose_missing_or_invalid"
    assert zero_row["motion_delta_valid"]
    assert zero_row["actual_linear_speed_mps"] == 0.0
    assert zero_row["actual_yaw_rate_radps"] == 0.0
    assert duplicate_row["motion_invalid_reason"] == "duplicate_pose_stamp"
    assert duplicate_row["actual_linear_speed_mps"] == ""
    assert abs(finish_row["actual_linear_speed_mps"] - 1.0) < 1.0e-9
    assert postfinish_row["motion_invalid_reason"] == "trajectory_complete"
    assert postfinish_row["actual_linear_speed_mps"] == ""
    assert motion["camera_pose_frame_count"] == 7
    assert motion["motion_sample_count"] == 3
    assert motion["lateral_offset_sample_count"] == 5
    assert abs(motion["mean_actual_linear_speed_mps"] - (2.0 / 3.0)) < 1.0e-9

    reset_start = motion_row(2.1, 0.0, 0.0, 2.0, 0.0, 2.0)
    reset_progress = motion_row(3.1, 5.0, 0.0, 2.0, 0.0, 3.0)
    reset_jump = motion_row(4.1, 0.0, 0.0, 2.0, 0.0, 4.0)
    reset_after = motion_row(5.1, 1.0, 0.0, 2.0, 0.0, 5.0)
    reset_motion = annotate_motion_frames([
        reset_after, reset_jump, reset_start, reset_progress,
    ], "dynamic", {
        "start_x": 0.0, "start_y": 0.0,
        "finish_x": 10.0, "finish_y": 0.0,
        "expected_speed_mps": 1.0, "update_rate_hz": 10.0,
        "steps": 10, "motion_start_source_stamp": 2.0,
        "motion_end_source_stamp": 5.0,
    })
    assert reset_jump["motion_invalid_reason"] == "trajectory_reset_jump"
    assert reset_jump["actual_linear_speed_mps"] == ""
    assert reset_after["motion_delta_valid"]
    assert abs(reset_after["actual_linear_speed_mps"] - 1.0) < 1.0e-9
    assert reset_motion["motion_sample_count"] == 2

    invalid_window_row = motion_row(1.0, 0.0, 0.0, 2.0, 0.0)
    invalid_window = annotate_motion_frames(
        [invalid_window_row], "dynamic", {
            "start_x": 0.0, "start_y": 0.0,
            "finish_x": 10.0, "finish_y": 0.0,
        })
    assert invalid_window_row["motion_invalid_reason"] == \
        "dynamic_motion_window_invalid"
    assert invalid_window["motion_sample_count"] == 0

    static_row = motion_row(1.0, 0.0, 0.0, 2.0, 0.0)
    static_motion = annotate_motion_frames(
        [static_row], "static", {})
    assert static_row["actual_linear_speed_mps"] == ""
    assert static_row["actual_yaw_rate_radps"] == ""
    assert static_row["path_lateral_offset_normalized"] == ""
    assert static_row["motion_invalid_reason"] == "static_trial"
    assert static_motion["motion_sample_count"] == 0
    assert static_motion["lateral_offset_sample_count"] == 0

    default_matrix = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "vsim04_trial_matrix.yaml")
    matrix_path = os.environ.get("VSIM04_MATRIX", default_matrix)
    matrix = load_trial_matrix(matrix_path)
    trials = matrix["trials"]
    assert len(trials) == 23
    assert sum(trial["kind"] == "static" for trial in trials) == 15
    assert sum(trial["kind"] == "dynamic" for trial in trials) == 8
    assert {trial["class_name"] for trial in trials} == {
        "tent", "pillbox", "bridge", "panzer", "red_cross"}
    assert all(trial["class_name"] != "tank" for trial in trials)
    surface_path = os.path.join(
        os.path.dirname(matrix_path), "vsim04_operating_surface_matrix.yaml")
    surface = load_trial_matrix(surface_path)
    assert len(surface["trials"]) == 85
    static25 = select_trial_matrix(surface, "", "static25")
    sparse30 = select_trial_matrix(surface, "", "sparse30")
    assert static25["evaluation_scope"] == "diagnostic"
    assert static25["trial_slice"] == "static25"
    assert len(static25["trials"]) == 25
    assert all(trial["kind"] == "static" for trial in static25["trials"])
    assert len(sparse30["trials"]) == 30
    assert all(trial["kind"] == "dynamic" for trial in sparse30["trials"])
    assert {trial["class_name"] for trial in sparse30["trials"]} == {
        "tent", "pillbox", "bridge", "panzer", "red_cross"}
    try:
        select_trial_matrix(surface, "static_tent_h1p2", "static25")
        raise AssertionError("selector and slice were accepted together")
    except ValueError:
        pass
    try:
        select_trial_matrix(surface, "", "missing_slice")
        raise AssertionError("unknown trial slice was accepted")
    except ValueError:
        pass
    selected_ids = [trials[1]["trial_id"], trials[-1]["trial_id"]]
    diagnostic_matrix = select_trial_matrix(matrix, ",".join(selected_ids))
    assert diagnostic_matrix["evaluation_scope"] == "diagnostic"
    assert [trial["trial_id"] for trial in diagnostic_matrix["trials"]] == selected_ids
    assert len(matrix["trials"]) == 23, "selector mutated the formal matrix"
    try:
        select_trial_matrix(matrix, [selected_ids[0], selected_ids[0]])
        raise AssertionError("duplicate trial selector was accepted")
    except ValueError:
        pass
    try:
        select_trial_matrix(matrix, "missing_trial")
        raise AssertionError("unknown trial selector was accepted")
    except ValueError:
        pass

    measured = planned_trial_result(trials[0])
    measured.update({
        "status": "completed",
        "p_confirm": True,
        "p_selected": True,
        "p_interrupt": None,
        "eligible_frames": 10,
        "raw_class_frames": 9,
        "raw_geometry_frames": 8,
        "resolved_frames": 8,
        "refined_frames": 8,
        "geometry_verified_frames": 7,
        "association_valid_frames": 7,
        "center_refined_frames": 7,
        "detection_frames": 8,
        "map_valid_frames": 6,
        "tf_failure_frames": 2,
        "map_errors_xy": [0.05, 0.10, 0.20],
        "confirmation_exposure_sec": 0.4,
        "confirmation_processing_ms": 35.0,
    })
    measured_summary = summarize_trial_results(
        [measured], "unit", actual_fps=20.0)
    measured_result = measured_summary["trials"][0]
    assert measured_summary["status"] == "INCOMPLETE"
    assert measured_summary["metrics"]["p_confirm"] == 1.0
    assert measured_summary["metrics"]["p_selected"] == 1.0
    assert measured_summary["metrics"]["p_interrupt"] is None
    assert abs(measured_result["map_invalid_rate"] - 0.25) < 1.0e-9
    assert abs(measured_result["map_unavailable_rate"] - 0.4) < 1.0e-9
    assert abs(measured_result["tf_failure_rate"] - 0.25) < 1.0e-9
    assert abs(measured_summary["metrics"]["map_invalid_rate"] - 0.25) < 1.0e-9
    assert abs(measured_summary["metrics"]["map_unavailable_rate"] - 0.4) < 1.0e-9
    assert abs(measured_summary["metrics"]["tf_failure_rate"] - 0.25) < 1.0e-9
    assert measured_summary["metric_denominators"]["map_error_samples"] == 3
    assert measured_summary["metric_denominators"]["raw_class_frames"] == 9
    assert abs(
        measured_summary["metrics"]["stage_frame_rates"]["raw_class_rate"] -
        0.9) < 1.0e-9

    failure = planned_trial_result(trials[0])
    failure.update({
        "status": "completed", "p_confirm": False,
        "eligible_frames": 10,
    })
    assert classify_failure_stage(failure) == "raw_classifier"
    failure["stage_trace_enabled"] = False
    assert classify_failure_stage(failure) == "stage_trace_disabled"
    failure["stage_trace_enabled"] = True
    failure["raw_class_frames"] = 8
    assert classify_failure_stage(failure) == "raw_geometry"
    failure["raw_geometry_frames"] = 8
    assert classify_failure_stage(failure) == "detection_fusion"
    failure["resolved_frames"] = 8
    assert classify_failure_stage(failure) == "target_refiner"
    failure["refined_frames"] = 8
    assert classify_failure_stage(failure) == "geometry_association"
    failure["association_valid_frames"] = 7
    assert classify_failure_stage(failure) == "geometry_refinement"
    failure["geometry_verified_frames"] = 7
    failure["center_refined_frames"] = 7
    assert classify_failure_stage(failure) == "map_projector_input"
    failure["detection_frames"] = 7
    assert classify_failure_stage(failure) == "map_projection"
    failure["map_valid_frames"] = 7
    assert classify_failure_stage(failure) == "target_memory_admission"

    terminal_results = []
    for trial in trials:
        result = planned_trial_result(trial)
        result.update({
            "status": "completed",
            "p_confirm": False,
            "p_selected": False,
            "entered_fully_in_frame": True,
            "left_fully_in_frame": True,
            "eligible_frames": 1,
            "complete_mapped_frames": 1,
            "partial_only_mapped_frames": 1,
            "detector_inference_ms_samples": [10.0, 20.0],
            "detector_processing_ms_samples": [12.0, 24.0],
        })
        if trial["kind"] == "dynamic":
            result.update({
                "expected_duration_sec": 2.0,
                "actual_duration_sec": 2.01,
                "actual_speed_mps": trial["speed_mps"],
                "camera_pose_frame_count": 3,
                "actual_linear_speed_mps_samples": [
                    trial["speed_mps"], trial["speed_mps"]],
                "actual_yaw_rate_radps_samples": [0.0, -0.1],
                "normalized_lateral_offset_samples": [0.01, -0.02],
            })
        terminal_results.append(result)
    terminal_summary = summarize_trial_results(
        terminal_results, "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 23,
                          "validation_errors": [],
                          "class_profile": "r2026",
                          "performance_contract": matrix[
                              "performance_contract"],
                          "actual_image_source_fps": 30.0})
    assert terminal_summary["status"] == "MEASURED"
    assert terminal_summary["completeness"]["status"] == "MEASURED"
    assert terminal_summary["performance_verdict"]["status"] == "FAIL"
    assert not terminal_summary["performance_verdict"]["is_gate_pass"]
    assert terminal_summary["evaluation_scope"] == "full"
    assert terminal_summary["metrics"]["actual_image_source_fps"] == 30.0
    assert terminal_summary["metrics"]["complete_mapped_rate"] == 0.5
    assert terminal_summary["metrics"]["p95_detector_inference_ms"] == 20.0
    assert terminal_summary["metrics"]["p95_detector_processing_ms"] == 24.0
    speed_groups = {
        group["value"]: group
        for group in terminal_summary["breakdowns"]["by_speed_mps"]}
    assert set(speed_groups) == {None, 0.5, 1.5}
    assert speed_groups[None]["label"] == "static"
    assert speed_groups[0.5]["completed_trial_count"] == 4
    assert speed_groups[0.5]["motion_sample_count"] == 8
    assert abs(speed_groups[0.5]["mean_actual_linear_speed_mps"] -
               0.5) < 1.0e-9
    assert abs(speed_groups[1.5]["p95_abs_normalized_lateral_offset"] -
               0.02) < 1.0e-9
    class_groups = {
        group["value"]: group
        for group in terminal_summary["breakdowns"]["by_class"]}
    assert class_groups["panzer"]["completed_trial_count"] == 7
    height_groups = {
        group["value"]: group
        for group in terminal_summary["breakdowns"]["by_height_m"]}
    assert height_groups[1.8]["completed_trial_count"] == 4
    performance_rows = decorate_performance_rows(
        terminal_summary["trials"], terminal_summary["breakdowns"])
    dynamic_half = next(
        row for row in performance_rows
        if row["kind"] == "dynamic" and row["speed_mps"] == 0.5)
    assert dynamic_half["speed_group_completed_trials"] == 4
    assert abs(dynamic_half["speed_group_mean_actual_linear_speed_mps"] -
               0.5) < 1.0e-9
    invalid_summary = summarize_trial_results(
        terminal_results[:-1], "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 23,
                          "validation_errors": ["completed_trials_22/23"]})
    assert invalid_summary["status"] == "INVALID"
    spoofed_22_summary = summarize_trial_results(
        terminal_results[:-1], "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 22,
                          "validation_errors": []})
    assert spoofed_22_summary["status"] == "INVALID"
    missing_leave = [dict(result) for result in terminal_results]
    missing_leave[0]["left_fully_in_frame"] = False
    missing_leave_summary = summarize_trial_results(
        missing_leave, "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 23,
                          "validation_errors": []})
    assert missing_leave_summary["status"] == "INVALID"
    diagnostic_summary = summarize_trial_results(
        terminal_results[:2], "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 2,
                          "evaluation_scope": "diagnostic",
                          "validation_errors": [],
                          "class_profile": "r2026",
                          "performance_contract": matrix[
                              "performance_contract"]})
    assert diagnostic_summary["status"] == "DIAGNOSTIC"
    assert diagnostic_summary["completed_trial_count"] == 2
    assert diagnostic_summary["performance_verdict"]["status"] == (
        "DIAGNOSTIC_ONLY")
    assert diagnostic_summary["performance_verdict"]["failure_reasons"] == [
        "diagnostic_subset_not_gate"]
    assert diagnostic_summary["performance_verdict"][
        "metric_failure_reasons"]

    audit_observations = [
        candidate_audit_observation(
            "confirmed", "tent", 1, "tent",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=True, trial_id=trials[0]["trial_id"]),
        candidate_audit_observation(
            "confirmed", "panzer", 2, "tent",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=True, trial_id=trials[0]["trial_id"]),
        candidate_audit_observation(
            "confirmed", "tank", 3, "tent",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=False, trial_id=trials[0]["trial_id"]),
        candidate_audit_observation(
            "selected", "panzer", 2, "tent",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=True, trial_id=trials[0]["trial_id"]),
        candidate_audit_observation(
            "selected", "tent", 4, "tent",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=False, trial_id=trials[0]["trial_id"]),
        candidate_audit_observation(
            "selected", "tank", 3, "",
            {"tent", "panzer", "red_cross"}, state=2,
            policy_selectable=False, trial_id=""),
    ]
    audit = candidate_audit_summary(audit_observations, "r2026")
    assert audit["confirmed"]["observations"] == 3
    assert audit["confirmed"]["unexpected_observations"] == 2
    assert audit["confirmed"]["disallowed_observations"] == 1
    assert audit["confirmed"]["tank_observations"] == 1
    assert audit["selected"]["observations"] == 3
    assert audit["selected"]["unexpected_observations"] == 1
    assert audit["selected"]["disallowed_observations"] == 1
    assert audit["selected"]["tank_observations"] == 1
    assert audit["selected"]["policy_rejected_observations"] == 2
    assert audit["unscoped_observation_count"] == 1
    assert audit["trials"][trials[0]["trial_id"]]["selected"][
        "unexpected_observations"] == 1

    contract = matrix["performance_contract"]
    hard_verdict = evaluate_performance_verdict(
        {"p_confirm": 1.0, "p_selected": 1.0,
         "p95_confirmation_processing_ms": 100.0,
         "p95_map_error_xy": 0.1, "tf_failure_rate": 0.0},
        "MEASURED", "full", audit, contract)
    assert hard_verdict["status"] == "FAIL"
    assert hard_verdict["hard_failure"]
    assert any(reason.startswith("disallowed_selected_observations:")
               for reason in hard_verdict["hard_failure_reasons"])
    assert any(reason.startswith("r2026_tank_selected_observations:")
               for reason in hard_verdict["hard_failure_reasons"])
    assert any(reason.startswith(
        "selected_rejected_by_current_policy_observations:")
        for reason in hard_verdict["hard_failure_reasons"])

    clean_audit = candidate_audit_summary([], "r2026")
    not_gated = evaluate_performance_verdict(
        {"p_confirm": 1.0, "p_selected": 1.0,
         "p95_confirmation_processing_ms": 100.0,
         "p95_map_error_xy": 0.1, "tf_failure_rate": 0.0},
        "MEASURED", "full", clean_audit, contract)
    assert not_gated["status"] == "NOT_GATED"
    assert not not_gated["is_gate_pass"]
    assert sorted(not_gated["failure_reasons"]) == [
        "threshold_unfrozen:max_tf_failure_rate",
        "threshold_unfrozen:min_p_confirm",
        "threshold_unfrozen:min_p_selected",
    ]
    diagnostic_hard = evaluate_performance_verdict(
        {}, "DIAGNOSTIC", "diagnostic", audit, contract)
    assert diagnostic_hard["status"] == "FAIL"
    assert diagnostic_hard["hard_failure"]
    incomplete_verdict = evaluate_performance_verdict(
        {}, "INCOMPLETE", "full", clean_audit, contract)
    assert incomplete_verdict["status"] == "NOT_EVALUATED"

    # selected may arrive at the recorder before /targets because ROS does not
    # order different topic connections.  Correlation must use ID + two clocks,
    # not callback order.  Events received after leave remain failures.
    window = {
        "enter_source_stamp": 1.0,
        "leave_source_stamp": 3.0,
        "leave_receipt_monotonic": 30.0,
    }
    candidates = [
        {"source_stamp": 2.0, "stamp_key": 200, "receipt_monotonic": 20.0,
         "stable_id": 7},
        {"source_stamp": 2.5, "stamp_key": 250, "receipt_monotonic": 31.0,
         "stable_id": 8},
    ]
    selected_first = [
        {"source_stamp": 2.0, "stamp_key": 200, "receipt_monotonic": 19.0,
         "stable_id": 7},
    ]
    correlated = correlate_admission_events(
        candidates, selected_first, window, {200: 10.0}, {200: 15.0})
    assert correlated["p_confirm"] and correlated["p_selected"]
    assert correlated["stable_id"] == 7
    assert abs(correlated["confirmation_exposure_sec"] - 1.0) < 1.0e-9
    assert abs(correlated["confirmation_processing_ms"] - 10000.0) < 1.0e-9
    assert abs(correlated["confirmation_pipeline_ms"] - 5000.0) < 1.0e-9
    image_after_candidate = correlate_admission_events(
        candidates[:1], selected_first, window, {200: 21.0})
    assert image_after_candidate["p_confirm"]
    assert image_after_candidate["confirmation_processing_ms"] == 0.0
    assert image_after_candidate["processing_receipt_reordered"]
    watermarks = {
        "image": 3.1, "truth": 3.0, "mapped": 3.2,
        "targets": 3.0, "perf": 3.3,
    }
    assert watermarks_cover_source_stamp(watermarks, 3.0)
    watermarks["targets"] = 2.99
    assert not watermarks_cover_source_stamp(watermarks, 3.0)
    assert not watermarks_cover_source_stamp(watermarks, None)
    assert not detector_diagnostic_errors(
        0, {"backend": "ultralytics", "model_path": "/tmp/model.pt"},
        "ultralytics", "/tmp/model.pt")
    assert detector_diagnostic_errors(
        1, {"backend": "empty", "model_path": "/tmp/other.pt"},
        "ultralytics", "/tmp/model.pt") == [
            "diagnostic_level_1", "backend_empty", "model_path_mismatch"]
    required_sources = {
        "target_detector", "circle_detector", "cross_detector"}
    assert completed_sources_cover(
        required_sources,
        ["cross_detector", "target_detector", "circle_detector"])
    assert not completed_sources_cover(
        required_sources, ["cross_detector", "circle_detector"])
    assert call_with_monotonic_deadline(
        lambda: 7, 0.5, "unit_service") == 7
    try:
        call_with_monotonic_deadline(
            lambda: time.sleep(0.05), 0.01, "stalled_service")
        raise AssertionError("blocking service call did not time out")
    except TimeoutError:
        pass
    assert handshake_timeout_is_safe(16.0, 10.0, 0.25, 0.25, 5.0)
    assert not handshake_timeout_is_safe(15.5, 10.0, 0.25, 0.25, 5.0)

    output_dir = tempfile.mkdtemp(prefix="vsim04_schema_")
    try:
        summary = dry_run_artifacts(matrix_path, output_dir, metadata={
            "revisions": {"vision": "test", "navigation": "test"},
            "thresholds": {"confirm_frames": 3,
                           "selected_max_age_sec": 0.5},
        })
        assert summary["trial_count"] == 23
        assert summary["completed_trial_count"] == 0
        assert summary["status"] == "DRY_RUN"
        assert summary["completeness"]["status"] == "DRY_RUN"
        assert summary["performance_verdict"]["status"] == "NOT_EVALUATED"
        assert summary["artifact_completeness"]["complete"]
        assert set(summary["artifact_completeness"]["present"]) == set(
            REQUIRED_ARTIFACTS)
        assert summary["artifact_completeness"]["missing"] == []
        assert summary["metrics"]["p_interrupt"] is None
        for artifact in REQUIRED_ARTIFACTS:
            artifact_path = os.path.join(output_dir, artifact)
            assert os.path.isfile(artifact_path), artifact
            assert os.path.getsize(artifact_path) > 0, artifact
        with open(os.path.join(output_dir, "manifest.json"),
                  "r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        assert manifest["seed"] == 11
        assert manifest["class_profile"] == "r2026"
        assert len(manifest["trials"]) == 23
        with open(os.path.join(output_dir, "events.csv"),
                  "r", encoding="utf-8") as stream:
            assert len(list(csv.DictReader(stream))) == 23
        with open(os.path.join(output_dir, "vision_search_performance.csv"),
                  "r", encoding="utf-8") as stream:
            performance_reader = csv.DictReader(stream)
            assert performance_reader.fieldnames == PERFORMANCE_FIELDS
            rows = list(performance_reader)
        assert len(rows) == 23
        assert all(row["p_interrupt"] == "" for row in rows)
        assert all(row["class_group_completed_trials"] == "0"
                   for row in rows)
        with open(os.path.join(output_dir, "report.md"),
                  "r", encoding="utf-8") as stream:
            report = stream.read()
        assert "## Breakdown by class" in report
        assert "## Breakdown by height" in report
        assert "## Breakdown by requested speed" in report
        assert all(row["measurement_completeness_status"] == "DRY_RUN"
                   for row in rows)
        assert all(row["artifact_set_complete"] == "True" for row in rows)
        assert all(row["performance_verdict"] == "NOT_EVALUATED"
                   for row in rows)
        with open(os.path.join(output_dir, "frames.csv"),
                  "r", encoding="utf-8") as stream:
            frame_reader = csv.DictReader(stream)
            assert frame_reader.fieldnames == FRAME_FIELDS
        with open(os.path.join(output_dir, "summary.json"),
                  "r", encoding="utf-8") as stream:
            persisted_summary = json.load(stream)
        assert persisted_summary["artifact_completeness"] == summary[
            "artifact_completeness"]
    finally:
        shutil.rmtree(output_dir)

    audit_output = tempfile.mkdtemp(prefix="vsim04_audit_schema_")
    try:
        audit_result = planned_trial_result(trials[0])
        audit_result.update({
            "status": "completed",
            "p_confirm": True,
            "p_selected": False,
            "entered_fully_in_frame": True,
            "left_fully_in_frame": True,
            "eligible_frames": 1,
            "detection_frames": 1,
            "map_valid_frames": 1,
            "map_errors_xy": [0.1],
            "confirmation_processing_ms": 100.0,
        })
        audit_summary = write_artifacts(
            audit_output, {"class_profile": "r2026"}, [], [],
            [audit_result], "unit", actual_fps=20.0,
            terminal_context={
                "run_complete": True,
                "evaluation_scope": "diagnostic",
                "expected_trial_count": 1,
                "validation_errors": [],
                "class_profile": "r2026",
                "candidate_audit_observations": audit_observations,
                "performance_contract": contract,
            }, actual_source_fps=20.0)
        assert audit_summary["status"] == "DIAGNOSTIC"
        assert audit_summary["performance_verdict"]["status"] == "FAIL"
        assert audit_summary["candidate_audit"]["selected"][
            "disallowed_observations"] == 1
        with open(os.path.join(
                audit_output, "vision_search_performance.csv"),
                "r", encoding="utf-8") as stream:
            audit_rows = list(csv.DictReader(stream))
        assert len(audit_rows) == 1
        assert audit_rows[0]["unexpected_confirmed_observations"] == "2"
        assert audit_rows[0]["disallowed_confirmed_observations"] == "1"
        assert audit_rows[0]["unexpected_selected_observations"] == "1"
        # The tank selection was deliberately emitted outside the trial and
        # remains visible in the run-level summary/hard verdict, not this row.
        assert audit_rows[0]["disallowed_selected_observations"] == "0"
        assert audit_rows[0][
            "policy_rejected_selected_observations"] == "1"
        assert audit_rows[0]["performance_hard_failure"] == "True"
        assert audit_rows[0]["performance_metric_failure_reasons"] == "[]"
        with open(os.path.join(audit_output, "report.md"),
                  "r", encoding="utf-8") as stream:
            report = stream.read()
        assert "Measurement completeness: `DIAGNOSTIC`" in report
        assert "Algorithm performance verdict: `FAIL`" in report
    finally:
        shutil.rmtree(audit_output)
    print("V-SIM-04 matrix/artifact schema PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("V-SIM-04 matrix/artifact schema FAIL: {}".format(error),
              file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
