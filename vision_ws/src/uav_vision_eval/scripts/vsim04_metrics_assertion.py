#!/usr/bin/env python3
"""Pure deterministic assertion for the V-SIM-04 matrix and artifact schema."""

import csv
import json
import os
import shutil
import sys
import tempfile
import time

from uav_vision_eval.vsim04_metrics import (
    REQUIRED_ARTIFACTS,
    FRAME_FIELDS,
    classify_failure_stage,
    completed_sources_cover,
    call_with_monotonic_deadline,
    correlate_admission_events,
    detector_diagnostic_errors,
    handshake_timeout_is_safe,
    dry_run_artifacts,
    load_trial_matrix,
    planned_trial_result,
    select_trial_matrix,
    summarize_trial_results,
    watermarks_cover_source_stamp,
)
from uav_vision_eval.stamped_pose_buffer import StampedPoseBuffer

from geometry_msgs.msg import PoseStamped
import rospy


def main():
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
        })
        if trial["kind"] == "dynamic":
            result.update({
                "expected_duration_sec": 2.0,
                "actual_duration_sec": 2.01,
                "actual_speed_mps": trial["speed_mps"],
            })
        terminal_results.append(result)
    terminal_summary = summarize_trial_results(
        terminal_results, "unit", actual_fps=30.0,
        terminal_context={"run_complete": True,
                          "expected_trial_count": 23,
                          "validation_errors": [],
                          "actual_image_source_fps": 30.0})
    assert terminal_summary["status"] == "MEASURED"
    assert terminal_summary["evaluation_scope"] == "full"
    assert terminal_summary["metrics"]["actual_image_source_fps"] == 30.0
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
                          "validation_errors": []})
    assert diagnostic_summary["status"] == "DIAGNOSTIC"
    assert diagnostic_summary["completed_trial_count"] == 2

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
        candidates, selected_first, window, {200: 10.0})
    assert correlated["p_confirm"] and correlated["p_selected"]
    assert correlated["stable_id"] == 7
    assert abs(correlated["confirmation_exposure_sec"] - 1.0) < 1.0e-9
    assert abs(correlated["confirmation_processing_ms"] - 10000.0) < 1.0e-9
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
        assert summary["metrics"]["p_interrupt"] is None
        for artifact in REQUIRED_ARTIFACTS:
            assert os.path.isfile(os.path.join(output_dir, artifact)), artifact
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
            rows = list(csv.DictReader(stream))
        assert len(rows) == 23
        assert all(row["p_interrupt"] == "" for row in rows)
        with open(os.path.join(output_dir, "frames.csv"),
                  "r", encoding="utf-8") as stream:
            frame_reader = csv.DictReader(stream)
            assert frame_reader.fieldnames == FRAME_FIELDS
    finally:
        shutil.rmtree(output_dir)
    print("V-SIM-04 matrix/artifact schema PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("V-SIM-04 matrix/artifact schema FAIL: {}".format(error),
              file=sys.stderr)
        sys.exit(1)
