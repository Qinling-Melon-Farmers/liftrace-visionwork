#!/usr/bin/env python3
"""Pure-function regression for the V-SIM-04 camera soak accounting."""

import os
import sys
from types import SimpleNamespace


SOURCE_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "src"))
if SOURCE_ROOT not in sys.path:
    sys.path.insert(0, SOURCE_ROOT)

from uav_vision_eval.vsim04_soak import (  # noqa: E402
    SoakAccounting,
    camera_orientation_drift_errors,
    camera_pose_tracking_errors,
    camera_info_snapshot,
    measurement_presence_errors,
    route_pose,
    selected_candidate_errors,
    truth_catalog_errors,
    validate_soak_config,
)


def config(duration=600.0):
    return {
        "duration_sec": duration,
        "route_period_sec": 40.0,
        "route_update_rate_hz": 10.0,
        "route_center_x_m": -0.6,
        "route_center_y_m": 0.5,
        "route_radius_x_m": 3.2,
        "route_radius_y_m": 3.0,
        "route_height_m": 2.4,
        "arena_limit_m": 4.8,
        "health_window_sec": 10.0,
        "heartbeat_timeout_sec": 3.0,
        "startup_timeout_sec": 45.0,
        "service_call_timeout_sec": 5.0,
        "node_check_timeout_sec": 2.0,
        "bucket_settle_sec": 0.5,
        "max_source_lag_sec": 1.0,
        "max_camera_pose_age_sec": 0.5,
        "max_camera_pose_error_m": 0.25,
        "max_camera_orientation_drift_rad": 0.05,
        "min_input_fps": 5.0,
        "min_complete_mapped_fps": 1.0,
        "max_partial_only_ratio": 0.8,
        "min_partial_samples": 5,
        "bad_windows_to_fail": 3,
    }


def feed_window(accounting, start, images=100, complete=20, partial=0,
                lag=0.1):
    for index in range(images):
        stamp = start + index * 0.09
        accounting.note_stream("image", stamp, stamp)
    for index in range(complete):
        stamp = start + index * 0.4
        accounting.note_mapped(stamp, stamp, True, stamp + lag)
    for index in range(partial):
        stamp = start + 0.001 + index * 0.3
        accounting.note_mapped(stamp, stamp, False, stamp + lag)
    for name in ("truth", "targets", "perf", "camera_pose"):
        for index in range(10):
            stamp = start + index
            accounting.note_stream(name, stamp, stamp)
    accounting.evaluate(start + 10.6)


def assert_soak_contract():
    cfg = validate_soak_config(config())
    first = route_pose(0.0, cfg)
    second = route_pose(40.0, cfg)
    assert first["trial_id"] == "soak_loop_0001"
    assert second["trial_id"] == "soak_loop_0002"
    assert first["trial_id"] != second["trial_id"]
    assert abs(first["x"] - second["x"]) < 1.0e-9

    actual_pose = {
        "receipt_monotonic": 9.8, "x": first["x"] + 0.05,
        "y": first["y"], "z": first["z"],
    }
    pose_errors, pose_age, pose_error = camera_pose_tracking_errors(
        actual_pose, first, 10.0, 0.5, 0.25)
    assert pose_errors == []
    assert abs(pose_age - 0.2) < 1.0e-9
    assert abs(pose_error - 0.05) < 1.0e-9
    assert "camera_pose_tracking_error" in camera_pose_tracking_errors(
        actual_pose, first, 10.0, 0.5, 0.01)[0]
    orientation_errors, orientation_drift = camera_orientation_drift_errors(
        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, -1.0), 0.05)
    assert orientation_errors == []
    assert abs(orientation_drift) < 1.0e-9
    assert "camera_orientation_drift" in camera_orientation_drift_errors(
        (0.0, 0.0, 0.1, 0.994987437),
        (0.0, 0.0, 0.0, 1.0), 0.05)[0]

    truth = [
        {"target_id": "tent_1", "pose_valid": True},
        {"target_id": "pillbox_1", "pose_valid": True},
    ]
    assert truth_catalog_errors(
        "scenario", truth, "scenario", {"tent_1", "pillbox_1"}) == []
    truth[1]["pose_valid"] = False
    assert "truth_pose_invalid:pillbox_1" in truth_catalog_errors(
        "scenario", truth, "scenario", {"tent_1", "pillbox_1"})

    presence = measurement_presence_errors(
        ("image", "truth"), {"image": 1, "truth": 0}, 0, 0)
    assert "measurement_stream_missing:truth" in presence
    assert "measurement_truth_valid_missing" in presence
    assert "measurement_actual_camera_pose_missing" in presence
    assert "measurement_truth_projection_missing" in presence
    assert "measurement_truth_fully_in_frame_missing" in presence
    assert measurement_presence_errors(
        ("image", "truth"), {"image": 1, "truth": 1}, 1, 1,
        1, 1) == []

    camera_info = SimpleNamespace(
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
        width=640, height=480, distortion_model="plumb_bob",
        K=[400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0],
        D=[0.0] * 5,
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[400.0, 0.0, 320.0, 0.0, 0.0, 400.0,
           240.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    snapshot = camera_info_snapshot(camera_info)
    assert snapshot["width"] == 640
    camera_info.K[0] = 0.0
    try:
        camera_info_snapshot(camera_info)
        raise AssertionError("invalid CameraInfo must fail")
    except ValueError:
        pass

    required = ("image", "truth", "mapped_complete", "targets", "perf",
                "camera_pose")
    warm = SoakAccounting(cfg, required)
    warm.note_stream("image", 1.0, 1.0)
    warm.note_mapped(1.0, 1.0, True, 1.1)
    warm.add_error("warmup_only")
    warm.begin_measurement(2.0)
    assert warm.counts["image"] == 0
    assert warm.complete_mapped_frames == 0
    assert warm.errors == []
    assert warm.last_receipt["image"] == 1.0
    assert warm.last_source_stamp == {}

    healthy = SoakAccounting(cfg, required)
    healthy.start(0.0)
    for window in range(3):
        feed_window(healthy, window * 10.6)
    summary = healthy.final_summary(600.0, 600.0, 570.0)
    assert summary["status"] == "SOAK_MEASURED"
    assert summary["qualification_status"] == "SOAK_600S_MEASURED"
    assert summary["soak_600s_pass"] is True
    assert summary["actual_wall_duration_sec"] == 600.0
    assert summary["actual_source_duration_sec"] == 570.0
    smoke = healthy.final_summary(20.0, 20.0, 25.0)
    assert smoke["status"] == "SOAK_MEASURED"
    assert smoke["qualification_status"] == "SMOKE_ONLY"
    assert smoke["soak_600s_pass"] is False

    regressed = SoakAccounting(cfg, required)
    regressed.note_stream("image", 1.0, 2.0)
    regressed.note_stream("image", 2.0, 1.0)
    assert "image_source_time_regressed" in regressed.errors

    mapped_reordered = SoakAccounting(cfg, required)
    mapped_reordered.note_mapped(1.0, 2.0, True, 2.1)
    mapped_reordered.note_mapped(2.0, 1.0, True, 2.1)
    assert mapped_reordered.errors == []
    assert mapped_reordered.source_reorder_counts["mapped"] == 1
    assert mapped_reordered.source_reorder_counts["mapped_complete"] == 1

    heartbeat_gap = SoakAccounting(cfg, required)
    heartbeat_gap.start(0.0)
    heartbeat_gap.note_stream("image", 0.1, 0.1)
    heartbeat_gap.note_stream("image", 3.2, 3.2)
    assert "image_heartbeat_gap_exceeded" in heartbeat_gap.errors

    degraded = SoakAccounting(cfg, required)
    degraded.start(0.0)
    for window in range(3):
        feed_window(degraded, window * 10.6, images=10, complete=1,
                    partial=12, lag=2.0)
    assert "input_throughput_sustained" in degraded.errors
    assert "complete_mapped_throughput_sustained" in degraded.errors
    assert "partial_only_trend_sustained" in degraded.errors
    assert "source_backlog_sustained" in degraded.errors

    stale_tank = {
        "class_name": "tank", "last_seen_sec": 1.0, "state": 2,
        "consecutive_observe_count": 3, "map_valid": True,
        "association_valid": True, "reject_reason": "",
    }
    errors = selected_candidate_errors(
        stale_tank, 2.0,
        {"tent", "pillbox", "bridge", "panzer", "red_cross"}, 3, 0.5)
    assert "tank_selected" in errors
    assert "selected_class_disallowed:tank" in errors
    assert "selected_stale" in errors

    incomplete = SoakAccounting(cfg, required)
    incomplete.start(0.0)
    result = incomplete.final_summary(600.0, 599.0, 610.0)
    assert result["status"] == "FAIL"
    assert "soak_duration_incomplete" in result["errors"]


def test_soak_contract():
    assert_soak_contract()


def main():
    assert_soak_contract()
    print("V-SIM-04 camera soak pure accounting PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
