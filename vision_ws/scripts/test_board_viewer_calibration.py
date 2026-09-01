#!/usr/bin/env python3
"""Offline contract tests for the board viewer camera calibration."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "top_level_scripts" / "board_realtime_rknn_viewer.py"
SPEC = importlib.util.spec_from_file_location("board_realtime_rknn_viewer", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_calibration_constants():
    expected_k = np.array(
        [[725.3510059644434, 0.0, 631.67186313702575],
         [0.0, 723.34035628450874, 397.56638133116269],
         [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    expected_d = np.array(
        [0.0058668600963917095, 0.017910549546758369,
         -0.0010064115869294274, 0.0014715593681005204,
         -0.026485100937585344], dtype=np.float32)
    assert np.allclose(MODULE.CAMERA_K, expected_k)
    assert np.allclose(MODULE.CAMERA_D, expected_d)


def test_undistort_maps_and_frame_shape():
    map1, map2, new_k = MODULE.build_undistort_maps(1280, 720)
    assert map1.shape[:2] == (720, 1280)
    assert map2.shape[:2] == (720, 1280)
    assert new_k.shape == (3, 3)

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[360, 640] = (255, 0, 0)
    rectified = MODULE.undistort_frame(frame, map1, map2)
    assert rectified.shape == frame.shape

    # Some board-side OpenCV builds expose the rotated metadata order for the
    # 2560x1080 real-target video. The map must follow the decoded frame shape.
    map1_wide, map2_wide, _ = MODULE.build_undistort_maps(
        2560, 1080, allow_scale=True)
    assert map1_wide.shape[:2] == (1080, 2560)
    assert map2_wide.shape[:2] == (1080, 2560)


def test_video_replay_defaults_to_raw_pixels():
    args = MODULE.parse_args(["--video", "real_target.mp4"])
    assert not args.no_rectify
    assert not args.rectify_video


def test_live_defaults_use_new_calibration_and_are_overridable():
    args = MODULE.parse_args([])
    assert args.model.endswith(
        "vision_ws/src/uav_vision/models/merged_standard_fp32.rknn")
    assert args.camera_width == 1280
    assert args.camera_height == 720
    assert args.camera_fps == 30.0
    assert args.calibration.endswith("calibration_1280x720.yaml")
    custom = MODULE.parse_args([
        "--camera", "/dev/v4l/by-id/test-camera", "--camera-width", "640",
        "--camera-height", "480", "--camera-fps", "20"])
    assert custom.camera == "/dev/v4l/by-id/test-camera"
    assert custom.camera_width == 640
    assert custom.camera_height == 480
    assert custom.camera_fps == 20.0


if __name__ == "__main__":
    test_calibration_constants()
    test_undistort_maps_and_frame_shape()
    test_live_defaults_use_new_calibration_and_are_overridable()
    print("[PASS] board viewer calibration contract")
