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
        [[581.2568, 0.0, 1043.5], [0.0, 580.9240, 513.0979], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    expected_d = np.array([0.0349, -0.0426, 0.0, 0.0, 0.0076], dtype=np.float32)
    assert np.allclose(MODULE.CAMERA_K, expected_k)
    assert np.allclose(MODULE.CAMERA_D, expected_d)


def test_undistort_maps_and_frame_shape():
    map1, map2, new_k = MODULE.build_undistort_maps(1920, 1080)
    assert map1.shape[:2] == (1080, 1920)
    assert map2.shape[:2] == (1080, 1920)
    assert new_k.shape == (3, 3)

    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    frame[540, 960] = (255, 0, 0)
    rectified = MODULE.undistort_frame(frame, map1, map2)
    assert rectified.shape == frame.shape

    # Some board-side OpenCV builds expose the rotated metadata order for the
    # 2560x1080 real-target video. The map must follow the decoded frame shape.
    map1_wide, map2_wide, _ = MODULE.build_undistort_maps(2560, 1080)
    assert map1_wide.shape[:2] == (1080, 2560)
    assert map2_wide.shape[:2] == (1080, 2560)


def test_video_replay_defaults_to_raw_pixels():
    args = MODULE.parse_args(["--video", "real_target.mp4"])
    assert not args.no_rectify
    assert not args.rectify_video


if __name__ == "__main__":
    test_calibration_constants()
    test_undistort_maps_and_frame_shape()
    print("[PASS] board viewer calibration contract")
