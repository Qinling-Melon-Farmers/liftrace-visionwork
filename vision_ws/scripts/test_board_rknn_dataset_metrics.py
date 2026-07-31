#!/usr/bin/env python3
"""Deterministic tests for the board dataset metric implementation."""
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "top_level_scripts" / "board_rknn_dataset_eval.py"
SPEC = importlib.util.spec_from_file_location("board_rknn_dataset_eval", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_exact_match():
    gt = {"a.jpg": [(0, [0.0, 0.0, 1.0, 1.0])]}
    pred = {"a.jpg": [(0, 0.9, [0.0, 0.0, 1.0, 1.0])]}
    result = MODULE.compute_threshold_metrics(gt, pred, 0.25, 0.5, 1)
    assert result["tp"] == 1
    assert result["fp"] == 0
    assert result["fn"] == 0


def test_wrong_class_is_fp_and_fn():
    gt = {"a.jpg": [(0, [0.0, 0.0, 1.0, 1.0])]}
    pred = {"a.jpg": [(1, 0.9, [0.0, 0.0, 1.0, 1.0])]}
    result = MODULE.compute_threshold_metrics(gt, pred, 0.25, 0.5, 2)
    assert result["tp"] == 0
    assert result["fp"] == 1
    assert result["fn"] == 1


if __name__ == "__main__":
    test_exact_match()
    test_wrong_class_is_fp_and_fn()
    print("[PASS] board dataset metric contract")
