#!/usr/bin/env python3
"""Pure deterministic assertion for the V-SIM-04 matrix and artifact schema."""

import csv
import json
import os
import shutil
import sys
import tempfile

from uav_vision_eval.vsim04_metrics import (
    REQUIRED_ARTIFACTS,
    dry_run_artifacts,
    load_trial_matrix,
    planned_trial_result,
    summarize_trial_results,
)


def main():
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

    measured = planned_trial_result(trials[0])
    measured.update({
        "status": "completed",
        "p_confirm": True,
        "p_selected": True,
        "p_interrupt": None,
        "eligible_frames": 10,
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
    assert measured_summary["metrics"]["p_confirm"] == 1.0
    assert measured_summary["metrics"]["p_selected"] == 1.0
    assert measured_summary["metrics"]["p_interrupt"] is None
    assert abs(measured_result["map_invalid_rate"] - 0.25) < 1.0e-9
    assert abs(measured_result["map_unavailable_rate"] - 0.4) < 1.0e-9
    assert abs(measured_result["tf_failure_rate"] - 0.25) < 1.0e-9

    output_dir = tempfile.mkdtemp(prefix="vsim04_schema_")
    try:
        summary = dry_run_artifacts(matrix_path, output_dir, metadata={
            "revisions": {"vision": "test", "navigation": "test"},
            "thresholds": {"confirm_frames": 3,
                           "selected_max_age_sec": 0.5},
        })
        assert summary["trial_count"] == 23
        assert summary["completed_trial_count"] == 0
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
