#!/usr/bin/env python3
"""No-Gazebo tests for V-SIM-04 repeat command and aggregation contracts."""

import json
import hashlib
import os
import tempfile
import unittest

from uav_vision_eval.repeat_runs import (
    REQUIRED_ARTIFACTS,
    aggregate_repeat_runs,
    build_repeat_commands,
    write_aggregate_outputs,
)


def _make_run(root, name, status="MEASURED", verdict="PASS",
              complete=True, failure_stage="", p_confirm=True,
              p_selected=True, missing_artifact=""):
    run_dir = os.path.join(root, name)
    vsim_dir = os.path.join(run_dir, "vsim04")
    os.makedirs(vsim_dir)
    for artifact in REQUIRED_ARTIFACTS:
        if artifact == missing_artifact or artifact == "summary.json":
            continue
        with open(os.path.join(vsim_dir, artifact), "w", encoding="utf-8") as stream:
            stream.write("fixture\n")
    summary = {
        "evaluation_id": "V-SIM-04",
        "status": status,
        "trial_count": 1,
        "completed_trial_count": 1 if complete else 0,
        "validation_errors": [],
        "completeness": {
            "run_complete": complete,
            "validation_errors": [],
        },
        "performance_verdict": {
            "status": verdict,
            "is_gate_pass": verdict == "PASS",
            "hard_failure": False,
        },
        "metrics": {
            "p95_confirmation_processing_ms": 120.0,
            "p95_map_error_xy": 0.08,
        },
        "trials": [{
            "trial_id": "static_pillbox_h3p6",
            "status": "completed" if complete else "incomplete",
            "p_confirm": p_confirm,
            "p_selected": p_selected,
            "p_interrupt": None,
            "failure_stage": failure_stage,
            "confirmation_processing_ms": 110.0,
            "p95_map_error_xy": 0.07,
        }],
    }
    with open(os.path.join(vsim_dir, "summary.json"), "w",
              encoding="utf-8") as stream:
        json.dump(summary, stream)
    return run_dir


class RepeatRunTests(unittest.TestCase):
    def test_commands_are_unique_and_always_use_sim_run(self):
        with tempfile.TemporaryDirectory() as root:
            matrix = os.path.join(root, "matrix.yaml")
            model = os.path.join(root, "model.pt")
            open(matrix, "w", encoding="utf-8").close()
            open(model, "w", encoding="utf-8").close()
            commands = build_repeat_commands(
                root, 3, "static_pillbox_h3p6", matrix, 640, model,
                "vision123", "nav456", "batch")
            self.assertEqual(3, len(commands))
            self.assertEqual(3, len({item["scene"] for item in commands}))
            for item in commands:
                command = item["command"]
                self.assertEqual("bash", command[0])
                self.assertTrue(command[1].endswith(
                    "/top_level_scripts/sim_run.sh"))
                self.assertIn("trial_selector:=static_pillbox_h3p6", command)
                self.assertIn("target_detector_imgsz:=640", command)
                self.assertEqual("vision123", item["environment"][
                    "VSIM04_VISION_REVISION"])
                self.assertEqual("nav456", item["environment"][
                    "VSIM04_NAVIGATION_REVISION"])

    def test_long_multi_trial_selector_is_bounded_hashed_and_unique(self):
        selector_a = ",".join([
            "dynamic_pillbox_h1p2_v2p0", "dynamic_pillbox_h3p6_v0p5",
            "dynamic_pillbox_h3p6_v2p0", "dynamic_bridge_h1p2_v2p0",
            "dynamic_panzer_h1p2_v2p0", "static_pillbox_h3p6_a",
        ])
        selector_b = selector_a[:-1] + "b"
        args = ("/tmp/root", 1, None, "/tmp/matrix", 640,
                "/tmp/model", "vision", "nav", "batch")
        command_a = build_repeat_commands(
            args[0], args[1], selector_a, *args[3:])[0]
        command_a_repeat = build_repeat_commands(
            args[0], args[1], selector_a, *args[3:])[0]
        command_b = build_repeat_commands(
            args[0], args[1], selector_b, *args[3:])[0]
        self.assertEqual(command_a["scene"], command_a_repeat["scene"])
        self.assertNotEqual(command_a["scene"], command_b["scene"])
        self.assertLessEqual(len(command_a["scene"]), 79)
        self.assertIn(
            hashlib.sha256(selector_a.encode("utf-8")).hexdigest()[:8],
            command_a["scene"])

    def test_invalid_repeat_parameters_fail_closed(self):
        with self.assertRaises(ValueError):
            build_repeat_commands(
                "/tmp/root", 0, "trial", "/tmp/matrix", 640,
                "/tmp/model", "vision", "nav", "batch")
        with self.assertRaises(ValueError):
            build_repeat_commands(
                "/tmp/root", 1, "trial", "/tmp/matrix", 0,
                "/tmp/model", "vision", "nav", "batch")
        with self.assertRaises(ValueError):
            build_repeat_commands(
                "/tmp/root", 1, "", "/tmp/matrix", 640,
                "/tmp/model", "vision", "nav", "batch")

    def test_formal_passes_aggregate_and_writes_three_outputs(self):
        with tempfile.TemporaryDirectory() as root:
            run1 = _make_run(root, "run1")
            run2 = _make_run(root, "run2", p_selected=False,
                             failure_stage="target_memory_admission")
            aggregate = aggregate_repeat_runs([run1, run2])
            self.assertEqual("PASS", aggregate["status"])
            trial = aggregate["trials"][0]
            self.assertEqual(2, trial["completed_run_count"])
            self.assertEqual(1.0, trial["p_confirm"])
            self.assertEqual(0.5, trial["p_selected"])
            self.assertIsNone(trial["p_interrupt"])
            self.assertEqual(
                1, trial["failure_stage_counts"]["target_memory_admission"])
            output_dir = os.path.join(root, "aggregate")
            paths = write_aggregate_outputs(aggregate, output_dir)
            self.assertEqual(3, len(paths))
            self.assertTrue(all(os.path.isfile(path) for path in paths))

    def test_diagnostic_and_missing_artifact_are_never_pass(self):
        with tempfile.TemporaryDirectory() as root:
            diagnostic = _make_run(
                root, "diagnostic", status="DIAGNOSTIC",
                verdict="DIAGNOSTIC_ONLY")
            incomplete = _make_run(
                root, "incomplete", missing_artifact="frames.csv")
            aggregate = aggregate_repeat_runs([diagnostic, incomplete])
            self.assertEqual("FAIL", aggregate["status"])
            self.assertFalse(aggregate["is_gate_pass"])
            self.assertEqual(
                ["DIAGNOSTIC_ONLY", "FAIL"],
                [source["source_verdict"]
                 for source in aggregate["source_runs"]])
            self.assertIn(
                "artifact_set_incomplete",
                aggregate["source_runs"][1]["errors"])
            self.assertIn(
                "frames.csv",
                aggregate["source_runs"][1]["missing_artifacts"])

    def test_nonterminal_summary_is_never_pass(self):
        with tempfile.TemporaryDirectory() as root:
            run_dir = _make_run(
                root, "nonterminal", status="INCOMPLETE", complete=False)
            aggregate = aggregate_repeat_runs([run_dir])
            self.assertEqual("FAIL", aggregate["status"])
            self.assertIn(
                "measurement_not_terminal",
                aggregate["source_runs"][0]["errors"])

    def test_nonzero_execution_and_duplicate_run_fail_closed(self):
        with tempfile.TemporaryDirectory() as root:
            run_dir = _make_run(root, "run")
            aggregate = aggregate_repeat_runs([run_dir], [9])
            self.assertEqual("FAIL", aggregate["status"])
            self.assertIn(
                "sim_run_exit_nonzero:9",
                aggregate["source_runs"][0]["errors"])
            duplicate = aggregate_repeat_runs([run_dir, run_dir])
            self.assertEqual("FAIL", duplicate["status"])
            self.assertIn(
                "duplicate_run_dir", duplicate["source_runs"][1]["errors"])


if __name__ == "__main__":
    unittest.main()
