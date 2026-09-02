#!/usr/bin/env python3
"""No-Gazebo tests for V-SIM-04 repeat command and aggregation contracts."""

import json
import hashlib
import os
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

from uav_vision_eval.repeat_runs import (
    REQUIRED_ARTIFACTS,
    aggregate_repeat_runs,
    append_unfinished_results,
    atomic_write_json,
    build_repeat_commands,
    create_new_batch_output,
    execute_repeat_commands,
    resolve_batch_output,
    write_aggregate_outputs,
)


def _make_run(root, name, status="MEASURED", verdict="PASS",
              complete=True, failure_stage="", p_confirm=True,
              p_selected=True, missing_artifact="", configuration=None):
    run_dir = os.path.join(root, name)
    vsim_dir = os.path.join(run_dir, "vsim04")
    os.makedirs(vsim_dir)
    for artifact in REQUIRED_ARTIFACTS:
        if artifact == missing_artifact or artifact in (
                "summary.json", "manifest.json"):
            continue
        with open(os.path.join(vsim_dir, artifact), "w", encoding="utf-8") as stream:
            stream.write("fixture\n")
    manifest_configuration = {
        "seed": 11,
        "class_profile": "r2026",
        "matrix_file": os.path.join(root, "matrix.yaml"),
        "model": {"path": os.path.join(root, "model.pt")},
        "thresholds": {"detector_imgsz": 640},
        "revisions": {"vision": "abcdef1", "navigation": "1234567"},
        "evaluation_design": {
            "trial_selector": ["static_pillbox_h3p6"],
        },
    }
    for key, value in (configuration or {}).items():
        if key == "vision_revision":
            manifest_configuration["revisions"]["vision"] = value
        elif key == "navigation_revision":
            manifest_configuration["revisions"]["navigation"] = value
        elif key == "model_path":
            manifest_configuration["model"]["path"] = value
        elif key == "imgsz":
            manifest_configuration["thresholds"]["detector_imgsz"] = value
        elif key == "trial_selector":
            manifest_configuration["evaluation_design"]["trial_selector"] = value
        else:
            manifest_configuration[key] = value
    if missing_artifact != "manifest.json":
        with open(os.path.join(vsim_dir, "manifest.json"), "w",
                  encoding="utf-8") as stream:
            json.dump(manifest_configuration, stream)
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

    def test_batch_output_is_bounded_local_and_refuses_escape(self):
        with tempfile.TemporaryDirectory() as root:
            os.makedirs(os.path.join(root, "logs"))
            token, output_dir = resolve_batch_output(
                root, "../../../../outside")
            self.assertLessEqual(len(token), 48)
            self.assertEqual(
                os.path.realpath(os.path.join(root, "logs")),
                os.path.commonpath([
                    os.path.realpath(os.path.join(root, "logs")), output_dir]))
            with self.assertRaises(ValueError):
                resolve_batch_output(
                    root, "batch", os.path.join(root, "outside"))
            create_new_batch_output(output_dir)
            with self.assertRaises(ValueError):
                create_new_batch_output(output_dir)

    def test_runner_rejects_duplicate_selector_before_dry_run(self):
        project_root = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "../../../.."))
        runner = os.path.join(
            project_root, "vision_ws/src/uav_vision_eval/scripts",
            "vsim04_repeat_runner.py")
        matrix = os.path.join(
            project_root, "vision_ws/src/uav_vision_eval/config",
            "vsim04_operating_surface_matrix.yaml")
        with tempfile.NamedTemporaryFile(suffix=".pt") as model:
            environment = os.environ.copy()
            source_path = os.path.join(
                project_root, "vision_ws/src/uav_vision_eval/src")
            vision_source_path = os.path.join(
                project_root, "vision_ws/src/uav_vision/src")
            environment["PYTHONPATH"] = os.pathsep.join([
                source_path, vision_source_path,
                environment.get("PYTHONPATH", ""),
            ])
            completed = subprocess.run([
                sys.executable, runner,
                "--project-root", project_root,
                "--repeats", "2",
                "--trial-selector",
                "static_pillbox_h3p6,static_pillbox_h3p6",
                "--matrix", matrix,
                "--imgsz", "640",
                "--model-path", model.name,
                "--vision-revision", "abcdef1",
                "--navigation-revision", "1234567",
                "--batch-id", "duplicate-selector-test",
                "--dry-run",
            ], env=environment, check=False, capture_output=True, text=True)
            self.assertNotEqual(0, completed.returncode)
            self.assertIn("duplicate", completed.stderr)

    @mock.patch("uav_vision_eval.repeat_runs.subprocess.run")
    def test_run_directory_uses_unique_set_difference(self, run_mock):
        with tempfile.TemporaryDirectory() as root:
            logs_dir = os.path.join(root, "logs")
            os.makedirs(logs_dir)
            scene = "vsim04_diag_repeat_trial_batch_r01"
            old_dir = os.path.join(logs_dir, scene + "_old")
            new_dir = os.path.join(logs_dir, scene + "_new")
            os.makedirs(old_dir)

            def create_new(*_args, **_kwargs):
                os.makedirs(new_dir)
                os.utime(old_dir, (4102444800, 4102444800))
                return SimpleNamespace(returncode=0)

            run_mock.side_effect = create_new
            command = {
                "repeat_index": 1,
                "scene": scene,
                "command": ["ignored"],
                "environment": {},
            }
            results = execute_repeat_commands([command], root)
            self.assertEqual(os.path.realpath(new_dir), results[0]["run_dir"])
            self.assertEqual("", results[0]["binding_error"])

        with tempfile.TemporaryDirectory() as root:
            logs_dir = os.path.join(root, "logs")
            os.makedirs(logs_dir)
            scene = "vsim04_diag_repeat_trial_batch_r02"

            def create_ambiguous(*_args, **_kwargs):
                os.makedirs(os.path.join(logs_dir, scene + "_a"))
                os.makedirs(os.path.join(logs_dir, scene + "_b"))
                return SimpleNamespace(returncode=0)

            run_mock.side_effect = create_ambiguous
            command = {
                "repeat_index": 2,
                "scene": scene,
                "command": ["ignored"],
                "environment": {},
            }
            results = execute_repeat_commands([command], root)
            self.assertIsNone(results[0]["run_dir"])
            self.assertEqual("new_run_dir_count:2",
                             results[0]["binding_error"])

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

    def test_complete_nonzero_diagnostic_keeps_trial_metrics(self):
        with tempfile.TemporaryDirectory() as root:
            diagnostic = _make_run(
                root, "diagnostic", status="DIAGNOSTIC",
                verdict="DIAGNOSTIC_ONLY", p_selected=False,
                failure_stage="target_memory_admission")
            aggregate = aggregate_repeat_runs([diagnostic], [1])
            self.assertEqual("FAIL", aggregate["status"])
            self.assertTrue(
                aggregate["source_runs"][0]["measurement_eligible"])
            self.assertFalse(
                aggregate["source_runs"][0]["source_pass_eligible"])
            self.assertEqual(1, aggregate["trials"][0]["completed_run_count"])
            self.assertEqual(0.0, aggregate["trials"][0]["p_selected"])

    def test_manifest_configuration_mismatch_fails_aggregation(self):
        with tempfile.TemporaryDirectory() as root:
            run1 = _make_run(root, "run1")
            run2 = _make_run(root, "run2", configuration={
                "navigation_revision": "7654321",
            })
            aggregate = aggregate_repeat_runs([run1, run2])
            self.assertEqual("FAIL", aggregate["status"])
            self.assertFalse(aggregate["configuration_consistent"])
            self.assertTrue(all(
                "configuration_mismatch" in source["errors"]
                for source in aggregate["source_runs"]))

    def test_checkpoint_write_is_atomic_and_replaces_content(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "batch_checkpoint.json")
            atomic_write_json(path, {"status": "RUNNING", "count": 1})
            atomic_write_json(path, {"status": "FINALIZED", "count": 2})
            with open(path, "r", encoding="utf-8") as stream:
                payload = json.load(stream)
            self.assertEqual("FINALIZED", payload["status"])
            self.assertEqual(2, payload["count"])
            self.assertFalse(any(name.startswith("batch_checkpoint.json.tmp")
                                 for name in os.listdir(root)))

    def test_interrupted_batch_marks_every_unfinished_repeat(self):
        commands = [{
            "repeat_index": index,
            "scene": "scene{}".format(index),
            "command": ["ignored"],
            "environment": {},
        } for index in range(1, 4)]
        results = [dict(
            commands[0], exit_code=0, run_dir="/tmp/run1",
            binding_error="", execution_error="")]
        append_unfinished_results(commands, results, "keyboard_interrupt")
        self.assertEqual(3, len(results))
        self.assertEqual([0, 130, 130], [item["exit_code"] for item in results])
        self.assertTrue(all(
            item["binding_error"] == "run_not_executed"
            for item in results[1:]))
        self.assertTrue(all(
            item["execution_error"] == "keyboard_interrupt"
            for item in results[1:]))
        self.assertTrue(all(
            item["state"] == "UNFINISHED" for item in results[1:]))

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
