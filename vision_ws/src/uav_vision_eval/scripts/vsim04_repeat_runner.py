#!/usr/bin/env python3
"""Run isolated V-SIM-04 repeats and aggregate their artifacts."""

import argparse
import datetime
import json
import os
import re
import sys

from uav_vision_eval.repeat_runs import (
    aggregate_repeat_runs,
    append_unfinished_results,
    atomic_write_json,
    build_repeat_commands,
    create_new_batch_output,
    execute_repeat_commands,
    resolve_batch_output,
    write_aggregate_outputs,
)
from uav_vision_eval.vsim04_metrics import (
    load_trial_matrix,
    select_trial_matrix,
)


REVISION_PATTERN = re.compile(r"^[0-9a-fA-F]{7,40}$")


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", required=True, type=int)
    parser.add_argument("--trial-selector", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--imgsz", required=True, type=int)
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--vision-revision", required=True)
    parser.add_argument("--navigation-revision", required=True)
    parser.add_argument("--project-root", default=os.environ.get(
        "PROJECT_ROOT", os.getcwd()))
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--batch-id", default="")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def _preflight(args):
    project_root = os.path.realpath(os.path.abspath(args.project_root))
    matrix_path = os.path.realpath(os.path.abspath(args.matrix))
    model_path = os.path.realpath(os.path.abspath(args.model_path))
    if not os.path.isfile(matrix_path):
        raise ValueError("matrix is not a file: " + matrix_path)
    if not os.path.isfile(model_path):
        raise ValueError("model path is not a file: " + model_path)
    if int(args.repeats) <= 0:
        raise ValueError("repeats must be positive")
    if int(args.imgsz) <= 0:
        raise ValueError("imgsz must be positive")
    for name, revision in (
            ("vision", args.vision_revision),
            ("navigation", args.navigation_revision)):
        if REVISION_PATTERN.fullmatch(str(revision).strip()) is None:
            raise ValueError(name + " revision must be a 7-40 digit git SHA")
    selected = select_trial_matrix(
        load_trial_matrix(matrix_path), args.trial_selector)
    selector = list(selected["trial_selector"])
    if not selector:
        raise ValueError("repeat runner requires a diagnostic trial selector")
    canonical_selector = ",".join(selector)
    default_batch = "{}-p{}".format(
        datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"), os.getpid())
    batch_token, output_dir = resolve_batch_output(
        project_root, args.batch_id or default_batch, args.output_dir)
    if os.path.exists(output_dir):
        raise ValueError("repeat batch output already exists: " + output_dir)
    expected_configuration = {
        "vision_revision": str(args.vision_revision).strip(),
        "navigation_revision": str(args.navigation_revision).strip(),
        "model_path": model_path,
        "imgsz": int(args.imgsz),
        "class_profile": selected["class_profile"],
        "matrix_file": matrix_path,
        "seed": int(selected["seed"]),
        "trial_selector": selector,
    }
    return {
        "project_root": project_root,
        "matrix_path": matrix_path,
        "model_path": model_path,
        "batch_token": batch_token,
        "output_dir": output_dir,
        "canonical_selector": canonical_selector,
        "expected_configuration": expected_configuration,
    }


def _checkpoint_payload(status, commands, results, preflight, error="",
                        aggregate=None, artifacts=None):
    completed = {int(result["repeat_index"]): result for result in results}
    repetitions = []
    for item in commands:
        result = completed.get(int(item["repeat_index"]))
        repetitions.append({
            "repeat_index": int(item["repeat_index"]),
            "scene": item["scene"],
            "command": list(item["command"]),
            "environment": dict(item["environment"]),
            "state": ("PENDING" if result is None else
                      result.get("state", "FINISHED")),
            "exit_code": None if result is None else result.get("exit_code"),
            "run_dir": None if result is None else result.get("run_dir"),
            "binding_error": ("" if result is None else
                              result.get("binding_error", "")),
            "execution_error": ("" if result is None else
                                result.get("execution_error", "")),
        })
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04-repeat-batch",
        "status": status,
        "error": error,
        "batch_token": preflight["batch_token"],
        "output_dir": preflight["output_dir"],
        "expected_configuration": preflight["expected_configuration"],
        "repeats_are_multi_seed": False,
        "repetitions": repetitions,
        "aggregate_status": None if aggregate is None else aggregate["status"],
        "aggregate_artifacts": list(artifacts or []),
    }


def main(argv=None):
    args = parse_args(argv)
    try:
        preflight = _preflight(args)
    except ValueError as error:
        raise SystemExit(str(error))
    commands = build_repeat_commands(
        preflight["project_root"], args.repeats,
        preflight["canonical_selector"], preflight["matrix_path"],
        args.imgsz, preflight["model_path"], args.vision_revision,
        args.navigation_revision, preflight["batch_token"])
    if args.dry_run:
        print(json.dumps({
            "dry_run": True,
            "output_dir": preflight["output_dir"],
            "expected_configuration": preflight["expected_configuration"],
            "repeats_are_multi_seed": False,
            "commands": commands,
        }, indent=2, sort_keys=True))
        return 0

    try:
        create_new_batch_output(preflight["output_dir"])
    except ValueError as error:
        raise SystemExit(str(error))
    checkpoint_path = os.path.join(
        preflight["output_dir"], "batch_checkpoint.json")
    results = []
    atomic_write_json(checkpoint_path, _checkpoint_payload(
        "RUNNING", commands, results, preflight))

    def checkpoint_result(_result, current_results):
        atomic_write_json(checkpoint_path, _checkpoint_payload(
            "RUNNING", commands, current_results, preflight))

    interrupted_error = ""
    try:
        execute_repeat_commands(
            commands, preflight["project_root"], on_result=checkpoint_result,
            results=results)
    except KeyboardInterrupt:
        interrupted_error = "keyboard_interrupt"
    except Exception as error:  # Preserve completed evidence on orchestration errors.
        interrupted_error = "orchestration_error:" + str(error)

    append_unfinished_results(commands, results, interrupted_error)
    run_dirs = [result["run_dir"] or os.path.join(
        preflight["project_root"], "logs", result["scene"] + "_MISSING")
                for result in results]
    aggregate = aggregate_repeat_runs(
        run_dirs, [result["exit_code"] for result in results],
        [result.get("binding_error", "") for result in results],
        [result.get("execution_error", "") for result in results],
        preflight["expected_configuration"])
    aggregate["repeat_execution"] = [{
        "repeat_index": result["repeat_index"],
        "scene": result["scene"],
        "exit_code": result["exit_code"],
        "run_dir": result["run_dir"],
        "binding_error": result.get("binding_error", ""),
        "execution_error": result.get("execution_error", ""),
    } for result in results]
    paths = write_aggregate_outputs(aggregate, preflight["output_dir"])
    final_status = "INTERRUPTED" if interrupted_error else "FINALIZED"
    atomic_write_json(checkpoint_path, _checkpoint_payload(
        final_status, commands, results, preflight, interrupted_error,
        aggregate, paths))
    print(json.dumps({
        "status": aggregate["status"],
        "batch_status": final_status,
        "output_dir": preflight["output_dir"],
        "artifacts": paths,
    }, sort_keys=True))
    if interrupted_error:
        return 130
    return 0 if aggregate["is_gate_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
