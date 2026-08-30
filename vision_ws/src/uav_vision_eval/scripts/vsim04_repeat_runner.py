#!/usr/bin/env python3
"""Run isolated V-SIM-04 repeats and aggregate their artifacts."""

import argparse
import datetime
import json
import os
import sys

from uav_vision_eval.repeat_runs import (
    aggregate_repeat_runs,
    build_repeat_commands,
    execute_repeat_commands,
    write_aggregate_outputs,
)


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


def main(argv=None):
    args = parse_args(argv)
    project_root = os.path.abspath(args.project_root)
    matrix = os.path.abspath(args.matrix)
    model_path = os.path.abspath(args.model_path)
    if not os.path.isfile(matrix):
        raise SystemExit("matrix is not a file: " + matrix)
    if not os.path.isfile(model_path):
        raise SystemExit("model path is not a file: " + model_path)
    batch_id = args.batch_id or "{}-p{}".format(
        datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"), os.getpid())
    output_dir = os.path.abspath(args.output_dir or os.path.join(
        project_root, "logs", "vsim04_repeat_aggregate_" + batch_id))
    commands = build_repeat_commands(
        project_root, args.repeats, args.trial_selector, matrix, args.imgsz,
        model_path, args.vision_revision, args.navigation_revision, batch_id)
    if args.dry_run:
        print(json.dumps({
            "dry_run": True,
            "output_dir": output_dir,
            "commands": commands,
        }, indent=2, sort_keys=True))
        return 0

    results = execute_repeat_commands(commands, project_root)
    run_dirs = [result["run_dir"] or os.path.join(
        project_root, "logs", result["scene"] + "_MISSING")
                for result in results]
    aggregate = aggregate_repeat_runs(
        run_dirs, [result["exit_code"] for result in results])
    aggregate["repeat_execution"] = [{
        "repeat_index": result["repeat_index"],
        "scene": result["scene"],
        "exit_code": result["exit_code"],
        "run_dir": result["run_dir"],
    } for result in results]
    paths = write_aggregate_outputs(aggregate, output_dir)
    print(json.dumps({
        "status": aggregate["status"],
        "output_dir": output_dir,
        "artifacts": paths,
    }, sort_keys=True))
    return 0 if aggregate["is_gate_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
