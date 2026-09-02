#!/usr/bin/env python3
"""Validate D50 and emit pure trajectory/association planning artifacts."""

import argparse
import json
import os
import sys

from uav_vision_eval.vsim04_d_matrix import (
    load_d50_runtime_matrix,
    write_d50_dry_run,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument("--list-runtime-supported", action="store_true")
    args = parser.parse_args()
    if args.list_runtime_supported:
        matrix = load_d50_runtime_matrix(os.path.abspath(args.matrix))
        for trial in matrix["trials"]:
            if trial["d50_runtime_status"] == "READY_FOR_SINGLE_SMOKE":
                print(trial["trial_id"])
        return 0
    if not args.output_dir:
        parser.error("--output-dir is required unless listing runtime support")
    summary = write_d50_dry_run(
        os.path.abspath(args.matrix), os.path.abspath(args.output_dir))
    print(json.dumps({
        "output_dir": os.path.abspath(args.output_dir),
        "status": summary["status"],
        "trial_count": summary["trial_count"],
        "pairwise_complete": summary["pairwise_coverage"]["complete"],
        "gazebo_execution_status": summary["gazebo_execution_status"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
