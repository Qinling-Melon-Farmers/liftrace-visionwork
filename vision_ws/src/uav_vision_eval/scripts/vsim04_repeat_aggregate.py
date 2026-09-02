#!/usr/bin/env python3
"""Aggregate existing V-SIM-04 run directories without Gazebo."""

import argparse
import json
import sys

from uav_vision_eval.repeat_runs import (
    aggregate_repeat_runs,
    write_aggregate_outputs,
)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("run_dirs", nargs="+")
    args = parser.parse_args(argv)
    aggregate = aggregate_repeat_runs(args.run_dirs)
    paths = write_aggregate_outputs(aggregate, args.output_dir)
    print(json.dumps({
        "status": aggregate["status"],
        "artifacts": paths,
    }, sort_keys=True))
    return 0 if aggregate["is_gate_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
