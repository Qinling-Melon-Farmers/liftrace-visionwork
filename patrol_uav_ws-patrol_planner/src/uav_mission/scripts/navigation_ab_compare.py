#!/usr/bin/env python3
"""Compare two 90-second fixed-route metrics files and emit Gate JSON."""

import argparse
import json
import os
import sys

from uav_mission.navigation_ab_policy import compare_navigation_ab


def _read(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _write(path, payload):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument(
        "--output", default=os.path.join(
            os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
    args = parser.parse_args()
    try:
        report = compare_navigation_ab(
            _read(args.baseline), _read(args.candidate))
    except Exception as exc:
        report = {
            "gate": "navigation_feature_ab",
            "status": "FAIL",
            "reason": "comparison_error",
            "error": str(exc),
            "promote_candidate": False,
        }
    report["baseline_path"] = os.path.abspath(args.baseline)
    report["candidate_path"] = os.path.abspath(args.candidate)
    _write(args.output, report)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
