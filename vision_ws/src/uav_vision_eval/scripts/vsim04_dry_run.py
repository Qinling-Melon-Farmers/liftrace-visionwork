#!/usr/bin/env python3
"""Expand the V-SIM-04 matrix and emit all artifact schemas without ROS/Gazebo."""

import argparse
import json
import os
import sys

from uav_vision_eval.vsim04_metrics import dry_run_artifacts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-path", default="")
    parser.add_argument("--extrinsic-profile", default="")
    parser.add_argument("--vision-revision", default="unknown")
    parser.add_argument("--navigation-revision", default="unknown")
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--selected-max-age", type=float, default=0.5)
    args = parser.parse_args()
    summary = dry_run_artifacts(
        args.matrix, args.output_dir,
        metadata={
            "model": {"path": args.model_path},
            "extrinsic_profile": args.extrinsic_profile,
            "revisions": {
                "vision": args.vision_revision,
                "navigation": args.navigation_revision,
            },
            "thresholds": {
                "confirm_frames": args.confirm_frames,
                "selected_max_age_sec": args.selected_max_age,
            },
            "camera_info": None,
        })
    print(json.dumps({
        "output_dir": os.path.abspath(args.output_dir),
        "trial_count": summary["trial_count"],
        "status": summary["status"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
