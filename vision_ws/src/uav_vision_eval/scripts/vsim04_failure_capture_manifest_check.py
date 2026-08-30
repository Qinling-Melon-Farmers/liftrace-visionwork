#!/usr/bin/env python3
"""Fail non-zero unless a V-SIM failure-capture dataset is complete."""

import json
import os
import sys

from uav_vision_eval.failure_capture import validate_capture_manifest


def main(arguments=None):
    values = list(sys.argv[1:] if arguments is None else arguments)
    if len(values) != 1:
        print("usage: vsim04_failure_capture_manifest_check.py MANIFEST",
              file=sys.stderr)
        return 2
    manifest_path = os.path.abspath(values[0])
    try:
        with open(manifest_path, "r", encoding="utf-8") as stream:
            payload = json.load(stream)
        validate_capture_manifest(payload, os.path.dirname(manifest_path))
    except Exception as error:
        print("V-SIM-04 failure capture INVALID: {}".format(error),
              file=sys.stderr)
        return 8
    print("V-SIM-04 failure capture manifest/files PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
