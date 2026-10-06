#!/usr/bin/env python3
"""Offline SD-card logger profile builder for PX4 30e763b. Never accesses a board.

Input: official source checkout with exact HEAD, or --source-files directory of
the three official logger files (explicit externally fetched files). Output is a
local staging directory, never the SD card. Existing output is refused.
"""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import re
import shutil
import subprocess

REVISION = "30e763b6780061d70a14894e3e8b06e6a656f9b8"
SOURCE_URL = "https://github.com/PX4/PX4-Autopilot/blob/" + REVISION
# FMUv6C historical logs select primary EKF instance 1. Every actual estimator
# instance must be retained; never infer 'primary=0 only' from vehicle outputs.
EXTRA = {
    "vehicle_visual_odometry": 0, "vehicle_odometry": 20,
    "vehicle_attitude": 20, "vehicle_local_position": 20,
    "estimator_aid_src_ev_yaw": 20, "estimator_aid_src_ev_hgt": 20,
    "estimator_aid_src_baro_hgt": 20, "estimator_aid_src_rng_hgt": 100,
    "estimator_aid_src_gnss_hgt": 100, "estimator_aid_src_gnss_yaw": 100,
    "estimator_aid_src_mag": 100, "estimator_aid_src_ev_pos": 20,
    "estimator_aid_src_ev_vel": 20,
}


def default_entries(source):
    """Expand exact DEFAULT profile's non-constrained, non-HITL hardware branch."""
    body = source.split("void LoggedTopics::add_default_topics()", 1)[1]
    body = body.split("void LoggedTopics::add_high_rate_topics()", 1)[0]
    body = body.split("// SYS_HITL:", 1)[0]
    body = re.sub(r"#if CONSTRAINED_MEMORY.*?#else", "", body, flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)
    result = OrderedDict()
    calls = re.findall(r'(add_(?:optional_)?topic(?:_multi)?)\("([a-z0-9_]+)"([^;]*)\);', body)
    if len(calls) < 140:
        raise ValueError("unexpected historical default topic function")
    for method, name, arguments in calls:
        args = [a.strip() for a in arguments.split(",") if a.strip()]
        interval = int(args[0]) if args else 0
        # ORB_MULTI_MAX_INSTANCES is 10 on non-constrained FMUv6C (uORB.h).
        instances = (6 if len(args) > 1 and args[1] == "MAX_ESTIMATOR_INSTANCES"
                     else int(args[1]) if len(args) > 1 else 10) if method.endswith("multi") else 1
        for instance in range(instances):
            result[(name, instance)] = {"interval_ms": interval, "optional": "optional" in method}
    return result


def build_profile(source, estimator_instances=2):
    if not 1 <= estimator_instances <= 6:
        raise ValueError("estimator instances must be in [1, 6]")
    defaults = default_entries(source)
    # A text file has no optional-subscription syntax. Full six-instance default
    # expansion alone may exceed MAX_TOPICS_NUM; retain all topic names and all
    # sensor/control default instances, bound optional estimator slots explicitly.
    profile = OrderedDict((key, value["interval_ms"]) for key, value in defaults.items()
        if not key[0].startswith(("estimator_", "yaw_estimator_")) or key[1] < estimator_instances)
    for name, interval in EXTRA.items():
        instances = estimator_instances if name.startswith("estimator_") else 1
        for instance in range(instances):
            profile[(name, instance)] = interval
    if len(profile) > 251:  # reserve four mission-profile slots (worst case)
        raise ValueError("%s subscriptions exceeds 251 (+4 mission); reduce ONLY after checking actual instances" % len(profile))
    text = "# PX4 " + REVISION + "\n# Complete DEFAULT names + diagnostics; explicit instances, interval ms.\n"
    text += "# Estimator slots: %s; verify actual instances on ground before use.\n" % estimator_instances
    text += "".join("%s %d %d\n" % (name, interval, instance) for (name, instance), interval in profile.items())
    manifest = {"firmware_revision": REVISION, "source": SOURCE_URL + "/src/modules/logger/logged_topics.cpp",
        "hardware": "FMUv6C / non-constrained / SYS_HITL=0; no SITL-only defaults",
        "logger_max_subscriptions": 255, "mission_slots_reserved": 4,
        "subscription_count": len(profile), "estimator_instances": estimator_instances,
        "default_names": sorted({name for name, _ in defaults}),
        "default_expanded_subscription_count": len(defaults),
        "bounded_default_slots": [{"topic": name, "instance": instance} for name, instance in defaults if (name, instance) not in profile],
        "additional_or_faster": EXTRA,
        "required_ground_check": "actual EKF instances must fit estimator_instances; SDLOG_PROFILE other bits replaced too",
        "file_parser": "topic interval_ms instance; explicitly include instance to avoid all-instance expansion"}
    return text, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--source", type=Path, help="official PX4 git checkout at exact revision")
    source.add_argument("--source-files", type=Path, help="directory with official logger.cpp/logged_topics.cpp/logged_topics.h")
    parser.add_argument("--output", type=Path, required=True, help="new LOCAL staging directory, not mounted SD")
    parser.add_argument("--estimator-instances", type=int, default=2)
    args = parser.parse_args(argv)
    if args.source:
        head = subprocess.check_output(["git", "-C", str(args.source), "rev-parse", "HEAD"], text=True).strip()
        if head != REVISION:
            parser.error("PX4 source HEAD differs from historical firmware; newer local checkout is not authority")
        root = args.source / "src/modules/logger"
    else:
        root = args.source_files
    logger = (root / "logger.cpp").read_text()
    header = (root / "logged_topics.h").read_text()
    if 'strcmp(argv[0], "on")' not in logger or 'strcmp(argv[0], "topic")' in logger:
        parser.error("unexpected logger command implementation")
    if not re.search(r"MAX_TOPICS_NUM\s*=\s*255", header):
        parser.error("unexpected topic capacity")
    text, manifest = build_profile((root / "logged_topics.cpp").read_text(), args.estimator_instances)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "logger_topics.txt").write_text(text, encoding="utf-8")
    (args.output / "profile_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    # Preserve original authoritative default function for offline review/tests.
    for name in ("logger.cpp", "logged_topics.cpp", "logged_topics.h"):
        shutil.copyfile(root / name, args.output / name)
    print(json.dumps({"local_staging": str(args.output), "subscriptions": manifest["subscription_count"],
                      "board_access": False}))


if __name__ == "__main__":
    main()
