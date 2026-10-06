#!/usr/bin/env python3
"""Select existing measured site profiles; delegate all flight logic unchanged."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FOLDERS = {
    "visual_interrupt": "01_visual_interrupt",
    "high_view": "02_high_view_revisit",
    "landing": "03_h_landing",
    "corridor_landing": "04_corridor_landing",
    "low_multi": "05_low_multi",
    "high_priority": "06_high_priority",
    "memory_only": "07_memory_only",
    "full_mission": "08_full_mission",
    "high_speed_capture": "09_high_speed_capture",
}
ALIASES = {
    "1": "visual_interrupt", "single": "visual_interrupt",
    "2": "low_multi", "multi": "low_multi",
    "3": "memory_only", "memory": "memory_only",
    "4": "high_view", "revisit": "high_view",
    "5": "high_priority", "priority": "high_priority",
    "6": "high_speed_capture", "capture": "high_speed_capture",
    "h": "landing", "high_view_revisit": "high_view",
    **{folder: name for name, folder in FOLDERS.items()},
}
HIGH_TRIALS = {"high_view", "high_priority", "memory_only", "full_mission", "high_speed_capture"}
DROP_TRIALS = {"visual_interrupt", "high_view", "low_multi", "high_priority", "full_mission"}
SITE_FILES = {
    "landing": "h_landing_test_area.yaml",
    "corridor_landing": "corridor_landing_test_area.yaml",
    "full_mission": "full_mission_test_area.yaml",
}


def command(argv, root=ROOT):
    defaults = json.loads((HERE / "defaults.json").read_text(encoding="utf-8"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", choices=sorted(set(FOLDERS) | set(ALIASES)))
    parser.add_argument("mode", choices=("preview", "flight"), nargs="?", default="preview")
    parser.add_argument("--survey-pattern", choices=("rect", "rectangle", "snake2", "snake3"))
    parser.add_argument("--check-config", action="store_true")
    parser.add_argument("--print-command", action="store_true", help="Print only; never starts ROS")
    parser.add_argument("--real-release", action="store_true", help="Explicitly select the existing guarded real release chain")
    parser.add_argument("--capture-speed", type=float, choices=(0.5, 1.0, 1.2))
    parser.add_argument("--capture-lighting", choices=("normal", "dim", "unspecified"))
    parser.add_argument("--resume-survey", choices=("on", "off"))
    parser.add_argument("--model")
    parser.add_argument("--metadata")
    args = parser.parse_args(argv)
    trial = ALIASES.get(args.trial, args.trial)
    if args.survey_pattern and trial not in HIGH_TRIALS:
        parser.error("survey patterns are only available for 02/06/07/08/09")
    if args.real_release and (args.mode != "flight" or trial not in DROP_TRIALS):
        parser.error("real release requires an explicitly selected delivery flight")
    if (args.capture_speed is not None or args.capture_lighting) and trial != "high_speed_capture":
        parser.error("capture options require high_speed_capture / 6")
    if args.resume_survey and trial not in {"high_priority", "full_mission"}:
        parser.error("survey resume requires high_priority / 5 or full_mission")
    site = root / defaults["site_directory"] / SITE_FILES.get(trial, "test_area.yaml")
    result = ["bash", str(root / "deployment/board_trials_4x4" / FOLDERS[trial] / "start.sh"),
              args.mode, "--site-config", str(site)]
    if defaults["motion_optimized"]:
        result.append("--motion-optimized")
    if trial in HIGH_TRIALS:
        pattern = args.survey_pattern or defaults["survey_pattern"]
        result += ["--survey-pattern", "rectangle" if pattern == "rect" else pattern]
    if trial == "high_speed_capture":
        speed = defaults["capture_speed"] if args.capture_speed is None else args.capture_speed
        result += ["--capture-speed", str(speed)]
    for option in ("capture_lighting", "resume_survey", "model", "metadata"):
        value = getattr(args, option)
        if value is not None:
            result += ["--" + option.replace("_", "-"), value]
    if args.real_release:
        result.append("--real-release")
    if args.check_config:
        result.append("--check-config")
    return args, result


def main(argv=None):
    args, result = command(argv)
    if args.print_command:
        print(shlex.join(result))
        return 0
    # No arming, mode, PWM or ROS commands are introduced by this wrapper.
    return subprocess.call(result)


if __name__ == "__main__":
    raise SystemExit(main())
