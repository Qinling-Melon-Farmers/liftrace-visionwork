#!/usr/bin/env python3
"""串行运行斜下辅助相机固定姿态矩阵；每个 Gazebo case 均经 sim_run.sh。"""

import argparse
import csv
import datetime
import glob
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import yaml


STANDARD_IDS = ("pillbox_1", "bridge_1", "tank_1", "tent_1", "panzer_1")


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def choose_camera_pose(target, height, angle_deg):
    distance = height / math.tan(math.radians(angle_deg))
    candidates = []
    for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
        x = target[0] - distance * math.cos(yaw)
        y = target[1] - distance * math.sin(yaw)
        if -4.6 <= x <= 4.6 and -4.6 <= y <= 4.6:
            margin = min(4.6 - abs(x), 4.6 - abs(y))
            candidates.append((margin, x, y, yaw))
    if not candidates:
        raise RuntimeError("no in-field camera pose for target=%s" % (target,))
    _margin, x, y, yaw = max(candidates)
    return x, y, yaw


def load_targets(catalog_path):
    with open(catalog_path, "r", encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)
    return {
        item["target_id"]: {
            "class_name": item["class_name"],
            "center": item["fallback_center_world"],
        }
        for item in catalog["targets"]}


def newest_run(logs_dir, scene, started_at):
    candidates = [Path(path) for path in glob.glob(
        str(logs_dir / (scene + "_*"))) if Path(path).stat().st_mtime >= started_at - 2.0]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def case_rows(case):
    path = Path(case["run_dir"]) / "metrics" / "frames.csv"
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def summarize(cases):
    groups = {}
    for mode in ("mono", "depth"):
        selected = [case for case in cases
                    if case["mode"] == mode and case["angle_deg"] == 55.0]
        tp = fp = fn = 0
        map_errors = []
        projection_observations = 0
        projection_valid = 0
        range_sources = {}
        depth_fraction_means = []
        for case in selected:
            for row in case_rows(case):
                status = row["match_status"]
                standard_truth = row.get("truth_class") in {
                    "bridge", "panzer", "pillbox", "tent", "tank"}
                if status == "true_positive" and standard_truth:
                    tp += 1
                    if row.get("map_error_xy"):
                        map_errors.append(float(row["map_error_xy"]))
                elif status == "false_negative" and standard_truth:
                    fn += 1
                elif status == "false_positive":
                    fp += 1
            projection_path = (Path(case["run_dir"]) / "metrics" /
                               "projection_summary.json")
            if projection_path.exists():
                projection = json.loads(projection_path.read_text(encoding="utf-8"))
                projection_observations += int(projection.get("observations", 0))
                projection_valid += int(projection.get("valid_observations", 0))
                for source, count in projection.get("range_sources", {}).items():
                    range_sources[source] = range_sources.get(source, 0) + int(count)
                if projection.get("mean_depth_valid_fraction") is not None:
                    depth_fraction_means.append(
                        float(projection["mean_depth_valid_fraction"]))
        precision = tp / float(tp + fp) if tp + fp else 1.0
        recall = tp / float(tp + fn) if tp + fn else 0.0
        p90 = percentile(map_errors, 0.90)
        groups[mode] = {
            "cases": len(selected),
            "true_positive": tp,
            "false_positive": fp,
            "false_negative": fn,
            "precision": precision,
            "standard_recall": recall,
            "p90_map_error_xy_m": p90,
            "projection_valid_rate": (
                projection_valid / float(projection_observations)
                if projection_observations else None),
            "range_sources": dict(sorted(range_sources.items())),
            "mean_depth_valid_fraction": (
                sum(depth_fraction_means) / float(len(depth_fraction_means))
                if depth_fraction_means else None),
            "fixed_gate_pass": (
                bool(selected) and recall >= 0.70 and precision >= 0.90 and
                p90 is not None and p90 <= 0.80),
        }
    return groups


def write_report(output_dir, payload):
    summary_path = output_dir / "matrix_summary.json"
    report_path = output_dir / "matrix_report.md"
    summary_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    lines = [
        "# 斜下辅助相机固定姿态矩阵报告",
        "",
        "运行档位：`{}`。".format(payload["profile"]),
        "",
        "本报告只判定固定姿态识别与粗定位；提前发现和下视交接必须由 coverage shadow 另行判定。",
        "",
        "| 路线 | 55° case | Precision | 标准靶 Recall | 地图误差 P90 | 当前判定 |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for mode in ("mono", "depth"):
        item = payload["fixed_55deg"][mode]
        p90 = item["p90_map_error_xy_m"]
        lines.append(
            "| {} | {} | {:.3f} | {:.3f} | {} | {} |".format(
                mode, item["cases"], item["precision"],
                item["standard_recall"],
                "N/A" if p90 is None else "{:.3f} m".format(p90),
                ("PASS" if item["fixed_gate_pass"] else "FAIL")
                if payload["profile"] == "full" else
                ("SMOKE OK" if item["fixed_gate_pass"] else "SMOKE FAIL")))
    lines.extend([
        "",
        "总体状态：`{}`。".format(payload["status"]),
        "",
        "不得仅凭本报告决定导航接入或硬件选型。",
    ])
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("smoke", "full"), default="full")
    parser.add_argument("--wall-timeout", type=float, default=40.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--continue-on-failure", action="store_true")
    args = parser.parse_args()

    project_root = Path(os.path.realpath(__file__)).parents[4]
    package_root = Path(os.path.realpath(__file__)).parents[1]
    logs_dir = project_root / "logs"
    catalog_path = package_root / "config/sim_target_catalog_toudi4.yaml"
    scenario_dir = package_root / "config/scenarios"
    model_path = Path(os.environ.get(
        "UAV_VISION_MODEL_PATH",
        str(project_root /
            "vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt")))
    if not model_path.is_file():
        parser.error("merged_standard model is not readable: %s" % model_path)

    targets = load_targets(catalog_path)
    if args.profile == "smoke":
        angles = (55.0,)
        heights = (2.0,)
        scene_ids = ("tent_1", "background")
    else:
        angles = (45.0, 55.0, 60.0)
        heights = (2.0, 3.0, 3.5)
        scene_ids = STANDARD_IDS + ("red_cross_1", "background")
    modes = ("mono", "depth")

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = logs_dir / ("oblique_fixed_matrix_" + timestamp)
    output_dir.mkdir(parents=True, exist_ok=False)
    cases = []
    environment = os.environ.copy()
    environment["SIM_NO_RECORD"] = "1"
    environment.setdefault("VISION_PYTHON", "/home/xhj/miniconda3/envs/rl_drone/bin/python")
    environment["UAV_VISION_MODEL_PATH"] = str(model_path)

    for mode in modes:
        for angle in angles:
            for height in heights:
                for scene_id in scene_ids:
                    spawn_red_cross = scene_id == "red_cross_1"
                    if scene_id == "background":
                        class_name = "background"
                        camera_x, camera_y, camera_yaw = 4.2, 4.2, math.pi / 4.0
                        scenario = scenario_dir / "background_only.yaml"
                    else:
                        target = targets[scene_id]
                        class_name = target["class_name"]
                        camera_x, camera_y, camera_yaw = choose_camera_pose(
                            target["center"], height, angle)
                        scenario = scenario_dir / (
                            "red_cross_dynamic.yaml" if spawn_red_cross
                            else "existing_targets.yaml")
                    scene = "oblq_{}_a{:02d}_h{:02d}_{}".format(
                        mode, int(angle), int(round(height * 10)), class_name)
                    command = [
                        "bash", str(project_root / "top_level_scripts/sim_run.sh"), scene,
                        "roslaunch", "uav_vision_eval", "oblique_static_eval.launch",
                        "gui:=" + ("true" if args.gui else "false"),
                        "projection_mode:=" + mode,
                        "camera_x:={:.6f}".format(camera_x),
                        "camera_y:={:.6f}".format(camera_y),
                        "camera_z:={:.3f}".format(height),
                        "camera_pitch_rad:={:.12f}".format(math.radians(angle)),
                        "camera_yaw_rad:={:.12f}".format(camera_yaw),
                        "scenario_file:=" + str(scenario),
                        "spawn_red_cross:=" + ("true" if spawn_red_cross else "false"),
                        "target_model_path:=" + str(model_path),
                        "wall_timeout_sec:={:.1f}".format(args.wall_timeout),
                    ]
                    started = datetime.datetime.now().timestamp()
                    result = subprocess.run(command, env=environment, check=False)
                    run_dir = newest_run(logs_dir, scene, started)
                    case = {
                        "mode": mode, "angle_deg": angle, "height_m": height,
                        "scene_id": scene_id, "class_name": class_name,
                        "camera_pose": [camera_x, camera_y, height, camera_yaw],
                        "return_code": result.returncode,
                        "run_dir": str(run_dir) if run_dir else "",
                    }
                    cases.append(case)
                    (output_dir / "cases.json").write_text(
                        json.dumps(cases, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
                    if result.returncode != 0 and not args.continue_on_failure:
                        print("case failed: %s" % scene, file=sys.stderr)
                        payload = {
                            "profile": args.profile,
                            "status": "INCOMPLETE_CASE_FAILED",
                            "cases": cases,
                            "fixed_55deg": summarize(cases),
                        }
                        write_report(output_dir, payload)
                        return result.returncode

    fixed = summarize(cases)
    fixed_any = any(item["fixed_gate_pass"] for item in fixed.values())
    if args.profile == "smoke":
        status = ("SMOKE_PASS_FULL_MATRIX_PENDING" if fixed_any
                  else "SMOKE_FAIL")
    else:
        status = ("FIXED_MATRIX_PASS_SHADOW_PENDING" if fixed_any
                  else "FIXED_MATRIX_FAIL")
    payload = {
        "profile": args.profile,
        "status": status,
        "formal_gate_eligible": args.profile == "full",
        "model": str(model_path),
        "cases": cases,
        "fixed_55deg": fixed,
        "thresholds": {
            "standard_recall_min": 0.70,
            "precision_min": 0.90,
            "p90_map_error_xy_max_m": 0.80,
        },
    }
    write_report(output_dir, payload)
    print(str(output_dir))
    return 0 if fixed_any or args.profile == "smoke" else 2


if __name__ == "__main__":
    sys.exit(main())
