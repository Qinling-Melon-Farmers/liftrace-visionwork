#!/usr/bin/env python3
"""重复运行单下视基线与斜下辅助 shadow，并生成统一的无 GUI A/B 报告。"""

import argparse
import datetime
import glob
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time


def newest_run(logs_dir, scene, started_at):
    candidates = [
        Path(path) for path in glob.glob(str(logs_dir / (scene + "_*")))
        if Path(path).stat().st_mtime >= started_at - 2.0
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) \
        if candidates else None


def load_json(path):
    if not path or not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def median(values):
    return statistics.median(values) if values else None


def population_std(values):
    return statistics.pstdev(values) if len(values) >= 2 else None


def fmt(value, suffix=""):
    return "N/A" if value is None else "{:.2f}{}".format(value, suffix)


def extract_record(case, channel):
    summary = case.get("summary")
    if not summary:
        return None
    channel_metrics = summary.get("channels", {}).get(channel, {})
    metrics = summary.get("metrics", {})
    checks = summary.get("checks", {})
    return {
        "run_index": case["run_index"],
        "return_code": case["return_code"],
        "route_completed": bool(checks.get("route_completed")),
        "all_required_detected": bool(
            channel_metrics.get("all_required_detected")),
        "required_coverage_rate": float(
            channel_metrics.get("required_coverage_rate", 0.0)),
        "distinct_required_targets": int(
            channel_metrics.get("distinct_required_targets", 0)),
        "time_to_all_required_sec":
            channel_metrics.get("time_to_all_required_sec"),
        "time_to_three_required_sec":
            channel_metrics.get("time_to_three_required_sec"),
        "p90_map_error_xy_m": channel_metrics.get("p90_map_error_xy_m"),
        "route_mission_elapsed_sec": metrics.get(
            "route_mission_elapsed_sec"),
        "route_search_active_sec": metrics.get("route_search_active_sec"),
        "forbidden_aux_publishers": summary.get(
            "forbidden_aux_publishers", []),
    }


def summarize_records(records):
    valid = [record for record in records if record is not None]
    search_times = [
        float(record["time_to_all_required_sec"]) for record in valid
        if record["time_to_all_required_sec"] is not None
    ]
    route_times = [
        float(record["route_mission_elapsed_sec"]) for record in valid
        if record["route_mission_elapsed_sec"] is not None
    ]
    search_active = [
        float(record["route_search_active_sec"]) for record in valid
        if record["route_search_active_sec"] is not None
    ]
    map_errors = [
        float(record["p90_map_error_xy_m"]) for record in valid
        if record["p90_map_error_xy_m"] is not None
    ]
    requested = len(records)
    return {
        "requested_runs": requested,
        "valid_reports": len(valid),
        "route_success_rate": (
            sum(record["route_completed"] for record in valid) /
            float(requested) if requested else 0.0),
        "all_target_success_rate": (
            sum(record["all_required_detected"] for record in valid) /
            float(requested) if requested else 0.0),
        "mean_required_coverage_rate": (
            sum(record["required_coverage_rate"] for record in valid) /
            float(requested) if requested else 0.0),
        "median_time_to_all_required_sec": median(search_times),
        "std_time_to_all_required_sec": population_std(search_times),
        "median_route_mission_elapsed_sec": median(route_times),
        "std_route_mission_elapsed_sec": population_std(route_times),
        "median_route_search_active_sec": median(search_active),
        "median_p90_map_error_xy_m": median(map_errors),
        "records": valid,
    }


def paired_metrics(baseline_cases, auxiliary_cases):
    baseline_by_index = {
        case["run_index"]: extract_record(case, "downward")
        for case in baseline_cases
    }
    auxiliary_by_index = {
        case["run_index"]: extract_record(case, "auxiliary")
        for case in auxiliary_cases
    }
    auxiliary_down_by_index = {
        case["run_index"]: extract_record(case, "downward")
        for case in auxiliary_cases
    }
    pairs = []
    for run_index in sorted(set(baseline_by_index) & set(auxiliary_by_index)):
        baseline = baseline_by_index[run_index]
        auxiliary = auxiliary_by_index[run_index]
        auxiliary_down = auxiliary_down_by_index[run_index]
        if not baseline or not auxiliary:
            continue
        baseline_search = baseline["time_to_all_required_sec"]
        auxiliary_search = auxiliary["time_to_all_required_sec"]
        baseline_route = baseline["route_mission_elapsed_sec"]
        auxiliary_route = auxiliary["route_mission_elapsed_sec"]
        pairs.append({
            "run_index": run_index,
            "baseline_time_to_all_sec": baseline_search,
            "auxiliary_time_to_all_sec": auxiliary_search,
            "potential_search_saving_sec": (
                None if baseline_search is None or auxiliary_search is None
                else float(baseline_search) - float(auxiliary_search)),
            "baseline_route_elapsed_sec": baseline_route,
            "auxiliary_route_elapsed_sec": auxiliary_route,
            "auxiliary_route_overhead_sec": (
                None if baseline_route is None or auxiliary_route is None
                else float(auxiliary_route) - float(baseline_route)),
            "auxiliary_run_downward_time_to_all_sec": (
                None if auxiliary_down is None else
                auxiliary_down["time_to_all_required_sec"]),
        })
    savings = [
        item["potential_search_saving_sec"] for item in pairs
        if item["potential_search_saving_sec"] is not None
    ]
    overheads = [
        item["auxiliary_route_overhead_sec"] for item in pairs
        if item["auxiliary_route_overhead_sec"] is not None
    ]
    return {
        "complete_search_pairs": len(savings),
        "median_potential_search_saving_sec": median(savings),
        "std_potential_search_saving_sec": population_std(savings),
        "median_auxiliary_route_overhead_sec": median(overheads),
        "pairs": pairs,
    }


def write_report(output_dir, payload):
    report_path = output_dir / "comparison_report.md"
    conditions = payload["conditions"]
    baseline = conditions["baseline_downward"]
    auxiliary = conditions["auxiliary_oblique"]
    auxiliary_down = conditions["auxiliary_run_downward"]
    paired = payload["paired_comparison"]
    lines = [
        "# 全场覆盖下视基线与斜下辅助搜索 A/B 报告",
        "",
        "状态：`{}`；重复次数：`{}`；证据等级：`{}`。".format(
            payload["status"], payload["repeats"],
            payload["evidence_level"]),
        "",
        "本试验中两种模式执行同一条覆盖路线，辅助链只做 shadow。"
        "因此“发现全部目标耗时”可直接比较；路线总耗时差主要反映额外相机、"
        "Gazebo 传感器和第二路推理的资源代价，不代表已经实现导航捷径。",
        "",
        "| 条件 | 路线成功率 | 五类发现成功率 | 平均目标覆盖率 | 全部发现耗时中位数 | 耗时标准差 | 路线耗时中位数 | 地图误差 P90 中位数 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, item in (
            ("原单下视基线", baseline),
            ("55°斜下辅助", auxiliary),
            ("辅助运行中的下视链", auxiliary_down)):
        lines.append(
            "| {} | {:.1%} | {:.1%} | {:.1%} | {} | {} | {} | {} |".format(
                label, item["route_success_rate"],
                item["all_target_success_rate"],
                item["mean_required_coverage_rate"],
                fmt(item["median_time_to_all_required_sec"], " s"),
                fmt(item["std_time_to_all_required_sec"], " s"),
                fmt(item["median_route_mission_elapsed_sec"], " s"),
                fmt(item["median_p90_map_error_xy_m"], " m")))
    lines.extend([
        "",
        "配对运行中，斜下辅助相机相对原下视基线的潜在搜索节省中位数为 "
        "`{}`；加入辅助传感器和第二路笔记本推理后的路线耗时变化中位数为 "
        "`{}`。".format(
            fmt(paired["median_potential_search_saving_sec"], " s"),
            fmt(paired["median_auxiliary_route_overhead_sec"], " s")),
        "",
        "稳定性按重复运行的路线完成率、五类发现成功率和全部发现耗时标准差衡量。"
        "少于 3 次只视为 smoke，不形成稳定性结论。",
        "",
        "所有辅助节点控制输出违规数：`{}`。".format(
            payload["forbidden_aux_publisher_count"]),
    ])
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def wait_for_ros_shutdown(timeout_sec=12.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["rosnode", "list"], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, check=False)
        if result.returncode != 0:
            return True
        time.sleep(1.0)
    return False


def run_case(project_root, logs_dir, model_path, vehicle_sdf, mode,
             angle, run_index, wall_timeout):
    condition = "baseline" if mode == "baseline" else "aux_" + mode
    scene = "oblique_ab_{}_r{:02d}".format(condition, run_index)
    command = [
        "bash", str(project_root / "top_level_scripts/sim_run.sh"), scene,
        "roslaunch", "uav_vision_eval", "oblique_coverage_shadow.launch",
        "gui:=false", "rviz:=false", "start_arming:=true",
        "wall_timeout:={:.1f}".format(wall_timeout),
        "recorder_wall_timeout:={:.1f}".format(wall_timeout + 30.0),
        "projection_mode:=" + ("mono" if mode == "baseline" else mode),
        "aux_angle_deg:={}".format(angle),
        "enable_aux:=" + ("false" if mode == "baseline" else "true"),
        "evaluation_mode:=" + (
            "baseline" if mode == "baseline" else "aux_shadow"),
        "vehicle_sdf:=" + str(vehicle_sdf),
        "target_model_path:=" + str(model_path),
    ]
    environment = os.environ.copy()
    environment["SIM_NO_RECORD"] = "1"
    environment.setdefault(
        "VISION_PYTHON", "/home/xhj/miniconda3/envs/rl_drone/bin/python")
    environment["UAV_VISION_MODEL_PATH"] = str(model_path)
    started_at = datetime.datetime.now().timestamp()
    result = subprocess.run(command, env=environment, check=False)
    run_dir = newest_run(logs_dir, scene, started_at)
    summary = load_json(
        run_dir / "aux_shadow_summary.json" if run_dir else None)
    return {
        "condition": condition,
        "run_index": run_index,
        "return_code": result.returncode,
        "run_dir": str(run_dir) if run_dir else "",
        "summary": summary,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--angle", type=int, choices=(45, 55, 60), default=55)
    parser.add_argument("--mode", choices=("mono", "depth"), default="mono")
    parser.add_argument("--wall-timeout", type=float, default=900.0)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be at least 1")

    project_root = Path(os.path.realpath(__file__)).parents[4]
    package_root = Path(os.path.realpath(__file__)).parents[1]
    logs_dir = project_root / "logs"
    model_path = Path(os.environ.get(
        "UAV_VISION_MODEL_PATH",
        str(project_root /
            "vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt")))
    if not model_path.is_file():
        parser.error("merged_standard model is not readable: %s" % model_path)
    if subprocess.run(
            ["rosnode", "list"], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, check=False).returncode == 0:
        parser.error("an existing ROS master is active; wait for the other run")

    base_sdf = (
        project_root / "patrol_uav_ws-patrol_planner/src/patrol_control/models/"
        "iris_mid360_downward_camera/model.sdf")
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = logs_dir / ("oblique_search_ab_{}_{}deg_{}".format(
        args.mode, args.angle, timestamp))
    output_dir.mkdir(parents=True, exist_ok=False)
    derived_sdf = Path("/tmp/iris_mid360_downward_aux_ab_{}_{}.sdf".format(
        args.angle, os.getpid()))
    generator = package_root / "scripts/generate_oblique_vehicle_sdf.py"
    subprocess.run([
        sys.executable, str(generator), "--base-sdf", str(base_sdf),
        "--output", str(derived_sdf), "--angle-deg", str(args.angle),
        "--sensor-mode", args.mode,
    ], check=True)

    cases = []
    try:
        for run_index in range(1, args.repeats + 1):
            # 交替 A/B 顺序，降低温度和缓存随时间单向漂移的影响。
            order = ("baseline", args.mode) if run_index % 2 else \
                (args.mode, "baseline")
            for mode in order:
                if not wait_for_ros_shutdown():
                    raise RuntimeError(
                        "previous ROS run did not shut down; no process was killed")
                case = run_case(
                    project_root, logs_dir, model_path,
                    base_sdf if mode == "baseline" else derived_sdf,
                    mode, args.angle, run_index, args.wall_timeout)
                cases.append(case)
                (output_dir / "cases.json").write_text(
                    json.dumps(cases, ensure_ascii=False, indent=2,
                               sort_keys=True) + "\n",
                    encoding="utf-8")
    finally:
        try:
            derived_sdf.unlink()
        except FileNotFoundError:
            pass

    baseline_cases = [case for case in cases
                      if case["condition"] == "baseline"]
    auxiliary_cases = [case for case in cases
                       if case["condition"] != "baseline"]
    baseline_records = [extract_record(case, "downward")
                        for case in baseline_cases]
    auxiliary_records = [extract_record(case, "auxiliary")
                         for case in auxiliary_cases]
    auxiliary_down_records = [extract_record(case, "downward")
                              for case in auxiliary_cases]
    forbidden_count = sum(
        len(record["forbidden_aux_publishers"])
        for record in auxiliary_records if record is not None)
    complete_reports = sum(case.get("summary") is not None for case in cases)
    payload = {
        "status": (
            "COMPARISON_COMPLETE" if complete_reports == 2 * args.repeats
            else "INCOMPLETE"),
        "evidence_level": "multi_run" if args.repeats >= 3 else "smoke",
        "repeats": args.repeats,
        "angle_deg": args.angle,
        "auxiliary_projection_mode": args.mode,
        "model": str(model_path),
        "comparison_boundary": (
            "same coverage route; auxiliary is shadow-only and never redirects navigation"),
        "conditions": {
            "baseline_downward": summarize_records(baseline_records),
            "auxiliary_oblique": summarize_records(auxiliary_records),
            "auxiliary_run_downward": summarize_records(
                auxiliary_down_records),
        },
        "paired_comparison": paired_metrics(
            baseline_cases, auxiliary_cases),
        "forbidden_aux_publisher_count": forbidden_count,
        "cases": [{key: value for key, value in case.items()
                   if key != "summary"} for case in cases],
    }
    (output_dir / "comparison_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    write_report(output_dir, payload)
    print(str(output_dir))
    return 0 if payload["status"] == "COMPARISON_COMPLETE" else 2


if __name__ == "__main__":
    sys.exit(main())
