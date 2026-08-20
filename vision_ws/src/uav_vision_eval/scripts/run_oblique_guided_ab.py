#!/usr/bin/env python3
"""重复运行原覆盖与 OpenCV 蓝环引导搜索，汇总实际路线裁剪收益。"""

import argparse
import datetime
import glob
import json
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


def load_report(run_dir):
    path = run_dir / "gate_status.json" if run_dir else None
    if path is None or not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def seeded_case(run_dir_text, strategy):
    """将已经完成的同配置实跑作为第一组，避免重复消耗仿真时间。"""
    run_dir = Path(run_dir_text).expanduser().resolve()
    report = load_report(run_dir)
    if report is None:
        raise ValueError("seed run has no readable gate_status.json: %s" % run_dir)
    return {
        "strategy": strategy,
        "run_index": 1,
        "return_code": 0 if report.get("status") == "PASS" else 2,
        "run_dir": str(run_dir),
        "report": report,
        "seeded": True,
    }
def median(values):
    return statistics.median(values) if values else None


def std(values):
    return statistics.pstdev(values) if len(values) >= 2 else None


def fmt(value, suffix=""):
    return "N/A" if value is None else "{:.2f}{}".format(value, suffix)


def summarize(cases):
    reports = [case["report"] for case in cases if case.get("report")]
    terminal = [report for report in reports
                if report.get("status") in ("PASS", "FAIL")]
    passed = [report for report in terminal if report.get("status") == "PASS"]
    search_times = [float(report["search_elapsed_sec"]) for report in terminal
                    if report.get("search_elapsed_sec") is not None]
    mission_times = [float(report["mission_elapsed_sec"]) for report in terminal
                     if report.get("mission_elapsed_sec") is not None]
    wall_times = [float(report["wall_elapsed_sec"]) for report in terminal
                  if report.get("wall_elapsed_sec") is not None]
    path_lengths = [float(report["path_length_m"]) for report in terminal
                    if report.get("path_length_m") is not None]
    requested = len(cases)
    candidate_states = [
        report.get("aux_candidate_state_counts", {}) for report in terminal]
    return {
        "requested_runs": requested,
        "valid_reports": len(terminal),
        "success_rate": len(passed) / float(requested) if requested else 0.0,
        "mean_downward_confirmed": (
            sum(int(report.get("downward_confirmed_count", 0))
                for report in terminal) / float(len(terminal))
            if terminal else 0.0),
        "median_search_elapsed_sec": median(search_times),
        "std_search_elapsed_sec": std(search_times),
        "median_mission_elapsed_sec": median(mission_times),
        "median_wall_elapsed_sec": median(wall_times),
        "median_path_length_m": median(path_lengths),
        "fallback_rate": (
            sum(bool(report.get("fallback_used")) for report in terminal) /
            float(len(terminal)) if terminal else 0.0),
        "aux_activation_rate": (
            sum(bool(report.get("aux_triggered")) for report in terminal) /
            float(len(terminal)) if terminal else 0.0),
        "mean_fallback_count": (
            sum(int(report.get(
                "fallback_count", bool(report.get("fallback_used"))))
                for report in terminal) / float(len(terminal))
            if terminal else 0.0),
        "mean_goal_progress_timeout_count": (
            sum(int(report.get("goal_progress_timeout_count", 0))
                for report in terminal) / float(len(terminal))
            if terminal else 0.0),
        "mean_aux_confirmed_count": (
            sum(int(states.get("CONFIRMED", 0))
                for states in candidate_states) / float(len(terminal))
            if terminal else 0.0),
        "mean_aux_rejected_count": (
            sum(int(states.get("REJECTED", 0))
                for states in candidate_states) / float(len(terminal))
            if terminal else 0.0),
        "median_aux_candidate_count": median([
            int(report.get("aux_candidate_count", 0)) for report in terminal]),
    }


def paired_comparison(baseline_cases, guided_cases):
    baseline = {case["run_index"]: case.get("report")
                for case in baseline_cases}
    guided = {case["run_index"]: case.get("report")
              for case in guided_cases}
    pairs = []
    for run_index in sorted(set(baseline) & set(guided)):
        left = baseline[run_index]
        right = guided[run_index]
        if not left or not right:
            continue
        left_time = left.get("search_elapsed_sec")
        right_time = right.get("search_elapsed_sec")
        left_path = left.get("path_length_m")
        right_path = right.get("path_length_m")
        pairs.append({
            "run_index": run_index,
            "baseline_status": left.get("status"),
            "guided_status": right.get("status"),
            "baseline_search_sec": left_time,
            "guided_search_sec": right_time,
            "search_saving_sec": (
                None if left_time is None or right_time is None else
                float(left_time) - float(right_time)),
            "search_reduction_rate": (
                None if left_time in (None, 0) or right_time is None else
                (float(left_time) - float(right_time)) / float(left_time)),
            "baseline_path_m": left_path,
            "guided_path_m": right_path,
            "path_saving_m": (
                None if left_path is None or right_path is None else
                float(left_path) - float(right_path)),
        })
    successful = [
        item for item in pairs
        if item["baseline_status"] == "PASS" and
        item["guided_status"] == "PASS" and
        item["search_saving_sec"] is not None
    ]
    return {
        "paired_runs": len(pairs),
        "successful_pairs": len(successful),
        "median_search_saving_sec": median([
            item["search_saving_sec"] for item in successful]),
        "median_search_reduction_rate": median([
            item["search_reduction_rate"] for item in successful]),
        "median_path_saving_m": median([
            item["path_saving_m"] for item in successful
            if item["path_saving_m"] is not None]),
        "pairs": pairs,
    }


def write_report(output_dir, payload):
    baseline = payload["conditions"]["baseline"]
    guided = payload["conditions"]["guided_opencv_blue"]
    paired = payload["paired"]
    lines = [
        "# 原全覆盖与 OpenCV 蓝环辅助搜索 A/B 报告",
        "",
        "状态：`{}`；重复次数：`{}`；证据等级：`{}`。".format(
            payload["status"], payload["repeats"],
            payload["evidence_level"]),
        "",
        "基线走原 12 点覆盖；guided 先走四点稀疏扫描，仅在仍缺目标时才访问斜下蓝环"
        "粗候选。两者均由下视链确认五类并返航，不执行投递和降落。",
        "",
        "| 条件 | 成功率 | 平均下视确认数 | 搜索耗时中位数 | 搜索耗时标准差 | 路径中位数 | 墙钟中位数 | 辅助激活率 | fallback 率 | 辅助交接确认/拒绝 | 目标无进展超时 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        "| 原 12 点全覆盖 | {:.1%} | {:.2f} | {} | {} | {} | {} | {:.1%} | {:.1%} | {:.2f}/{:.2f} | {:.2f} |".format(
            baseline["success_rate"], baseline["mean_downward_confirmed"],
            fmt(baseline["median_search_elapsed_sec"], " s"),
            fmt(baseline["std_search_elapsed_sec"], " s"),
            fmt(baseline["median_path_length_m"], " m"),
            fmt(baseline["median_wall_elapsed_sec"], " s"),
            baseline["aux_activation_rate"],
            baseline["fallback_rate"],
            baseline["mean_aux_confirmed_count"],
            baseline["mean_aux_rejected_count"],
            baseline["mean_goal_progress_timeout_count"]),
        "| 55° 稀疏扫描 + OpenCV 候选兜底 | {:.1%} | {:.2f} | {} | {} | {} | {} | {:.1%} | {:.1%} | {:.2f}/{:.2f} | {:.2f} |".format(
            guided["success_rate"], guided["mean_downward_confirmed"],
            fmt(guided["median_search_elapsed_sec"], " s"),
            fmt(guided["std_search_elapsed_sec"], " s"),
            fmt(guided["median_path_length_m"], " m"),
            fmt(guided["median_wall_elapsed_sec"], " s"),
            guided["aux_activation_rate"],
            guided["fallback_rate"],
            guided["mean_aux_confirmed_count"],
            guided["mean_aux_rejected_count"],
            guided["mean_goal_progress_timeout_count"]),
        "",
        "成功配对的搜索节省中位数：`{}`；相对缩短：`{}`；路径节省：`{}`。".format(
            fmt(paired["median_search_saving_sec"], " s"),
            ("N/A" if paired["median_search_reduction_rate"] is None else
             "{:.1%}".format(paired["median_search_reduction_rate"])),
            fmt(paired["median_path_saving_m"], " m")),
        "",
        "收益归因：`{}`。{}".format(
            payload["benefit_attribution"],
            ("本轮辅助访问激活率为 0，节省只能归因于稀疏覆盖路线，不能归因于辅助相机。"
             if payload["benefit_attribution"] == "sparse_route_only" else
             "至少一轮实际激活了辅助访问，仍需结合逐轮交接结果解释收益。")),
        "",
        "OpenCV 蓝环只回答“这里像标准投放区”，不能给出 tent/tank 等图案类别，"
        "也不能覆盖只有黑色外环的红十字；类别和最终地图点仍由下视生产链确认。",
    ]
    (output_dir / "comparison_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


def persist_summary(output_dir, cases, args):
    """每个子运行后都落盘，外层超时也保留可读的部分结论。"""
    baseline_cases = [case for case in cases
                      if case["strategy"] == "baseline"]
    guided_cases = [case for case in cases
                    if case["strategy"] == "guided"]
    complete_reports = sum(
        case.get("report", {}).get("status") in ("PASS", "FAIL")
        for case in cases)
    release_violations = sum(
        len(case["report"].get("raw_servo_calls", [])) +
        len(case["report"].get("release_results", []))
        for case in cases if case.get("report"))
    comparison_complete = complete_reports == 2 * args.repeats
    baseline_summary = summarize(baseline_cases)
    guided_summary = summarize(guided_cases)
    benefit_attribution = (
        "sparse_route_only"
        if guided_summary["aux_activation_rate"] == 0.0 else
        "mixed_sparse_and_aux")
    payload = {
        "status": (
            "COMPARISON_COMPLETE" if comparison_complete else "INCOMPLETE"),
        "evidence_level": (
            "multi_run" if comparison_complete and args.repeats >= 3
            else "partial"),
        "repeats": args.repeats,
        "angle_deg": args.angle,
        "auxiliary_algorithm": "opencv_hsv_blue_ellipse",
        "interrupt_policy": {
            "semantic_classes": ["red_cross", "tank"],
            "anonymous_interrupt_count": args.anonymous_interrupt_count,
            "anonymous_behavior": (
                "queue_only" if args.anonymous_interrupt_count == 0 else
                "interrupt_at_count"),
        },
        "conditions": {
            "baseline": baseline_summary,
            "guided_opencv_blue": guided_summary,
        },
        "benefit_attribution": benefit_attribution,
        "paired": paired_comparison(baseline_cases, guided_cases),
        "release_violation_count": release_violations,
        "cases": [{key: value for key, value in case.items()
                   if key != "report"} for case in cases],
    }
    (output_dir / "comparison_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2,
                   sort_keys=True) + "\n", encoding="utf-8")
    write_report(output_dir, payload)
    return payload


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


def run_case(project_root, logs_dir, model_path, vehicle_sdf, strategy,
             angle, run_index, wall_timeout, anonymous_interrupt_count):
    scene = "oblique_guided_ab_{}_r{:02d}".format(strategy, run_index)
    command = [
        "bash", str(project_root / "top_level_scripts/sim_run.sh"), scene,
        "roslaunch", "uav_vision_eval", "oblique_guided_search.launch",
        "gui:=false", "rviz:=false", "start_arming:=true",
        "strategy:=" + strategy,
        "enable_aux:=" + ("true" if strategy == "guided" else "false"),
        "wall_timeout:={:.1f}".format(wall_timeout),
        "aux_angle_deg:={}".format(angle),
        "anonymous_interrupt_count:={}".format(
            anonymous_interrupt_count),
        "vehicle_sdf:=" + str(vehicle_sdf),
        "target_model_path:=" + str(model_path),
    ]
    environment = os.environ.copy()
    environment["SIM_NO_RECORD"] = "1"
    environment["UAV_VISION_MODEL_PATH"] = str(model_path)
    started_at = datetime.datetime.now().timestamp()
    result = subprocess.run(command, env=environment, check=False)
    run_dir = newest_run(logs_dir, scene, started_at)
    return {
        "strategy": strategy,
        "run_index": run_index,
        "return_code": result.returncode,
        "run_dir": str(run_dir) if run_dir else "",
        "report": load_report(run_dir),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--angle", type=int, choices=(45, 55, 60), default=55)
    parser.add_argument("--wall-timeout", type=float, default=1000.0)
    parser.add_argument(
        "--anonymous-interrupt-count", type=int, default=0,
        help="匿名蓝环达到该数量时中断；0 表示只入队，默认推荐")
    parser.add_argument(
        "--seed-baseline-run", default="",
        help="已完成的基线 run 目录；需与 --seed-guided-run 同时提供")
    parser.add_argument(
        "--seed-guided-run", default="",
        help="已完成的辅助 run 目录；需与 --seed-baseline-run 同时提供")
    parser.add_argument(
        "--summarize-cases", default="",
        help="只重新汇总指定 cases.json，不启动 ROS/Gazebo")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be at least 1")
    if args.anonymous_interrupt_count < 0:
        parser.error("anonymous interrupt count cannot be negative")
    if bool(args.seed_baseline_run) != bool(args.seed_guided_run):
        parser.error("both seed run directories must be provided together")

    if args.summarize_cases:
        case_file = Path(args.summarize_cases).expanduser().resolve()
        try:
            cases = json.loads(case_file.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            parser.error("cannot read cases file: %s" % exc)
        payload = persist_summary(case_file.parent, cases, args)
        print(str(case_file.parent))
        return 0 if payload["status"] == "COMPARISON_COMPLETE" else 2

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
        parser.error("an existing ROS master is active")

    base_sdf = (
        project_root / "patrol_uav_ws-patrol_planner/src/patrol_control/models/"
        "iris_mid360_downward_camera/model.sdf")
    derived_sdf = Path("/tmp/iris_mid360_downward_aux_guided_{}_{}.sdf".format(
        args.angle, os.getpid()))
    subprocess.run([
        sys.executable,
        str(package_root / "scripts/generate_oblique_vehicle_sdf.py"),
        "--base-sdf", str(base_sdf), "--output", str(derived_sdf),
        "--angle-deg", str(args.angle), "--sensor-mode", "mono",
    ], check=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = logs_dir / (
        "oblique_guided_search_ab_{}deg_{}".format(args.angle, timestamp))
    output_dir.mkdir(parents=True, exist_ok=False)

    cases = []
    first_new_index = 1
    if args.seed_baseline_run:
        try:
            cases.extend([
                seeded_case(args.seed_baseline_run, "baseline"),
                seeded_case(args.seed_guided_run, "guided"),
            ])
        except ValueError as exc:
            parser.error(str(exc))
        first_new_index = 2
        (output_dir / "cases.json").write_text(
            json.dumps(cases, ensure_ascii=False, indent=2,
                       sort_keys=True) + "\n", encoding="utf-8")
        persist_summary(output_dir, cases, args)
    try:
        for run_index in range(first_new_index, args.repeats + 1):
            order = ("baseline", "guided") if run_index % 2 else \
                ("guided", "baseline")
            for strategy in order:
                if not wait_for_ros_shutdown():
                    raise RuntimeError("previous ROS run did not shut down")
                case = run_case(
                    project_root, logs_dir, model_path,
                    base_sdf if strategy == "baseline" else derived_sdf,
                    strategy, args.angle, run_index, args.wall_timeout,
                    args.anonymous_interrupt_count)
                cases.append(case)
                (output_dir / "cases.json").write_text(
                    json.dumps(cases, ensure_ascii=False, indent=2,
                               sort_keys=True) + "\n", encoding="utf-8")
                persist_summary(output_dir, cases, args)
    finally:
        try:
            derived_sdf.unlink()
        except FileNotFoundError:
            pass

    payload = persist_summary(output_dir, cases, args)
    print(str(output_dir))
    return 0 if payload["status"] == "COMPARISON_COMPLETE" else 2


if __name__ == "__main__":
    sys.exit(main())
