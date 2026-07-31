#!/usr/bin/env python3
"""Aggregate fixed-seed uav_vision_eval runs into one auditable report."""

import argparse
from collections import Counter, defaultdict
import csv
import json
import os
import sys


def _load_matrix(path):
    with open(path, "r", encoding="utf-8") as stream:
        rows = [line for line in stream if not line.lstrip().startswith("#")]
    return list(csv.DictReader(rows, delimiter="\t"))


def _display(value):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return "{:.4f}".format(value)
    return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()

    matrix_path = os.path.abspath(args.matrix)
    run_dir = os.path.abspath(args.run_dir)
    records = []
    missing = []
    for definition in _load_matrix(matrix_path):
        seed = int(definition["seed"])
        seed_dir = os.path.join(run_dir, "seed_{:02d}_{}".format(
            seed, definition["scenario"]))
        summary_path = os.path.join(seed_dir, "summary.json")
        manifest_path = os.path.join(seed_dir, "manifest.json")
        launch_log_path = os.path.join(seed_dir, "launch.log")
        if not os.path.isfile(summary_path) or not os.path.isfile(manifest_path):
            missing.append(seed)
            continue
        with open(summary_path, "r", encoding="utf-8") as stream:
            summary = json.load(stream)
        with open(manifest_path, "r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        launch_log = ""
        if os.path.isfile(launch_log_path):
            with open(launch_log_path, "r", encoding="utf-8", errors="replace") as stream:
                launch_log = stream.read()
        infrastructure_ok = (
            "Spawn status: SpawnModel: Successfully spawned entity" in launch_log and
            "Spawn service failed" not in launch_log and
            "run_id on parameter server does not match" not in launch_log
        )
        manifest_seed = int(manifest.get("evaluation_seed", -1))
        pose = manifest.get("camera_pose", {})
        pose_matches = (
            manifest_seed == seed and
            all(abs(float(pose.get(axis, 1.0e9)) - float(definition["camera_" + axis])) < 1.0e-5
                for axis in ("x", "y", "z")) and
            abs(float(pose.get("yaw", 1.0e9)) - float(definition["camera_yaw"])) < 1.0e-5
        )
        records.append({
            "seed": seed,
            "scenario": definition["scenario"],
            "gate_profile": definition["gate_profile"],
            "pose_matches_manifest": pose_matches,
            "infrastructure_ok": infrastructure_ok,
            "passed": (bool(summary.get("passed", False)) and pose_matches and
                       infrastructure_ok),
            "metrics": summary["metrics"],
            "checks": summary["checks"],
            "input_coverage": summary.get("input_coverage", {}),
            "summary": summary_path,
        })

    tp = sum(item["metrics"]["true_positive"] for item in records)
    fp = sum(item["metrics"]["false_positive"] for item in records)
    fn = sum(item["metrics"]["false_negative"] for item in records)
    precision = tp / float(tp + fp) if tp + fp else 1.0
    recall = tp / float(tp + fn) if tp + fn else 1.0
    failed_seeds = [item["seed"] for item in records if not item["passed"]]
    failure_checks = Counter()
    grouped = defaultdict(list)
    for item in records:
        grouped[item["scenario"]].append(item)
        failure_checks.update(
            name for name, passed in item["checks"].items() if not passed)
    scenario_aggregates = {}
    for scenario, items in sorted(grouped.items()):
        scenario_tp = sum(item["metrics"]["true_positive"] for item in items)
        scenario_fp = sum(item["metrics"]["false_positive"] for item in items)
        scenario_fn = sum(item["metrics"]["false_negative"] for item in items)
        latencies = [item["metrics"].get("p95_latency_ms") for item in items
                     if item["metrics"].get("p95_latency_ms") is not None]
        scenario_aggregates[scenario] = {
            "seed_count": len(items),
            "passed_seed_count": sum(item["passed"] for item in items),
            "true_positive": scenario_tp,
            "false_positive": scenario_fp,
            "false_negative": scenario_fn,
            "precision": scenario_tp / float(scenario_tp + scenario_fp)
            if scenario_tp + scenario_fp else 1.0,
            "recall": scenario_tp / float(scenario_tp + scenario_fn)
            if scenario_tp + scenario_fn else 1.0,
            "max_seed_p95_latency_ms": max(latencies) if latencies else None,
            "zero_true_positive_seeds": [
                item["seed"] for item in items
                if item["metrics"]["true_positive"] == 0
            ],
            "mean_input_coverage": (
                sum(item["metrics"].get("input_coverage") or 0.0 for item in items) /
                float(len(items))
            ),
        }
    result = {
        "matrix": matrix_path,
        "run_dir": run_dir,
        "expected_seed_count": len(_load_matrix(matrix_path)),
        "completed_seed_count": len(records),
        "missing_seeds": missing,
        "failed_seeds": failed_seeds,
        "failure_check_counts": dict(sorted(failure_checks.items())),
        "passed": not missing and not failed_seeds,
        "aggregate": {
            "true_positive": tp,
            "false_positive": fp,
            "false_negative": fn,
            "precision": precision,
            "recall": recall,
        },
        "scenario_aggregates": scenario_aggregates,
        "seeds": records,
    }
    summary_output = os.path.join(run_dir, "matrix_summary.json")
    report_output = os.path.join(run_dir, "matrix_report.md")
    os.makedirs(run_dir, exist_ok=True)
    with open(summary_output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    lines = [
        "# UAV Vision Fixed-Seed Matrix Report", "",
        "- Result: **{}**".format("PASS" if result["passed"] else "FAIL"),
        "- Completed: `{}/{}`".format(len(records), result["expected_seed_count"]),
        "- Aggregate precision/recall: `{:.4f}` / `{:.4f}`".format(precision, recall),
        "- Missing seeds: `{}`".format(missing),
        "- Failed seeds: `{}`".format(failed_seeds), "",
        "## Scenario aggregates", "",
        "| Scenario | Seeds passed | Precision | Recall | Mean input coverage | FP | Max seed P95 ms | Zero-TP seeds |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for scenario, aggregate in scenario_aggregates.items():
        lines.append(
            "| {} | {}/{} | {} | {} | {} | {} | {} | `{}` |".format(
                scenario, aggregate["passed_seed_count"], aggregate["seed_count"],
                _display(aggregate["precision"]), _display(aggregate["recall"]),
                _display(aggregate["mean_input_coverage"]),
                aggregate["false_positive"],
                _display(aggregate["max_seed_p95_latency_ms"]),
                aggregate["zero_true_positive_seeds"],
            )
        )
    lines.extend([
        "", "- Failed check counts: `{}`".format(dict(sorted(failure_checks.items()))), "",
        "## Per-seed results", "",
        "| Seed | Scenario | Infra | Pose | Result | Precision | Recall | Coverage | FP | P95 px | P95 map m | P95 ms |",
        "|---:|---|:---:|:---:|:---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for item in records:
        metrics = item["metrics"]
        lines.append(
            "| {seed} | {scenario} | {infra} | {pose} | {result} | {precision} | {recall} | {coverage} | {fp} | "
            "{pixel} | {map_error} | {latency} |".format(
                seed=item["seed"], scenario=item["scenario"],
                infra="PASS" if item["infrastructure_ok"] else "FAIL",
                pose="PASS" if item["pose_matches_manifest"] else "FAIL",
                result="PASS" if item["passed"] else "FAIL",
                precision=_display(metrics.get("precision")),
                recall=_display(metrics.get("recall")),
                coverage=_display(metrics.get("input_coverage")),
                fp=_display(metrics.get("false_positive")),
                pixel=_display(metrics.get("p95_pixel_error")),
                map_error=_display(metrics.get("p95_map_error_xy")),
                latency=_display(metrics.get("p95_latency_ms")),
            )
        )
    with open(report_output, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    print("matrix: {} ({}/{})".format(
        "PASS" if result["passed"] else "FAIL", len(records), result["expected_seed_count"]))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
