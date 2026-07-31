#!/usr/bin/env python3
"""Render a compact Markdown report and fail when configured gates fail."""

import argparse
import json
import os
import sys


def _display(value):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return "{:.4f}".format(value)
    return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(args.summary, "r", encoding="utf-8") as stream:
        summary = json.load(stream)
    metrics = summary["metrics"]
    lines = [
        "# UAV Vision Evaluation Report",
        "",
        "- Scenario: `{}`".format(summary["scenario_id"]),
        "- Gate result: **{}**".format("PASS" if summary["passed"] else "FAIL"),
        "",
        "| Metric | Value | Gate | Result |",
        "|---|---:|---:|:---:|",
    ]
    gate_mapping = [
        ("scored_frames", "min_scored_frames", ">="),
        ("precision", "min_precision", ">="),
        ("recall", "min_recall", ">="),
        ("input_coverage", "min_input_coverage", ">="),
        ("false_positive", "max_false_positives", "<="),
        ("mean_pixel_error", "max_mean_pixel_error", "<="),
        ("p95_pixel_error", "max_p95_pixel_error", "<="),
        ("mean_map_error_xy", "max_mean_map_error_xy", "<="),
        ("p95_map_error_xy", "max_p95_map_error_xy", "<="),
        ("p95_latency_ms", "max_p95_latency_ms", "<="),
    ]
    for metric_name, gate_name, operator in gate_mapping:
        if gate_name not in summary["gates"]:
            continue
        lines.append("| {} | {} | {} {} | {} |".format(
            metric_name, _display(metrics.get(metric_name)), operator,
            _display(summary["gates"][gate_name]),
            "PASS" if summary["checks"].get(metric_name, True) else "FAIL",
        ))
    lines.extend([
        "", "## Counts", "",
        "TP: {} · FP: {} · FN: {}".format(
            metrics["true_positive"], metrics["false_positive"], metrics["false_negative"]
        ),
        "Input coverage: {} / {} frames via `{}`".format(
            summary.get("input_coverage", {}).get("processed_frames", 0),
            summary.get("input_coverage", {}).get("reference_camera_frames", 0),
            summary.get("input_coverage", {}).get("required_scoring_source", ""),
        ),
        "",
        "Raw CSV: `{}`".format(summary["files"]["csv"]),
    ])
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    print("{}: {}".format(summary["scenario_id"], "PASS" if summary["passed"] else "FAIL"))
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
