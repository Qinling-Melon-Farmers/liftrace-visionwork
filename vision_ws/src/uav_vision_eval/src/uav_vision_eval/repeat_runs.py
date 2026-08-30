"""Build repeat-run commands and aggregate V-SIM-04 run artifacts."""

import csv
import datetime
import hashlib
import json
import math
import os
import re
import subprocess


REQUIRED_ARTIFACTS = (
    "manifest.json", "frames.csv", "events.csv", "summary.json",
    "report.md", "vision_search_performance.csv",
)
TERMINAL_MEASUREMENT_STATUSES = {"MEASURED", "DIAGNOSTIC"}


def _bounded_scene_token(value, max_length=48, always_hash=False):
    """Return a readable bounded token with a hash preserving uniqueness."""
    if int(max_length) < 10:
        raise ValueError("scene token max_length must be at least 10")
    raw = str(value).strip()
    token = re.sub(r"[^A-Za-z0-9_-]+", "-", raw).strip("-") or "all"
    if not always_hash and len(token) <= int(max_length):
        return token
    digest = hashlib.sha256(raw.encode("utf-8")).hexdigest()[:8]
    prefix_length = int(max_length) - len(digest) - 1
    return "{}-{}".format(token[:prefix_length].rstrip("-"), digest)


def build_repeat_commands(project_root, repeats, trial_selector, matrix,
                          imgsz, model_path, vision_revision,
                          navigation_revision, batch_id=None):
    """Return deterministic, independently executable sim_run commands."""
    if int(repeats) <= 0:
        raise ValueError("repeats must be positive")
    if int(imgsz) <= 0:
        raise ValueError("imgsz must be positive")
    for name, value in (
            ("trial_selector", trial_selector), ("matrix", matrix),
            ("model_path", model_path), ("vision_revision", vision_revision),
            ("navigation_revision", navigation_revision)):
        if not str(value).strip():
            raise ValueError("{} must be non-empty".format(name))

    root = os.path.abspath(project_root)
    matrix = os.path.abspath(matrix)
    model_path = os.path.abspath(model_path)
    batch = _bounded_scene_token(
        batch_id or datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"))
    # A comma-separated selector can contain many boundary IDs. Always attach
    # a digest so two equally truncated readable prefixes cannot collide.
    selector_token = _bounded_scene_token(
        trial_selector, max_length=48, always_hash=True)
    commands = []
    for repeat_index in range(1, int(repeats) + 1):
        scene = "vsim04_diag_repeat_{}_{}_r{:02d}".format(
            selector_token, batch, repeat_index)
        command = [
            "bash", os.path.join(root, "top_level_scripts", "sim_run.sh"),
            scene,
            "roslaunch", "uav_vision_eval", "vsim04_stability.launch",
            "gui:=false",
            "matrix_file:={}".format(matrix),
            "trial_selector:={}".format(trial_selector),
            "target_detector_imgsz:={}".format(int(imgsz)),
            "start_vision:=true",
            "start_landing_detector:=false",
            "enable_stage_trace:=true",
        ]
        commands.append({
            "repeat_index": repeat_index,
            "scene": scene,
            "command": command,
            "environment": {
                "SIM_NO_RECORD": "1",
                "UAV_VISION_MODEL_PATH": model_path,
                "VSIM04_VISION_REVISION": str(vision_revision),
                "VSIM04_NAVIGATION_REVISION": str(navigation_revision),
            },
        })
    if len({item["scene"] for item in commands}) != len(commands):
        raise ValueError("repeat scenes must be unique")
    return commands


def execute_repeat_commands(commands, project_root, dry_run=False):
    """Execute every command, continuing after failures to preserve evidence."""
    results = []
    logs_dir = os.path.join(os.path.abspath(project_root), "logs")
    for item in commands:
        command = list(item["command"])
        if dry_run:
            results.append(dict(item, exit_code=None, run_dir=None))
            continue
        env = os.environ.copy()
        env.update(item["environment"])
        completed = subprocess.run(command, cwd=project_root, env=env,
                                   check=False)
        prefix = item["scene"] + "_"
        candidates = [
            os.path.join(logs_dir, name) for name in os.listdir(logs_dir)
            if name.startswith(prefix) and
            os.path.isdir(os.path.join(logs_dir, name))
        ] if os.path.isdir(logs_dir) else []
        run_dir = max(candidates, key=os.path.getmtime) if candidates else None
        results.append(dict(
            item, exit_code=int(completed.returncode), run_dir=run_dir))
    return results


def _finite_number(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _integer(value, default=-1):
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def _percentile(values, percentile=0.95):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    index = (len(values) - 1) * float(percentile)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def _inspect_run(run_dir, source_index, execution_exit_code=None):
    record = {
        "source_index": source_index,
        "run_dir": os.path.abspath(run_dir),
        "execution_exit_code": execution_exit_code,
        "missing_artifacts": [],
        "summary_status": "MISSING",
        "performance_verdict": "MISSING",
        "artifact_complete": False,
        "measurement_terminal": False,
        "metrics_eligible": False,
        "source_verdict": "FAIL",
        "errors": [],
        "trials": [],
        "processing_p95_ms": None,
        "map_p95_m": None,
    }
    vsim_dir = os.path.join(record["run_dir"], "vsim04")
    if not os.path.isdir(record["run_dir"]):
        record["errors"].append("run_dir_missing")
        record["missing_artifacts"] = list(REQUIRED_ARTIFACTS)
        return record
    missing = [name for name in REQUIRED_ARTIFACTS
               if not os.path.isfile(os.path.join(vsim_dir, name))]
    record["missing_artifacts"] = missing
    record["artifact_complete"] = not missing
    summary_path = os.path.join(vsim_dir, "summary.json")
    if not os.path.isfile(summary_path):
        record["errors"].append("summary_missing")
        return record
    try:
        with open(summary_path, "r", encoding="utf-8") as stream:
            summary = json.load(stream)
    except (OSError, ValueError) as error:
        record["errors"].append("summary_invalid:" + str(error))
        return record
    if not isinstance(summary, dict):
        record["errors"].append("summary_root_not_object")
        return record

    record["summary_status"] = str(summary.get("status", "")) or "MISSING"
    performance = summary.get("performance_verdict", {})
    if not isinstance(performance, dict):
        performance = {}
        record["errors"].append("performance_verdict_not_object")
    record["performance_verdict"] = str(
        performance.get("status", "")) or "MISSING"
    completeness = summary.get("completeness", {})
    if not isinstance(completeness, dict):
        completeness = {}
        record["errors"].append("completeness_not_object")
    trial_count = _integer(summary.get("trial_count"))
    completed_trial_count = _integer(summary.get("completed_trial_count"))
    trials = summary.get("trials", [])
    if not isinstance(trials, list):
        trials = []
        record["errors"].append("trials_not_array")
    valid_trials = [trial for trial in trials if isinstance(trial, dict)]
    trial_ids = [str(trial.get("trial_id", "")).strip()
                 for trial in valid_trials]
    trial_contract_valid = (
        len(valid_trials) == trial_count and
        len(set(trial_ids)) == trial_count and
        all(trial_ids) and
        sum(trial.get("status") == "completed"
            for trial in valid_trials) == completed_trial_count
    )
    record["measurement_terminal"] = (
        record["summary_status"] in TERMINAL_MEASUREMENT_STATUSES and
        completeness.get("run_complete") is True and
        trial_count > 0 and completed_trial_count == trial_count and
        trial_contract_valid and
        not summary.get("validation_errors") and
        not completeness.get("validation_errors")
    )
    if summary.get("evaluation_id") != "V-SIM-04":
        record["errors"].append("evaluation_id_invalid")
        record["measurement_terminal"] = False
    if not record["artifact_complete"]:
        record["errors"].append("artifact_set_incomplete")
    if not record["measurement_terminal"]:
        record["errors"].append("measurement_not_terminal")
    if performance.get("hard_failure") is True:
        record["errors"].append("performance_hard_failure")
    if execution_exit_code is not None and int(execution_exit_code) != 0:
        record["errors"].append(
            "sim_run_exit_nonzero:{}".format(int(execution_exit_code)))

    metrics = summary.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
        record["errors"].append("metrics_not_object")
    record["processing_p95_ms"] = _finite_number(
        metrics.get("p95_confirmation_processing_ms"))
    record["map_p95_m"] = _finite_number(metrics.get("p95_map_error_xy"))
    record["trials"] = valid_trials
    if any(trial.get("p_interrupt") is not None
           for trial in record["trials"] if isinstance(trial, dict)):
        record["errors"].append("p_interrupt_not_null_in_visual_only_run")
    record["metrics_eligible"] = (
        record["artifact_complete"] and record["measurement_terminal"] and
        not record["errors"])
    if (record["metrics_eligible"] and
            record["summary_status"] == "MEASURED" and
            record["performance_verdict"] == "PASS" and
            performance.get("is_gate_pass") is True and
            performance.get("hard_failure") is not True):
        record["source_verdict"] = "PASS"
    elif (record["metrics_eligible"] and
          record["performance_verdict"] == "DIAGNOSTIC_ONLY"):
        record["source_verdict"] = "DIAGNOSTIC_ONLY"
    elif record["metrics_eligible"]:
        record["source_verdict"] = record["performance_verdict"]
    return record


def aggregate_repeat_runs(run_dirs, execution_exit_codes=None):
    """Aggregate run directories without weakening source run verdicts."""
    if execution_exit_codes is None:
        execution_exit_codes = [None] * len(run_dirs)
    if len(execution_exit_codes) != len(run_dirs):
        raise ValueError("execution_exit_codes must match run_dirs")
    sources = [_inspect_run(path, index + 1, execution_exit_codes[index])
               for index, path in enumerate(run_dirs)]
    seen_run_dirs = set()
    for source in sources:
        if source["run_dir"] in seen_run_dirs:
            source["errors"].append("duplicate_run_dir")
            source["metrics_eligible"] = False
            source["source_verdict"] = "FAIL"
        seen_run_dirs.add(source["run_dir"])
    expected_trial_ids = None
    for source in sources:
        if not source["metrics_eligible"]:
            continue
        source_trial_ids = {
            str(trial.get("trial_id", "")).strip()
            for trial in source["trials"]
        }
        if expected_trial_ids is None:
            expected_trial_ids = source_trial_ids
        elif source_trial_ids != expected_trial_ids:
            source["errors"].append("trial_set_mismatch")
            source["metrics_eligible"] = False
            source["source_verdict"] = "FAIL"
    by_trial = {}
    for source in sources:
        if not source["metrics_eligible"]:
            continue
        for trial in source["trials"]:
            trial_id = str(trial.get("trial_id", "")).strip()
            if not trial_id:
                continue
            bucket = by_trial.setdefault(trial_id, {
                "trial_id": trial_id,
                "source_run_count": len(sources),
                "completed_run_count": 0,
                "p_confirm_count": 0,
                "p_selected_count": 0,
                "p_interrupt": None,
                "failure_stage_counts": {},
                "confirmation_processing_ms_samples": [],
                "processing_p95_samples_ms": [],
                "map_p95_samples_m": [],
                "source_verdicts": [],
            })
            if trial.get("status") != "completed":
                continue
            bucket["completed_run_count"] += 1
            bucket["p_confirm_count"] += int(trial.get("p_confirm") is True)
            bucket["p_selected_count"] += int(trial.get("p_selected") is True)
            stage = str(trial.get("failure_stage", "")).strip() or "none"
            bucket["failure_stage_counts"][stage] = (
                bucket["failure_stage_counts"].get(stage, 0) + 1)
            processing = _finite_number(trial.get(
                "confirmation_processing_ms"))
            map_p95 = _finite_number(trial.get("p95_map_error_xy"))
            if processing is not None:
                bucket["confirmation_processing_ms_samples"].append(processing)
            if source["processing_p95_ms"] is not None:
                bucket["processing_p95_samples_ms"].append(
                    source["processing_p95_ms"])
            if map_p95 is not None:
                bucket["map_p95_samples_m"].append(map_p95)

    for bucket in by_trial.values():
        bucket["source_verdicts"] = [{
            "source_index": source["source_index"],
            "run_dir": source["run_dir"],
            "verdict": source["source_verdict"],
        } for source in sources]
        missing = bucket["source_run_count"] - bucket["completed_run_count"]
        if missing:
            bucket["failure_stage_counts"]["missing_or_incomplete_run"] = missing
        denominator = bucket["completed_run_count"]
        bucket["p_confirm"] = (
            bucket["p_confirm_count"] / float(denominator)
            if denominator else None)
        bucket["p_selected"] = (
            bucket["p_selected_count"] / float(denominator)
            if denominator else None)
        bucket["processing_p95_across_samples_ms"] = _percentile(
            bucket["confirmation_processing_ms_samples"])
        bucket["map_p95_across_samples_m"] = _percentile(
            bucket["map_p95_samples_m"])

    all_pass = bool(sources) and all(
        source["source_verdict"] == "PASS" for source in sources)
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04-repeat-aggregate",
        "status": "PASS" if all_pass else "FAIL",
        "is_gate_pass": all_pass,
        "source_run_count": len(sources),
        "source_runs": sources,
        "trials": [by_trial[key] for key in sorted(by_trial)],
        "definitions": {
            "source_verdict": (
                "PASS requires MEASURED, complete six-artifact set, terminal "
                "measurement, and performance PASS; DIAGNOSTIC_ONLY is not PASS."),
            "p_interrupt": (
                "Always null in visual-only repeats; navigation acceptance is "
                "required to measure SEARCH-to-APPROACH interruption."),
            "processing_p95_samples_ms": (
                "Source-run p95_confirmation_processing_ms values for runs "
                "containing the trial."),
            "map_p95_samples_m": (
                "Per-trial p95_map_error_xy values across source runs."),
        },
    }


def write_aggregate_outputs(aggregate, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "repeat_summary.json")
    csv_path = os.path.join(output_dir, "repeat_trials.csv")
    report_path = os.path.join(output_dir, "repeat_report.md")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(aggregate, stream, ensure_ascii=False, indent=2,
                  sort_keys=True)
        stream.write("\n")
    fields = [
        "trial_id", "source_run_count", "completed_run_count",
        "p_confirm", "p_selected", "p_interrupt", "failure_stage_counts",
        "processing_p95_samples_ms", "map_p95_samples_m",
        "processing_p95_across_samples_ms", "map_p95_across_samples_m",
        "source_verdicts",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for trial in aggregate["trials"]:
            row = {key: trial.get(key) for key in fields}
            row["failure_stage_counts"] = json.dumps(
                row["failure_stage_counts"], sort_keys=True)
            row["processing_p95_samples_ms"] = json.dumps(
                row["processing_p95_samples_ms"])
            row["map_p95_samples_m"] = json.dumps(row["map_p95_samples_m"])
            row["source_verdicts"] = json.dumps(
                row["source_verdicts"], sort_keys=True)
            writer.writerow(row)

    with open(report_path, "w", encoding="utf-8") as stream:
        stream.write("# V-SIM-04 重复运行聚合\n\n")
        stream.write("- 总判定：`{}`\n".format(aggregate["status"]))
        stream.write("- 源 run 数：{}\n".format(
            aggregate["source_run_count"]))
        stream.write("- `P_interrupt`：`null`（visual-only）\n\n")
        stream.write("## 源 run 判定\n\n")
        stream.write("| # | verdict | summary | performance | run |\n")
        stream.write("|---:|---|---|---|---|\n")
        for source in aggregate["source_runs"]:
            stream.write("| {} | {} | {} | {} | `{}` |\n".format(
                source["source_index"], source["source_verdict"],
                source["summary_status"], source["performance_verdict"],
                source["run_dir"]))
        stream.write("\n## Trial 聚合\n\n")
        stream.write("| trial | completed/runs | P_confirm | P_selected | "
                     "failure stages | processing P95 samples (ms) | "
                     "map P95 samples (m) |\n")
        stream.write("|---|---:|---:|---:|---|---|---|\n")
        for trial in aggregate["trials"]:
            stream.write("| {} | {}/{} | {} | {} | `{}` | `{}` | `{}` |\n".format(
                trial["trial_id"], trial["completed_run_count"],
                trial["source_run_count"], trial["p_confirm"],
                trial["p_selected"], json.dumps(
                    trial["failure_stage_counts"], sort_keys=True),
                json.dumps(trial["processing_p95_samples_ms"]),
                json.dumps(trial["map_p95_samples_m"])))
    return [json_path, csv_path, report_path]
