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


def bounded_scene_token(value, max_length=48, always_hash=False):
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


def resolve_batch_output(project_root, batch_id, requested_output=""):
    """Resolve a bounded batch token and a run-local aggregate directory."""
    project_root = os.path.realpath(os.path.abspath(project_root))
    logs_root = os.path.realpath(os.path.join(project_root, "logs"))
    batch_token = bounded_scene_token(
        batch_id, max_length=48, always_hash=True)
    if requested_output:
        output_dir = os.path.realpath(os.path.abspath(requested_output))
    else:
        output_dir = os.path.realpath(os.path.join(
            logs_root, "vsim04_repeat_aggregate_" + batch_token))
    try:
        inside_logs = os.path.commonpath([logs_root, output_dir]) == logs_root
    except ValueError:
        inside_logs = False
    if not inside_logs or output_dir == logs_root:
        raise ValueError("repeat output_dir must stay inside project/logs")
    return batch_token, output_dir


def create_new_batch_output(output_dir):
    """Create a batch directory exactly once; never reuse existing evidence."""
    try:
        os.makedirs(output_dir, exist_ok=False)
    except FileExistsError as error:
        raise ValueError(
            "repeat batch output already exists: " + output_dir) from error


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
    batch = bounded_scene_token(
        batch_id or datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"))
    # A comma-separated selector can contain many boundary IDs. Always attach
    # a digest so two equally truncated readable prefixes cannot collide.
    selector_token = bounded_scene_token(
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


def _scene_run_dirs(logs_dir, scene):
    prefix = scene + "_"
    if not os.path.isdir(logs_dir):
        return set()
    return {
        os.path.realpath(os.path.join(logs_dir, name))
        for name in os.listdir(logs_dir)
        if name.startswith(prefix) and
        os.path.isdir(os.path.join(logs_dir, name))
    }


def execute_repeat_commands(commands, project_root, dry_run=False,
                            on_result=None, results=None):
    """Execute every command, continuing after failures to preserve evidence."""
    results = [] if results is None else results
    logs_dir = os.path.join(os.path.abspath(project_root), "logs")
    for item in commands:
        command = list(item["command"])
        if dry_run:
            results.append(dict(item, exit_code=None, run_dir=None))
            continue
        env = os.environ.copy()
        env.update(item["environment"])
        before = _scene_run_dirs(logs_dir, item["scene"])
        interrupted = None
        execution_error = ""
        try:
            completed = subprocess.run(
                command, cwd=project_root, env=env, check=False)
            exit_code = int(completed.returncode)
        except KeyboardInterrupt as error:
            exit_code = 130
            interrupted = error
            execution_error = "keyboard_interrupt"
        except OSError as error:
            exit_code = 127
            execution_error = "subprocess_error:" + str(error)
        after = _scene_run_dirs(logs_dir, item["scene"])
        created = sorted(after - before)
        binding_error = ""
        if len(created) == 1:
            run_dir = created[0]
        else:
            run_dir = None
            binding_error = "new_run_dir_count:{}".format(len(created))
        result = dict(
            item, exit_code=exit_code, run_dir=run_dir,
            binding_error=binding_error, execution_error=execution_error)
        results.append(result)
        if on_result is not None:
            on_result(result, list(results))
        if interrupted is not None:
            raise interrupted
    return results


def atomic_write_json(path, payload):
    """Atomically replace a JSON checkpoint in its destination directory."""
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    temporary = path + ".tmp.{}".format(os.getpid())
    try:
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def append_unfinished_results(commands, results, interruption_error=""):
    """Append fail-closed placeholders for repeats that never returned."""
    completed_indexes = {int(result["repeat_index"]) for result in results}
    for item in commands:
        if int(item["repeat_index"]) in completed_indexes:
            continue
        results.append(dict(
            item, exit_code=130 if interruption_error else 127, run_dir=None,
            binding_error="run_not_executed",
            execution_error=interruption_error or "run_not_executed",
            state="UNFINISHED"))
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


def _manifest_configuration(manifest):
    if not isinstance(manifest, dict):
        raise ValueError("manifest root is not an object")
    revisions = manifest.get("revisions", {})
    model = manifest.get("model", {})
    thresholds = manifest.get("thresholds", {})
    design = manifest.get("evaluation_design", {})
    for name, value in (
            ("revisions", revisions), ("model", model),
            ("thresholds", thresholds), ("evaluation_design", design)):
        if not isinstance(value, dict):
            raise ValueError(name + " is not an object")
    selector = design.get("trial_selector", [])
    if not isinstance(selector, list):
        raise ValueError("trial_selector is not an array")
    model_path = str(model.get("path", "")).strip()
    matrix_file = str(manifest.get("matrix_file", "")).strip()
    configuration = {
        "vision_revision": str(revisions.get("vision", "")).strip(),
        "navigation_revision": str(revisions.get("navigation", "")).strip(),
        "model_path": os.path.realpath(model_path) if model_path else "",
        "imgsz": _integer(thresholds.get("detector_imgsz")),
        "class_profile": str(manifest.get("class_profile", "")).strip(),
        "matrix_file": os.path.realpath(matrix_file) if matrix_file else "",
        "seed": _integer(manifest.get("seed")),
        "trial_selector": [str(value).strip() for value in selector],
    }
    if (not configuration["vision_revision"] or
            not configuration["navigation_revision"] or
            not configuration["model_path"] or configuration["imgsz"] <= 0 or
            not configuration["class_profile"] or
            not configuration["matrix_file"] or configuration["seed"] <= 0 or
            any(not value for value in configuration["trial_selector"])):
        raise ValueError("manifest repeat configuration is incomplete")
    return configuration


def _inspect_run(run_dir, source_index, execution_exit_code=None,
                 binding_error="", execution_error=""):
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
        "measurement_eligible": False,
        "source_pass_eligible": False,
        "configuration_consistent": True,
        "trial_set_consistent": True,
        "configuration": None,
        "source_verdict": "FAIL",
        "errors": [],
        "measurement_errors": [],
        "verdict_errors": [],
        "trials": [],
        "processing_p95_ms": None,
        "map_p95_m": None,
    }
    if binding_error:
        record["measurement_errors"].append(str(binding_error))
    if execution_error:
        record["verdict_errors"].append(str(execution_error))
    if execution_exit_code is not None and int(execution_exit_code) != 0:
        record["verdict_errors"].append(
            "sim_run_exit_nonzero:{}".format(int(execution_exit_code)))
    vsim_dir = os.path.join(record["run_dir"], "vsim04")
    if not os.path.isdir(record["run_dir"]):
        record["measurement_errors"].append("run_dir_missing")
        record["missing_artifacts"] = list(REQUIRED_ARTIFACTS)
        record["errors"] = (
            list(record["measurement_errors"]) +
            list(record["verdict_errors"]))
        return record
    missing = [name for name in REQUIRED_ARTIFACTS
               if not os.path.isfile(os.path.join(vsim_dir, name))]
    record["missing_artifacts"] = missing
    record["artifact_complete"] = not missing
    summary_path = os.path.join(vsim_dir, "summary.json")
    if not os.path.isfile(summary_path):
        record["measurement_errors"].append("summary_missing")
        record["errors"] = (
            list(record["measurement_errors"]) +
            list(record["verdict_errors"]))
        return record
    try:
        with open(summary_path, "r", encoding="utf-8") as stream:
            summary = json.load(stream)
    except (OSError, ValueError) as error:
        record["measurement_errors"].append("summary_invalid:" + str(error))
        record["errors"] = (
            list(record["measurement_errors"]) +
            list(record["verdict_errors"]))
        return record
    if not isinstance(summary, dict):
        record["measurement_errors"].append("summary_root_not_object")
        record["errors"] = (
            list(record["measurement_errors"]) +
            list(record["verdict_errors"]))
        return record

    manifest_path = os.path.join(vsim_dir, "manifest.json")
    try:
        with open(manifest_path, "r", encoding="utf-8") as stream:
            record["configuration"] = _manifest_configuration(
                json.load(stream))
    except (OSError, ValueError) as error:
        record["measurement_errors"].append(
            "manifest_configuration_invalid:" + str(error))

    record["summary_status"] = str(summary.get("status", "")) or "MISSING"
    performance = summary.get("performance_verdict", {})
    if not isinstance(performance, dict):
        performance = {}
        record["measurement_errors"].append(
            "performance_verdict_not_object")
    record["performance_verdict"] = str(
        performance.get("status", "")) or "MISSING"
    completeness = summary.get("completeness", {})
    if not isinstance(completeness, dict):
        completeness = {}
        record["measurement_errors"].append("completeness_not_object")
    trial_count = _integer(summary.get("trial_count"))
    completed_trial_count = _integer(summary.get("completed_trial_count"))
    trials = summary.get("trials", [])
    if not isinstance(trials, list):
        trials = []
        record["measurement_errors"].append("trials_not_array")
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
        record["measurement_errors"].append("evaluation_id_invalid")
        record["measurement_terminal"] = False
    if not record["artifact_complete"]:
        record["measurement_errors"].append("artifact_set_incomplete")
    if not record["measurement_terminal"]:
        record["measurement_errors"].append("measurement_not_terminal")
    if performance.get("hard_failure") is True:
        record["verdict_errors"].append("performance_hard_failure")
    metrics = summary.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
        record["measurement_errors"].append("metrics_not_object")
    record["processing_p95_ms"] = _finite_number(
        metrics.get("p95_confirmation_processing_ms"))
    record["map_p95_m"] = _finite_number(metrics.get("p95_map_error_xy"))
    record["trials"] = valid_trials
    if any(trial.get("p_interrupt") is not None
           for trial in record["trials"] if isinstance(trial, dict)):
        record["measurement_errors"].append(
            "p_interrupt_not_null_in_visual_only_run")
    record["measurement_eligible"] = (
        record["artifact_complete"] and record["measurement_terminal"] and
        not record["measurement_errors"])
    record["metrics_eligible"] = record["measurement_eligible"]
    record["source_pass_eligible"] = (
        record["measurement_eligible"] and not record["verdict_errors"])
    record["errors"] = (
        list(record["measurement_errors"]) + list(record["verdict_errors"]))
    if (record["source_pass_eligible"] and
            record["summary_status"] == "MEASURED" and
            record["performance_verdict"] == "PASS" and
            performance.get("is_gate_pass") is True and
            performance.get("hard_failure") is not True):
        record["source_verdict"] = "PASS"
    elif (record["source_pass_eligible"] and
          record["performance_verdict"] == "DIAGNOSTIC_ONLY"):
        record["source_verdict"] = "DIAGNOSTIC_ONLY"
    elif record["source_pass_eligible"]:
        record["source_verdict"] = record["performance_verdict"]
    return record


def aggregate_repeat_runs(run_dirs, execution_exit_codes=None,
                          binding_errors=None, execution_errors=None,
                          expected_configuration=None):
    """Aggregate run directories without weakening source run verdicts."""
    if execution_exit_codes is None:
        execution_exit_codes = [None] * len(run_dirs)
    if len(execution_exit_codes) != len(run_dirs):
        raise ValueError("execution_exit_codes must match run_dirs")
    binding_errors = binding_errors or [""] * len(run_dirs)
    execution_errors = execution_errors or [""] * len(run_dirs)
    if (len(binding_errors) != len(run_dirs) or
            len(execution_errors) != len(run_dirs)):
        raise ValueError("run error lists must match run_dirs")
    sources = [_inspect_run(
        path, index + 1, execution_exit_codes[index], binding_errors[index],
        execution_errors[index])
               for index, path in enumerate(run_dirs)]
    seen_run_dirs = set()
    for source in sources:
        if source["run_dir"] in seen_run_dirs:
            source["measurement_errors"].append("duplicate_run_dir")
            source["errors"] = (
                list(source["measurement_errors"]) +
                list(source["verdict_errors"]))
            source["measurement_eligible"] = False
            source["metrics_eligible"] = False
            source["source_pass_eligible"] = False
            source["source_verdict"] = "FAIL"
        seen_run_dirs.add(source["run_dir"])
    measured_configurations = {
        json.dumps(source["configuration"], sort_keys=True)
        for source in sources
        if source["measurement_eligible"] and source["configuration"] is not None
    }
    expected_serialized = (json.dumps(expected_configuration, sort_keys=True)
                           if expected_configuration is not None else None)
    configuration_mismatch = len(measured_configurations) > 1
    if (expected_serialized is not None and measured_configurations and
            measured_configurations != {expected_serialized}):
        configuration_mismatch = True
    if configuration_mismatch:
        for source in sources:
            if source["measurement_eligible"]:
                source["configuration_consistent"] = False
                source["source_pass_eligible"] = False
                source["source_verdict"] = "FAIL"
                source["verdict_errors"].append("configuration_mismatch")
                source["errors"] = (
                    list(source["measurement_errors"]) +
                    list(source["verdict_errors"]))
    trial_sets = {
        tuple(sorted(str(trial.get("trial_id", "")).strip()
                     for trial in source["trials"]))
        for source in sources if source["measurement_eligible"]
    }
    if len(trial_sets) > 1:
        for source in sources:
            if source["measurement_eligible"]:
                source["trial_set_consistent"] = False
                source["source_pass_eligible"] = False
                source["source_verdict"] = "FAIL"
                source["verdict_errors"].append("trial_set_mismatch")
                source["errors"] = (
                    list(source["measurement_errors"]) +
                    list(source["verdict_errors"]))
    for source in sources:
        source["metrics_eligible"] = (
            source["measurement_eligible"] and
            source["configuration_consistent"] and
            source["trial_set_consistent"])
    by_trial = {}
    for source in sources:
        if (not source["measurement_eligible"] or
                not source["configuration_consistent"] or
                not source["trial_set_consistent"]):
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
    configuration_verified = (
        bool(measured_configurations) and not configuration_mismatch and
        (expected_serialized is None or
         measured_configurations == {expected_serialized}))
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04-repeat-aggregate",
        "status": "PASS" if all_pass else "FAIL",
        "is_gate_pass": all_pass,
        "source_run_count": len(sources),
        "source_runs": sources,
        "configuration_consistent": not configuration_mismatch,
        "configuration_verified": configuration_verified,
        "expected_configuration": expected_configuration,
        "seeds": sorted({
            source["configuration"]["seed"] for source in sources
            if source["configuration"] is not None
        }),
        "trials": [by_trial[key] for key in sorted(by_trial)],
        "repeats_are_multi_seed": False,
        "definitions": {
            "source_verdict": (
                "PASS requires MEASURED, complete six-artifact set, terminal "
                "measurement, and performance PASS; DIAGNOSTIC_ONLY is not PASS."),
            "measurement_eligible": (
                "The six-artifact schema and terminal trial measurement are "
                "complete. Performance or sim_run failure does not erase "
                "these diagnostic samples."),
            "source_pass_eligible": (
                "Measurement is eligible and no execution, hard-performance, "
                "configuration, or trial-set error blocks verdict evaluation."),
            "p_interrupt": (
                "Always null in visual-only repeats; navigation acceptance is "
                "required to measure SEARCH-to-APPROACH interruption."),
            "processing_p95_samples_ms": (
                "Source-run p95_confirmation_processing_ms values for runs "
                "containing the trial."),
            "map_p95_samples_m": (
                "Per-trial p95_map_error_xy values across source runs."),
            "seed": (
                "A fixed matrix/design identifier. Repeats reuse the same "
                "seed and are runtime repeats, not independent multi-seed "
                "samples."),
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
        stream.write("- 配置元组已核验：`{}`\n".format(
            aggregate["configuration_verified"]))
        stream.write("- repeats 是否为多 seed：`false`\n")
        stream.write("- `P_interrupt`：`null`（visual-only）\n\n")
        stream.write("## 源 run 判定\n\n")
        stream.write("| # | verdict | measured | summary | performance | "
                     "seed | vision/nav | run |\n")
        stream.write("|---:|---|---|---|---|---:|---|---|\n")
        for source in aggregate["source_runs"]:
            configuration = source.get("configuration") or {}
            stream.write("| {} | {} | {} | {} | {} | {} | `{}/{}` | `{}` |\n".format(
                source["source_index"], source["source_verdict"],
                source["measurement_eligible"],
                source["summary_status"], source["performance_verdict"],
                configuration.get("seed"),
                configuration.get("vision_revision", ""),
                configuration.get("navigation_revision", ""),
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
