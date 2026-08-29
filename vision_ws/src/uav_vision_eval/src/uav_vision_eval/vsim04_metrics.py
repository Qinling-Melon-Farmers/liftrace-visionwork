"""V-SIM-04 trial schema, aggregation, and artifact writers."""

import csv
import json
import math
import os
import statistics

import yaml

from uav_vision.target_selection_policy import resolve_class_profile


FRAME_FIELDS = [
    "trial_id", "stamp", "target_id", "class_name", "fully_in_frame",
    "center_in_frame", "detection_present", "map_valid",
    "transform_failure", "reject_reason", "map_error_xy",
    "current_confirmed", "current_selected", "stable_id",
]
EVENT_FIELDS = [
    "event_seq", "trial_id", "event", "source_stamp", "monotonic_sec",
    "class_name", "stable_id", "details",
]
PERFORMANCE_FIELDS = [
    "trial_id", "kind", "class_name", "height_m", "speed_mps", "status",
    "p_confirm", "p_selected", "p_interrupt", "stable_id",
    "confirmation_exposure_sec", "confirmation_processing_ms",
    "eligible_frames", "detection_frames", "map_valid_frames",
    "map_invalid_rate", "map_unavailable_rate", "tf_failure_rate",
    "mean_map_error_xy", "p95_map_error_xy",
]
REQUIRED_ARTIFACTS = (
    "manifest.json", "frames.csv", "events.csv", "summary.json",
    "report.md", "vision_search_performance.csv",
)


def _height_token(value):
    return ("{:.1f}".format(float(value))).replace(".", "p")


def _speed_token(value):
    return ("{:.1f}".format(float(value))).replace(".", "p")


def load_trial_matrix(path):
    with open(path, "r", encoding="utf-8") as stream:
        matrix = yaml.safe_load(stream)
    if not isinstance(matrix, dict) or matrix.get("evaluation_id") != "V-SIM-04":
        raise ValueError("matrix evaluation_id must be V-SIM-04")
    seed = int(matrix.get("seed", 0))
    if seed <= 0:
        raise ValueError("V-SIM-04 seed must be a fixed positive integer")
    profile_name, allowed = resolve_class_profile(matrix.get("class_profile", ""))
    matrix["class_profile"] = profile_name
    trials = expand_trial_matrix(matrix)
    forbidden = sorted({trial["class_name"] for trial in trials} - set(allowed))
    if forbidden:
        raise ValueError(
            "matrix classes are not selectable in {}: {}".format(
                profile_name, ",".join(forbidden)))
    expected = int(matrix.get("expected_trial_count", len(trials)))
    if len(trials) != expected:
        raise ValueError(
            "matrix expanded to {} trials, expected {}".format(
                len(trials), expected))
    anchors = matrix.get("target_anchors", {})
    missing = sorted({trial["class_name"] for trial in trials} - set(anchors))
    if missing:
        raise ValueError("missing target anchors: {}".format(",".join(missing)))
    matrix["trials"] = trials
    return matrix


def expand_trial_matrix(matrix):
    trials = []
    static = matrix.get("static", {})
    for class_name in static.get("classes", []):
        for height in static.get("heights_m", []):
            trials.append({
                "trial_id": "static_{}_h{}".format(
                    class_name, _height_token(height)),
                "kind": "static",
                "class_name": str(class_name),
                "height_m": float(height),
                "speed_mps": None,
            })
    dynamic = matrix.get("dynamic", {})
    for class_name in dynamic.get("classes", []):
        for height in dynamic.get("heights_m", []):
            for speed in dynamic.get("speeds_mps", []):
                trials.append({
                    "trial_id": "dynamic_{}_h{}_v{}".format(
                        class_name, _height_token(height),
                        _speed_token(speed)),
                    "kind": "dynamic",
                    "class_name": str(class_name),
                    "height_m": float(height),
                    "speed_mps": float(speed),
                })
    identifiers = [trial["trial_id"] for trial in trials]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("matrix contains duplicate trial identifiers")
    return trials


def percentile(values, percentile_value):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    position = (len(values) - 1) * float(percentile_value) / 100.0
    lower, upper = int(math.floor(position)), int(math.ceil(position))
    if lower == upper:
        return values[lower]
    return (values[lower] * (upper - position) +
            values[upper] * (position - lower))


def planned_trial_result(trial):
    result = dict(trial)
    result.update({
        "status": "planned",
        "p_confirm": None,
        "p_selected": None,
        "p_interrupt": None,
        "stable_id": None,
        "confirmation_exposure_sec": None,
        "confirmation_processing_ms": None,
        "eligible_frames": 0,
        "detection_frames": 0,
        "map_valid_frames": 0,
        "map_invalid_rate": None,
        "map_unavailable_rate": None,
        "tf_failure_rate": None,
        "map_errors_xy": [],
        "mean_map_error_xy": None,
        "p95_map_error_xy": None,
    })
    return result


def finalize_trial_result(result):
    result = dict(result)
    eligible = int(result.get("eligible_frames", 0))
    detected = int(result.get("detection_frames", 0))
    valid = int(result.get("map_valid_frames", 0))
    tf_failures = int(result.get("tf_failure_frames", 0))
    errors = [float(value) for value in result.pop("map_errors_xy", [])]
    result["map_invalid_rate"] = (
        max(0, detected - valid) / float(detected) if detected else None)
    result["map_unavailable_rate"] = (
        max(0, eligible - valid) / float(eligible) if eligible else None)
    result["tf_failure_rate"] = (
        tf_failures / float(detected) if detected else None)
    result["mean_map_error_xy"] = statistics.mean(errors) if errors else None
    result["p95_map_error_xy"] = percentile(errors, 95)
    result.pop("tf_failure_frames", None)
    return result


def summarize_trial_results(results, run_mode, actual_fps=None):
    finalized = [finalize_trial_result(result) for result in results]
    completed = [result for result in finalized
                 if result.get("status") == "completed"]
    confirmation_exposure = [
        result["confirmation_exposure_sec"] for result in completed
        if result.get("confirmation_exposure_sec") is not None]
    confirmation_processing = [
        result["confirmation_processing_ms"] for result in completed
        if result.get("confirmation_processing_ms") is not None]
    map_errors = [
        result["p95_map_error_xy"] for result in completed
        if result.get("p95_map_error_xy") is not None]
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04",
        "run_mode": run_mode,
        "status": "MEASURED" if completed else "DRY_RUN",
        "trial_count": len(finalized),
        "completed_trial_count": len(completed),
        "metrics": {
            "p_confirm": (
                sum(bool(result.get("p_confirm")) for result in completed) /
                float(len(completed)) if completed else None),
            "p_selected": (
                sum(bool(result.get("p_selected")) for result in completed) /
                float(len(completed)) if completed else None),
            "p_interrupt": None,
            "p_interrupt_reason": "visual_only_no_navigation_acceptance_event",
            "median_confirmation_exposure_sec": (
                statistics.median(confirmation_exposure)
                if confirmation_exposure else None),
            "p95_confirmation_processing_ms": percentile(
                confirmation_processing, 95),
            "max_trial_p95_map_error_xy": max(map_errors) if map_errors else None,
            "actual_image_fps": actual_fps,
        },
        "definitions": {
            "p_confirm": (
                "trial reaches current full candidate admission before the "
                "target leaves the fully-in-frame window"),
            "p_selected": (
                "the same stable_id confirmed in the trial is published on "
                "selected_target before leaving"),
            "p_interrupt": (
                "null in visual-only runs; requires navigation adapter "
                "SEARCH-to-APPROACH acceptance"),
            "confirmation_exposure_sec": (
                "candidate last_seen source stamp minus first fully-in-frame "
                "truth stamp"),
            "confirmation_processing_ms": (
                "monotonic recorder receipt of confirmation minus receipt of "
                "the image at candidate last_seen"),
            "map_invalid_rate": (
                "mapped detection frames without a valid map point divided by "
                "matching detection frames"),
            "map_unavailable_rate": (
                "eligible truth frames without a valid map observation divided "
                "by eligible truth frames"),
        },
        "trials": finalized,
    }


def _atomic_json(path, value):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def _write_csv(path, fields, rows):
    temporary = path + ".tmp"
    with open(temporary, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})
    os.replace(temporary, path)


def _report(summary):
    metrics = summary["metrics"]
    return "\n".join([
        "# V-SIM-04 Vision Search Performance",
        "",
        "- Run mode: `{}`".format(summary["run_mode"]),
        "- Completed trials: `{}/{}`".format(
            summary["completed_trial_count"], summary["trial_count"]),
        "- P_confirm: `{}`".format(metrics["p_confirm"]),
        "- P_selected: `{}`".format(metrics["p_selected"]),
        "- P_interrupt: `null` (visual-only; navigation acceptance is absent)",
        "- P95 processing latency: `{}` ms".format(
            metrics["p95_confirmation_processing_ms"]),
        "- Actual image FPS: `{}`".format(metrics["actual_image_fps"]),
        "",
        "## Semantics",
        "",
        "P_confirm uses current consecutive-frame/map/association/reject/age "
        "admission before the target leaves the fully-in-frame window. "
        "P_selected requires the same stable ID. Exposure time uses ROS/image "
        "stamps; processing time uses a monotonic wall clock. Map-invalid and "
        "TF-failure rates are reported separately from map error.",
        "",
        "A dry run validates only matrix and artifact schemas; it is not a Gate PASS.",
        "",
    ])


def write_artifacts(output_dir, manifest, frame_rows, event_rows, results,
                    run_mode, actual_fps=None):
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    summary = summarize_trial_results(results, run_mode, actual_fps)
    manifest = dict(manifest)
    manifest["schema_version"] = 1
    manifest["evaluation_id"] = "V-SIM-04"
    manifest["run_mode"] = run_mode
    manifest["actual_image_fps"] = actual_fps
    _atomic_json(os.path.join(output_dir, "manifest.json"), manifest)
    _write_csv(os.path.join(output_dir, "frames.csv"), FRAME_FIELDS, frame_rows)
    _write_csv(os.path.join(output_dir, "events.csv"), EVENT_FIELDS, event_rows)
    _atomic_json(os.path.join(output_dir, "summary.json"), summary)
    performance_rows = [finalize_trial_result(result) for result in results]
    _write_csv(
        os.path.join(output_dir, "vision_search_performance.csv"),
        PERFORMANCE_FIELDS, performance_rows)
    report_path = os.path.join(output_dir, "report.md")
    temporary = report_path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write(_report(summary))
    os.replace(temporary, report_path)
    return summary


def dry_run_artifacts(matrix_path, output_dir, metadata=None):
    matrix_path = os.path.abspath(matrix_path)
    matrix = load_trial_matrix(matrix_path)
    results = [planned_trial_result(trial) for trial in matrix["trials"]]
    events = [{
        "event_seq": index,
        "trial_id": trial["trial_id"],
        "event": "trial_planned",
        "class_name": trial["class_name"],
        "details": json.dumps(trial, sort_keys=True),
    } for index, trial in enumerate(matrix["trials"], 1)]
    manifest = {
        "seed": matrix["seed"],
        "class_profile": matrix["class_profile"],
        "matrix_file": matrix_path,
        "trials": matrix["trials"],
        "model": (metadata or {}).get("model", {"path": ""}),
        "thresholds": (metadata or {}).get("thresholds", {}),
        "camera_info": (metadata or {}).get("camera_info"),
        "extrinsic_profile": (metadata or {}).get("extrinsic_profile", ""),
        "revisions": (metadata or {}).get("revisions", {}),
    }
    return write_artifacts(
        output_dir, manifest, [], events, results, "dry_run", None)
