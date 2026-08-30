"""V-SIM-04 trial schema, aggregation, and artifact writers."""

import csv
import json
import math
import os
import statistics
import threading
import time
import uuid

import yaml

from uav_vision.target_selection_policy import resolve_class_profile


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}
STAGE_COUNT_FIELDS = (
    "raw_class_frames", "raw_geometry_frames", "resolved_frames",
    "refined_frames", "geometry_verified_frames",
    "association_valid_frames", "center_refined_frames",
)
FRAME_FIELDS = [
    "trial_id", "stamp", "target_id", "class_name", "fully_in_frame",
    "center_in_frame", "co_visible_classes",
    "raw_class_present", "raw_geometry_present",
    "raw_class_confidence", "raw_geometry_confidence",
    "resolved_present", "resolved_class_confidence",
    "resolved_geometry_confidence", "refined_present",
    "geometry_verified", "center_refined", "association_valid",
    "refined_class_confidence", "refined_geometry_confidence",
    "refined_reject_reason", "detection_present",
    "mapped_class_confidence", "mapped_geometry_confidence", "map_valid",
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
    "processing_receipt_reordered",
    "eligible_frames", "raw_class_frames", "raw_geometry_frames",
    "resolved_frames", "refined_frames", "geometry_verified_frames",
    "association_valid_frames", "center_refined_frames",
    "detection_frames", "map_valid_frames", "failure_stage",
    "map_invalid_rate", "map_unavailable_rate", "tf_failure_rate",
    "mean_map_error_xy", "p95_map_error_xy", "map_error_sample_count",
    "entered_fully_in_frame", "left_fully_in_frame",
    "expected_duration_sec", "actual_duration_sec", "expected_speed_mps",
    "actual_speed_mps",
]
REQUIRED_ARTIFACTS = (
    "manifest.json", "frames.csv", "events.csv", "summary.json",
    "report.md", "vision_search_performance.csv",
)
EXPECTED_TRIAL_COUNT = 23


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
        "processing_receipt_reordered": False,
        "eligible_frames": 0,
        "raw_class_frames": 0,
        "raw_geometry_frames": 0,
        "resolved_frames": 0,
        "refined_frames": 0,
        "geometry_verified_frames": 0,
        "association_valid_frames": 0,
        "center_refined_frames": 0,
        "detection_frames": 0,
        "map_valid_frames": 0,
        "failure_stage": "",
        "map_invalid_rate": None,
        "map_unavailable_rate": None,
        "tf_failure_rate": None,
        "map_errors_xy": [],
        "mean_map_error_xy": None,
        "p95_map_error_xy": None,
        "map_error_sample_count": 0,
        "entered_fully_in_frame": False,
        "left_fully_in_frame": False,
        "enter_source_stamp": None,
        "leave_source_stamp": None,
        "enter_receipt_monotonic": None,
        "leave_receipt_monotonic": None,
        "expected_duration_sec": None,
        "actual_duration_sec": None,
        "expected_speed_mps": None,
        "actual_speed_mps": None,
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
    result["map_error_sample_count"] = len(errors)
    result.pop("tf_failure_frames", None)
    result["failure_stage"] = classify_failure_stage(result)
    return result


def classify_failure_stage(result):
    """Return the first pipeline stage that blocked trial confirmation."""
    if result.get("status") != "completed" or result.get("p_confirm"):
        return ""
    if int(result.get("eligible_frames", 0)) <= 0:
        return "truth_visibility"
    if int(result.get("raw_class_frames", 0)) <= 0:
        return "raw_classifier"
    if int(result.get("raw_geometry_frames", 0)) <= 0:
        return "raw_geometry"
    if int(result.get("resolved_frames", 0)) <= 0:
        return "detection_fusion"
    if int(result.get("refined_frames", 0)) <= 0:
        return "target_refiner"
    class_name = str(result.get("class_name", ""))
    if (class_name in STANDARD_CLASSES and
            int(result.get("association_valid_frames", 0)) <= 0):
        return "geometry_association"
    if (int(result.get("geometry_verified_frames", 0)) <= 0 or
            int(result.get("center_refined_frames", 0)) <= 0):
        return "geometry_refinement"
    if int(result.get("detection_frames", 0)) <= 0:
        return "map_projector_input"
    if int(result.get("map_valid_frames", 0)) <= 0:
        return "map_projection"
    return "target_memory_admission"


def _ratio(numerator, denominator):
    return numerator / float(denominator) if denominator else None


def watermarks_cover_source_stamp(watermarks, source_stamp):
    """Return true only when every required output processed the source stamp."""
    try:
        boundary = float(source_stamp)
        values = [float(value) for value in watermarks.values()]
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    return (math.isfinite(boundary) and bool(values) and
            all(math.isfinite(value) and value >= boundary
                for value in values))


def completed_sources_cover(required_sources, completed_sources):
    """Require every formal detector branch to reach the mapped output."""
    required = {str(value).strip() for value in required_sources
                if str(value).strip()}
    completed = {str(value).strip() for value in completed_sources
                 if str(value).strip()}
    return bool(required) and required.issubset(completed)


def detector_diagnostic_errors(level, values, expected_backend,
                               expected_model_path):
    """Validate the dev/sim detector diagnostic without trusting heartbeats."""
    errors = []
    if int(level) != 0:
        errors.append("diagnostic_level_{}".format(int(level)))
    backend = str(values.get("backend", ""))
    if backend != str(expected_backend):
        errors.append("backend_{}".format(backend or "missing"))
    reported_model = str(values.get("model_path", "")).strip()
    expected_model = str(expected_model_path).strip()
    if (not reported_model or not expected_model or
            os.path.realpath(os.path.expanduser(reported_model)) !=
            os.path.realpath(os.path.expanduser(expected_model))):
        errors.append("model_path_mismatch")
    return errors


def call_with_monotonic_deadline(operation, timeout_sec, operation_name,
                                 cancelled=None):
    """Run a potentially blocking local/ROS call behind a wall-clock limit."""
    timeout_sec = float(timeout_sec)
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive")
    completed = threading.Event()
    outcome = {}

    def invoke():
        try:
            outcome["response"] = operation()
        except Exception as error:  # re-raised on the calling thread
            outcome["error"] = error
        finally:
            completed.set()

    worker = threading.Thread(
        target=invoke, name="{}_deadline_worker".format(operation_name))
    worker.daemon = True
    worker.start()
    deadline = time.monotonic() + timeout_sec
    while not completed.wait(0.01):
        if cancelled is not None and cancelled():
            raise RuntimeError(
                "cancelled during {} operation".format(operation_name))
        if time.monotonic() >= deadline:
            raise TimeoutError(
                "{} exceeded {:.2f}s monotonic deadline".format(
                    operation_name, timeout_sec))
    if "error" in outcome:
        raise outcome["error"]
    return outcome.get("response")


def handshake_timeout_is_safe(timeout_sec, drain_sec, quiet_sec,
                              status_period_sec, write_margin_sec):
    values = [timeout_sec, drain_sec, quiet_sec, status_period_sec,
              write_margin_sec]
    try:
        values = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError):
        return False
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        return False
    return values[0] > sum(values[1:])


def event_inside_trial_window(event, result):
    """Use both source time and monotonic receipt time for trial admission."""
    enter_source = result.get("enter_source_stamp")
    leave_source = result.get("leave_source_stamp")
    leave_receipt = result.get("leave_receipt_monotonic")
    values = (enter_source, leave_source, leave_receipt,
              event.get("source_stamp"), event.get("receipt_monotonic"))
    if any(value is None or not math.isfinite(float(value))
           for value in values):
        return False
    return (
        enter_source is not None and leave_source is not None and
        leave_receipt is not None and
        enter_source <= event["source_stamp"] <= leave_source and
        event["receipt_monotonic"] <= leave_receipt)


def correlate_admission_events(candidate_events, selected_events, result,
                               image_receipts):
    """Join cross-topic events without depending on ROS callback order."""
    output = {
        "p_confirm": False,
        "p_selected": False,
        "stable_id": None,
        "confirmation_exposure_sec": None,
        "confirmation_processing_ms": None,
        "processing_receipt_reordered": False,
    }
    confirms = [event for event in candidate_events
                if event_inside_trial_window(event, result)]
    if not confirms:
        return output
    confirmation = min(confirms, key=lambda event: (
        event["receipt_monotonic"], event["source_stamp"],
        event["stable_id"]))
    output["p_confirm"] = True
    output["stable_id"] = confirmation["stable_id"]
    output["confirmation_exposure_sec"] = max(
        0.0, confirmation["source_stamp"] - result["enter_source_stamp"])
    image_receipt = image_receipts.get(confirmation["stamp_key"])
    if image_receipt is not None:
        delta = confirmation["receipt_monotonic"] - image_receipt
        output["processing_receipt_reordered"] = delta < 0.0
        output["confirmation_processing_ms"] = max(0.0, delta) * 1000.0
    output["p_selected"] = any(
        event["stable_id"] == output["stable_id"] and
        event_inside_trial_window(event, result)
        for event in selected_events)
    return output


def summarize_trial_results(results, run_mode, actual_fps=None,
                            terminal_context=None):
    finalized = [finalize_trial_result(result) for result in results]
    completed = [result for result in finalized
                 if result.get("status") == "completed"]
    confirmation_exposure = [
        result["confirmation_exposure_sec"] for result in completed
        if result.get("confirmation_exposure_sec") is not None]
    confirmation_processing = [
        result["confirmation_processing_ms"] for result in completed
        if result.get("confirmation_processing_ms") is not None]
    raw_map_errors = [
        float(value) for result in results
        if result.get("status") == "completed"
        for value in result.get("map_errors_xy", [])]
    eligible_frames = sum(int(result.get("eligible_frames", 0))
                          for result in completed)
    stage_frame_counts = {
        field: sum(int(result.get(field, 0)) for result in completed)
        for field in STAGE_COUNT_FIELDS
    }
    detection_frames = sum(int(result.get("detection_frames", 0))
                           for result in completed)
    map_valid_frames = sum(int(result.get("map_valid_frames", 0))
                           for result in completed)
    tf_failure_frames = sum(int(result.get("tf_failure_frames", 0))
                            for result in results
                            if result.get("status") == "completed")
    validation_errors = list(
        (terminal_context or {}).get("validation_errors", []))
    terminal_complete = (terminal_context or {}).get("run_complete", False)
    if terminal_complete:
        expected_count = (terminal_context or {}).get(
            "expected_trial_count", EXPECTED_TRIAL_COUNT)
        if int(expected_count) != EXPECTED_TRIAL_COUNT:
            validation_errors.append("expected_trial_count_must_be_23")
        if len(finalized) != int(expected_count):
            validation_errors.append("trial_count_{}/{}".format(
                len(finalized), int(expected_count)))
        if len(completed) != len(finalized):
            validation_errors.append("completed_trials_{}/{}".format(
                len(completed), len(finalized)))
        for result in completed:
            if not result.get("entered_fully_in_frame"):
                validation_errors.append(
                    "{}:never_entered_fully_in_frame".format(
                        result.get("trial_id", "unknown")))
            if not result.get("left_fully_in_frame"):
                validation_errors.append(
                    "{}:never_left_fully_in_frame".format(
                        result.get("trial_id", "unknown")))
        validation_errors = sorted(set(validation_errors))
    if run_mode == "dry_run":
        status = "DRY_RUN"
    elif not terminal_complete:
        status = "INCOMPLETE"
    else:
        status = "INVALID" if validation_errors else "MEASURED"
    failure_stage_counts = {
        stage: sum(result.get("failure_stage") == stage
                   for result in completed)
        for stage in sorted({result.get("failure_stage")
                             for result in completed
                             if result.get("failure_stage")})
    }
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04",
        "run_mode": run_mode,
        "status": status,
        "trial_count": len(finalized),
        "completed_trial_count": len(completed),
        "validation_errors": validation_errors,
        "metrics": {
            "p_confirm": (
                sum(bool(result.get("p_confirm")) for result in completed) /
                float(len(completed)) if completed else None),
            "p_selected": (
                sum(bool(result.get("p_selected")) for result in completed) /
                float(len(completed)) if completed else None),
            "p_interrupt": None,
            "p_interrupt_reason": "visual_only_no_navigation_acceptance_event",
            "stage_frame_rates": {
                field.replace("_frames", "_rate"): _ratio(
                    count, eligible_frames)
                for field, count in stage_frame_counts.items()
            },
            "failure_stage_counts": failure_stage_counts,
            "median_confirmation_exposure_sec": (
                statistics.median(confirmation_exposure)
                if confirmation_exposure else None),
            "p95_confirmation_exposure_sec": percentile(
                confirmation_exposure, 95),
            "p95_confirmation_processing_ms": percentile(
                confirmation_processing, 95),
            "map_invalid_rate": _ratio(
                max(0, detection_frames - map_valid_frames),
                detection_frames),
            "map_unavailable_rate": _ratio(
                max(0, eligible_frames - map_valid_frames), eligible_frames),
            "tf_failure_rate": _ratio(tf_failure_frames, detection_frames),
            "mean_map_error_xy": (
                statistics.mean(raw_map_errors) if raw_map_errors else None),
            "p95_map_error_xy": percentile(raw_map_errors, 95),
            "actual_image_fps": actual_fps,
            "actual_image_source_fps": (terminal_context or {}).get(
                "actual_image_source_fps"),
        },
        "metric_denominators": {
            "completed_trials": len(completed),
            "eligible_frames": eligible_frames,
            **stage_frame_counts,
            "detection_frames": detection_frames,
            "map_valid_frames": map_valid_frames,
            "tf_failure_frames": tf_failure_frames,
            "map_error_samples": len(raw_map_errors),
            "confirmation_exposure_samples": len(confirmation_exposure),
            "confirmation_processing_samples": len(confirmation_processing),
            "processing_receipt_reordered_samples": sum(
                bool(result.get("processing_receipt_reordered"))
                for result in completed),
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
            "stage_frame_rates": (
                "per-stage presence on eligible truth frames; these are "
                "diagnostic frame coverages, not independent probabilities"),
        },
        "trials": finalized,
    }


def _atomic_json(path, value):
    temporary = "{}.tmp.{}.{}".format(path, os.getpid(), uuid.uuid4().hex)
    try:
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _write_csv(path, fields, rows):
    temporary = "{}.tmp.{}.{}".format(path, os.getpid(), uuid.uuid4().hex)
    try:
        with open(temporary, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row.get(field, "")
                                 for field in fields})
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _report(summary):
    metrics = summary["metrics"]
    denominators = summary["metric_denominators"]
    validation = summary.get("validation_errors", [])
    validation_label = (
        "PASS" if summary["status"] == "MEASURED" else
        summary["status"] + (
            ": " + "; ".join(validation) if validation else ""))
    return "\n".join([
        "# V-SIM-04 Vision Search Performance",
        "",
        "- Run mode: `{}`".format(summary["run_mode"]),
        "- Completed trials: `{}/{}`".format(
            summary["completed_trial_count"], summary["trial_count"]),
        "- P_confirm: `{}`".format(metrics["p_confirm"]),
        "- P_selected: `{}`".format(metrics["p_selected"]),
        "- P_interrupt: `null` (visual-only; navigation acceptance is absent)",
        "- Stage frame rates (raw class/raw geometry/resolved/refined/"
        "geometry/association/center): `{}`".format(
            metrics["stage_frame_rates"]),
        "- Failed trial first-blocking stages: `{}`".format(
            metrics["failure_stage_counts"]),
        "- P95 processing latency: `{}` ms".format(
            metrics["p95_confirmation_processing_ms"]),
        "- Median/P95 exposure: `{}` / `{}` s".format(
            metrics["median_confirmation_exposure_sec"],
            metrics["p95_confirmation_exposure_sec"]),
        "- Map-invalid rate: `{}` ({}/{})".format(
            metrics["map_invalid_rate"],
            max(0, denominators["detection_frames"] -
                denominators["map_valid_frames"]),
            denominators["detection_frames"]),
        "- Map-unavailable rate: `{}` ({}/{})".format(
            metrics["map_unavailable_rate"],
            max(0, denominators["eligible_frames"] -
                denominators["map_valid_frames"]),
            denominators["eligible_frames"]),
        "- TF-failure rate: `{}` ({}/{})".format(
            metrics["tf_failure_rate"], denominators["tf_failure_frames"],
            denominators["detection_frames"]),
        "- Mean/P95 map error: `{}` / `{}` m (n={})".format(
            metrics["mean_map_error_xy"], metrics["p95_map_error_xy"],
            denominators["map_error_samples"]),
        "- Actual image FPS: `{}`".format(metrics["actual_image_fps"]),
        "- Source/sim-time image FPS: `{}`".format(
            metrics["actual_image_source_fps"]),
        "- Processing receipt reorder samples: `{}`".format(
            denominators["processing_receipt_reordered_samples"]),
        "- Terminal validation: `{}`".format(
            validation_label),
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
                    run_mode, actual_fps=None, terminal_context=None,
                    actual_source_fps=None):
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    terminal_context = dict(terminal_context or {
        "run_complete": False, "validation_errors": []})
    terminal_context["actual_image_source_fps"] = actual_source_fps
    summary = summarize_trial_results(
        results, run_mode, actual_fps, terminal_context)
    manifest = dict(manifest)
    manifest["schema_version"] = 1
    manifest["evaluation_id"] = "V-SIM-04"
    manifest["run_mode"] = run_mode
    manifest["actual_image_fps"] = actual_fps
    manifest["actual_image_source_fps"] = actual_source_fps
    manifest["terminal_validation"] = terminal_context or {
        "run_complete": False, "validation_errors": []}
    _atomic_json(os.path.join(output_dir, "manifest.json"), manifest)
    _write_csv(os.path.join(output_dir, "frames.csv"), FRAME_FIELDS, frame_rows)
    _write_csv(os.path.join(output_dir, "events.csv"), EVENT_FIELDS, event_rows)
    _atomic_json(os.path.join(output_dir, "summary.json"), summary)
    performance_rows = [finalize_trial_result(result) for result in results]
    _write_csv(
        os.path.join(output_dir, "vision_search_performance.csv"),
        PERFORMANCE_FIELDS, performance_rows)
    report_path = os.path.join(output_dir, "report.md")
    temporary = "{}.tmp.{}.{}".format(
        report_path, os.getpid(), uuid.uuid4().hex)
    try:
        with open(temporary, "w", encoding="utf-8") as stream:
            stream.write(_report(summary))
        os.replace(temporary, report_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
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
