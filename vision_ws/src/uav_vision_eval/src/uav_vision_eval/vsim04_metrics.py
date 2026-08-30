"""V-SIM-04 trial schema, aggregation, and artifact writers."""

import csv
import copy
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
    "camera_pose_valid", "camera_pose_source_stamp",
    "camera_pose_age_sec", "camera_position_x_m", "camera_position_y_m",
    "camera_position_z_m", "camera_yaw_rad", "camera_pose_invalid_reason",
    "motion_delta_valid", "actual_linear_speed_mps",
    "actual_yaw_rate_radps", "motion_invalid_reason",
    "path_lateral_offset_m", "path_lateral_offset_normalized",
    "path_lateral_invalid_reason",
    "raw_class_present", "raw_geometry_present",
    "raw_class_confidence", "raw_geometry_confidence",
    "resolved_present", "resolved_class_confidence",
    "resolved_geometry_confidence", "refined_present",
    "geometry_verified", "center_refined", "association_valid",
    "refined_class_confidence", "refined_geometry_confidence",
    "refined_reject_reason", "detection_present",
    "mapped_class_confidence", "mapped_geometry_confidence", "map_valid",
    "transform_failure", "reject_reason", "map_error_xy",
    "detector_inference_ms", "detector_processing_ms",
    "detector_callback_start_monotonic_sec",
    "detector_callback_end_monotonic_sec",
    "detector_perf_receipt_monotonic_sec",
    "current_confirmed", "current_selected", "stable_id",
]
EVENT_FIELDS = [
    "event_seq", "trial_id", "event", "source_stamp", "monotonic_sec",
    "class_name", "stable_id", "details",
]
PERFORMANCE_FIELDS = [
    "trial_id", "kind", "class_name", "height_m", "speed_mps", "status",
    "measurement_completeness_status", "artifact_set_complete",
    "performance_verdict",
    "performance_hard_failure", "performance_failure_reasons",
    "performance_metric_failure_reasons",
    "p_confirm", "p_selected", "p_interrupt", "stable_id",
    "confirmation_exposure_sec", "confirmation_processing_ms",
    "confirmation_pipeline_ms", "processing_receipt_reordered",
    "stage_trace_enabled", "complete_mapped_frames",
    "partial_only_mapped_frames", "complete_mapped_rate",
    "partial_source_sets", "p95_detector_inference_ms",
    "p95_detector_processing_ms",
    "eligible_frames", "raw_class_frames", "raw_geometry_frames",
    "resolved_frames", "refined_frames", "geometry_verified_frames",
    "association_valid_frames", "center_refined_frames",
    "detection_frames", "map_valid_frames", "failure_stage",
    "map_invalid_rate", "map_unavailable_rate", "tf_failure_rate",
    "mean_map_error_xy", "p95_map_error_xy", "map_error_sample_count",
    "entered_fully_in_frame", "left_fully_in_frame",
    "expected_duration_sec", "actual_duration_sec", "expected_speed_mps",
    "actual_speed_mps",
    "camera_pose_frame_count", "motion_sample_count",
    "lateral_offset_sample_count", "mean_actual_linear_speed_mps",
    "p95_actual_linear_speed_mps", "mean_abs_actual_yaw_rate_radps",
    "p95_abs_actual_yaw_rate_radps",
    "mean_abs_normalized_lateral_offset",
    "p95_abs_normalized_lateral_offset",
    "class_group_completed_trials", "class_group_p_confirm",
    "class_group_p_selected", "class_group_mean_actual_linear_speed_mps",
    "class_group_p95_abs_normalized_lateral_offset",
    "height_group_completed_trials", "height_group_p_confirm",
    "height_group_p_selected", "height_group_mean_actual_linear_speed_mps",
    "height_group_p95_abs_normalized_lateral_offset",
    "speed_group_completed_trials", "speed_group_p_confirm",
    "speed_group_p_selected", "speed_group_mean_actual_linear_speed_mps",
    "speed_group_p95_abs_normalized_lateral_offset",
    "confirmed_observations", "unexpected_confirmed_observations",
    "disallowed_confirmed_observations", "unique_confirmed_targets",
    "policy_rejected_confirmed_observations",
    "selected_observations", "unexpected_selected_observations",
    "disallowed_selected_observations", "unique_selected_targets",
    "policy_rejected_selected_observations",
    "tank_confirmed_observations", "tank_selected_observations",
]
REQUIRED_ARTIFACTS = (
    "manifest.json", "frames.csv", "events.csv", "summary.json",
    "report.md", "vision_search_performance.csv",
)
EXPECTED_TRIAL_COUNT = 23
CONFIRMED_STATE = 2
AUDIT_EVENT_KINDS = ("confirmed", "selected")


def candidate_audit_observation(event_kind, class_name, stable_id,
                                expected_class, allowed_classes,
                                state=None, policy_selectable=False,
                                trial_id="", source_stamp=None):
    """Build one JSON-safe audit observation without filtering by class.

    ``confirmed`` means the sticky target-memory state is CONFIRMED.  It does
    not imply that the current observation is selectable.  ``selected`` is
    recorded exactly as published, including malformed or disallowed values.
    """
    event_kind = str(event_kind).strip().lower()
    if event_kind not in AUDIT_EVENT_KINDS:
        raise ValueError("unsupported candidate audit event: " + event_kind)
    class_name = str(class_name).strip()
    expected_class = str(expected_class or "").strip()
    try:
        stable_id = int(stable_id)
    except (TypeError, ValueError, OverflowError):
        stable_id = -1
    allowed = class_name in {str(value) for value in allowed_classes}
    return {
        "event_kind": event_kind,
        "trial_id": str(trial_id or ""),
        "class_name": class_name,
        "stable_id": stable_id,
        "state": None if state is None else int(state),
        "expected_class": expected_class,
        "unexpected": bool(expected_class and class_name != expected_class),
        "allowed_by_profile": allowed,
        "disallowed_by_profile": not allowed,
        "policy_selectable": bool(policy_selectable),
        "source_stamp": source_stamp,
    }


def candidate_audit_summary(observations, class_profile=""):
    """Aggregate all confirmed/selected observations and unique identities."""
    records = [dict(record) for record in (observations or [])]
    summary = {
        "class_profile": str(class_profile or ""),
        "observation_count": len(records),
        "unscoped_observation_count": sum(
            not str(record.get("trial_id", "")) for record in records),
        "by_class": {},
        "trials": {},
    }

    def aggregate(subset):
        values = {}
        for event_kind in AUDIT_EVENT_KINDS:
            matching = [record for record in subset
                        if record.get("event_kind") == event_kind]
            unique = sorted({
                "{}:{}".format(record.get("class_name", ""),
                               int(record.get("stable_id", -1)))
                for record in matching
            })
            values[event_kind] = {
                "observations": len(matching),
                "unexpected_observations": sum(
                    bool(record.get("unexpected")) for record in matching),
                "disallowed_observations": sum(
                    bool(record.get("disallowed_by_profile"))
                    for record in matching),
                "policy_rejected_observations": sum(
                    not bool(record.get("policy_selectable"))
                    for record in matching),
                "tank_observations": sum(
                    record.get("class_name") == "tank" for record in matching),
                "unique_targets": unique,
                "unique_target_count": len(unique),
            }
        return values

    summary.update(aggregate(records))
    classes = sorted({str(record.get("class_name", "")) for record in records
                      if str(record.get("class_name", ""))})
    summary["by_class"] = {
        class_name: aggregate([
            record for record in records
            if record.get("class_name") == class_name
        ]) for class_name in classes
    }
    trial_ids = sorted({str(record.get("trial_id", "")) for record in records
                        if str(record.get("trial_id", ""))})
    summary["trials"] = {
        trial_id: aggregate([
            record for record in records
            if record.get("trial_id") == trial_id
        ]) for trial_id in trial_ids
    }
    return summary


def evaluate_performance_verdict(metrics, completeness_status,
                                 evaluation_scope, candidate_audit,
                                 contract=None):
    """Evaluate only thresholds that an input contract explicitly freezes."""
    contract = copy.deepcopy(contract or {})
    thresholds = dict(contract.get("thresholds", {}))
    unfrozen = sorted({str(value) for value in
                       contract.get("unfrozen_thresholds", [])
                       if str(value).strip()})
    checks = []
    failures = []
    hard_failures = []

    metric_specs = {
        "max_p95_confirmation_processing_ms": (
            "p95_confirmation_processing_ms", "max"),
        "max_p95_map_error_xy_m": ("p95_map_error_xy", "max"),
        "max_tf_failure_rate": ("tf_failure_rate", "max"),
        "min_p_confirm": ("p_confirm", "min"),
        "min_p_selected": ("p_selected", "min"),
    }
    for threshold_name, threshold_value in sorted(thresholds.items()):
        if threshold_name not in metric_specs:
            failures.append("unsupported_threshold:" + threshold_name)
            continue
        metric_name, operator = metric_specs[threshold_name]
        actual = metrics.get(metric_name)
        try:
            threshold_value = float(threshold_value)
            valid_threshold = math.isfinite(threshold_value)
        except (TypeError, ValueError, OverflowError):
            valid_threshold = False
        passed = False
        if not valid_threshold:
            reason = "invalid_threshold:" + threshold_name
        elif actual is None:
            reason = "metric_missing:" + metric_name
        else:
            actual = float(actual)
            passed = (actual <= threshold_value if operator == "max" else
                      actual >= threshold_value)
            reason = "" if passed else "threshold_failed:{}:{}:{}".format(
                metric_name, actual, threshold_value)
        checks.append({
            "threshold": threshold_name,
            "metric": metric_name,
            "operator": operator,
            "limit": threshold_value,
            "actual": actual,
            "passed": passed,
            "failure_reason": reason,
        })
        if reason:
            failures.append(reason)

    selected = candidate_audit.get("selected", {})
    disallowed_selected = int(selected.get("disallowed_observations", 0))
    tank_selected = int(selected.get("tank_observations", 0))
    policy_rejected_selected = int(
        selected.get("policy_rejected_observations", 0))
    if disallowed_selected:
        hard_failures.append(
            "disallowed_selected_observations:{}".format(disallowed_selected))
    if str(candidate_audit.get("class_profile", "")) == "r2026" and tank_selected:
        hard_failures.append(
            "r2026_tank_selected_observations:{}".format(tank_selected))
    if policy_rejected_selected:
        hard_failures.append(
            "selected_rejected_by_current_policy_observations:{}".format(
                policy_rejected_selected))

    eligible_statuses = {"MEASURED", "DIAGNOSTIC"}
    if completeness_status not in eligible_statuses:
        status = "NOT_EVALUATED"
        reasons = ["measurement_not_complete:" + str(completeness_status)]
    elif hard_failures:
        status = "FAIL"
        reasons = sorted(set(hard_failures + failures))
    elif evaluation_scope == "diagnostic":
        status = "DIAGNOSTIC_ONLY"
        reasons = ["diagnostic_subset_not_gate"]
    elif failures:
        status = "FAIL"
        reasons = sorted(set(failures))
    elif not contract:
        status = "NOT_GATED"
        reasons = ["performance_contract_missing"]
    elif unfrozen:
        status = "NOT_GATED"
        reasons = ["threshold_unfrozen:" + name for name in unfrozen]
    else:
        status = "PASS"
        reasons = []
    return {
        "status": status,
        "is_gate_pass": status == "PASS",
        "hard_failure": bool(hard_failures),
        "hard_failure_reasons": sorted(set(hard_failures)),
        "failure_reasons": reasons,
        "metric_failure_reasons": sorted(set(failures)),
        "contract_id": str(contract.get("contract_id", "")),
        "contract_sources": list(contract.get("sources", [])),
        "threshold_checks": checks,
        "unfrozen_thresholds": unfrozen,
        "meaning": (
            "Algorithm verdict; independent from artifact and measurement "
            "completeness. Diagnostic subsets are never Gate PASS."),
    }


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
    contract = matrix.get("performance_contract", {})
    if not isinstance(contract, dict):
        raise ValueError("performance_contract must be a mapping")
    thresholds = contract.get("thresholds", {})
    if not isinstance(thresholds, dict):
        raise ValueError("performance_contract thresholds must be a mapping")
    for name, value in thresholds.items():
        try:
            valid = math.isfinite(float(value)) and float(value) >= 0.0
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            raise ValueError("invalid performance threshold: " + str(name))
    unfrozen = contract.get("unfrozen_thresholds", [])
    if not isinstance(unfrozen, list) or any(
            not str(value).strip() for value in unfrozen):
        raise ValueError(
            "performance_contract unfrozen_thresholds must be a list")
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
    trial_ids = {trial["trial_id"] for trial in trials}
    slices = matrix.get("trial_slices", {})
    if slices is None:
        slices = {}
    if not isinstance(slices, dict):
        raise ValueError("trial_slices must be a mapping")
    for slice_name, identifiers in slices.items():
        if not str(slice_name).strip() or not isinstance(identifiers, list):
            raise ValueError("trial slice names and values must be non-empty lists")
        values = [str(value).strip() for value in identifiers]
        if not values or any(not value for value in values):
            raise ValueError("trial slice {} is empty or invalid".format(
                slice_name))
        if len(values) != len(set(values)):
            raise ValueError("trial slice {} contains duplicates".format(
                slice_name))
        unknown = sorted(set(values) - trial_ids)
        if unknown:
            raise ValueError("trial slice {} has unknown IDs: {}".format(
                slice_name, ",".join(unknown)))
        slices[slice_name] = values
    matrix["trial_slices"] = slices
    return matrix


def select_trial_matrix(matrix, selector, slice_name=""):
    """Select an ordered diagnostic subset without weakening the full Gate."""
    selected_matrix = copy.deepcopy(matrix)
    slice_name = str(slice_name or "").strip()
    if slice_name and str(selector or "").strip():
        raise ValueError("trial_selector and trial_slice are mutually exclusive")
    if slice_name:
        slices = selected_matrix.get("trial_slices", {})
        if slice_name not in slices:
            raise ValueError("unknown diagnostic trial slice: " + slice_name)
        selector = slices[slice_name]
    if isinstance(selector, (list, tuple)):
        identifiers = [str(value).strip() for value in selector
                       if str(value).strip()]
    else:
        identifiers = [value.strip() for value in str(selector or "").split(",")
                       if value.strip()]
    if not identifiers:
        selected_matrix["evaluation_scope"] = "full"
        selected_matrix["trial_selector"] = []
        selected_matrix["trial_slice"] = ""
        return selected_matrix
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("trial selector contains duplicate identifiers")
    trials_by_id = {
        trial["trial_id"]: trial for trial in selected_matrix["trials"]
    }
    unknown = [identifier for identifier in identifiers
               if identifier not in trials_by_id]
    if unknown:
        raise ValueError("unknown diagnostic trial identifiers: " +
                         ",".join(unknown))
    selected_matrix["trials"] = [trials_by_id[identifier]
                                  for identifier in identifiers]
    selected_matrix["expected_trial_count"] = len(identifiers)
    selected_matrix["evaluation_scope"] = "diagnostic"
    selected_matrix["trial_selector"] = identifiers
    selected_matrix["trial_slice"] = slice_name
    return selected_matrix


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


def quaternion_yaw(x_value, y_value, z_value, w_value):
    """Return finite ZYX yaw in radians, or None for an invalid quaternion."""
    try:
        values = [float(value) for value in (
            x_value, y_value, z_value, w_value)]
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        return None
    x_value, y_value, z_value, w_value = [
        value / norm for value in values]
    sin_yaw = 2.0 * (w_value * z_value + x_value * y_value)
    cos_yaw = 1.0 - 2.0 * (
        y_value * y_value + z_value * z_value)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return yaw if math.isfinite(yaw) else None


def _shortest_angle_delta(current, previous):
    return math.atan2(
        math.sin(float(current) - float(previous)),
        math.cos(float(current) - float(previous)))


def _finite_frame_value(row, field):
    try:
        value = float(row.get(field, ""))
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def motion_frame_sort_key(row):
    """Return a deterministic source-time-first order for one trial's rows."""
    image_stamp = _finite_frame_value(row, "stamp")
    pose_stamp = _finite_frame_value(row, "camera_pose_source_stamp")
    source_or_image = pose_stamp if pose_stamp is not None else image_stamp
    return (
        source_or_image is None,
        0.0 if source_or_image is None else source_or_image,
        image_stamp is None,
        0.0 if image_stamp is None else image_stamp,
        str(row.get("target_id", "")),
        str(row.get("class_name", "")),
        str(row.get("stable_id", "")),
    )


def _dynamic_path(trajectory):
    try:
        start_x = float(trajectory["start_x"])
        start_y = float(trajectory["start_y"])
        finish_x = float(trajectory["finish_x"])
        finish_y = float(trajectory["finish_y"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    values = (start_x, start_y, finish_x, finish_y)
    if not all(math.isfinite(value) for value in values):
        return None
    dx_value = finish_x - start_x
    dy_value = finish_y - start_y
    length = math.hypot(dx_value, dy_value)
    if length <= 1.0e-9:
        return None
    return start_x, start_y, dx_value, dy_value, length


def _dynamic_trajectory_window(trajectory):
    try:
        start_stamp = float(trajectory["motion_start_source_stamp"])
        end_stamp = float(trajectory["motion_end_source_stamp"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if (not math.isfinite(start_stamp) or not math.isfinite(end_stamp) or
            end_stamp < start_stamp):
        return None
    return start_stamp, end_stamp


def _path_progress_m(path, position):
    start_x, start_y, dx_value, dy_value, path_length = path
    return (
        (position[0] - start_x) * dx_value +
        (position[1] - start_y) * dy_value) / path_length


def _reset_regression_tolerance_m(trajectory, path_length):
    try:
        steps = int(trajectory.get("steps", 0))
    except (TypeError, ValueError, OverflowError):
        steps = 0
    planned_step_m = path_length / steps if steps > 0 else 0.0
    return max(0.25, 2.0 * planned_step_m)


def _trajectory_start_reference(trajectory, path, pose_stamp, window_start):
    """Return the expected first-pose XY and a fail-closed sampling slack."""
    start_x, start_y, dx_value, dy_value, path_length = path
    try:
        speed_mps = float(trajectory.get("expected_speed_mps", 0.0))
        elapsed_sec = float(pose_stamp) - float(window_start)
    except (TypeError, ValueError, OverflowError):
        return start_x, start_y, 0.05
    if (not math.isfinite(speed_mps) or not math.isfinite(elapsed_sec) or
            speed_mps < 0.0 or elapsed_sec < 0.0):
        return start_x, start_y, 0.05

    expected_progress_m = min(path_length, speed_mps * elapsed_sec)
    reference_x = start_x + dx_value * expected_progress_m / path_length
    reference_y = start_y + dy_value * expected_progress_m / path_length

    planned_step_m = None
    try:
        update_rate_hz = float(trajectory.get("update_rate_hz", 0.0))
        if math.isfinite(update_rate_hz) and update_rate_hz > 0.0:
            planned_step_m = speed_mps / update_rate_hz
    except (TypeError, ValueError, OverflowError):
        pass
    if planned_step_m is None:
        try:
            steps = int(trajectory.get("steps", 0))
            if steps > 0:
                planned_step_m = path_length / steps
        except (TypeError, ValueError, OverflowError):
            pass
    sampling_slack_m = max(
        0.05, 1.5 * (planned_step_m if planned_step_m is not None else 0.0))
    return reference_x, reference_y, sampling_slack_m


def annotate_motion_frames(frame_rows, trial_kind, trajectory):
    """Annotate rows with fail-closed motion telemetry.

    Rows are processed in deterministic matched-pose-source/image time order,
    independent of callback arrival order.  Linear speed is the 3-D distance
    between adjacent valid in-window pose samples divided by their strictly
    positive pose-source-stamp delta.  Yaw rate uses the shortest signed
    ZYX-yaw delta.  The signed lateral offset is the horizontal cross-track
    distance to the planned start-to-finish line; its normalized value divides
    that distance by the planned path length.  Dynamic pre-start resets,
    post-finish samples and backwards reset jumps do not enter motion samples.
    Static trials do not have a motion path, and their speed/yaw/lateral cells
    stay empty.
    """
    is_dynamic = str(trial_kind) == "dynamic"
    path = _dynamic_path(trajectory) if is_dynamic else None
    trajectory_window = (
        _dynamic_trajectory_window(trajectory) if is_dynamic else None)
    reset_tolerance_m = (
        _reset_regression_tolerance_m(trajectory, path[4])
        if path is not None else None)
    previous = None
    linear_samples = []
    yaw_rate_samples = []
    lateral_samples = []
    pose_count = 0
    for row in sorted(frame_rows, key=motion_frame_sort_key):
        row.update({
            "motion_delta_valid": False,
            "actual_linear_speed_mps": "",
            "actual_yaw_rate_radps": "",
            "motion_invalid_reason": "",
            "path_lateral_offset_m": "",
            "path_lateral_offset_normalized": "",
            "path_lateral_invalid_reason": "",
        })
        image_stamp = _finite_frame_value(row, "stamp")
        pose_stamp = _finite_frame_value(row, "camera_pose_source_stamp")
        position = tuple(_finite_frame_value(row, field) for field in (
            "camera_position_x_m", "camera_position_y_m",
            "camera_position_z_m"))
        yaw = _finite_frame_value(row, "camera_yaw_rad")
        pose_valid = bool(row.get("camera_pose_valid"))
        pose_valid = bool(
            pose_valid and image_stamp is not None and pose_stamp is not None and
            pose_stamp <= image_stamp and yaw is not None and
            all(value is not None for value in position))
        if not pose_valid:
            row["camera_pose_valid"] = False
            if not str(row.get("camera_pose_invalid_reason", "")).strip():
                row["camera_pose_invalid_reason"] = (
                    "camera_pose_fields_invalid")
            row["motion_invalid_reason"] = "camera_pose_missing_or_invalid"
            row["path_lateral_invalid_reason"] = (
                "static_trial" if not is_dynamic else
                "camera_pose_missing_or_invalid")
            continue
        pose_count += 1

        if not is_dynamic:
            row["motion_invalid_reason"] = "static_trial"
            row["path_lateral_invalid_reason"] = "static_trial"
            continue

        if trajectory_window is None:
            row["motion_invalid_reason"] = "dynamic_motion_window_invalid"
            row["path_lateral_invalid_reason"] = (
                "dynamic_motion_window_invalid")
            continue
        window_start, window_end = trajectory_window
        if pose_stamp < window_start:
            row["motion_invalid_reason"] = "trajectory_reset_prestart"
            row["path_lateral_invalid_reason"] = (
                "trajectory_reset_prestart")
            continue
        if pose_stamp > window_end:
            row["motion_invalid_reason"] = "trajectory_complete"
            row["path_lateral_invalid_reason"] = "trajectory_complete"
            continue

        if path is None:
            row["path_lateral_invalid_reason"] = "dynamic_path_invalid"
            progress_m = None
        else:
            start_x, start_y, dx_value, dy_value, path_length = path
            cross_track_m = (
                dx_value * (position[1] - start_y) -
                dy_value * (position[0] - start_x)) / path_length
            normalized = cross_track_m / path_length
            row["path_lateral_offset_m"] = cross_track_m
            row["path_lateral_offset_normalized"] = normalized
            lateral_samples.append(normalized)
            progress_m = _path_progress_m(path, position)

        current = (pose_stamp, position, yaw, progress_m)
        if previous is None:
            if path is not None:
                reference_x, reference_y, start_tolerance_m = (
                    _trajectory_start_reference(
                        trajectory, path, pose_stamp, window_start))
                if math.hypot(
                        position[0] - reference_x,
                        position[1] - reference_y) > start_tolerance_m:
                    row["path_lateral_offset_m"] = ""
                    row["path_lateral_offset_normalized"] = ""
                    lateral_samples.pop()
                    row["motion_invalid_reason"] = (
                        "trajectory_start_pose_not_ready")
                    row["path_lateral_invalid_reason"] = (
                        "trajectory_start_pose_not_ready")
                    continue
            row["motion_invalid_reason"] = "first_valid_pose"
            previous = current
            continue
        delta_sec = pose_stamp - previous[0]
        if not math.isfinite(delta_sec) or delta_sec < 0.0:
            row["motion_invalid_reason"] = "non_monotonic_stamp"
            continue
        if delta_sec == 0.0:
            row["motion_invalid_reason"] = "duplicate_pose_stamp"
            continue
        if (progress_m is not None and previous[3] is not None and
                progress_m < previous[3] - reset_tolerance_m):
            row["motion_invalid_reason"] = "trajectory_reset_jump"
            previous = current
            continue
        distance = math.sqrt(sum(
            (position[index] - previous[1][index]) ** 2
            for index in range(3)))
        linear_speed = distance / delta_sec
        yaw_rate = _shortest_angle_delta(yaw, previous[2]) / delta_sec
        if not all(math.isfinite(value) for value in (
                linear_speed, yaw_rate)):
            row["motion_invalid_reason"] = "motion_delta_nonfinite"
            continue
        row["motion_delta_valid"] = True
        row["actual_linear_speed_mps"] = linear_speed
        row["actual_yaw_rate_radps"] = yaw_rate
        linear_samples.append(linear_speed)
        yaw_rate_samples.append(yaw_rate)
        previous = current

    abs_yaw_rates = [abs(value) for value in yaw_rate_samples]
    abs_lateral = [abs(value) for value in lateral_samples]
    return {
        "camera_pose_frame_count": pose_count,
        "motion_sample_count": len(linear_samples),
        "lateral_offset_sample_count": len(lateral_samples),
        "actual_linear_speed_mps_samples": linear_samples,
        "actual_yaw_rate_radps_samples": yaw_rate_samples,
        "normalized_lateral_offset_samples": lateral_samples,
        "mean_actual_linear_speed_mps": (
            statistics.mean(linear_samples) if linear_samples else None),
        "p95_actual_linear_speed_mps": percentile(linear_samples, 95),
        "mean_abs_actual_yaw_rate_radps": (
            statistics.mean(abs_yaw_rates) if abs_yaw_rates else None),
        "p95_abs_actual_yaw_rate_radps": percentile(abs_yaw_rates, 95),
        "mean_abs_normalized_lateral_offset": (
            statistics.mean(abs_lateral) if abs_lateral else None),
        "p95_abs_normalized_lateral_offset": percentile(abs_lateral, 95),
    }


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
        "confirmation_pipeline_ms": None,
        "processing_receipt_reordered": False,
        "stage_trace_enabled": True,
        "complete_mapped_frames": 0,
        "partial_only_mapped_frames": 0,
        "complete_mapped_rate": None,
        "partial_source_sets": "{}",
        "detector_inference_ms_samples": [],
        "detector_processing_ms_samples": [],
        "p95_detector_inference_ms": None,
        "p95_detector_processing_ms": None,
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
        "camera_pose_frame_count": 0,
        "motion_sample_count": 0,
        "lateral_offset_sample_count": 0,
        "actual_linear_speed_mps_samples": [],
        "actual_yaw_rate_radps_samples": [],
        "normalized_lateral_offset_samples": [],
        "mean_actual_linear_speed_mps": None,
        "p95_actual_linear_speed_mps": None,
        "mean_abs_actual_yaw_rate_radps": None,
        "p95_abs_actual_yaw_rate_radps": None,
        "mean_abs_normalized_lateral_offset": None,
        "p95_abs_normalized_lateral_offset": None,
    })
    return result


def finalize_trial_result(result):
    result = dict(result)
    eligible = int(result.get("eligible_frames", 0))
    detected = int(result.get("detection_frames", 0))
    valid = int(result.get("map_valid_frames", 0))
    tf_failures = int(result.get("tf_failure_frames", 0))
    errors = [float(value) for value in result.pop("map_errors_xy", [])]
    detector_inference = [float(value) for value in result.pop(
        "detector_inference_ms_samples", [])]
    detector_processing = [float(value) for value in result.pop(
        "detector_processing_ms_samples", [])]
    linear_speed = [float(value) for value in result.pop(
        "actual_linear_speed_mps_samples", [])]
    yaw_rate = [float(value) for value in result.pop(
        "actual_yaw_rate_radps_samples", [])]
    lateral_offset = [float(value) for value in result.pop(
        "normalized_lateral_offset_samples", [])]
    result["map_invalid_rate"] = (
        max(0, detected - valid) / float(detected) if detected else None)
    result["map_unavailable_rate"] = (
        max(0, eligible - valid) / float(eligible) if eligible else None)
    result["tf_failure_rate"] = (
        tf_failures / float(detected) if detected else None)
    result["mean_map_error_xy"] = statistics.mean(errors) if errors else None
    result["p95_map_error_xy"] = percentile(errors, 95)
    result["map_error_sample_count"] = len(errors)
    result["p95_detector_inference_ms"] = percentile(
        detector_inference, 95)
    result["p95_detector_processing_ms"] = percentile(
        detector_processing, 95)
    result["camera_pose_frame_count"] = int(result.get(
        "camera_pose_frame_count", 0))
    result["motion_sample_count"] = len(linear_speed)
    result["lateral_offset_sample_count"] = len(lateral_offset)
    result["mean_actual_linear_speed_mps"] = (
        statistics.mean(linear_speed) if linear_speed else None)
    result["p95_actual_linear_speed_mps"] = percentile(linear_speed, 95)
    absolute_yaw_rate = [abs(value) for value in yaw_rate]
    result["mean_abs_actual_yaw_rate_radps"] = (
        statistics.mean(absolute_yaw_rate) if absolute_yaw_rate else None)
    result["p95_abs_actual_yaw_rate_radps"] = percentile(
        absolute_yaw_rate, 95)
    absolute_lateral = [abs(value) for value in lateral_offset]
    result["mean_abs_normalized_lateral_offset"] = (
        statistics.mean(absolute_lateral) if absolute_lateral else None)
    result["p95_abs_normalized_lateral_offset"] = percentile(
        absolute_lateral, 95)
    mapped_buckets = (
        int(result.get("complete_mapped_frames", 0)) +
        int(result.get("partial_only_mapped_frames", 0)))
    result["complete_mapped_rate"] = _ratio(
        int(result.get("complete_mapped_frames", 0)), mapped_buckets)
    result.pop("tf_failure_frames", None)
    result["failure_stage"] = classify_failure_stage(result)
    return result


def classify_failure_stage(result):
    """Return the first pipeline stage that blocked trial confirmation."""
    if result.get("status") != "completed" or result.get("p_confirm"):
        return ""
    if int(result.get("eligible_frames", 0)) <= 0:
        return "truth_visibility"
    if not bool(result.get("stage_trace_enabled", True)):
        return "stage_trace_disabled"
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
                               image_receipts, detector_callback_starts=None):
    """Join cross-topic events without depending on ROS callback order."""
    output = {
        "p_confirm": False,
        "p_selected": False,
        "stable_id": None,
        "confirmation_exposure_sec": None,
        "confirmation_processing_ms": None,
        "confirmation_pipeline_ms": None,
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
    detector_start = (detector_callback_starts or {}).get(
        confirmation["stamp_key"])
    if detector_start is not None:
        delta = confirmation["receipt_monotonic"] - detector_start
        if math.isfinite(delta) and delta >= 0.0:
            output["confirmation_pipeline_ms"] = delta * 1000.0
    output["p_selected"] = any(
        event["stable_id"] == output["stable_id"] and
        event_inside_trial_window(event, result)
        for event in selected_events)
    return output


def _group_value(result, field):
    value = result.get(field)
    if field in {"height_m", "speed_mps"} and value is not None:
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return numeric if math.isfinite(numeric) else None
    return str(value) if field == "class_name" else value


def _group_label(field, value):
    if field == "class_name":
        return str(value)
    if field == "height_m":
        return "{:.3g} m".format(float(value))
    if value is None:
        return "static"
    return "{:.3g} m/s".format(float(value))


def _group_sort_key(value):
    if value is None:
        return (0, 0.0)
    if isinstance(value, (int, float)):
        return (1, float(value))
    return (1, str(value))


def _finite_samples(result, field):
    samples = []
    for value in result.get(field, []):
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(value):
            samples.append(value)
    return samples


def build_metric_breakdowns(results):
    """Aggregate completed trials by class, height and requested speed."""
    dimensions = (
        ("by_class", "class_name"),
        ("by_height_m", "height_m"),
        ("by_speed_mps", "speed_mps"),
    )
    breakdowns = {}
    for output_name, field in dimensions:
        values = sorted(
            {_group_value(result, field) for result in results},
            key=_group_sort_key)
        groups = []
        for value in values:
            members = [result for result in results
                       if _group_value(result, field) == value]
            completed = [result for result in members
                         if result.get("status") == "completed"]
            linear = [sample for result in completed
                      for sample in _finite_samples(
                          result, "actual_linear_speed_mps_samples")]
            yaw_rate = [abs(sample) for result in completed
                        for sample in _finite_samples(
                            result, "actual_yaw_rate_radps_samples")]
            lateral = [abs(sample) for result in completed
                       for sample in _finite_samples(
                           result, "normalized_lateral_offset_samples")]
            map_errors = [sample for result in completed
                          for sample in _finite_samples(
                              result, "map_errors_xy")]
            groups.append({
                "dimension": field,
                "value": value,
                "label": _group_label(field, value),
                "trial_count": len(members),
                "completed_trial_count": len(completed),
                "p_confirm": (
                    sum(bool(result.get("p_confirm"))
                        for result in completed) / float(len(completed))
                    if completed else None),
                "p_selected": (
                    sum(bool(result.get("p_selected"))
                        for result in completed) / float(len(completed))
                    if completed else None),
                "eligible_frames": sum(
                    int(result.get("eligible_frames", 0))
                    for result in completed),
                "camera_pose_frame_count": sum(
                    int(result.get("camera_pose_frame_count", 0))
                    for result in completed),
                "motion_sample_count": len(linear),
                "lateral_offset_sample_count": len(lateral),
                "mean_actual_linear_speed_mps": (
                    statistics.mean(linear) if linear else None),
                "p95_actual_linear_speed_mps": percentile(linear, 95),
                "mean_abs_actual_yaw_rate_radps": (
                    statistics.mean(yaw_rate) if yaw_rate else None),
                "p95_abs_actual_yaw_rate_radps": percentile(yaw_rate, 95),
                "mean_abs_normalized_lateral_offset": (
                    statistics.mean(lateral) if lateral else None),
                "p95_abs_normalized_lateral_offset": percentile(
                    lateral, 95),
                "mean_map_error_xy": (
                    statistics.mean(map_errors) if map_errors else None),
                "p95_map_error_xy": percentile(map_errors, 95),
                "map_error_sample_count": len(map_errors),
            })
        breakdowns[output_name] = groups
    return breakdowns


def decorate_performance_rows(rows, breakdowns):
    """Append repeated group metrics without adding aggregate CSV rows."""
    lookup = {}
    for output_name, field, prefix in (
            ("by_class", "class_name", "class_group"),
            ("by_height_m", "height_m", "height_group"),
            ("by_speed_mps", "speed_mps", "speed_group")):
        lookup[field] = {
            group["value"]: (prefix, group)
            for group in breakdowns.get(output_name, [])
        }
    decorated = []
    for source in rows:
        row = dict(source)
        for field in ("class_name", "height_m", "speed_mps"):
            prefix_group = lookup[field].get(_group_value(row, field))
            if prefix_group is None:
                continue
            prefix, group = prefix_group
            row[prefix + "_completed_trials"] = group[
                "completed_trial_count"]
            row[prefix + "_p_confirm"] = group["p_confirm"]
            row[prefix + "_p_selected"] = group["p_selected"]
            row[prefix + "_mean_actual_linear_speed_mps"] = group[
                "mean_actual_linear_speed_mps"]
            row[prefix + "_p95_abs_normalized_lateral_offset"] = group[
                "p95_abs_normalized_lateral_offset"]
        decorated.append(row)
    return decorated


def summarize_trial_results(results, run_mode, actual_fps=None,
                            terminal_context=None):
    terminal_context = dict(terminal_context or {})
    finalized = [finalize_trial_result(result) for result in results]
    breakdowns = build_metric_breakdowns(results)
    completed = [result for result in finalized
                 if result.get("status") == "completed"]
    confirmation_exposure = [
        result["confirmation_exposure_sec"] for result in completed
        if result.get("confirmation_exposure_sec") is not None]
    confirmation_processing = [
        result["confirmation_processing_ms"] for result in completed
        if result.get("confirmation_processing_ms") is not None]
    confirmation_pipeline = [
        result["confirmation_pipeline_ms"] for result in completed
        if result.get("confirmation_pipeline_ms") is not None]
    detector_inference = [
        float(value) for result in results
        if result.get("status") == "completed"
        for value in result.get("detector_inference_ms_samples", [])]
    detector_processing = [
        float(value) for result in results
        if result.get("status") == "completed"
        for value in result.get("detector_processing_ms_samples", [])]
    complete_mapped_frames = sum(
        int(result.get("complete_mapped_frames", 0)) for result in completed)
    partial_only_mapped_frames = sum(
        int(result.get("partial_only_mapped_frames", 0))
        for result in completed)
    mapped_bucket_frames = complete_mapped_frames + partial_only_mapped_frames
    raw_map_errors = [
        float(value) for result in results
        if result.get("status") == "completed"
        for value in result.get("map_errors_xy", [])]
    linear_speed = [
        value for result in results
        if result.get("status") == "completed"
        for value in _finite_samples(
            result, "actual_linear_speed_mps_samples")]
    yaw_rate = [
        abs(value) for result in results
        if result.get("status") == "completed"
        for value in _finite_samples(
            result, "actual_yaw_rate_radps_samples")]
    lateral_offset = [
        abs(value) for result in results
        if result.get("status") == "completed"
        for value in _finite_samples(
            result, "normalized_lateral_offset_samples")]
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
    validation_errors = list(terminal_context.get("validation_errors", []))
    terminal_complete = terminal_context.get("run_complete", False)
    evaluation_scope = str(terminal_context.get("evaluation_scope", "full"))
    if terminal_complete:
        expected_count = terminal_context.get(
            "expected_trial_count", EXPECTED_TRIAL_COUNT)
        if evaluation_scope not in {"full", "diagnostic"}:
            validation_errors.append("evaluation_scope_invalid")
        if (evaluation_scope == "full" and
                int(expected_count) != EXPECTED_TRIAL_COUNT):
            validation_errors.append("expected_trial_count_must_be_23")
        if int(expected_count) <= 0:
            validation_errors.append("expected_trial_count_must_be_positive")
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
        status = (
            "INVALID" if validation_errors else
            "DIAGNOSTIC" if evaluation_scope == "diagnostic" else
            "MEASURED")
    failure_stage_counts = {
        stage: sum(result.get("failure_stage") == stage
                   for result in completed)
        for stage in sorted({result.get("failure_stage")
                             for result in completed
                             if result.get("failure_stage")})
    }
    metrics = {
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
        "p95_confirmation_pipeline_ms": percentile(
            confirmation_pipeline, 95),
        "p95_detector_inference_ms": percentile(detector_inference, 95),
        "p95_detector_processing_ms": percentile(detector_processing, 95),
        "complete_mapped_rate": _ratio(
            complete_mapped_frames, mapped_bucket_frames),
        "map_invalid_rate": _ratio(
            max(0, detection_frames - map_valid_frames), detection_frames),
        "map_unavailable_rate": _ratio(
            max(0, eligible_frames - map_valid_frames), eligible_frames),
        "tf_failure_rate": _ratio(tf_failure_frames, detection_frames),
        "mean_map_error_xy": (
            statistics.mean(raw_map_errors) if raw_map_errors else None),
        "p95_map_error_xy": percentile(raw_map_errors, 95),
        "actual_image_fps": actual_fps,
        "actual_image_source_fps": terminal_context.get(
            "actual_image_source_fps"),
        "mean_actual_linear_speed_mps": (
            statistics.mean(linear_speed) if linear_speed else None),
        "p95_actual_linear_speed_mps": percentile(linear_speed, 95),
        "mean_abs_actual_yaw_rate_radps": (
            statistics.mean(yaw_rate) if yaw_rate else None),
        "p95_abs_actual_yaw_rate_radps": percentile(yaw_rate, 95),
        "mean_abs_normalized_lateral_offset": (
            statistics.mean(lateral_offset) if lateral_offset else None),
        "p95_abs_normalized_lateral_offset": percentile(
            lateral_offset, 95),
    }
    audit = candidate_audit_summary(
        terminal_context.get("candidate_audit_observations", []),
        terminal_context.get("class_profile", ""))
    performance_verdict = evaluate_performance_verdict(
        metrics, status, evaluation_scope, audit,
        terminal_context.get("performance_contract"))
    completeness = {
        "status": status,
        "run_complete": bool(terminal_complete),
        "trial_count": len(finalized),
        "completed_trial_count": len(completed),
        "validation_errors": validation_errors,
        "meaning": (
            "Artifact/schema and trial measurement completeness only; "
            "MEASURED is not an algorithm performance PASS."),
    }
    return {
        "schema_version": 1,
        "evaluation_id": "V-SIM-04",
        "run_mode": run_mode,
        "evaluation_scope": evaluation_scope,
        "status": status,
        "trial_count": len(finalized),
        "completed_trial_count": len(completed),
        "validation_errors": validation_errors,
        "completeness": completeness,
        "performance_verdict": performance_verdict,
        "candidate_audit": audit,
        "artifact_completeness": {
            "required": list(REQUIRED_ARTIFACTS),
            "present": [],
            "missing": list(REQUIRED_ARTIFACTS),
            "complete": False,
        },
        "metrics": metrics,
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
            "confirmation_pipeline_samples": len(confirmation_pipeline),
            "detector_inference_samples": len(detector_inference),
            "detector_processing_samples": len(detector_processing),
            "complete_mapped_frames": complete_mapped_frames,
            "partial_only_mapped_frames": partial_only_mapped_frames,
            "processing_receipt_reordered_samples": sum(
                bool(result.get("processing_receipt_reordered"))
                for result in completed),
            "camera_pose_frames": sum(
                int(result.get("camera_pose_frame_count", 0))
                for result in completed),
            "motion_samples": len(linear_speed),
            "yaw_rate_samples": len(yaw_rate),
            "lateral_offset_samples": len(lateral_offset),
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
            "confirmation_pipeline_ms": (
                "same-host monotonic confirmation receipt minus detector "
                "callback start embedded for the same source stamp"),
            "complete_mapped_rate": (
                "unique active-trial mapped source stamps completed by all "
                "required detector branches divided by complete plus "
                "partial-only unique stamps"),
            "map_invalid_rate": (
                "mapped detection frames without a valid map point divided by "
                "matching detection frames"),
            "map_unavailable_rate": (
                "eligible truth frames without a valid map observation divided "
                "by eligible truth frames"),
            "stage_frame_rates": (
                "per-stage presence on eligible truth frames; these are "
                "diagnostic frame coverages, not independent probabilities"),
            "camera_pose": (
                "world-frame camera position and ZYX yaw from the latest "
                "stamped Gazebo camera pose not newer than the image stamp; "
                "camera_pose_age_sec is image stamp minus pose stamp"),
            "motion_window": (
                "runner ROS/source-time interval after the discontinuous "
                "path-start reset and before trajectory completion; dynamic "
                "telemetry is fail-closed when this interval is invalid"),
            "actual_speed_mps": (
                "route-level distance divided by runner elapsed ROS time; "
                "this achieved whole-trajectory average remains separate "
                "from the frame-delta speed distribution"),
            "actual_linear_speed_mps": (
                "3-D camera displacement divided by the strictly positive "
                "matched pose-source-stamp delta between deterministically "
                "time-sorted adjacent valid in-window dynamic samples; real "
                "zero displacement remains a valid zero, while duplicate "
                "pose stamps, reset/prestart, reset jumps, post-finish, "
                "static, first-valid and missing-pose samples are blank with "
                "motion_invalid_reason; mean/P95 are unweighted frame-delta "
                "distribution diagnostics, not route-level average speed"),
            "actual_yaw_rate_radps": (
                "shortest signed ZYX-yaw delta divided by the same dynamic "
                "sample interval"),
            "path_lateral_offset_normalized": (
                "signed horizontal cross-track distance from the planned "
                "dynamic start-to-finish line divided by that line length; "
                "dimensionless, positive on the path-left side, and blank "
                "outside the motion window and for static/invalid paths or "
                "missing poses"),
            "breakdowns": (
                "completed-trial and exact frame-sample aggregates grouped "
                "independently by class_name, height_m and requested "
                "speed_mps; the null speed group denotes static trials"),
        },
        "breakdowns": breakdowns,
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


def _artifact_inventory(output_dir):
    present = sorted(
        name for name in REQUIRED_ARTIFACTS
        if os.path.isfile(os.path.join(output_dir, name)) and
        os.path.getsize(os.path.join(output_dir, name)) > 0)
    missing = sorted(set(REQUIRED_ARTIFACTS) - set(present))
    return {
        "required": list(REQUIRED_ARTIFACTS),
        "present": present,
        "missing": missing,
        "complete": not missing,
    }


def _breakdown_report(title, rows):
    lines = [
        "## {}".format(title),
        "",
        "| Value | Completed/total | P_confirm | P_selected | Unweighted "
        "frame-delta mean speed (m/s) | Frame-delta P95 speed (m/s) | "
        "P95 abs lateral ratio |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {} | {}/{} | {} | {} | {} | {} | {} |".format(
                row["label"], row["completed_trial_count"],
                row["trial_count"], row["p_confirm"], row["p_selected"],
                row["mean_actual_linear_speed_mps"],
                row["p95_actual_linear_speed_mps"],
                row["p95_abs_normalized_lateral_offset"]))
    lines.append("")
    return lines


def _report(summary):
    metrics = summary["metrics"]
    denominators = summary["metric_denominators"]
    validation = summary.get("validation_errors", [])
    completeness = summary["completeness"]
    artifacts = summary["artifact_completeness"]
    verdict = summary["performance_verdict"]
    audit = summary["candidate_audit"]
    lines = [
        "# V-SIM-04 Vision Search Performance",
        "",
        "- Run mode: `{}`".format(summary["run_mode"]),
        "- Evaluation scope: `{}`".format(summary["evaluation_scope"]),
        "- Artifact set complete: `{}` (missing: `{}`)".format(
            artifacts["complete"], artifacts["missing"]),
        "- Measurement completeness: `{}`".format(
            completeness["status"]),
        "- Algorithm performance verdict: `{}` (Gate PASS: `{}`)".format(
            verdict["status"], verdict["is_gate_pass"]),
        "- Performance verdict reasons: `{}`".format(
            verdict["failure_reasons"]),
        "- Frozen-threshold findings: `{}`".format(
            verdict["metric_failure_reasons"]),
        "- Performance hard failures: `{}`".format(
            verdict["hard_failure_reasons"]),
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
        "- P95 same-host pipeline latency: `{}` ms".format(
            metrics["p95_confirmation_pipeline_ms"]),
        "- Detector inference/processing P95: `{}` / `{}` ms".format(
            metrics["p95_detector_inference_ms"],
            metrics["p95_detector_processing_ms"]),
        "- Complete mapped rate: `{}` ({}/{})".format(
            metrics["complete_mapped_rate"],
            denominators["complete_mapped_frames"],
            denominators["complete_mapped_frames"] +
            denominators["partial_only_mapped_frames"]),
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
        "- Camera pose/motion/lateral samples: `{}` / `{}` / `{}`".format(
            denominators["camera_pose_frames"],
            denominators["motion_samples"],
            denominators["lateral_offset_samples"]),
        "- Unweighted frame-delta mean/P95 linear speed: `{}` / `{}` "
        "m/s".format(
            metrics["mean_actual_linear_speed_mps"],
            metrics["p95_actual_linear_speed_mps"]),
        "- Mean/P95 absolute yaw rate: `{}` / `{}` rad/s".format(
            metrics["mean_abs_actual_yaw_rate_radps"],
            metrics["p95_abs_actual_yaw_rate_radps"]),
        "- Mean/P95 absolute normalized lateral offset: `{}` / `{}`".format(
            metrics["mean_abs_normalized_lateral_offset"],
            metrics["p95_abs_normalized_lateral_offset"]),
        "- Confirmed audit (all/unexpected/disallowed/policy-rejected/tank): "
        "`{}/{}/{}/{}/{}`".format(
            audit["confirmed"]["observations"],
            audit["confirmed"]["unexpected_observations"],
            audit["confirmed"]["disallowed_observations"],
            audit["confirmed"]["policy_rejected_observations"],
            audit["confirmed"]["tank_observations"]),
        "- Selected audit (all/unexpected/disallowed/policy-rejected/tank): "
        "`{}/{}/{}/{}/{}`".format(
            audit["selected"]["observations"],
            audit["selected"]["unexpected_observations"],
            audit["selected"]["disallowed_observations"],
            audit["selected"]["policy_rejected_observations"],
            audit["selected"]["tank_observations"]),
        "- Terminal validation errors: `{}`".format(validation),
        "",
        "## Semantics",
        "",
        "P_confirm uses current consecutive-frame/map/association/reject/age "
        "admission before the target leaves the fully-in-frame window. "
        "P_selected requires the same stable ID. Exposure time uses ROS/image "
        "stamps; processing time uses a monotonic wall clock. Map-invalid and "
        "TF-failure rates are reported separately from map error.",
        "",
        "MEASURED means that the formal trial set and artifacts are complete; "
        "it is not an algorithm PASS. A dry run validates only matrix and "
        "artifact schemas; a diagnostic subset isolates failures. Neither is "
        "a Gate PASS. Confirmed/selected audit counts include every class "
        "published during the session; r2026 tank selection and every other "
        "profile-disallowed selection are hard failures.",
        "",
    ]
    breakdowns = summary.get("breakdowns", {})
    lines.extend(_breakdown_report(
        "Breakdown by class", breakdowns.get("by_class", [])))
    lines.extend(_breakdown_report(
        "Breakdown by height", breakdowns.get("by_height_m", [])))
    lines.extend(_breakdown_report(
        "Breakdown by requested speed",
        breakdowns.get("by_speed_mps", [])))
    return "\n".join(lines)


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
    manifest["evaluation_scope"] = terminal_context.get(
        "evaluation_scope", manifest.get("evaluation_scope", "full"))
    manifest["actual_image_fps"] = actual_fps
    manifest["actual_image_source_fps"] = actual_source_fps
    manifest["terminal_validation"] = {
        key: copy.deepcopy(terminal_context.get(key)) for key in (
            "run_complete", "expected_trial_count", "evaluation_scope",
            "validation_errors") if key in terminal_context
    }
    _atomic_json(os.path.join(output_dir, "manifest.json"), manifest)
    _write_csv(os.path.join(output_dir, "frames.csv"), FRAME_FIELDS, frame_rows)
    _write_csv(os.path.join(output_dir, "events.csv"), EVENT_FIELDS, event_rows)
    audit_trials = summary["candidate_audit"]["trials"]
    performance_rows = []
    for result in summary["trials"]:
        row = dict(result)
        trial_audit = audit_trials.get(result.get("trial_id", ""), {})
        confirmed = trial_audit.get("confirmed", {})
        selected = trial_audit.get("selected", {})
        row.update({
            "measurement_completeness_status": summary[
                "completeness"]["status"],
            "artifact_set_complete": False,
            "performance_verdict": summary["performance_verdict"]["status"],
            "performance_hard_failure": summary["performance_verdict"][
                "hard_failure"],
            "performance_failure_reasons": json.dumps(
                summary["performance_verdict"]["failure_reasons"],
                sort_keys=True),
            "performance_metric_failure_reasons": json.dumps(
                summary["performance_verdict"]["metric_failure_reasons"],
                sort_keys=True),
            "confirmed_observations": confirmed.get("observations", 0),
            "unexpected_confirmed_observations": confirmed.get(
                "unexpected_observations", 0),
            "disallowed_confirmed_observations": confirmed.get(
                "disallowed_observations", 0),
            "policy_rejected_confirmed_observations": confirmed.get(
                "policy_rejected_observations", 0),
            "unique_confirmed_targets": confirmed.get(
                "unique_target_count", 0),
            "selected_observations": selected.get("observations", 0),
            "unexpected_selected_observations": selected.get(
                "unexpected_observations", 0),
            "disallowed_selected_observations": selected.get(
                "disallowed_observations", 0),
            "policy_rejected_selected_observations": selected.get(
                "policy_rejected_observations", 0),
            "unique_selected_targets": selected.get("unique_target_count", 0),
            "tank_confirmed_observations": confirmed.get(
                "tank_observations", 0),
            "tank_selected_observations": selected.get(
                "tank_observations", 0),
        })
        performance_rows.append(row)
    performance_rows = decorate_performance_rows(
        performance_rows, summary.get("breakdowns", {}))
    _write_csv(
        os.path.join(output_dir, "vision_search_performance.csv"),
        PERFORMANCE_FIELDS, performance_rows)
    # Write provisional summary/report first, then derive completeness from the
    # actual non-empty files.  This avoids declaring the writer's own outputs
    # complete before they exist on disk.
    _atomic_json(os.path.join(output_dir, "summary.json"), summary)
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
    summary["artifact_completeness"] = _artifact_inventory(output_dir)
    for row in performance_rows:
        row["artifact_set_complete"] = summary[
            "artifact_completeness"]["complete"]
    _write_csv(
        os.path.join(output_dir, "vision_search_performance.csv"),
        PERFORMANCE_FIELDS, performance_rows)
    _atomic_json(os.path.join(output_dir, "summary.json"), summary)
    # Refresh the human report with the final artifact inventory.
    try:
        with open(temporary, "w", encoding="utf-8") as stream:
            stream.write(_report(summary))
        os.replace(temporary, report_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    final_inventory = _artifact_inventory(output_dir)
    if final_inventory != summary["artifact_completeness"]:
        summary["artifact_completeness"] = final_inventory
        for row in performance_rows:
            row["artifact_set_complete"] = final_inventory["complete"]
        _write_csv(
            os.path.join(output_dir, "vision_search_performance.csv"),
            PERFORMANCE_FIELDS, performance_rows)
        _atomic_json(os.path.join(output_dir, "summary.json"), summary)
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
        "performance_contract": copy.deepcopy(
            matrix.get("performance_contract", {})),
    }
    return write_artifacts(
        output_dir, manifest, [], events, results, "dry_run", None,
        terminal_context={
            "run_complete": False,
            "evaluation_scope": "full",
            "expected_trial_count": len(matrix["trials"]),
            "validation_errors": [],
            "class_profile": matrix["class_profile"],
            "performance_contract": matrix.get("performance_contract", {}),
        })
