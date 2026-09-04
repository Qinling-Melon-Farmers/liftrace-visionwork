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
    # C25 fields are append-only.  path_lateral_* above remains the camera's
    # tracking error; these fields describe the physical target relative to
    # the planned flight line and to the calibrated image principal point.
    "visibility_profile", "visibility_eligible", "projection_valid",
    "truth_world_x_m", "truth_world_y_m", "truth_pixel_u",
    "truth_pixel_v", "target_path_lateral_offset_m",
    "target_pixel_offset_x_normalized", "target_pixel_offset_y_normalized",
    # D50 trial factors. The measured camera pose remains the source of motion;
    # these columns identify the requested operating-surface cell.
    "design_kind", "relative_angle_deg", "motion_profile", "framing",
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
    # Historical columns above are append-only.  Keep the navigation metrics
    # contiguous and in their frozen order so existing positional readers can
    # still locate that block before ignoring a newer tail.
    "navigation_metrics_mode", "navigation_target_stage_capability",
    "navigation_metrics_reason", "p_decision", "p_dispatch",
    "p_planner_arrival", "p_interrupt_reason",
    "navigation_binding_keys", "navigation_validation_errors",
    # C25 visibility/lateral columns are appended after the frozen navigation
    # surface so older readers can ignore an unknown tail safely.
    "lateral_bin", "visibility_profile", "p_confirm_visibility",
    "p_selected_visibility", "entered_visibility_window",
    "left_visibility_window", "requested_target_path_lateral_offset_m",
    "requested_pixel_offset_x_normalized",
    "target_lateral_sample_count", "mean_target_path_lateral_offset_m",
    "p95_abs_target_path_lateral_offset_m",
    "mean_target_pixel_offset_x_normalized",
    "p95_abs_target_pixel_offset_x_normalized",
    "lateral_group_completed_trials", "lateral_group_p_confirm",
    "lateral_group_p_selected", "lateral_group_p_confirm_visibility",
    "lateral_group_p_selected_visibility",
    "lateral_group_mean_target_path_lateral_offset_m",
    "lateral_group_mean_target_pixel_offset_x_normalized",
    # D50 diagnostic factors and the actual pose-sequence contract.
    "design_kind", "relative_angle_deg", "motion_profile", "framing",
    "relative_angle_measurement", "planned_sample_count",
    "expected_primary_target_id",
]
REQUIRED_ARTIFACTS = (
    "manifest.json", "frames.csv", "events.csv", "summary.json",
    "report.md", "vision_search_performance.csv",
)
EXPECTED_TRIAL_COUNT = 23
FORMAL_DESIGN_COUNTS = {
    "formal23": 23,
    "C25-lateral-offset": 25,
}
CONFIRMED_STATE = 2
AUDIT_EVENT_KINDS = ("confirmed", "selected")
NAVIGATION_METRICS_MODES = (
    "visual_only", "typed_contract", "target_stage")
NAVIGATION_SCHEMA_VERSION = 1
NAVIGATION_APPROACH_COMMAND = 1
NAVIGATION_VALID_COMMANDS = frozenset(range(8))
NAVIGATION_VALID_STATUSES = frozenset(range(8))
NAVIGATION_VALID_STAGES = frozenset(range(7))
NAVIGATION_ACCEPTED_STATUS = 0
NAVIGATION_STARTED_STATUS = 1
NAVIGATION_PROGRESS_STATUS = 2
NAVIGATION_DISPATCH_STAGE = 0
NAVIGATION_PLANNER_STAGE = 1
NAVIGATION_CAPTURE_STAGE = 2
VISUAL_ONLY_INTERRUPT_REASON = "visual_only_no_navigation_acceptance_event"


def navigation_metrics_metadata(mode):
    """Return the frozen capability declaration for one evaluation mode."""
    normalized = str(mode or "visual_only").strip().lower()
    if normalized not in NAVIGATION_METRICS_MODES:
        raise ValueError(
            "navigation_metrics_mode must be one of {}".format(
                ",".join(NAVIGATION_METRICS_MODES)))
    capability = normalized == "target_stage"
    reasons = {
        "visual_only": "navigation_topics_not_subscribed",
        "typed_contract": (
            "typed_navigation_contract_without_target_stage_capability"),
        "target_stage": "typed_target_stage_capability_enabled",
    }
    return {
        "mode": normalized,
        "target_stage_capability": capability,
        "reason": reasons[normalized],
    }


def navigation_drain_ready(now_monotonic, requested_monotonic,
                           last_receipt_monotonic, quiet_sec, timeout_sec):
    """Return ``(ready, timed_out)`` for the typed final callback drain."""
    values = (now_monotonic, requested_monotonic, quiet_sec, timeout_sec)
    try:
        now, requested, quiet, timeout = [float(value) for value in values]
        last_receipt = (requested if last_receipt_monotonic is None else
                        float(last_receipt_monotonic))
    except (TypeError, ValueError, OverflowError):
        return False, False
    if (not all(math.isfinite(value) for value in
                (now, requested, last_receipt, quiet, timeout)) or
            quiet <= 0.0 or timeout <= 0.0 or timeout < quiet or
            now < requested):
        return False, False
    timed_out = now - requested >= timeout
    quiet_reached = now - max(requested, last_receipt) >= quiet
    return quiet_reached or timed_out, timed_out


def _navigation_int(record, field, minimum=None):
    value = record.get(field)
    if isinstance(value, bool):
        raise ValueError(field + "_invalid")
    if isinstance(value, float) and (
            not math.isfinite(value) or not value.is_integer()):
        raise ValueError(field + "_invalid")
    try:
        value = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(field + "_invalid") from error
    if minimum is not None and value < minimum:
        raise ValueError(field + "_invalid")
    return value


def _navigation_text(record, field):
    value = str(record.get(field, "")).strip()
    if not value:
        raise ValueError(field + "_missing")
    return value


def _navigation_bool(record, field):
    value = record.get(field)
    if not isinstance(value, bool):
        raise ValueError(field + "_invalid")
    return value


def _navigation_target_key(record):
    """Return the exact cross-process target/action binding key."""
    return (
        _navigation_text(record, "mission_id"),
        _navigation_int(record, "decision_seq", 1),
        # uint32 zero is a valid target identity; has_target carries presence.
        _navigation_int(record, "target_id", 0),
        _navigation_int(record, "target_first_seen_ns", 1),
        _navigation_int(record, "attempt", 1),
        _navigation_int(record, "payload_slot", 1),
    )


def _navigation_wire_fingerprint(record, kind):
    ignored = {"receipt_monotonic", "trial_id_at_receipt"}
    if kind == "decision":
        fields = (
            "schema_version", "mission_id", "decision_seq",
            "header_seq", "header_stamp_ns", "header_frame_id",
            "deadline_ns", "command", "class_profile", "has_goal",
            "has_target", "target_id", "target_first_seen_ns",
            "target_observation_stamp_ns", "target_class", "attempt",
            "payload_slot", "goal_stamp_ns", "goal_frame_id", "goal_x",
            "goal_y", "goal_z", "goal_qx", "goal_qy", "goal_qz",
            "goal_qw", "reason")
    else:
        fields = (
            "schema_version", "mission_id", "executor_id", "event_seq",
            "decision_seq", "header_seq", "header_stamp_ns",
            "header_frame_id", "command", "status",
            "stage", "terminal", "retryable", "payload_committed",
            "has_target", "target_id", "target_first_seen_ns",
            "target_class", "attempt", "payload_slot", "reason",
            "evidence_source")
    return tuple((field, copy.deepcopy(record.get(field)))
                 for field in fields if field not in ignored)


def _navigation_record_label(record, kind):
    mission = str(record.get("mission_id", "") or "missing")
    if kind == "decision":
        sequence = record.get("decision_seq", "invalid")
        return "decision:{}:{}".format(mission, sequence)
    executor = str(record.get("executor_id", "") or "missing")
    sequence = record.get("event_seq", "invalid")
    return "result:{}:{}:{}".format(mission, executor, sequence)


def _navigation_decision_error(record, class_profile, allowed_classes):
    try:
        if _navigation_int(record, "schema_version") != \
                NAVIGATION_SCHEMA_VERSION:
            return "schema_version_mismatch"
        _navigation_text(record, "mission_id")
        _navigation_int(record, "decision_seq", 1)
        issued_ns = _navigation_int(record, "header_stamp_ns", 1)
        deadline_ns = _navigation_int(record, "deadline_ns", 1)
        if deadline_ns <= issued_ns:
            return "deadline_not_after_decision"
        command = _navigation_int(record, "command")
        if command not in NAVIGATION_VALID_COMMANDS:
            return "command_invalid"
        profile = _navigation_text(record, "class_profile")
        if profile != str(class_profile):
            return "profile_mismatch"
        _navigation_bool(record, "has_goal")
        has_target = _navigation_bool(record, "has_target")
        if command == NAVIGATION_APPROACH_COMMAND:
            if not has_target:
                return "approach_target_missing"
            _navigation_target_key(record)
            observation_ns = _navigation_int(
                record, "target_observation_stamp_ns", 1)
            first_seen_ns = _navigation_int(
                record, "target_first_seen_ns", 1)
            if observation_ns < first_seen_ns:
                return "target_observation_precedes_first_seen"
            if observation_ns > issued_ns:
                return "target_observation_after_decision"
            target_class = _navigation_text(record, "target_class")
            if allowed_classes and target_class not in set(allowed_classes):
                return "target_class_disallowed_by_profile"
        return ""
    except ValueError as error:
        return str(error)


def _navigation_result_error(record):
    try:
        if _navigation_int(record, "schema_version") != \
                NAVIGATION_SCHEMA_VERSION:
            return "schema_version_mismatch"
        _navigation_text(record, "mission_id")
        _navigation_text(record, "executor_id")
        _navigation_int(record, "event_seq", 1)
        _navigation_int(record, "decision_seq", 1)
        _navigation_int(record, "header_stamp_ns", 1)
        command = _navigation_int(record, "command")
        if command not in NAVIGATION_VALID_COMMANDS:
            return "command_invalid"
        if _navigation_int(record, "status") not in NAVIGATION_VALID_STATUSES:
            return "status_invalid"
        if _navigation_int(record, "stage") not in NAVIGATION_VALID_STAGES:
            return "stage_invalid"
        _navigation_bool(record, "terminal")
        _navigation_bool(record, "retryable")
        _navigation_bool(record, "payload_committed")
        has_target = _navigation_bool(record, "has_target")
        if command == NAVIGATION_APPROACH_COMMAND:
            if not has_target:
                return "approach_target_missing"
            _navigation_target_key(record)
            _navigation_text(record, "target_class")
        return ""
    except ValueError as error:
        return str(error)


def _deduplicate_navigation_records(records, kind):
    """Deduplicate retransmissions and reject identity/event-seq conflicts."""
    unique = []
    errors = []
    duplicates = 0
    identities = {}
    highest_result_seq = {}
    for raw in records or []:
        record = copy.deepcopy(dict(raw))
        label = _navigation_record_label(record, kind)
        try:
            if kind == "decision":
                identity = (
                    _navigation_text(record, "mission_id"),
                    _navigation_int(record, "decision_seq", 1))
            else:
                identity = (
                    _navigation_text(record, "mission_id"),
                    _navigation_text(record, "executor_id"),
                    _navigation_int(record, "event_seq", 1))
        except ValueError:
            # The schema validator below emits the precise field error.  Keep
            # malformed records distinct so they cannot hide one another.
            unique.append(record)
            continue
        fingerprint = _navigation_wire_fingerprint(record, kind)
        previous = identities.get(identity)
        if previous is not None:
            if previous == fingerprint:
                duplicates += 1
            else:
                errors.append(label + ":" + (
                    "decision_identity_conflict" if kind == "decision" else
                    "event_seq_conflict"))
            continue
        identities[identity] = fingerprint
        if kind == "result":
            stream = identity[:2]
            sequence = identity[2]
            previous_seq = highest_result_seq.get(stream)
            if previous_seq is not None and sequence < previous_seq:
                errors.append(label + ":event_seq_out_of_order")
                continue
            highest_result_seq[stream] = max(
                sequence, previous_seq if previous_seq is not None else 0)
        unique.append(record)
    return unique, errors, duplicates


def correlate_navigation_events(trials, decision_records, result_records,
                                mode="visual_only", class_profile="",
                                allowed_classes=None):
    """Validate typed navigation events and backfill completed visual trials.

    A result is usable only after an exact
    mission/decision/target/first-seen/attempt/slot join.  The function is
    deliberately ROS-free and independent of callback order, so a result that
    arrives after ``trial_end`` can still update that trial at snapshot time.
    """
    metadata = navigation_metrics_metadata(mode)
    updated = [copy.deepcopy(result) for result in (trials or [])]
    for result in updated:
        result.update({
            "navigation_metrics_mode": metadata["mode"],
            "navigation_target_stage_capability": metadata[
                "target_stage_capability"],
            "navigation_metrics_reason": metadata["reason"],
            "p_decision": None,
            "p_dispatch": None,
            "p_planner_arrival": None,
            "p_interrupt": None,
            "p_interrupt_reason": (
                VISUAL_ONLY_INTERRUPT_REASON
                if metadata["mode"] == "visual_only" else
                metadata["reason"]),
            "navigation_binding_keys": "[]",
            "navigation_validation_errors": "[]",
        })
    if metadata["mode"] == "visual_only":
        return {
            **metadata,
            "trials": updated,
            "validation_errors": [],
            "decision_event_count": 0,
            "result_event_count": 0,
            "deduplicated_decision_count": 0,
            "deduplicated_result_count": 0,
            "matched_result_count": 0,
        }

    allowed = set(str(value) for value in (allowed_classes or []))
    profile_errors = []
    if not allowed:
        try:
            resolved_profile, resolved_allowed = resolve_class_profile(
                class_profile)
            if resolved_profile != str(class_profile).strip().lower():
                profile_errors.append("navigation_profile_normalization_error")
            allowed = set(resolved_allowed)
        except ValueError:
            profile_errors.append("navigation_profile_allowlist_unavailable")
    decisions, errors, duplicate_decisions = \
        _deduplicate_navigation_records(decision_records, "decision")
    errors.extend(profile_errors)
    navigation_results, result_identity_errors, duplicate_results = \
        _deduplicate_navigation_records(result_records, "result")
    errors.extend(result_identity_errors)

    valid_decisions = {}
    valid_approach = []
    for decision in decisions:
        label = _navigation_record_label(decision, "decision")
        error = _navigation_decision_error(
            decision, class_profile, allowed)
        if error:
            errors.append(label + ":" + error)
            continue
        identity = (str(decision["mission_id"]), int(decision["decision_seq"]))
        valid_decisions[identity] = decision
        if int(decision["command"]) == NAVIGATION_APPROACH_COMMAND:
            valid_approach.append(decision)

    matched_results = []
    for event in navigation_results:
        label = _navigation_record_label(event, "result")
        error = _navigation_result_error(event)
        if error:
            errors.append(label + ":" + error)
            continue
        identity = (str(event["mission_id"]), int(event["decision_seq"]))
        decision = valid_decisions.get(identity)
        if decision is None:
            errors.append(label + ":decision_missing_or_invalid")
            continue
        if int(event["command"]) != int(decision["command"]):
            errors.append(label + ":command_mismatch")
            continue
        if bool(event.get("has_target")) != bool(
                decision.get("has_target")):
            errors.append(label + ":target_flag_mismatch")
            continue
        if bool(decision.get("has_target")):
            try:
                exact_key = _navigation_target_key(event)
                decision_key = _navigation_target_key(decision)
            except ValueError as validation_error:
                errors.append(label + ":" + str(validation_error))
                continue
            if exact_key != decision_key:
                errors.append(label + ":full_binding_key_mismatch")
                continue
            if str(event.get("target_class", "")) != str(
                    decision.get("target_class", "")):
                errors.append(label + ":target_class_mismatch")
                continue
        event_stamp = int(event["header_stamp_ns"])
        if event_stamp < int(decision["header_stamp_ns"]):
            errors.append(label + ":result_precedes_decision")
            continue
        # The navigation core treats deadline as an exclusive action lease:
        # no dispatch, planner-arrival, or target-stage milestone after this
        # boundary may contribute to evaluation.
        if event_stamp >= int(decision["deadline_ns"]):
            error = (
                "dispatch_at_or_after_deadline"
                if int(event["status"]) == NAVIGATION_ACCEPTED_STATUS and
                int(event["stage"]) == NAVIGATION_DISPATCH_STAGE else
                "result_at_or_after_deadline")
            errors.append(label + ":" + error)
            continue
        matched_results.append((decision, event))

    errors = sorted(set(errors))
    navigation_errors_json = json.dumps(errors, sort_keys=True)
    for trial in updated:
        if trial.get("status") != "completed":
            continue
        trial["navigation_validation_errors"] = navigation_errors_json
        trial["p_decision"] = False
        trial["p_dispatch"] = False
        trial["p_planner_arrival"] = False
        if metadata["target_stage_capability"]:
            trial["p_interrupt"] = False
            trial["p_interrupt_reason"] = \
                "matching_started_capture_event_absent"
        selected_id = trial.get("stable_id")
        selected_first_seen = trial.get("selected_target_first_seen_ns")
        selected_observation_stamps = {
            int(value) for value in
            (trial.get("selected_target_observation_stamps_ns") or [])
            if isinstance(value, int) and not isinstance(value, bool) and
            value > 0
        }
        if (not trial.get("p_selected") or selected_id is None or
                selected_first_seen is None or
                not selected_observation_stamps):
            continue
        binding_decisions = [
            decision for decision in valid_approach
            if int(decision.get("target_id", -1)) == int(selected_id) and
            int(decision.get("target_first_seen_ns", -1)) ==
            int(selected_first_seen) and
            int(decision.get("target_observation_stamp_ns", -1)) in
            selected_observation_stamps and
            str(decision.get("target_class", "")) ==
            str(trial.get("class_name", ""))
        ]
        if not binding_decisions:
            continue
        trial["p_decision"] = True
        keys = sorted({_navigation_target_key(item)
                       for item in binding_decisions})
        trial["navigation_binding_keys"] = json.dumps(
            [list(item) for item in keys], sort_keys=True)
        binding_key_set = set(keys)
        events = [
            (decision, event) for decision, event in matched_results
            if _navigation_target_key(decision) in binding_key_set
        ]
        trial["p_dispatch"] = any(
            int(event["status"]) == NAVIGATION_ACCEPTED_STATUS and
            int(event["stage"]) == NAVIGATION_DISPATCH_STAGE and
            int(event["header_stamp_ns"]) < int(decision["deadline_ns"])
            for decision, event in events)
        trial["p_planner_arrival"] = any(
            int(event["status"]) == NAVIGATION_PROGRESS_STATUS and
            int(event["stage"]) == NAVIGATION_PLANNER_STAGE and
            str(event.get("reason", "")) ==
            "approach_arrival_confirmed"
            for _decision, event in events)
        if metadata["target_stage_capability"]:
            trial["p_interrupt"] = any(
                int(event["status"]) == NAVIGATION_STARTED_STATUS and
                int(event["stage"]) == NAVIGATION_CAPTURE_STAGE
                for _decision, event in events)
            if trial["p_interrupt"]:
                trial["p_interrupt_reason"] = \
                    "matching_started_capture_event_observed"
    return {
        **metadata,
        "trials": updated,
        "validation_errors": errors,
        "decision_event_count": len(decisions),
        "result_event_count": len(navigation_results),
        "deduplicated_decision_count": duplicate_decisions,
        "deduplicated_result_count": duplicate_results,
        "matched_result_count": len(matched_results),
    }


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
        "max_p95_confirmation_pipeline_ms": (
            "p95_confirmation_pipeline_ms", "max"),
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
    if isinstance(matrix, dict) and matrix.get("matrix_kind") == \
            "vsim04_d50_trajectory_association":
        # D50 owns its explicit pairwise/directed schema. Keep one thin loader
        # hook here so it can reuse the established recorder without copying a
        # second metrics stack.
        from uav_vision_eval.vsim04_d_matrix import load_d50_runtime_matrix
        return load_d50_runtime_matrix(path)
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
    # Only frozen, named designs may report a full MEASURED run. An ad-hoc
    # matrix remains useful for diagnostics, but its self-declared count must
    # not silently redefine the formal surface.
    design_id = str(matrix.get("design_id", "")).strip()
    formal_count = FORMAL_DESIGN_COUNTS.get(design_id)
    if formal_count is None:
        matrix["diagnostic_only"] = True
        matrix["formal_expected_trial_count"] = expected
    else:
        if expected != formal_count:
            raise ValueError(
                "formal design {} must contain {} trials".format(
                    design_id, formal_count))
        matrix["diagnostic_only"] = bool(
            matrix.get("diagnostic_only", False))
        matrix["formal_expected_trial_count"] = formal_count
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
        selected_matrix["evaluation_scope"] = (
            "diagnostic" if selected_matrix.get("diagnostic_only") else
            "full")
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
    lateral = matrix.get("lateral", {})
    if lateral:
        try:
            height = float(lateral["height_m"])
            speed = float(lateral["speed_mps"])
            horizontal_fov = float(lateral["horizontal_fov_rad"])
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("lateral matrix scalar is invalid") from error
        if (not all(math.isfinite(value) for value in
                    (height, speed, horizontal_fov)) or
                height <= 0.0 or speed <= 0.0 or
                not 0.0 < horizontal_fov < math.pi):
            raise ValueError("lateral matrix scalar is out of range")
        widths = lateral.get("target_width_m", {})
        bins = lateral.get("bins", [])
        if not isinstance(widths, dict) or not isinstance(bins, list) or not bins:
            raise ValueError("lateral target widths and bins are required")
        bin_ids = []
        for definition in bins:
            if not isinstance(definition, dict):
                raise ValueError("lateral bin must be a mapping")
            bin_id = str(definition.get("id", "")).strip()
            profile = str(definition.get(
                "visibility_profile", "full")).strip().lower()
            try:
                side = int(definition.get("side", 0))
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError("lateral bin side is invalid") from error
            if not bin_id or profile not in {"full", "partial"} or \
                    side not in {-1, 0, 1}:
                raise ValueError("lateral bin contract is invalid")
            if side == 0 and bin_id != "center":
                raise ValueError("only center may use lateral side zero")
            if profile == "partial" and side == 0:
                raise ValueError("partial lateral bin requires a side")
            bin_ids.append(bin_id)
        if len(bin_ids) != len(set(bin_ids)):
            raise ValueError("lateral bins contain duplicate identifiers")
        for class_name in lateral.get("classes", []):
            class_name = str(class_name)
            try:
                width = float(widths[class_name])
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                raise ValueError(
                    "lateral target width missing: " + class_name) from error
            if not math.isfinite(width) or width <= 0.0:
                raise ValueError(
                    "lateral target width invalid: " + class_name)
            for definition in bins:
                bin_id = str(definition["id"]).strip()
                trial = {
                    "trial_id": "lateral_{}_{}".format(
                        class_name, bin_id),
                    # Keep the existing dynamic capture/telemetry contract;
                    # lateral_bin selects the generalized trajectory planner.
                    "kind": "dynamic",
                    "class_name": class_name,
                    "height_m": height,
                    "speed_mps": speed,
                    "lateral_bin": bin_id,
                    "visibility_profile": str(definition.get(
                        "visibility_profile", "full")).strip().lower(),
                    "lateral_side": int(definition.get("side", 0)),
                    "target_width_m": width,
                }
                if "full_visible_fraction" in definition:
                    trial["full_visible_fraction"] = float(
                        definition["full_visible_fraction"])
                if "partial_clip_fraction" in definition:
                    trial["partial_clip_fraction"] = float(
                        definition["partial_clip_fraction"])
                trials.append(trial)
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


def annotate_target_lateral_frames(frame_rows, trajectory):
    """Annotate independent C25 target-to-track and image offsets.

    Unlike ``path_lateral_offset_*`` (camera tracking error), the signed metre
    value here measures the truth target centre against the planned horizontal
    start-to-finish line.  Pixel offsets are produced by the truth projector
    and CameraInfo, not inferred from the requested bin.
    """
    path = _dynamic_path(trajectory)
    metre_samples = []
    pixel_samples = []
    if path is None:
        return {
            "target_lateral_sample_count": 0,
            "target_path_lateral_offset_m_samples": [],
            "target_pixel_offset_x_normalized_samples": [],
            "mean_target_path_lateral_offset_m": None,
            "p95_abs_target_path_lateral_offset_m": None,
            "mean_target_pixel_offset_x_normalized": None,
            "p95_abs_target_pixel_offset_x_normalized": None,
        }
    start_x, start_y, dx_value, dy_value, path_length = path
    for row in frame_rows:
        target_x = _finite_frame_value(row, "truth_world_x_m")
        target_y = _finite_frame_value(row, "truth_world_y_m")
        pixel_offset = _finite_frame_value(
            row, "target_pixel_offset_x_normalized")
        if target_x is not None and target_y is not None:
            cross_track = (
                dx_value * (target_y - start_y) -
                dy_value * (target_x - start_x)) / path_length
            if math.isfinite(cross_track):
                row["target_path_lateral_offset_m"] = cross_track
                if bool(row.get("visibility_eligible")):
                    metre_samples.append(cross_track)
        if pixel_offset is not None and bool(row.get("visibility_eligible")):
            pixel_samples.append(pixel_offset)
    return {
        "target_lateral_sample_count": len(metre_samples),
        "target_path_lateral_offset_m_samples": metre_samples,
        "target_pixel_offset_x_normalized_samples": pixel_samples,
        "mean_target_path_lateral_offset_m": (
            statistics.mean(metre_samples) if metre_samples else None),
        "p95_abs_target_path_lateral_offset_m": percentile(
            [abs(value) for value in metre_samples], 95),
        "mean_target_pixel_offset_x_normalized": (
            statistics.mean(pixel_samples) if pixel_samples else None),
        "p95_abs_target_pixel_offset_x_normalized": percentile(
            [abs(value) for value in pixel_samples], 95),
    }


def planned_trial_result(trial):
    result = dict(trial)
    result.update({
        "status": "planned",
        "p_confirm": None,
        "p_selected": None,
        "p_confirm_visibility": None,
        "p_selected_visibility": None,
        "p_interrupt": None,
        "p_decision": None,
        "p_dispatch": None,
        "p_planner_arrival": None,
        "p_interrupt_reason": VISUAL_ONLY_INTERRUPT_REASON,
        "navigation_metrics_mode": "visual_only",
        "navigation_target_stage_capability": False,
        "navigation_metrics_reason": "navigation_topics_not_subscribed",
        "navigation_binding_keys": "[]",
        "navigation_validation_errors": "[]",
        "stable_id": None,
        "selected_target_first_seen_ns": None,
        "selected_target_observation_stamps_ns": [],
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
        "entered_visibility_window": False,
        "left_visibility_window": False,
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
        "target_lateral_sample_count": 0,
        "target_path_lateral_offset_m_samples": [],
        "target_pixel_offset_x_normalized_samples": [],
        "mean_target_path_lateral_offset_m": None,
        "p95_abs_target_path_lateral_offset_m": None,
        "mean_target_pixel_offset_x_normalized": None,
        "p95_abs_target_pixel_offset_x_normalized": None,
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
    target_lateral = [float(value) for value in result.pop(
        "target_path_lateral_offset_m_samples", [])]
    target_pixel = [float(value) for value in result.pop(
        "target_pixel_offset_x_normalized_samples", [])]
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
    result["target_lateral_sample_count"] = len(target_lateral)
    result["mean_target_path_lateral_offset_m"] = (
        statistics.mean(target_lateral) if target_lateral else None)
    result["p95_abs_target_path_lateral_offset_m"] = percentile(
        [abs(value) for value in target_lateral], 95)
    result["mean_target_pixel_offset_x_normalized"] = (
        statistics.mean(target_pixel) if target_pixel else None)
    result["p95_abs_target_pixel_offset_x_normalized"] = percentile(
        [abs(value) for value in target_pixel], 95)
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
    if (result.get("status") != "completed" or
            result.get("p_confirm_visibility") or result.get("p_confirm")):
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


def trial_output_drain_boundary(result, source_event):
    """Choose a finite source-time boundary for draining one trial.

    A valid visibility leave is the closest boundary to the metric window. If
    a discretely driven simulation never samples that window, the runner's
    trial-end stamp still lets every output drain and lets terminal validation
    report the missing enter/leave as a design-invalid trial instead of
    deadlocking the entire matrix.
    """
    candidates = (
        ("visibility_leave", (result or {}).get("leave_source_stamp")),
        ("trial_end", (source_event or {}).get("stamp")),
        ("motion_end", (source_event or {}).get(
            "trajectory", {}).get("motion_end_source_stamp")),
    )
    for kind, candidate in candidates:
        try:
            value = float(candidate)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(value):
            return value, kind
    return None, "unavailable"


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
        "p_confirm": (None if result.get("visibility_profile") == "partial"
                       else False),
        "p_selected": (None if result.get("visibility_profile") == "partial"
                        else False),
        "p_confirm_visibility": False,
        "p_selected_visibility": False,
        "stable_id": None,
        "selected_target_first_seen_ns": None,
        "selected_target_observation_stamps_ns": [],
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
    output["p_confirm_visibility"] = True
    if result.get("visibility_profile") != "partial":
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
    matching_selected = [
        event for event in selected_events
        if event["stable_id"] == output["stable_id"] and
        event_inside_trial_window(event, result)]
    output["p_selected_visibility"] = bool(matching_selected)
    if result.get("visibility_profile") != "partial":
        output["p_selected"] = bool(matching_selected)
    if matching_selected:
        selected = min(matching_selected, key=lambda event: (
            event["receipt_monotonic"], event["source_stamp"]))
        first_seen_ns = selected.get("target_first_seen_ns")
        if first_seen_ns is not None:
            try:
                first_seen_ns = int(first_seen_ns)
            except (TypeError, ValueError, OverflowError):
                first_seen_ns = None
        output["selected_target_first_seen_ns"] = first_seen_ns
        output["selected_target_observation_stamps_ns"] = sorted({
            int(event["stamp_key"])
            for event in matching_selected
            if event.get("stamp_key") is not None and
            first_seen_ns is not None and
            int(event.get("target_first_seen_ns", -1)) == first_seen_ns
        })
    return output


def _group_value(result, field):
    value = result.get(field)
    if field in {"height_m", "speed_mps"} and value is not None:
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return numeric if math.isfinite(numeric) else None
    return str(value) if field in {
        "class_name", "lateral_bin", "visibility_profile"} else value


def _group_label(field, value):
    if field in {"class_name", "lateral_bin", "visibility_profile"}:
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
        ("by_lateral_bin", "lateral_bin"),
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
            target_lateral = [sample for result in completed
                              for sample in _finite_samples(
                                  result,
                                  "target_path_lateral_offset_m_samples")]
            target_pixel = [sample for result in completed
                            for sample in _finite_samples(
                                result,
                                "target_pixel_offset_x_normalized_samples")]
            confirm_members = [result for result in completed
                               if result.get("p_confirm") is not None]
            selected_members = [result for result in completed
                                if result.get("p_selected") is not None]
            groups.append({
                "dimension": field,
                "value": value,
                "label": _group_label(field, value),
                "trial_count": len(members),
                "completed_trial_count": len(completed),
                "p_confirm": (
                    sum(bool(result.get("p_confirm"))
                        for result in confirm_members) /
                    float(len(confirm_members)) if confirm_members else None),
                "p_selected": (
                    sum(bool(result.get("p_selected"))
                        for result in selected_members) /
                    float(len(selected_members))
                    if selected_members else None),
                "p_confirm_visibility": (
                    sum(bool(result.get("p_confirm_visibility"))
                        for result in completed) / float(len(completed))
                    if completed else None),
                "p_selected_visibility": (
                    sum(bool(result.get("p_selected_visibility"))
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
                "target_lateral_sample_count": len(target_lateral),
                "mean_target_path_lateral_offset_m": (
                    statistics.mean(target_lateral)
                    if target_lateral else None),
                "p95_abs_target_path_lateral_offset_m": percentile(
                    [abs(sample) for sample in target_lateral], 95),
                "mean_target_pixel_offset_x_normalized": (
                    statistics.mean(target_pixel) if target_pixel else None),
                "p95_abs_target_pixel_offset_x_normalized": percentile(
                    [abs(sample) for sample in target_pixel], 95),
            })
        breakdowns[output_name] = groups
    return breakdowns


def decorate_performance_rows(rows, breakdowns):
    """Append repeated group metrics without adding aggregate CSV rows."""
    lookup = {}
    for output_name, field, prefix in (
            ("by_class", "class_name", "class_group"),
            ("by_height_m", "height_m", "height_group"),
            ("by_speed_mps", "speed_mps", "speed_group"),
            ("by_lateral_bin", "lateral_bin", "lateral_group")):
        lookup[field] = {
            group["value"]: (prefix, group)
            for group in breakdowns.get(output_name, [])
        }
    decorated = []
    for source in rows:
        row = dict(source)
        for field in ("class_name", "height_m", "speed_mps",
                      "lateral_bin"):
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
            if prefix == "lateral_group":
                row["lateral_group_p_confirm_visibility"] = group[
                    "p_confirm_visibility"]
                row["lateral_group_p_selected_visibility"] = group[
                    "p_selected_visibility"]
                row["lateral_group_mean_target_path_lateral_offset_m"] = group[
                    "mean_target_path_lateral_offset_m"]
                row["lateral_group_mean_target_pixel_offset_x_normalized"] = group[
                    "mean_target_pixel_offset_x_normalized"]
        decorated.append(row)
    return decorated


def summarize_trial_results(results, run_mode, actual_fps=None,
                            terminal_context=None):
    terminal_context = dict(terminal_context or {})
    navigation_audit = correlate_navigation_events(
        results,
        terminal_context.get("navigation_decision_records", []),
        terminal_context.get("navigation_result_records", []),
        terminal_context.get("navigation_metrics_mode", "visual_only"),
        terminal_context.get("class_profile", ""),
        terminal_context.get("allowed_classes", []))
    results = navigation_audit["trials"]
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
    target_lateral = [
        value for result in results
        if result.get("status") == "completed"
        for value in _finite_samples(
            result, "target_path_lateral_offset_m_samples")]
    target_pixel = [
        value for result in results
        if result.get("status") == "completed"
        for value in _finite_samples(
            result, "target_pixel_offset_x_normalized_samples")]
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
        validation_errors.extend(
            "navigation_contract:" + error
            for error in navigation_audit["validation_errors"])
        expected_count = terminal_context.get(
            "expected_trial_count", EXPECTED_TRIAL_COUNT)
        formal_expected_count = terminal_context.get(
            "formal_expected_trial_count", EXPECTED_TRIAL_COUNT)
        if evaluation_scope not in {"full", "diagnostic"}:
            validation_errors.append("evaluation_scope_invalid")
        if (evaluation_scope == "full" and
                int(expected_count) != int(formal_expected_count)):
            validation_errors.append(
                "expected_trial_count_must_match_formal_design")
        if int(expected_count) <= 0:
            validation_errors.append("expected_trial_count_must_be_positive")
        if len(finalized) != int(expected_count):
            validation_errors.append("trial_count_{}/{}".format(
                len(finalized), int(expected_count)))
        if len(completed) != len(finalized):
            validation_errors.append("completed_trials_{}/{}".format(
                len(completed), len(finalized)))
        for result in completed:
            visibility_profile = str(
                result.get("visibility_profile") or "full")
            entered_visibility = bool(
                result.get("entered_visibility_window") or
                (visibility_profile != "partial" and
                 result.get("entered_fully_in_frame")))
            left_visibility = bool(
                result.get("left_visibility_window") or
                (visibility_profile != "partial" and
                 result.get("left_fully_in_frame")))
            if not entered_visibility:
                validation_errors.append(
                    "{}:never_entered_visibility_window".format(
                        result.get("trial_id", "unknown")))
            if not left_visibility:
                validation_errors.append(
                    "{}:never_left_visibility_window".format(
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
    navigation_enabled = navigation_audit["mode"] != "visual_only"
    target_stage_capability = navigation_audit[
        "target_stage_capability"]
    confirm_trials = [result for result in completed
                      if result.get("p_confirm") is not None]
    selected_trials = [result for result in completed
                       if result.get("p_selected") is not None]
    metrics = {
        "p_confirm": (
            sum(bool(result.get("p_confirm")) for result in confirm_trials) /
            float(len(confirm_trials)) if confirm_trials else None),
        "p_selected": (
            sum(bool(result.get("p_selected")) for result in selected_trials) /
            float(len(selected_trials)) if selected_trials else None),
        "p_confirm_visibility": (
            sum(bool(result.get("p_confirm_visibility"))
                for result in completed) / float(len(completed))
            if completed else None),
        "p_selected_visibility": (
            sum(bool(result.get("p_selected_visibility"))
                for result in completed) / float(len(completed))
            if completed else None),
        "p_decision": (
            sum(bool(result.get("p_decision")) for result in completed) /
            float(len(completed))
            if navigation_enabled and completed else None),
        "p_dispatch": (
            sum(bool(result.get("p_dispatch")) for result in completed) /
            float(len(completed))
            if navigation_enabled and completed else None),
        "p_planner_arrival": (
            sum(bool(result.get("p_planner_arrival"))
                for result in completed) / float(len(completed))
            if navigation_enabled and completed else None),
        "p_interrupt": (
            sum(bool(result.get("p_interrupt")) for result in completed) /
            float(len(completed))
            if target_stage_capability and completed else None),
        "p_interrupt_reason": (
            VISUAL_ONLY_INTERRUPT_REASON
            if navigation_audit["mode"] == "visual_only" else
            navigation_audit["reason"]),
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
        "mean_target_path_lateral_offset_m": (
            statistics.mean(target_lateral) if target_lateral else None),
        "p95_abs_target_path_lateral_offset_m": percentile(
            [abs(value) for value in target_lateral], 95),
        "mean_target_pixel_offset_x_normalized": (
            statistics.mean(target_pixel) if target_pixel else None),
        "p95_abs_target_pixel_offset_x_normalized": percentile(
            [abs(value) for value in target_pixel], 95),
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
        "navigation_metrics": {
            key: copy.deepcopy(value)
            for key, value in navigation_audit.items()
            if key != "trials"
        },
        "artifact_completeness": {
            "required": list(REQUIRED_ARTIFACTS),
            "present": [],
            "missing": list(REQUIRED_ARTIFACTS),
            "complete": False,
        },
        "metrics": metrics,
        "metric_denominators": {
            "completed_trials": len(completed),
            "fully_visible_metric_trials": len(confirm_trials),
            "visibility_metric_trials": len(completed),
            "navigation_metric_trials": (
                len(completed) if navigation_enabled else 0),
            "target_stage_metric_trials": (
                len(completed) if target_stage_capability else 0),
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
            "target_lateral_samples": len(target_lateral),
            "target_pixel_offset_samples": len(target_pixel),
        },
        "definitions": {
            "p_confirm": (
                "trial reaches current full candidate admission before the "
                "target leaves the fully-in-frame window; null for a partial "
                "visibility profile"),
            "p_confirm_visibility": (
                "candidate admission inside the trial's declared truth "
                "visibility window; partial trials use projection-valid, "
                "center-in-frame and not-fully-in-frame truth"),
            "p_selected": (
                "the same stable_id confirmed in the trial is published on "
                "selected_target before leaving"),
            "p_decision": (
                "a schema/profile/deadline-valid APPROACH decision binds the "
                "selected target by mission_id, decision_seq, target_id, "
                "target_first_seen, attempt and payload_slot"),
            "p_dispatch": (
                "an exact-key NavigationResult reports ACCEPTED/DISPATCH; "
                "selected or decision evidence cannot substitute it"),
            "p_planner_arrival": (
                "an exact-key NavigationResult reports PROGRESS/PLANNER with "
                "reason approach_arrival_confirmed"),
            "p_interrupt": (
                "true only in target_stage mode after an exact-key "
                "STARTED/CAPTURE result; null in visual_only and "
                "typed_contract modes"),
            "confirmation_exposure_sec": (
                "candidate last_seen source stamp minus first fully-in-frame "
                "truth stamp"),
            "confirmation_processing_ms": (
                "monotonic recorder receipt of confirmation minus receipt of "
                "the image at candidate last_seen; independent-subscriber "
                "transport diagnostic and optional when that subscriber "
                "skips the detector-processed frame"),
            "confirmation_pipeline_ms": (
                "same-host monotonic confirmation receipt minus detector "
                "callback start embedded for the same source stamp; canonical "
                "end-to-end latency and performance-contract metric"),
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
            "target_path_lateral_offset_m": (
                "signed independent-truth target-centre cross-track distance "
                "to the planned start-to-finish line; this is not camera "
                "path tracking error"),
            "target_pixel_offset_x_normalized": (
                "truth projected pixel u minus CameraInfo principal point, "
                "divided by image half-width; requested bin values never "
                "substitute this measured quantity"),
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
        "P95 abs path-error ratio | Visibility confirm/selected | "
        "Mean target cross-track (m) | Mean pixel half-frame offset |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {} | {}/{} | {} | {} | {} | {} | {} | {}/{} | {} | {} |".format(
                row["label"], row["completed_trial_count"],
                row["trial_count"], row["p_confirm"], row["p_selected"],
                row["mean_actual_linear_speed_mps"],
                row["p95_actual_linear_speed_mps"],
                row["p95_abs_normalized_lateral_offset"],
                row["p_confirm_visibility"],
                row["p_selected_visibility"],
                row["mean_target_path_lateral_offset_m"],
                row["mean_target_pixel_offset_x_normalized"]))
    lines.append("")
    return lines


def _report_scalar(value):
    return "null" if value is None else str(value)


def _report(summary):
    metrics = summary["metrics"]
    denominators = summary["metric_denominators"]
    validation = summary.get("validation_errors", [])
    completeness = summary["completeness"]
    artifacts = summary["artifact_completeness"]
    verdict = summary["performance_verdict"]
    audit = summary["candidate_audit"]
    navigation = summary["navigation_metrics"]
    lines = [
        "# V-SIM-04 Vision Search Performance",
        "",
        "- Run mode: `{}`".format(summary["run_mode"]),
        "- Evaluation scope: `{}`".format(summary["evaluation_scope"]),
        "- Navigation metrics mode: `{}`".format(navigation["mode"]),
        "- Target-stage capability: `{}`".format(
            navigation["target_stage_capability"]),
        "- Navigation metrics reason: `{}`".format(navigation["reason"]),
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
        "- P_confirm: `{}`".format(_report_scalar(metrics["p_confirm"])),
        "- P_selected: `{}`".format(_report_scalar(metrics["p_selected"])),
        "- Visibility-window P_confirm/P_selected: `{}` / `{}`".format(
            _report_scalar(metrics["p_confirm_visibility"]),
            _report_scalar(metrics["p_selected_visibility"])),
        "- P_decision: `{}`".format(_report_scalar(metrics["p_decision"])),
        "- P_dispatch: `{}`".format(_report_scalar(metrics["p_dispatch"])),
        "- P_planner_arrival: `{}`".format(
            _report_scalar(metrics["p_planner_arrival"])),
        "- P_interrupt: `{}` (reason: `{}`)".format(
            _report_scalar(metrics["p_interrupt"]),
            metrics["p_interrupt_reason"]),
        "- Typed decision/result events: `{}` / `{}`; exact-key matched "
        "results: `{}`".format(
            navigation["decision_event_count"],
            navigation["result_event_count"],
            navigation["matched_result_count"]),
        "- Navigation contract validation errors: `{}`".format(
            navigation["validation_errors"]),
        "- Stage frame rates (raw class/raw geometry/resolved/refined/"
        "geometry/association/center): `{}`".format(
            metrics["stage_frame_rates"]),
        "- Failed trial first-blocking stages: `{}`".format(
            metrics["failure_stage_counts"]),
        "- P95 recorder-observed transport latency (diagnostic): `{}` ms".format(
            metrics["p95_confirmation_processing_ms"]),
        "- P95 same-host end-to-end latency (contract metric): `{}` ms".format(
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
        "- Target cross-track mean/P95 absolute: `{}` / `{}` m".format(
            metrics["mean_target_path_lateral_offset_m"],
            metrics["p95_abs_target_path_lateral_offset_m"]),
        "- Truth pixel half-frame offset mean/P95 absolute: `{}` / `{}`".format(
            metrics["mean_target_pixel_offset_x_normalized"],
            metrics["p95_abs_target_pixel_offset_x_normalized"]),
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
        "P_decision, P_dispatch and P_planner_arrival require independent "
        "typed evidence joined by the complete navigation action key. None of "
        "selected, decision, dispatch or planner arrival substitutes for "
        "P_interrupt. P_interrupt is null unless target_stage capability was "
        "explicitly enabled, then requires STARTED/CAPTURE on that exact key.",
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
    if any(group.get("value") is not None
           for group in breakdowns.get("by_lateral_bin", [])):
        lines.extend(_breakdown_report(
            "Breakdown by lateral bin",
            [group for group in breakdowns.get("by_lateral_bin", [])
             if group.get("value") is not None]))
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
    navigation_metadata = navigation_metrics_metadata(
        terminal_context.get("navigation_metrics_mode", "visual_only"))
    manifest["navigation_metrics"] = {
        **navigation_metadata,
        "decision_topic": terminal_context.get(
            "navigation_decision_topic", ""),
        "result_topic": terminal_context.get(
            "navigation_result_topic", ""),
        "decision_schema_version": NAVIGATION_SCHEMA_VERSION,
        "result_schema_version": NAVIGATION_SCHEMA_VERSION,
        "exact_binding_key": [
            "mission_id", "decision_seq", "target_id",
            "target_first_seen", "attempt", "payload_slot"],
        "decision_event_count": summary["navigation_metrics"][
            "decision_event_count"],
        "result_event_count": summary["navigation_metrics"][
            "result_event_count"],
        "matched_result_count": summary["navigation_metrics"][
            "matched_result_count"],
        "deduplicated_decision_count": summary["navigation_metrics"][
            "deduplicated_decision_count"],
        "deduplicated_result_count": summary["navigation_metrics"][
            "deduplicated_result_count"],
        "validation_errors": copy.deepcopy(
            summary["navigation_metrics"]["validation_errors"]),
    }
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
