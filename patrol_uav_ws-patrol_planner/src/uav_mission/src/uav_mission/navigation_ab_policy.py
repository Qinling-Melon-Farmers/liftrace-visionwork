"""Pure acceptance policy for the same-seed navigation feature A/B."""

import math


IDENTITY_FIELDS = (
    "field_seed",
    "class_profile",
    "world",
    "target_model_path",
    "truth_targets",
    "route_spec",
)


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _safe_int(value, default=-1):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def compare_navigation_ab(baseline, candidate, max_pose_gap=0.5,
                          max_height=4.0, wall_regression_ratio=1.10,
                          improvement_epsilon=1e-3):
    """Return a structured, fail-closed A/B acceptance report."""
    identity_checks = {
        field: baseline.get(field) == candidate.get(field)
        for field in IDENTITY_FIELDS
    }
    baseline_arrivals = list(baseline.get("arrival_wall_times") or [])
    candidate_arrivals = list(candidate.get("arrival_wall_times") or [])
    common_arrivals = min(len(baseline_arrivals), len(candidate_arrivals))
    if common_arrivals:
        baseline_wall = _finite_float(
            baseline_arrivals[common_arrivals - 1])
        candidate_wall = _finite_float(
            candidate_arrivals[common_arrivals - 1])
        wall_ok = (
            baseline_wall is not None and candidate_wall is not None and
            baseline_wall > 0.0 and
            candidate_wall <= baseline_wall * float(wall_regression_ratio))
    else:
        baseline_wall = None
        candidate_wall = None
        wall_ok = False

    baseline_failures = _safe_int(baseline.get("planning_failure_count"))
    candidate_failures = _safe_int(candidate.get("planning_failure_count"))
    failure_improved = (
        baseline_failures >= 0 and candidate_failures >= 0 and
        candidate_failures < baseline_failures)
    baseline_drift = _finite_float(baseline.get("height_drift_rms"))
    candidate_drift = _finite_float(candidate.get("height_drift_rms"))
    height_improved = (
        baseline_drift is not None and candidate_drift is not None and
        candidate_drift + float(improvement_epsilon) < baseline_drift)

    def run_safe(run):
        pose_gap = _finite_float(run.get("pose_max_gap_wall"))
        altitude = _finite_float(run.get("max_altitude"))
        return bool(
            run.get("status") == "PASS" and
            run.get("assets_ready") is True and
            pose_gap is not None and pose_gap <= float(max_pose_gap) and
            altitude is not None and altitude <= float(max_height) and
            _safe_int(run.get("actual_collision_count")) == 0 and
            _safe_int(run.get("boundary_violation_count")) == 0 and
            _safe_int(run.get("invalid_pose_count")) == 0 and
            _safe_int(run.get("unexpected_goal_count")) == 0 and
            run.get("planner_goal_publishers") == [
                "/navigation_visual_delivery_adapter"] and
            run.get("raw_goal_publishers") == [
                "/target_search_manager_py"])

    checks = {
        "baseline_profile": baseline.get("nav_feature_profile") == "baseline",
        "candidate_profile": (
            candidate.get("nav_feature_profile") == "a68925d"),
        "same_seed_world_truth_model_route": all(identity_checks.values()),
        "baseline_safe": run_safe(baseline),
        "candidate_safe": run_safe(candidate),
        "common_route_progress": common_arrivals > 0,
        "wall_time_regression_le_10pct": wall_ok,
        "planning_failure_or_height_improved": (
            failure_improved or height_improved),
    }
    passed = all(checks.values())
    return {
        "gate": "navigation_feature_ab",
        "status": "PASS" if passed else "FAIL",
        "reason": ("candidate_meets_ab_contract" if passed else
                   "candidate_does_not_meet_ab_contract"),
        "checks": checks,
        "identity_checks": identity_checks,
        "common_arrivals": common_arrivals,
        "common_progress_wall_time": {
            "baseline": baseline_wall,
            "candidate": candidate_wall,
            "ratio": (None if baseline_wall in (None, 0.0) or
                      candidate_wall is None else
                      candidate_wall / baseline_wall),
        },
        "improvement": {
            "planning_failure_improved": failure_improved,
            "height_drift_improved": height_improved,
            "baseline_planning_failures": baseline_failures,
            "candidate_planning_failures": candidate_failures,
            "baseline_height_drift_rms": baseline_drift,
            "candidate_height_drift_rms": candidate_drift,
        },
        # Promotion is deliberately only a report fact.  No config/default is
        # mutated automatically; a reviewed follow-up commit owns promotion.
        "promote_candidate": passed,
    }
