"""Pure target-selection admission policy shared by target_memory and tests."""

import math


CLASS_PROFILES = {
    "full": frozenset({
        "tent", "pillbox", "bridge", "panzer", "tank", "red_cross",
    }),
    "r2026": frozenset({
        "tent", "pillbox", "bridge", "panzer", "red_cross",
    }),
}


def resolve_class_profile(profile_name):
    """Return the allowed selectable classes or fail closed on unknown input."""
    normalized = str(profile_name).strip().lower()
    if normalized not in CLASS_PROFILES:
        raise ValueError(
            "unsupported class_profile={!r}; expected one of {}".format(
                profile_name, ", ".join(sorted(CLASS_PROFILES))))
    return normalized, CLASS_PROFILES[normalized]


def _seconds(value):
    return float(value.to_sec()) if hasattr(value, "to_sec") else float(value)


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _finite_point(point):
    return all(_finite(getattr(point, axis, None)) for axis in ("x", "y", "z"))


def candidate_is_currently_selectable(candidate, now, confirm_frames,
                                      selected_max_age, priorities,
                                      allowed_classes, confirmed_state=2):
    """Check current admission without changing the candidate's sticky state."""
    try:
        class_name = str(candidate.class_name)
        now_sec = _seconds(now)
        last_seen_sec = _seconds(candidate.last_seen)
        maximum_age = float(selected_max_age)
        priority = float(priorities.get(class_name, 0.0))
        state = int(candidate.state)
        observe_count = int(candidate.consecutive_observe_count)
        required_observe_count = int(confirm_frames)
        required_state = int(confirmed_state)
        candidate_id = int(candidate.id)
        map_valid = bool(candidate.map_valid)
        association_valid = bool(candidate.association_valid)
        reject_reason = str(candidate.reject_reason).strip()
        class_confidence = candidate.class_confidence
        map_quality = candidate.map_quality
        map_point = candidate.map_point
        map_frame = str(candidate.map_frame).strip()
        class_allowed = class_name in allowed_classes
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    if not class_allowed or candidate_id < 0:
        return False
    if not math.isfinite(priority) or priority <= 0.0:
        return False
    if state != required_state:
        return False
    if required_observe_count < 1 or observe_count < required_observe_count:
        return False
    if not map_valid or not association_valid:
        return False
    if reject_reason:
        return False
    if (not math.isfinite(now_sec) or not math.isfinite(last_seen_sec) or
            not math.isfinite(maximum_age) or maximum_age < 0.0):
        return False
    if (not _finite(class_confidence) or not _finite(map_quality) or
            not _finite_point(map_point) or not map_frame):
        return False
    observation_age = now_sec - last_seen_sec
    return 0.0 <= observation_age <= maximum_age


def choose_selected_candidate(candidates, now, confirm_frames,
                              selected_max_age, priorities, allowed_classes,
                              confirmed_state=2):
    """Return the highest-priority currently admissible candidate, if any."""
    eligible = [
        candidate for candidate in candidates
        if candidate_is_currently_selectable(
            candidate, now, confirm_frames, selected_max_age, priorities,
            allowed_classes, confirmed_state)
    ]
    if not eligible:
        return None
    return max(
        eligible,
        key=lambda candidate: (
            float(priorities.get(candidate.class_name, 0.0)),
            float(candidate.class_confidence),
            float(candidate.map_quality),
            -int(candidate.id),
        ))
