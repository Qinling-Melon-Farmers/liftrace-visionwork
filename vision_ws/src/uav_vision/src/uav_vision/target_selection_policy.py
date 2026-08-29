"""Pure target-selection admission policy shared by target_memory and tests."""


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


def candidate_is_currently_selectable(candidate, now, confirm_frames,
                                      selected_max_age, priorities,
                                      allowed_classes, confirmed_state=2):
    """Check current admission without changing the candidate's sticky state."""
    if candidate.class_name not in allowed_classes:
        return False
    if float(priorities.get(candidate.class_name, 0.0)) <= 0.0:
        return False
    if int(candidate.state) != int(confirmed_state):
        return False
    if int(candidate.consecutive_observe_count) < int(confirm_frames):
        return False
    if not bool(candidate.map_valid) or not bool(candidate.association_valid):
        return False
    if str(candidate.reject_reason).strip():
        return False
    observation_age = max(0.0, _seconds(now) - _seconds(candidate.last_seen))
    return observation_age <= float(selected_max_age)


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
