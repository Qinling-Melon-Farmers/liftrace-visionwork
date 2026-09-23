"""Pure helpers for filtering Gazebo contact pairs and counting episodes."""


def relevant_contact_pairs(pairs, ignored_patterns=()):
    """Return normalized non-ignored collision pairs.

    A pair is ignored when either scoped collision name contains a configured
    pattern.  Ground contact is intentionally excluded because takeoff and
    landing are not obstacle collisions.
    """
    normalized = []
    patterns = tuple(str(item) for item in ignored_patterns if str(item))
    for first, second in pairs:
        first = str(first)
        second = str(second)
        if any(pattern in first or pattern in second for pattern in patterns):
            continue
        normalized.append(tuple(sorted((first, second))))
    return sorted(set(normalized))


def contact_episode_transition(was_active, pairs):
    """Return ``(active, increment)`` for a debounced contact sample."""
    active = bool(pairs)
    return active, int(active and not bool(was_active))
