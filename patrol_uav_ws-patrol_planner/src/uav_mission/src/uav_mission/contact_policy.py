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


def update_contact_episode(event, details, ros_stamp):
    """Accumulate sampled depth/force peaks independently for each pair.

    ``details`` retains geometry from the deepest sample of each pair; force
    peaks have their own stamps because they may occur in another frame.
    Duration spans the first and last positive samples, not a guessed physical
    separation time. Duplicate pair entries in one frame count as one sample.
    """
    stamp = float(ros_stamp)
    event.setdefault("last_ros_stamp", event["ros_stamp"])
    event["last_ros_stamp"] = max(event["last_ros_stamp"], stamp)
    event["duration_sec"] = max(
        0.0, event["last_ros_stamp"] - event["ros_stamp"])
    event["sample_count"] = event.get("sample_count", 0) + 1
    summaries = {tuple(item["pair"]): item
                 for item in event.get("details", [])}
    seen = set()
    for detail in details:
        pair = tuple(sorted(detail["pair"]))
        depth = detail["max_depth_m"]
        force = detail["total_force_norm_n"]
        if pair not in summaries:
            summary = dict(detail, pair=list(pair), first_ros_stamp=stamp,
                           last_ros_stamp=stamp, duration_sec=0.0,
                           sample_count=0, peak_depth_ros_stamp=None,
                           peak_force_ros_stamp=stamp)
            if depth is not None:
                summary["peak_depth_ros_stamp"] = stamp
            summaries[pair] = summary
        else:
            summary = summaries[pair]
            if depth is not None and (summary["max_depth_m"] is None or
                                      depth > summary["max_depth_m"]):
                for key in ("max_depth_m", "info", "positions", "normals"):
                    summary[key] = detail[key]
                summary["peak_depth_ros_stamp"] = stamp
            if force > summary["total_force_norm_n"]:
                summary["total_force_norm_n"] = force
                summary["peak_force_ros_stamp"] = stamp
        summary["last_ros_stamp"] = max(summary["last_ros_stamp"], stamp)
        summary["duration_sec"] = max(
            0.0, summary["last_ros_stamp"] - summary["first_ros_stamp"])
        if pair not in seen:
            summary["sample_count"] += 1
            seen.add(pair)
    event["details"] = [summaries[pair] for pair in sorted(summaries)]
    event["pairs"] = [list(pair) for pair in sorted(summaries)]
    event["peak_sampled_depth_m"] = max(
        (item["max_depth_m"] for item in summaries.values()
         if item["max_depth_m"] is not None), default=None)
    event["peak_sampled_force_n"] = max(
        (item["total_force_norm_n"] for item in summaries.values()), default=0.0)
