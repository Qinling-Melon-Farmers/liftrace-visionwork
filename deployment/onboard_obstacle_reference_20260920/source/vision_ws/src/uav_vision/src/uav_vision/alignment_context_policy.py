"""Pure validation and association rules for frozen alignment contexts."""

import math


SCHEMA_VERSION = 1
COMMAND_ALIGN = 2
STANDARD_CLASSES = frozenset({"tent", "pillbox", "bridge", "panzer", "tank"})


def _seconds(value):
    return float(value.to_sec()) if hasattr(value, "to_sec") else float(value)


def _stamp_key(value):
    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return int(value.secs), int(value.nsecs)
    seconds = _seconds(value)
    whole = int(math.floor(seconds))
    return whole, int(round((seconds - whole) * 1_000_000_000.0))


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _point_tuple(point):
    return tuple(float(getattr(point, axis)) for axis in ("x", "y", "z"))


def _point_is_finite(point):
    try:
        return all(math.isfinite(value) for value in _point_tuple(point))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def context_fence_key(context):
    """Return the complete transaction fence; target id zero is not a sentinel."""
    return (
        str(context.mission_id),
        int(context.decision_seq),
        bool(context.has_target),
        int(context.semantic_target_id),
        _stamp_key(context.semantic_target_first_seen),
        int(context.attempt),
        int(context.payload_slot),
    )


def context_frozen_key(context):
    """Return all fields that must not mutate while a decision is refreshed."""
    pose = context.target_pose
    return context_fence_key(context) + (
        str(context.source),
        int(context.schema_version),
        bool(context.active),
        _stamp_key(context.deadline),
        int(context.command),
        str(context.class_profile),
        str(context.align_mode),
        _stamp_key(context.target_observation_stamp),
        str(context.semantic_target_class),
        str(pose.header.frame_id),
        _point_tuple(pose.pose.position),
        float(context.max_association_distance_m),
    )


def geometry_identity_key(target):
    return int(target.id), _stamp_key(target.first_seen)


def validate_alignment_context(context, now, required_profile,
                               allowed_commands, align_mode,
                               maximum_context_age_sec,
                               allowed_semantic_classes=None):
    """Validate transport freshness, decision lease and frozen semantic data."""
    if context is None:
        return False, "alignment_context_missing"
    try:
        if int(context.schema_version) != SCHEMA_VERSION:
            return False, "alignment_context_schema_mismatch"
        if not str(context.source).strip():
            return False, "alignment_context_source_missing"
        if not bool(context.active):
            return False, "alignment_context_inactive"
        if not str(context.mission_id).strip():
            return False, "alignment_context_mission_missing"
        if int(context.command) not in {int(value) for value in allowed_commands}:
            return False, "alignment_context_command_mismatch"
        if str(context.class_profile).strip() != str(required_profile).strip():
            return False, "alignment_context_profile_mismatch"
        if str(context.align_mode).strip() != str(align_mode).strip():
            return False, "alignment_context_mode_mismatch"
        if not bool(context.has_target):
            return False, "alignment_context_target_missing"

        now_sec = _seconds(now)
        stamp_sec = _seconds(context.header.stamp)
        deadline_sec = _seconds(context.deadline)
        first_seen_sec = _seconds(context.semantic_target_first_seen)
        observation_sec = _seconds(context.target_observation_stamp)
        max_age = float(maximum_context_age_sec)
        if not all(math.isfinite(value) for value in (
                now_sec, stamp_sec, deadline_sec, first_seen_sec,
                observation_sec, max_age)):
            return False, "alignment_context_non_finite"
        if max_age < 0.0:
            return False, "alignment_context_max_age_invalid"
        if stamp_sec <= 0.0:
            return False, "alignment_context_unstamped"
        age = now_sec - stamp_sec
        if age < 0.0:
            return False, "alignment_context_future_stamp"
        if age > max_age:
            return False, "alignment_context_stale"
        if deadline_sec <= now_sec:
            return False, "alignment_context_deadline_expired"
        if first_seen_sec <= 0.0:
            return False, "alignment_context_target_first_seen_invalid"
        if observation_sec < first_seen_sec:
            return False, "alignment_context_observation_stamp_invalid"
        if not str(context.semantic_target_class).strip():
            return False, "alignment_context_target_class_missing"
        if (align_mode in ("drop_circle", "drop_cross") and
                allowed_semantic_classes is not None and
                str(context.semantic_target_class).strip() not in
                allowed_semantic_classes):
            return False, "alignment_context_profile_target_disallowed"
        if not str(context.target_pose.header.frame_id).strip():
            return False, "alignment_context_pose_frame_missing"
        if not _point_is_finite(context.target_pose.pose.position):
            return False, "alignment_context_pose_invalid"
        distance = float(context.max_association_distance_m)
        if not math.isfinite(distance) or distance <= 0.0:
            return False, "alignment_context_distance_invalid"
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, "alignment_context_malformed"
    return True, "alignment_context_valid"


def associate_geometry(context, target, align_mode):
    """Associate an observed geometry instance with the frozen semantic target."""
    if context is None or target is None:
        return False, "alignment_context_geometry_missing", float("inf")
    try:
        semantic_class = str(context.semantic_target_class).strip()
        geometry_class = str(target.class_name).strip()
        if align_mode == "drop_cross":
            if semantic_class != "red_cross" or geometry_class != "red_cross":
                return False, "alignment_context_geometry_class_mismatch", float("inf")
            exact = (
                int(target.id) == int(context.semantic_target_id) and
                _stamp_key(target.first_seen) ==
                _stamp_key(context.semantic_target_first_seen)
            )
            return (
                (True, "alignment_context_valid", 0.0)
                if exact else
                (False, "alignment_context_geometry_identity_mismatch", float("inf"))
            )

        if align_mode == "drop_circle":
            if semantic_class not in STANDARD_CLASSES or geometry_class != "circle":
                return False, "alignment_context_geometry_class_mismatch", float("inf")
            semantic_frame = str(context.target_pose.header.frame_id).strip()
            geometry_frame = str(target.map_frame).strip()
            if (not bool(target.map_valid) or not semantic_frame or
                    geometry_frame != semantic_frame):
                return False, "alignment_context_geometry_frame_mismatch", float("inf")
            semantic_point = context.target_pose.pose.position
            geometry_point = target.map_point
            if not _point_is_finite(semantic_point) or not _point_is_finite(geometry_point):
                return False, "alignment_context_geometry_pose_invalid", float("inf")
            distance = math.sqrt(sum(
                (left - right) ** 2 for left, right in
                zip(_point_tuple(semantic_point), _point_tuple(geometry_point))))
            if distance > float(context.max_association_distance_m):
                return False, "alignment_context_geometry_distance_exceeded", distance
            return True, "alignment_context_valid", distance

        if align_mode == "landing":
            if semantic_class != "landing_pad" or geometry_class != "landing_pad":
                return False, "alignment_context_geometry_class_mismatch", float("inf")
            exact = (
                int(target.id) == int(context.semantic_target_id) and
                _stamp_key(target.first_seen) ==
                _stamp_key(context.semantic_target_first_seen)
            )
            return (
                (True, "alignment_context_valid", 0.0)
                if exact else
                (False, "alignment_context_geometry_identity_mismatch", float("inf"))
            )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, "alignment_context_geometry_malformed", float("inf")
    return False, "alignment_context_mode_unsupported", float("inf")
