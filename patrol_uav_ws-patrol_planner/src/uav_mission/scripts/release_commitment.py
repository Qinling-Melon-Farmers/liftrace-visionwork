#!/usr/bin/env python3
"""Pure policy for locking a visually verified target through final descent."""

from dataclasses import dataclass
import math


MODE_TARGET_CLASS = {
    "drop_circle": "circle",
    "drop_cross": "red_cross",
}


def _seconds(value):
    return float(value.to_sec()) if hasattr(value, "to_sec") else float(value)


def _stamp_key(value):
    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return int(value.secs), int(value.nsecs)
    seconds = _seconds(value)
    whole = int(math.floor(seconds))
    return whole, int(round((seconds - whole) * 1_000_000_000.0))


def strict_context_source(context, now, maximum_age, class_profile,
                          align_mode, payload_slot):
    """Return semantic release identity from one valid context wrapper.

    ``ReleaseEvidence`` identifies the observed geometry (for example a blue
    circle), while the mission queue owns the semantic target (for example a
    tent).  Strict mode accepts the geometry only through the already-frozen
    ``ReleaseEvidenceContext`` and returns the semantic identity for the
    permission/result path.
    """

    if context is None:
        return False, "release_evidence_context_missing", None
    try:
        evidence = context.evidence
        now_sec = _seconds(now)
        evidence_stamp = _seconds(evidence.header.stamp)
        context_stamp = _seconds(context.context_header.stamp)
        deadline = _seconds(context.deadline)
        max_age = float(maximum_age)
        if not all(math.isfinite(value) for value in (
                now_sec, evidence_stamp, context_stamp, deadline, max_age)):
            return False, "release_evidence_context_time_invalid", None
        if (max_age < 0.0 or evidence_stamp <= 0.0 or
                context_stamp <= 0.0):
            return False, "release_evidence_context_unstamped", None
        if (now_sec - evidence_stamp < 0.0 or
                now_sec - context_stamp < 0.0):
            return False, "release_evidence_context_from_future", None
        if (now_sec - evidence_stamp > max_age or
                now_sec - context_stamp > max_age):
            return False, "release_evidence_context_stale", None
        if deadline <= now_sec:
            return False, "release_evidence_context_deadline", None
        if (not context.context_valid or not context.context_active or
                not context.has_semantic_target or
                not context.semantic_geometry_match):
            return False, "release_evidence_context_invalid", None
        if (context.context_schema_version != 1 or
                not str(context.context_source).strip() or
                not str(context.mission_id).strip() or
                int(context.decision_seq) <= 0):
            return False, "release_evidence_context_fence_invalid", None
        if (str(context.class_profile) != str(class_profile) or
                str(context.align_mode) != str(align_mode) or
                int(context.command) != 2 or
                int(context.payload_slot) != int(payload_slot)):
            return False, "release_evidence_context_fence_mismatch", None
        if (not evidence.evidence_valid or not evidence.aligned or
                str(evidence.align_mode) != str(align_mode) or
                not context.geometry_target_present or
                not context.geometry_map_valid):
            return False, "release_evidence_context_geometry_invalid", None
        if (int(evidence.target_id) != int(context.geometry_target_id) or
                str(evidence.target_class) !=
                str(context.geometry_target_class)):
            return False, "release_evidence_context_geometry_mismatch", None

        semantic_class = str(context.semantic_target_class).strip()
        geometry_class = str(context.geometry_target_class).strip()
        if align_mode == "drop_cross":
            if semantic_class != "red_cross" or geometry_class != "red_cross":
                return False, "release_evidence_context_class_mismatch", None
            if (int(context.semantic_target_id) !=
                    int(context.geometry_target_id) or
                    _stamp_key(context.semantic_target_first_seen) !=
                    _stamp_key(context.geometry_target_first_seen)):
                return False, "release_evidence_context_identity_mismatch", None
        elif align_mode == "drop_circle":
            if not semantic_class or semantic_class == "red_cross" or \
                    geometry_class != "circle":
                return False, "release_evidence_context_class_mismatch", None
        else:
            return False, "release_evidence_context_mode_invalid", None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, "release_evidence_context_malformed", None

    return True, "permission_granted", {
        "target_id": int(context.semantic_target_id),
        "target_class": semantic_class,
        "geometry_target_class": geometry_class,
        "evidence_stamp": evidence.header.stamp,
        "stable_frames": int(evidence.stable_frames),
    }


@dataclass(frozen=True)
class ReleaseCommitment:
    align_mode: str
    target_id: int
    target_class: str
    payload_slot: int
    locked_at: float
    evidence_stamp_nsec: int
    locked_x: float
    locked_y: float
    stable_frames: int


class ReleaseCommitmentPolicy:
    def __init__(self, required_control_state, commitment_timeout,
                 max_horizontal_drift):
        self._required_control_state = int(required_control_state)
        self._commitment_timeout = float(commitment_timeout)
        self._max_horizontal_drift = float(max_horizontal_drift)

    def observe(self, now, evidence, control_state, pose, next_slot,
                released_targets, evidence_fresh=True, pose_fresh=True,
                control_state_fresh=True):
        if not evidence_fresh or not pose_fresh or not control_state_fresh:
            return None
        if not evidence or not evidence.get("evidence_valid"):
            return None
        mode = evidence.get("align_mode", "")
        expected_class = MODE_TARGET_CLASS.get(mode)
        geometry_class = evidence.get(
            "geometry_target_class", evidence.get("target_class"))
        if expected_class is None or geometry_class != expected_class:
            return None
        if int(control_state) != self._required_control_state or pose is None:
            return None
        target_key = (mode, int(evidence.get("target_id", -1)))
        if target_key in released_targets:
            return None
        return ReleaseCommitment(
            align_mode=mode,
            target_id=target_key[1],
            target_class=evidence["target_class"],
            payload_slot=int(next_slot),
            locked_at=float(now),
            evidence_stamp_nsec=int(
                evidence.get("evidence_stamp_nsec",
                             round(float(now) * 1000000000.0))),
            locked_x=float(pose[0]),
            locked_y=float(pose[1]),
            stable_frames=int(evidence.get("stable_frames", 0)),
        )

    def evaluate(self, commitment, now, align_mode, control_state, pose,
                 next_slot, released_targets, current_evidence_valid=False,
                 current_target_key=None):
        if commitment is None:
            return False, "no_release_commitment"
        if float(now) - commitment.locked_at > self._commitment_timeout:
            return False, "commitment_expired"
        if align_mode != commitment.align_mode:
            return False, "commitment_mode_changed"
        if int(control_state) != self._required_control_state:
            return False, "control_not_aligning"
        if int(next_slot) != commitment.payload_slot:
            return False, "commitment_slot_changed"
        commitment_target_key = (
            commitment.align_mode, commitment.target_id)
        if (current_evidence_valid and current_target_key is not None and
                tuple(current_target_key) != commitment_target_key):
            return False, "commitment_target_changed"
        if commitment_target_key in released_targets:
            return False, "target_already_released"
        if pose is None:
            return False, "no_vehicle_pose"
        drift = math.hypot(float(pose[0]) - commitment.locked_x,
                           float(pose[1]) - commitment.locked_y)
        if drift > self._max_horizontal_drift:
            return False, "commitment_position_drift"
        return True, "permission_granted_from_commitment"
