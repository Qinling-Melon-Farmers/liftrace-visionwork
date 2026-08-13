#!/usr/bin/env python3
"""Pure policy for locking a visually verified target through final descent."""

from dataclasses import dataclass
import math


MODE_TARGET_CLASS = {
    "drop_circle": "circle",
    "drop_cross": "red_cross",
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
        if expected_class is None or evidence.get("target_class") != expected_class:
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
