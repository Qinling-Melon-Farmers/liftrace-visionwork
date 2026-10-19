#!/usr/bin/env python3

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from release_commitment import (  # noqa: E402
    ReleaseCommitmentPolicy,
    commitment_context_rejection_is_terminal,
    commitment_matches_fence,
    commitment_rejection_is_terminal,
    strict_commitment_fence,
)


def _context(stamp=164.3, deadline=253.9, decision_seq=9,
             active=True, context_valid=False):
    return SimpleNamespace(
        context_header=SimpleNamespace(stamp=stamp),
        deadline=deadline,
        context_active=active,
        context_valid=context_valid,
        context_schema_version=1,
        context_source="planner_bridge",
        mission_id="mission-r36",
        decision_seq=decision_seq,
        class_profile="r2026",
        align_mode="drop_circle",
        command=2,
        payload_slot=2,
        has_semantic_target=True,
        semantic_target_id=4,
        semantic_target_first_seen=133.604,
        semantic_target_class="bridge",
        attempt=1,
    )


class ReleaseCommitmentTest(unittest.TestCase):
    def setUp(self):
        self.policy = ReleaseCommitmentPolicy(
            required_control_state=2,
            commitment_timeout=45.0,
            max_horizontal_drift=0.20,
        )
        self.evidence = {
            "evidence_valid": True,
            "align_mode": "drop_circle",
            "target_id": 4,
            "target_class": "bridge",
            "geometry_target_class": "circle",
            "stable_frames": 5,
            "evidence_stamp_nsec": 148_356_000_000,
            "mission_id": "mission-r36",
            "decision_seq": 9,
            "attempt": 1,
            "target_first_seen_nsec": 133_604_000_000,
            "deadline_at": 253.9,
        }
        self.commitment = self.policy.observe(
            now=148.356,
            evidence=self.evidence,
            control_state=2,
            pose=(-1.133, 4.125),
            next_slot=2,
            released_targets=set(),
        )

    def _evaluate(self, now, pose):
        return self.policy.evaluate(
            commitment=self.commitment,
            now=now,
            align_mode="drop_circle",
            control_state=2,
            pose=pose,
            next_slot=2,
            released_targets=set(),
        )

    def test_r36_transient_drift_does_not_destroy_commitment(self):
        valid, reason = self._evaluate(164.3, (-0.9587, 4.2276))
        self.assertFalse(valid)
        self.assertEqual(reason, "commitment_position_drift")
        self.assertFalse(commitment_rejection_is_terminal(reason))

        valid, reason = self._evaluate(171.9, (-1.0344, 4.1008))
        self.assertTrue(valid)
        self.assertEqual(reason, "permission_granted_from_commitment")

    def test_manager_deadline_replaces_local_timeout_for_strict_context(self):
        valid, _ = self._evaluate(200.0, (-1.0344, 4.1008))
        self.assertTrue(valid)
        valid, reason = self._evaluate(253.9, (-1.0344, 4.1008))
        self.assertFalse(valid)
        self.assertEqual(reason, "commitment_deadline_reached")
        self.assertTrue(commitment_rejection_is_terminal(reason))

    def test_fence_survives_geometry_dropout_but_not_new_generation(self):
        valid, reason, fence = strict_commitment_fence(
            _context(), 164.3, 0.5, "r2026", "drop_circle", 2)
        self.assertTrue(valid, reason)
        self.assertTrue(commitment_matches_fence(self.commitment, fence))

        valid, reason, new_fence = strict_commitment_fence(
            _context(decision_seq=10), 164.3, 0.5,
            "r2026", "drop_circle", 2)
        self.assertTrue(valid, reason)
        self.assertFalse(
            commitment_matches_fence(self.commitment, new_fence))

    def test_context_deadline_and_inactive_state_end_commitment(self):
        valid, reason, _ = strict_commitment_fence(
            _context(deadline=164.3), 164.3, 0.5,
            "r2026", "drop_circle", 2)
        self.assertFalse(valid)
        self.assertEqual(reason, "commitment_context_deadline")
        self.assertTrue(commitment_context_rejection_is_terminal(reason))

        valid, reason, _ = strict_commitment_fence(
            _context(active=False), 164.3, 0.5,
            "r2026", "drop_circle", 2)
        self.assertFalse(valid)
        self.assertEqual(reason, "commitment_context_inactive")
        self.assertTrue(commitment_context_rejection_is_terminal(reason))

    def test_release_height_band_covers_r36_near_ground_oscillation(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "release_guard.yaml").read_text())
        self.assertEqual(config["max_release_altitude"], 0.30)
        self.assertLessEqual(0.2658, config["max_release_altitude"])


if __name__ == "__main__":
    unittest.main()
