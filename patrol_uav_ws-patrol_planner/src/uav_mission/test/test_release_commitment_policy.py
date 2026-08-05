#!/usr/bin/env python3
import unittest

from release_commitment import ReleaseCommitmentPolicy


class ReleaseCommitmentPolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = ReleaseCommitmentPolicy(
            required_control_state=2,
            commitment_timeout=30.0,
            max_horizontal_drift=0.20,
        )
        self.evidence = {
            "evidence_valid": True,
            "align_mode": "drop_circle",
            "target_id": 17,
            "target_class": "circle",
            "stable_frames": 5,
            "evidence_stamp_nsec": 99000000123,
        }

    def test_stable_lock_survives_stale_visual_during_controlled_descent(self):
        commitment = self.policy.observe(
            now=100.0,
            evidence=self.evidence,
            control_state=2,
            pose=(1.0, -2.0),
            next_slot=1,
            released_targets=set(),
        )

        permitted, reason = self.policy.evaluate(
            commitment=commitment,
            now=118.0,
            align_mode="drop_circle",
            control_state=2,
            pose=(1.12, -2.08),
            next_slot=1,
            released_targets=set(),
            current_evidence_valid=False,
        )

        self.assertIsNotNone(commitment)
        self.assertEqual(commitment.evidence_stamp_nsec, 99000000123)
        self.assertTrue(permitted)
        self.assertEqual(reason, "permission_granted_from_commitment")

    def test_no_valid_lock_never_allows_release(self):
        invalid = dict(self.evidence, evidence_valid=False)
        commitment = self.policy.observe(
            now=100.0,
            evidence=invalid,
            control_state=2,
            pose=(1.0, -2.0),
            next_slot=1,
            released_targets=set(),
        )

        permitted, reason = self.policy.evaluate(
            commitment=commitment,
            now=101.0,
            align_mode="drop_circle",
            control_state=2,
            pose=(1.0, -2.0),
            next_slot=1,
            released_targets=set(),
            current_evidence_valid=False,
        )

        self.assertIsNone(commitment)
        self.assertFalse(permitted)
        self.assertEqual(reason, "no_release_commitment")

    def test_stale_inputs_cannot_create_commitment(self):
        freshness_cases = [
            (False, True, True),
            (True, False, True),
            (True, True, False),
        ]
        for evidence_fresh, pose_fresh, control_state_fresh in freshness_cases:
            with self.subTest(
                    evidence_fresh=evidence_fresh,
                    pose_fresh=pose_fresh,
                    control_state_fresh=control_state_fresh):
                commitment = self.policy.observe(
                    now=100.0,
                    evidence=self.evidence,
                    control_state=2,
                    pose=(1.0, -2.0),
                    next_slot=1,
                    released_targets=set(),
                    evidence_fresh=evidence_fresh,
                    pose_fresh=pose_fresh,
                    control_state_fresh=control_state_fresh,
                )
                self.assertIsNone(commitment)

    def test_timeout_drift_mode_and_replay_invalidate_lock(self):
        commitment = self.policy.observe(
            now=100.0,
            evidence=self.evidence,
            control_state=2,
            pose=(1.0, -2.0),
            next_slot=1,
            released_targets=set(),
        )
        cases = [
            (131.0, "drop_circle", 2, (1.0, -2.0), 1, set(), None, "commitment_expired"),
            (110.0, "drop_circle", 2, (1.21, -2.0), 1, set(), None, "commitment_position_drift"),
            (110.0, "drop_cross", 2, (1.0, -2.0), 1, set(), None, "commitment_mode_changed"),
            (110.0, "drop_circle", 1, (1.0, -2.0), 1, set(), None, "control_not_aligning"),
            (110.0, "drop_circle", 2, (1.0, -2.0), 2, set(), None, "commitment_slot_changed"),
            (110.0, "drop_circle", 2, (1.0, -2.0), 1,
             {("drop_circle", 17)}, None, "target_already_released"),
            (110.0, "drop_circle", 2, (1.0, -2.0), 1, set(),
             ("drop_circle", 18), "commitment_target_changed"),
        ]
        for args in cases:
            with self.subTest(reason=args[-1]):
                permitted, reason = self.policy.evaluate(
                    commitment=commitment,
                    now=args[0],
                    align_mode=args[1],
                    control_state=args[2],
                    pose=args[3],
                    next_slot=args[4],
                    released_targets=args[5],
                    current_evidence_valid=args[6] is not None,
                    current_target_key=args[6],
                )
                self.assertFalse(permitted)
                self.assertEqual(reason, args[-1])


if __name__ == "__main__":
    unittest.main()
