#!/usr/bin/env python3
import unittest
from types import SimpleNamespace

from release_commitment import ReleaseCommitmentPolicy, strict_context_source


class Stamp:
    def __init__(self, seconds):
        self.secs = int(seconds)
        self.nsecs = int(round(
            (float(seconds) - self.secs) * 1_000_000_000.0))

    def to_sec(self):
        return self.secs + self.nsecs / 1_000_000_000.0


def strict_context(mode="drop_circle"):
    semantic_class = "red_cross" if mode == "drop_cross" else "tent"
    geometry_class = "red_cross" if mode == "drop_cross" else "circle"
    semantic_id = 7
    geometry_id = 7 if mode == "drop_cross" else 42
    first_seen = Stamp(90.0)
    evidence = SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(100.0)),
        evidence_valid=True,
        aligned=True,
        align_mode=mode,
        target_id=geometry_id,
        target_class=geometry_class,
        stable_frames=5,
    )
    return SimpleNamespace(
        evidence=evidence,
        context_valid=True,
        context_active=True,
        has_semantic_target=True,
        semantic_geometry_match=True,
        context_schema_version=1,
        context_source="vcl06_execution_bridge",
        context_header=SimpleNamespace(stamp=Stamp(100.0)),
        mission_id="mission-1",
        decision_seq=3,
        deadline=Stamp(110.0),
        class_profile="r2026",
        align_mode=mode,
        command=2,
        payload_slot=1,
        semantic_target_id=semantic_id,
        semantic_target_first_seen=first_seen,
        semantic_target_class=semantic_class,
        geometry_target_present=True,
        geometry_map_valid=True,
        geometry_target_id=geometry_id,
        geometry_target_first_seen=first_seen,
        geometry_target_class=geometry_class,
    )


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

    def test_strict_circle_context_commits_semantic_not_geometry_identity(self):
        valid, reason, source = strict_context_source(
            strict_context(), Stamp(100.1), 0.5, "r2026",
            "drop_circle", 1)
        self.assertTrue(valid, reason)
        self.assertEqual(source["target_id"], 7)
        self.assertEqual(source["target_class"], "tent")
        self.assertEqual(source["geometry_target_class"], "circle")

        evidence = {
            "evidence_valid": True,
            "align_mode": "drop_circle",
            "target_id": source["target_id"],
            "target_class": source["target_class"],
            "geometry_target_class": source["geometry_target_class"],
            "stable_frames": source["stable_frames"],
            "evidence_stamp_nsec": 100_000_000_000,
        }
        commitment = self.policy.observe(
            now=100.1,
            evidence=evidence,
            control_state=2,
            pose=(1.0, -2.0),
            next_slot=1,
            released_targets=set(),
        )
        self.assertIsNotNone(commitment)
        self.assertEqual(commitment.target_id, 7)
        self.assertEqual(commitment.target_class, "tent")

    def test_strict_context_fence_and_cross_identity_are_enforced(self):
        wrong_slot = strict_context()
        valid, reason, _ = strict_context_source(
            wrong_slot, Stamp(100.1), 0.5, "r2026",
            "drop_circle", 2)
        self.assertFalse(valid)
        self.assertEqual(reason, "release_evidence_context_fence_mismatch")

        wrong_cross = strict_context("drop_cross")
        wrong_cross.geometry_target_id = 8
        wrong_cross.evidence.target_id = 8
        valid, reason, _ = strict_context_source(
            wrong_cross, Stamp(100.1), 0.5, "r2026",
            "drop_cross", 1)
        self.assertFalse(valid)
        self.assertEqual(reason, "release_evidence_context_identity_mismatch")


if __name__ == "__main__":
    unittest.main()
