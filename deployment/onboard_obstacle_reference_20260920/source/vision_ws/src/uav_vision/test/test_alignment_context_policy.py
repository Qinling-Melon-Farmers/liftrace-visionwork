#!/usr/bin/env python3
"""Pure regression tests for decision fencing and geometry association."""

import copy
import unittest
from types import SimpleNamespace

from uav_vision.alignment_context_policy import (
    associate_geometry, context_fence_key, context_frozen_key,
    validate_alignment_context,
)


class Stamp:
    def __init__(self, seconds):
        self.secs = int(seconds)
        self.nsecs = int(round((float(seconds) - self.secs) * 1_000_000_000.0))

    def to_sec(self):
        return float(self.secs) + float(self.nsecs) / 1_000_000_000.0


def point(x=1.0, y=2.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def pose(frame="camera_init", x=1.0, y=2.0, z=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        pose=SimpleNamespace(position=point(x, y, z)),
    )


def context(**updates):
    value = SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(100.0)),
        source="navigation_coordinator",
        schema_version=1,
        active=True,
        mission_id="mission-1",
        decision_seq=7,
        deadline=Stamp(110.0),
        command=2,
        class_profile="r2026",
        align_mode="drop_circle",
        has_target=True,
        semantic_target_id=0,
        semantic_target_first_seen=Stamp(90.0),
        target_observation_stamp=Stamp(99.0),
        semantic_target_class="tent",
        attempt=1,
        payload_slot=2,
        target_pose=pose(),
        max_association_distance_m=0.5,
    )
    for name, update in updates.items():
        setattr(value, name, update)
    return value


def target(**updates):
    value = SimpleNamespace(
        id=42,
        first_seen=Stamp(95.0),
        class_name="circle",
        map_valid=True,
        map_frame="camera_init",
        map_point=point(1.1, 2.0, 0.0),
    )
    for name, update in updates.items():
        setattr(value, name, update)
    return value


class AlignmentContextPolicyTest(unittest.TestCase):
    def validate(self, value, now=100.1, profile="r2026",
                 allowed=frozenset({"tent", "pillbox", "bridge", "panzer",
                                    "red_cross"})):
        return validate_alignment_context(
            value, Stamp(now), profile, {2}, value.align_mode, 0.5, allowed)

    def test_target_id_zero_is_valid_when_has_target_is_true(self):
        valid, reason = self.validate(context(semantic_target_id=0))
        self.assertTrue(valid, reason)

    def test_source_stamp_profile_command_and_exclusive_deadline_fail_closed(self):
        cases = [
            (context(source=""), "alignment_context_source_missing"),
            (context(header=SimpleNamespace(stamp=Stamp(99.0))),
             "alignment_context_stale"),
            (context(header=SimpleNamespace(stamp=Stamp(100.2))),
             "alignment_context_future_stamp"),
            (context(command=1), "alignment_context_command_mismatch"),
            (context(class_profile="full"),
             "alignment_context_profile_mismatch"),
            (context(deadline=Stamp(100.1)),
             "alignment_context_deadline_expired"),
            (context(semantic_target_class="tank"),
             "alignment_context_profile_target_disallowed"),
        ]
        for value, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(self.validate(value), (False, expected))

        full_tank = context(
            class_profile="full", semantic_target_class="tank")
        valid, reason = self.validate(
            full_tank, profile="full",
            allowed=frozenset({"tent", "pillbox", "bridge", "panzer",
                               "tank", "red_cross"}))
        self.assertTrue(valid, reason)

    def test_fence_key_covers_complete_navigation_transaction(self):
        original = context_fence_key(context())
        mutations = {
            "mission_id": "mission-2",
            "decision_seq": 8,
            "semantic_target_id": 1,
            "semantic_target_first_seen": Stamp(91.0),
            "attempt": 2,
            "payload_slot": 3,
        }
        for field, update in mutations.items():
            changed = context()
            setattr(changed, field, update)
            with self.subTest(field=field):
                self.assertNotEqual(original, context_fence_key(changed))

    def test_frozen_key_allows_heartbeat_only(self):
        original = context()
        heartbeat = copy.deepcopy(original)
        heartbeat.header.stamp = Stamp(100.2)
        self.assertEqual(
            context_frozen_key(original), context_frozen_key(heartbeat))

        changed = copy.deepcopy(original)
        changed.target_observation_stamp = Stamp(99.5)
        self.assertNotEqual(
            context_frozen_key(original), context_frozen_key(changed))
        changed = copy.deepcopy(original)
        changed.target_pose.pose.position.x = 1.2
        self.assertNotEqual(
            context_frozen_key(original), context_frozen_key(changed))

    def test_standard_ring_uses_same_frame_distance_not_equal_id(self):
        semantic = context(semantic_target_id=0)
        geometry = target(id=42)
        matched, reason, distance = associate_geometry(
            semantic, geometry, "drop_circle")
        self.assertTrue(matched, reason)
        self.assertAlmostEqual(distance, 0.1)

        mismatched, reason, _ = associate_geometry(
            semantic, target(map_frame="map"), "drop_circle")
        self.assertFalse(mismatched)
        self.assertEqual(reason, "alignment_context_geometry_frame_mismatch")
        mismatched, reason, distance = associate_geometry(
            semantic, target(map_point=point(2.0, 2.0, 0.0)),
            "drop_circle")
        self.assertFalse(mismatched)
        self.assertEqual(reason, "alignment_context_geometry_distance_exceeded")
        self.assertAlmostEqual(distance, 1.0)

    def test_red_cross_matches_exact_id_and_first_seen_including_zero(self):
        first_seen = Stamp(90.0)
        semantic = context(
            align_mode="drop_cross", semantic_target_id=0,
            semantic_target_first_seen=first_seen,
            semantic_target_class="red_cross")
        geometry = target(
            id=0, first_seen=Stamp(90.0), class_name="red_cross")
        matched, reason, _ = associate_geometry(
            semantic, geometry, "drop_cross")
        self.assertTrue(matched, reason)

        geometry.first_seen = Stamp(90.1)
        matched, reason, _ = associate_geometry(
            semantic, geometry, "drop_cross")
        self.assertFalse(matched)
        self.assertEqual(
            reason, "alignment_context_geometry_identity_mismatch")


if __name__ == "__main__":
    unittest.main()
