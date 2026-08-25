#!/usr/bin/env python3

import types
import unittest

from uav_mission.candidate_policy import CandidatePolicy
from uav_mission.search_policy import SearchPolicy


def candidate(target_id=1, **overrides):
    values = {
        "id": target_id,
        "class_name": "bridge",
        "map_valid": True,
        "state": 2,
        "map_point": types.SimpleNamespace(x=1.0, y=2.0, z=0.0),
    }
    values.update(overrides)
    return types.SimpleNamespace(**values)


class NavigationUpstreamPolicyTest(unittest.TestCase):
    def test_upstream_toudi4_parameters_generate_sixteen_points(self):
        policy = SearchPolicy(-3.6, 2.6, -2.0, 6.0, 1.2, 2.2)
        self.assertEqual(len(policy.waypoints), 16)
        self.assertEqual(policy.waypoints[0].as_tuple(), (-3.6, -2.0, 2.2))
        self.assertEqual(policy.waypoints[1].as_tuple(), (2.6, -2.0, 2.2))
        self.assertEqual(policy.waypoints[-1].as_tuple(), (-3.6, 6.0, 2.2))

    def test_restore_keeps_interrupted_waypoint(self):
        policy = SearchPolicy(-3.6, 2.6, -2.0, 6.0, 1.2, 2.2)
        policy.advance()
        policy.advance()
        self.assertEqual(policy.restore(1), policy.waypoints[1])

    def test_upstream_candidate_policy_validates_and_deduplicates_id(self):
        policy = CandidatePolicy(minimum_state=2)
        first = candidate(7)
        self.assertTrue(policy.accept(first))
        self.assertFalse(policy.accept(first))
        self.assertFalse(policy.accept(candidate(8, map_valid=False)))
        self.assertFalse(policy.accept(candidate(9, state=1)))
        self.assertEqual(policy.known_target_ids, frozenset({7}))

    def test_non_finite_map_point_is_rejected(self):
        policy = CandidatePolicy()
        invalid = candidate(
            10, map_point=types.SimpleNamespace(x=float("nan"), y=0.0, z=0.0))
        self.assertFalse(policy.accept(invalid))


if __name__ == "__main__":
    unittest.main()
