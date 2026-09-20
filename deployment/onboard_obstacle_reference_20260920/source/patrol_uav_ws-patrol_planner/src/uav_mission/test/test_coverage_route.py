#!/usr/bin/env python3

import unittest

from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint


class CoverageRouteTest(unittest.TestCase):
    def make_route(self):
        return CoverageRoute(
            (Waypoint(0.0, 0.0, 2.2), Waypoint(1.0, 0.0, 2.2)),
            route_revision="coverage-r1",
            max_failures_per_waypoint=2,
        )

    def test_interrupt_preserves_nominal_cursor(self):
        route = self.make_route()
        search = route.bind(1, "SEARCH")
        self.assertEqual(search.waypoint_index, 0)
        outcome = route.interrupt(1)
        self.assertTrue(outcome.accepted)
        self.assertEqual(route.current_index, 0)
        retired = route.finish(1, True)
        self.assertFalse(retired.accepted)
        self.assertEqual(retired.reason, "route_result_retired")
        with self.assertRaisesRegex(ValueError, "monotonically"):
            route.bind(1, "RESUME")
        resume = route.bind(2, "RESUME")
        self.assertEqual(resume.waypoint_index, 0)
        self.assertEqual(resume.nominal_waypoint, search.nominal_waypoint)

    def test_success_advances_once_and_duplicate_is_rejected(self):
        route = self.make_route()
        route.bind(1, "SEARCH")
        outcome = route.finish(1, True)
        self.assertTrue(outcome.accepted)
        self.assertTrue(outcome.advanced)
        self.assertEqual(route.current_index, 1)
        duplicate = route.finish(1, True)
        self.assertFalse(duplicate.accepted)
        self.assertEqual(duplicate.reason, "route_result_duplicate")
        self.assertEqual(route.current_index, 1)

    def test_mismatched_result_does_not_mutate_cursor(self):
        route = self.make_route()
        route.bind(4, "SEARCH")
        outcome = route.finish(5, True)
        self.assertFalse(outcome.accepted)
        self.assertEqual(outcome.reason, "route_decision_mismatch")
        self.assertEqual(route.current_index, 0)
        self.assertIsNotNone(route.active)

    def test_bounded_failures_retry_then_skip(self):
        route = self.make_route()
        route.bind(1, "SEARCH")
        retry = route.finish(1, False)
        self.assertEqual(retry.reason, "route_waypoint_retry")
        self.assertEqual(route.current_index, 0)
        route.bind(2, "RESUME")
        skipped = route.finish(2, False)
        self.assertEqual(skipped.reason, "route_waypoint_skipped")
        self.assertTrue(skipped.skipped)
        self.assertEqual(route.skipped_indices, [0])
        self.assertEqual(route.current_index, 1)

    def test_last_waypoint_sets_complete(self):
        route = CoverageRoute(
            (Waypoint(0.0, 0.0, 2.2),), "coverage-r1")
        route.bind(1, "SEARCH")
        outcome = route.finish(1, True)
        self.assertTrue(outcome.complete)
        self.assertTrue(route.is_complete)
        self.assertIsNone(route.current_waypoint)
        self.assertEqual(route.coverage_ratio, 1.0)

    def test_waypoint_altitude_hard_limit(self):
        for altitude in (-0.1, 4.01):
            with self.subTest(altitude=altitude):
                with self.assertRaisesRegex(ValueError, "altitude"):
                    CoverageRoute(
                        (Waypoint(0.0, 0.0, altitude),), "coverage-r1")


if __name__ == "__main__":
    unittest.main()
