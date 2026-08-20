#!/usr/bin/env python3
from pathlib import Path
import sys
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from uav_vision_eval.aux_search_policy import (  # noqa: E402
    AuxCandidateBook,
    SOURCE_AUX_CV,
    STATUS_APPROACHING,
    STATUS_CONFIRMED,
    STATUS_DETECTED,
    STATUS_REJECTED,
    STATUS_VERIFYING,
    fresh_spatial_match,
    handoff_gate_status,
    sparse_scan_interrupt_decision,
)


class AuxSearchPolicyTest(unittest.TestCase):
    def test_observations_cluster_and_keep_stable_id(self):
        book = AuxCandidateBook(match_distance_m=0.8)
        first = book.observe(4, 1.0, 2.0, 0.7, 0.5, 10.0)
        repeated = book.observe(4, 1.2, 2.0, 0.8, 0.9, 10.2)
        separate = book.observe(7, 3.0, 2.0, 0.6, 0.7, 10.3)

        self.assertIs(first, repeated)
        self.assertEqual(first.id, 1)
        self.assertEqual(first.source, SOURCE_AUX_CV)
        self.assertEqual(first.observations, 2)
        self.assertEqual(first.status, STATUS_DETECTED)
        self.assertEqual(separate.id, 2)

    def test_lifecycle_requires_ordered_transitions(self):
        candidate = AuxCandidateBook(0.8).observe(
            1, 1.0, 2.0, 0.8, 0.7, 10.0)
        self.assertFalse(candidate.start_verify(11.0))
        self.assertTrue(candidate.start_approach(11.0))
        self.assertEqual(candidate.status, STATUS_APPROACHING)
        self.assertTrue(candidate.start_verify(12.0))
        self.assertEqual(candidate.status, STATUS_VERIFYING)
        self.assertTrue(candidate.confirm(12.5, 9, "tank", 0.2))
        self.assertEqual(candidate.status, STATUS_CONFIRMED)
        self.assertFalse(candidate.reject(13.0, "late_reject"))

    def test_handoff_rejects_stale_far_and_preverify_targets(self):
        candidate = AuxCandidateBook(0.8).observe(
            1, 1.0, 2.0, 0.8, 0.7, 10.0)
        candidate.start_approach(11.0)
        candidate.start_verify(12.0)
        targets = [
            {"id": 1, "class_name": "tent", "x": 1.1, "y": 2.0,
             "last_seen_sec": 11.0},
            {"id": 2, "class_name": "tank", "x": 3.0, "y": 2.0,
             "last_seen_sec": 12.4},
            {"id": 3, "class_name": "bridge", "x": 1.2, "y": 2.0,
             "last_seen_sec": 12.1},
        ]
        matched = fresh_spatial_match(
            candidate, targets, now_sec=12.5, max_distance_m=1.0,
            max_age_sec=0.75)
        self.assertIsNotNone(matched)
        self.assertEqual(matched[1]["id"], 3)

        self.assertIsNone(fresh_spatial_match(
            candidate, targets, now_sec=13.0, max_distance_m=1.0,
            max_age_sec=0.75))

    def test_rejected_candidate_is_not_scheduled_again(self):
        book = AuxCandidateBook(0.8)
        rejected = book.observe(1, 0.0, 0.0, 0.8, 0.8, 1.0)
        available = book.observe(2, 2.0, 0.0, 0.8, 0.8, 1.0)
        rejected.start_approach(2.0)
        rejected.reject(3.0, "goal_progress_timeout")

        route = book.visit_order(0.0, 0.0)
        self.assertEqual(route, [available])
        self.assertEqual(rejected.status, STATUS_REJECTED)
        self.assertEqual(book.state_counts()[STATUS_REJECTED], 1)

    def test_handoff_subgate_is_independent_from_mission_return(self):
        book = AuxCandidateBook(0.8)
        candidate = book.observe(1, 0.0, 0.0, 0.8, 0.8, 1.0)
        self.assertEqual(
            handoff_gate_status(book.records, False), "NOT_EXERCISED")
        self.assertEqual(handoff_gate_status(book.records, True), "PENDING")
        candidate.start_approach(2.0)
        candidate.start_verify(3.0)
        candidate.confirm(3.2, 9, "tank", 0.2)
        self.assertEqual(handoff_gate_status(book.records, True), "PASS")

        rejected = book.observe(2, 2.0, 0.0, 0.8, 0.8, 3.3)
        rejected.start_approach(3.4)
        rejected.reject(3.5, "downward_verify_timeout")
        self.assertEqual(handoff_gate_status(book.records, True), "FAIL")

    def test_anonymous_candidates_queue_but_semantic_high_value_interrupts(self):
        book = AuxCandidateBook(0.8)
        book.observe(1, 0.0, 0.0, 0.8, 0.8, 1.0,
                     class_hint="circle")
        book.observe(2, 2.0, 0.0, 0.8, 0.8, 1.0,
                     class_hint="circle")
        self.assertEqual(
            sparse_scan_interrupt_decision(
                book.records, {"red_cross", "tank"}, 0),
            (False, "queue_only"))
        self.assertEqual(
            sparse_scan_interrupt_decision(
                book.records, {"red_cross", "tank"}, 2),
            (True, "anonymous_interrupt_count_2"))

        semantic = AuxCandidateBook(0.8)
        semantic.observe(3, 1.0, 1.0, 0.9, 0.8, 2.0,
                         source="AUX_YOLO", class_hint="red_cross")
        self.assertEqual(
            sparse_scan_interrupt_decision(
                semantic.records, {"red_cross", "tank"}, 0),
            (True, "semantic_interrupt_red_cross"))


if __name__ == "__main__":
    unittest.main()
