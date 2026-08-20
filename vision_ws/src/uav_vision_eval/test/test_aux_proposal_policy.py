#!/usr/bin/env python3
from pathlib import Path
import sys
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from uav_vision_eval.aux_proposal_policy import (  # noqa: E402
    map_quality_uncertainty,
    validate_aux_proposal,
)


class AuxProposalPolicyTest(unittest.TestCase):
    def _validate(self, **overrides):
        values = {
            "class_hint": "circle",
            "confidence": 0.8,
            "map_valid": True,
            "map_frame": "camera_init",
            "map_quality": 0.7,
            "state": 2,
            "reject_reason": "",
            "x": 1.0,
            "y": 2.0,
            "source_stamp_sec": 9.5,
            "now_sec": 10.0,
            "accepted_classes": {"circle"},
            "expected_frame": "camera_init",
            "min_confidence": 0.35,
            "min_map_quality": 0.1,
            "min_state": 2,
            "max_age_sec": 1.0,
            "max_future_skew_sec": 0.1,
        }
        values.update(overrides)
        return validate_aux_proposal(**values)

    def test_accepts_fresh_confirmed_candidate(self):
        self.assertEqual(self._validate(), (True, ""))

    def test_rejects_stale_and_wrong_class(self):
        self.assertEqual(
            self._validate(source_stamp_sec=8.0),
            (False, "source_candidate_stale"))
        self.assertEqual(
            self._validate(class_hint="red_cross"),
            (False, "class_not_accepted"))

    def test_rejects_future_and_non_finite_map_point(self):
        self.assertEqual(
            self._validate(source_stamp_sec=10.2),
            (False, "source_stamp_in_future"))
        self.assertEqual(
            self._validate(x=float("nan")),
            (False, "map_point_non_finite"))

    def test_uncertainty_proxy_decreases_with_quality(self):
        self.assertAlmostEqual(map_quality_uncertainty(1.0, 0.35, 0.65), 0.35)
        self.assertAlmostEqual(map_quality_uncertainty(0.0, 0.35, 0.65), 1.0)
        self.assertLess(
            map_quality_uncertainty(0.8, 0.35, 0.65),
            map_quality_uncertainty(0.2, 0.35, 0.65))


if __name__ == "__main__":
    unittest.main()
