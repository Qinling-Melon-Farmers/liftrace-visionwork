#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from uav_mission.position_settle import PositionSettleWindow  # noqa: E402


class PositionSettleWindowTest(unittest.TestCase):
    def setUp(self):
        self.window = PositionSettleWindow(
            dwell_ns=500_000_000,
            radius_m=0.15,
            max_sample_gap_ns=250_000_000,
        )

    def test_r18_pose_trace_settles_without_instantaneous_velocity_gate(self):
        samples = (
            (75_957_000_000, -1.053337216, 1.054927707, 1.006945133),
            (76_059_000_000, -1.055449486, 1.054373622, 1.019916415),
            (76_159_000_000, -1.052954793, 1.050400734, 1.025365472),
            (76_259_000_000, -1.050285935, 1.046085477, 1.029625177),
            (76_360_000_000, -1.044283628, 1.038095593, 1.031804681),
            (76_459_000_000, -1.037270427, 1.029210925, 1.034561753),
        )
        results = [
            self.window.update(stamp, x, y, z)
            for stamp, x, y, z in samples
        ]
        self.assertFalse(results[-2].ready)
        self.assertTrue(results[-1].ready)
        self.assertEqual(results[-1].reason, "settled")
        self.assertAlmostEqual(results[-1].displacement_m, 0.041014294)

    def test_motion_outside_radius_restarts_full_dwell(self):
        self.window.update(1_000_000_000, 0.0, 0.0, 1.0)
        result = self.window.update(1_200_000_000, 0.2, 0.0, 1.0)
        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "motion_restart")
        self.assertAlmostEqual(result.displacement_m, 0.2)
        self.window.update(1_400_000_000, 0.2, 0.0, 1.0)
        self.assertFalse(
            self.window.update(1_600_000_000, 0.2, 0.0, 1.0).ready)
        self.assertTrue(
            self.window.update(1_700_000_000, 0.2, 0.0, 1.0).ready)

    def test_sample_gap_restarts_full_dwell(self):
        self.window.update(1_000_000_000, 0.0, 0.0, 1.0)
        result = self.window.update(1_300_000_001, 0.0, 0.0, 1.0)
        self.assertEqual(result.reason, "sample_gap_restart")
        self.window.update(1_500_000_000, 0.0, 0.0, 1.0)
        self.assertFalse(
            self.window.update(1_700_000_000, 0.0, 0.0, 1.0).ready)
        self.assertTrue(
            self.window.update(1_800_000_001, 0.0, 0.0, 1.0).ready)

    def test_duplicate_is_idempotent_and_conflict_resets(self):
        first = self.window.update(1_000_000_000, 0.0, 0.0, 1.0)
        self.assertIs(
            self.window.update(1_000_000_000, 0.0, 0.0, 1.0), first)
        conflict = self.window.update(1_000_000_000, 0.01, 0.0, 1.0)
        self.assertEqual(conflict.reason, "sample_stamp_conflict")
        self.assertEqual(self.window.result.elapsed_ns, 0)


if __name__ == "__main__":
    unittest.main()
