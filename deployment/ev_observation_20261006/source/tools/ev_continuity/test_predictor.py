"""Pure numerical regression; no ROS master, simulated vehicle or hardware."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    'patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts'))
from ev_predictor import Predictor, Limits, State, Imu, exp_rot


def state(t=1., **kwargs):
    return replace(State(t, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3),
                         np.zeros(3), np.array([0., 0., -9.81]),
                         np.eye(18)*1e-5, np.ones(12)*1e-4), **kwargs)


def samples(p, start=0.995, end=1.2, acc=(0., 0., 9.81), gyro=(0., 0., 0.), now=None):
    for t in np.arange(start, end+1e-7, 0.005):
        p.add_imu(Imu(float(t), np.array(acc), np.array(gyro)), float(t if now is None else now))


class PredictorTests(unittest.TestCase):
    def test_stationary_gravity_cancels(self):
        p=Predictor(); samples(p)
        self.assertTrue(p.correction(state(), 1.2))
        raw, smooth=p.output(1.2)
        np.testing.assert_allclose(raw.p, 0., atol=1e-10)
        self.assertAlmostEqual(raw.t, 1.2)
        self.assertGreater(raw.cov[0, 0], 1e-5)

    def test_delayed_accelerated_motion_not_low_passed(self):
        p=Predictor(); samples(p, acc=(2., 0., 9.81))
        self.assertTrue(p.correction(state(v=np.array([1., 0., 0.])), 1.2))
        raw, smooth=p.output(1.2)
        self.assertAlmostEqual(raw.p[0], .24, places=8)
        self.assertAlmostEqual(raw.v[0], 1.4, places=8)
        # 100 ms-old correction must first propagate to 1.2, not replace 1.2
        # with the past position 0.11 and then smooth a fictitious 13 cm error.
        s=state(1.1, p=np.array([.11, 0., 0.]), v=np.array([1.2, 0., 0.]))
        self.assertTrue(p.correction(s, 1.2))
        raw, smooth=p.output(1.2)
        np.testing.assert_allclose(smooth.p, [.24, 0., 0.], atol=1e-10)
        np.testing.assert_allclose(p.p_offset, 0., atol=1e-10)

    def test_small_correction_blends_at_same_time(self):
        p=Predictor(); samples(p)
        p.correction(state(), 1.2)
        self.assertTrue(p.correction(state(1.1, p=np.array([0., 0., .05])), 1.2))
        raw, smooth=p.output(1.2)
        self.assertAlmostEqual(raw.p[2], .05)
        self.assertAlmostEqual(smooth.p[2], 0.)
        samples(p, 1.205, 1.3)
        raw, smooth=p.output(1.3)
        self.assertGreater(smooth.p[2], 0.)
        self.assertLess(smooth.p[2], .05)
        self.assertGreater(smooth.cov[2, 2], raw.cov[2, 2])

    def test_large_jump_latches_and_is_not_hidden(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        self.assertFalse(p.correction(state(1.1, p=np.array([0., 0., .5])), 1.2))
        self.assertEqual(p.fault, 'correction_jump_requires_restart')
        self.assertIsNone(p.output(1.2))
        self.assertFalse(p.correction(state(1.15), 1.2))

    def test_attitude_motion_and_small_correction(self):
        p=Predictor(); samples(p, gyro=(0., 0., 1.))
        p.correction(state(), 1.2)
        raw, _=p.output(1.2)
        np.testing.assert_allclose(raw.r, exp_rot(np.array([0., 0., .2])), atol=1e-9)
        s=state(1.1, r=exp_rot(np.array([0., 0., .12])))
        p.correction(s, 1.2)
        _, smooth=p.output(1.2)
        np.testing.assert_allclose(smooth.r, raw.r, atol=1e-9)

    def test_large_attitude_change_rejected(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        self.assertFalse(p.correction(state(1.1, r=exp_rot(np.array([0., 0., 1.]))), 1.2))

    def test_long_radar_loss_despite_fresh_imu(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        samples(p, 1.205, 1.305)
        self.assertIsNone(p.output(1.305))
        self.assertEqual(p.fault, 'correction_timeout_requires_restart')

    def test_imu_loss_stops_not_retimestamps(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        raw, _=p.output(1.25)
        self.assertAlmostEqual(raw.t, 1.2)
        self.assertIsNone(p.output(1.301))

    def test_imu_gap_detected(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        samples(p, 1.25, 1.26)
        self.assertIsNone(p.output(1.26))
        self.assertEqual(p.fault, 'imu_gap')

    def test_duplicates_do_not_integrate_twice(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        s=Imu(1.1, np.array([100., 0., 9.81]), np.zeros(3))
        self.assertFalse(p.add_imu(s, 1.2))
        self.assertFalse(p.correction(state(), 1.2))
        raw,_=p.output(1.2)
        np.testing.assert_allclose(raw.p, 0.)

    def test_epoch_change_cannot_resume_automatically(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        self.assertFalse(p.correction(state(1.1, epoch=2), 1.2))
        self.assertEqual(p.fault, 'state_epoch_or_calibration_changed')

    def test_clock_regression(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        self.assertIsNone(p.output(1.1))
        self.assertEqual(p.fault, 'clock_regression')

    def test_bias_scale_offset_and_tilt(self):
        p=Predictor()
        r=exp_rot(np.array([.05, -.08, 0.]))
        ba=np.array([.2, .1, -.1]); bg=np.array([.01, .02, .03])
        body_acc=(r.T@np.array([0., 0., 9.81])+ba)/9.81
        samples(p, start=.98, end=1.18, acc=body_acc, gyro=bg, now=1.2)
        p.correction(state(r=r, ba=ba, bg=bg, scale=9.81, offset=.02), 1.2)
        raw,_=p.output(1.2)
        np.testing.assert_allclose(raw.p, 0., atol=1e-9)
        self.assertAlmostEqual(raw.t, 1.2)

    def test_negative_covariance_invalid(self):
        p=Predictor(); samples(p)
        self.assertFalse(p.correction(state(cov=-np.eye(18)), 1.2))

    def test_missing_boundary_or_wrong_frame(self):
        p=Predictor(); samples(p, start=1.01)
        self.assertFalse(p.correction(state(), 1.2))
        self.assertEqual(p.reason, 'missing_imu_boundary')
        q=Predictor(); samples(q)
        self.assertFalse(q.correction(state(imu_frame='different'), 1.2))

    def test_bounded_cache(self):
        p=Predictor(Limits(buffer_samples=10)); samples(p)
        self.assertLessEqual(len(p.imu), 10)
        self.assertFalse(p.correction(state(), 1.2))

    def test_independent_subscription_order_waits_for_imu(self):
        p=Predictor(); samples(p, end=1.01); p.correction(state(), 1.01)
        self.assertFalse(p.correction(state(1.02), 1.025))
        self.assertIsNone(p.fault)
        self.assertIsNotNone(p.pending)
        samples(p, 1.015, 1.025, now=1.03)
        self.assertIsNone(p.pending)
        self.assertAlmostEqual(p.correction_t, 1.02)
        self.assertIsNotNone(p.output(1.03))

    def test_smoothing_cannot_mask_opposite_raw_jump(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        self.assertTrue(p.correction(state(1.1, p=np.array([.15, 0., 0.])), 1.2))
        self.assertFalse(p.correction(state(1.15, p=np.array([-.15, 0., 0.])), 1.2))
        self.assertEqual(p.fault, 'correction_jump_requires_restart')

    def test_angular_smoothing_cannot_mask_opposite_raw_jump(self):
        p=Predictor(); samples(p); p.correction(state(), 1.2)
        angle=np.deg2rad(9)
        self.assertTrue(p.correction(state(1.1, r=exp_rot(np.array([0., 0., angle]))), 1.2))
        raw, smooth=p.output(1.2)
        self.assertGreater(smooth.cov[5, 5], raw.cov[5, 5]+.01)
        self.assertFalse(p.correction(state(1.15, r=exp_rot(np.array([0., 0., -angle]))), 1.2))


if __name__ == '__main__':
    unittest.main()
