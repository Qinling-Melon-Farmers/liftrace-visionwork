#!/usr/bin/env python3
"""Bounded IMU prediction with delayed LIO replay. Shadow experiments only.

No FC feedback, ROS clock retiming, extrapolation past the newest IMU, or
low-pass filtering of physical motion. The smoothing offset is only the
same-time difference when a new LIO correction replaces the prediction.
"""
from collections import deque
from dataclasses import dataclass, replace
import math

import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_rot(v):
    theta = float(np.linalg.norm(v))
    k = skew(v)
    if theta < 1e-8:
        return np.eye(3) + k + 0.5 * k @ k
    return np.eye(3) + math.sin(theta) / theta * k + (1-math.cos(theta))/theta**2 * k @ k


def log_rot(r):
    theta = math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1)))
    # Large corrections are rejected before this function is used.
    v = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    return 0.5*v if theta < 1e-8 else theta/(2*math.sin(theta))*v


@dataclass(frozen=True)
class Limits:
    imu_gap: float = 0.025
    output_age: float = 0.10
    correction_age: float = 0.30
    future_tolerance: float = 0.005
    buffer_seconds: float = 0.8
    buffer_samples: int = 1000
    correction_position: float = 0.20
    correction_angle: float = math.radians(10)
    smoothing_tau: float = 0.15
    position_sigma: float = 0.30

    def __post_init__(self):
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in vars(self).values()):
            raise ValueError('all limits must be finite and positive')
        if self.buffer_seconds <= self.correction_age or self.buffer_samples < 3:
            raise ValueError('IMU cache must cover the permitted correction interval')


@dataclass(frozen=True)
class Imu:
    t: float
    acc: np.ndarray
    gyro: np.ndarray
    frame: str = 'livox_frame'


@dataclass(frozen=True)
class State:
    t: float
    p: np.ndarray
    r: np.ndarray
    v: np.ndarray
    bg: np.ndarray
    ba: np.ndarray
    g: np.ndarray
    cov: np.ndarray
    noise: np.ndarray
    scale: float = 1.0
    offset: float = 0.0
    epoch: int = 1
    frame: str = 'camera_init'
    imu_frame: str = 'livox_frame'


def propagate(s, a, b):
    """Midpoint strapdown integration, right attitude-error covariance."""
    dt = b.t - a.t
    w = (a.gyro+b.gyro)*0.5-s.bg
    acc = (a.acc+b.acc)*0.5*s.scale-s.ba
    r_mid = s.r @ exp_rot(w*dt*0.5)
    world_acc = r_mid @ acc+s.g
    f = np.zeros((18, 18))
    f[0:3, 6:9] = np.eye(3)
    f[3:6, 3:6] = -skew(w)
    f[3:6, 9:12] = -np.eye(3)
    f[6:9, 3:6] = -r_mid @ skew(acc)
    f[6:9, 12:15] = -r_mid
    f[6:9, 15:18] = np.eye(3)
    a_mat = np.eye(18)+dt*f+0.5*dt*dt*(f@f)
    g_mat = np.zeros((18, 12))
    g_mat[3:6, 0:3] = -np.eye(3)
    g_mat[6:9, 3:6] = -r_mid
    g_mat[9:12, 6:9] = np.eye(3)
    g_mat[12:15, 9:12] = np.eye(3)
    # FAST-LIO's Q is per-sample variance (G*dt Q G*dt^T), not a density.
    cov = a_mat @ s.cov @ a_mat.T + dt*dt*(g_mat*s.noise) @ g_mat.T
    return replace(s, t=b.t, p=s.p+s.v*dt+0.5*world_acc*dt*dt,
                   v=s.v+world_acc*dt, r=s.r @ exp_rot(w*dt),
                   cov=(cov+cov.T)*0.5)


class Predictor:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.imu = deque()
        self.state = None
        self.pending = None
        self.correction_t = None
        self.last_now = None
        self.fault = None
        self.reason = 'waiting_for_state'
        self.p_offset = np.zeros(3)
        self.r_offset = np.zeros(3)
        self.offset_t = 0.
        self.stats = dict(imu_rejected=0, corrections_rejected=0, corrections=0, outputs=0)

    def fail(self, reason):
        # Explicit restart required after discontinuity. No silent auto-recovery.
        self.fault = self.reason = reason
        return False

    def clock(self, now):
        if not math.isfinite(now):
            return self.fail('invalid_clock')
        if self.last_now is not None and now < self.last_now-1e-6:
            return self.fail('clock_regression')
        self.last_now = now
        return not self.fault

    def add_imu(self, sample, now):
        if not self.clock(now):
            return False
        if not all(np.all(np.isfinite(v)) for v in (sample.t, sample.acc, sample.gyro)):
            return self.fail('invalid_imu')
        if np.shape(sample.acc) != (3,) or np.shape(sample.gyro) != (3,):
            return self.fail('invalid_imu_shape')
        if self.imu and sample.t <= self.imu[-1].t:
            self.stats['imu_rejected'] += 1
            # Duplicate/late transport packets are ignored, not integrated twice.
            self.reason = 'nonmonotonic_imu_rejected'
            return False
        self.imu.append(sample)
        while len(self.imu) > self.limits.buffer_samples or (
                len(self.imu) > 2 and self.imu[-1].t-self.imu[1].t > self.limits.buffer_seconds):
            self.imu.popleft()
        if self.pending is not None and sample.t+self.pending.offset >= self.pending.t:
            self.correction(self.pending, now)
        return True

    def replay(self, s):
        samples = [replace(x, t=x.t+s.offset) for x in self.imu]
        if not samples or samples[-1].t < s.t:
            raise ValueError('waiting_for_imu_coverage')
        left = None
        for right in samples:
            if right.frame != s.imu_frame:
                raise ValueError('imu_frame_mismatch')
            if right.t <= s.t:
                left = right
                continue
            if left is None:
                raise ValueError('missing_imu_boundary')
            if right.t-left.t > self.limits.imu_gap:
                raise ValueError('imu_gap')
            if left.t < s.t:
                alpha = (s.t-left.t)/(right.t-left.t)
                left = Imu(s.t, left.acc+(right.acc-left.acc)*alpha,
                           left.gyro+(right.gyro-left.gyro)*alpha, left.frame)
            s = propagate(s, left, right)
            left = right
        return s

    def correction(self, s, now):
        if not self.clock(now):
            return False
        values = (s.t, s.p, s.r, s.v, s.bg, s.ba, s.g, s.cov, s.noise, s.scale, s.offset)
        if not all(np.all(np.isfinite(x)) for x in values) or s.scale <= 0:
            return self.fail('invalid_state')
        if (any(np.shape(x) != (3,) for x in (s.p, s.v, s.bg, s.ba, s.g)) or
                s.r.shape != (3, 3) or s.cov.shape != (18, 18) or s.noise.shape != (12,)):
            return self.fail('invalid_state_shape')
        if (not np.allclose(s.r.T@s.r, np.eye(3), atol=1e-6) or
                abs(np.linalg.det(s.r)-1) > 1e-6 or np.any(s.noise < 0) or
                not np.allclose(s.cov, s.cov.T, atol=1e-8) or
                np.linalg.eigvalsh(s.cov).min() < -1e-8):
            return self.fail('invalid_state_covariance_or_rotation')
        if self.state and (s.epoch, s.frame, s.imu_frame, s.offset, s.scale) != (
                self.state.epoch, self.state.frame, self.state.imu_frame, self.state.offset, self.state.scale):
            return self.fail('state_epoch_or_calibration_changed')
        if self.correction_t is not None and s.t <= self.correction_t:
            self.stats['corrections_rejected'] += 1
            self.reason = 'nonmonotonic_correction_rejected'
            return False
        if self.pending is not None and s.t < self.pending.t:
            self.stats['corrections_rejected'] += 1
            self.reason = 'older_than_pending_correction'
            return False
        if not -self.limits.future_tolerance <= now-s.t <= self.limits.correction_age:
            self.stats['corrections_rejected'] += 1
            self.reason = 'stale_or_future_correction'
            return False
        if self.state and now-self.correction_t > self.limits.correction_age:
            return self.fail('correction_timeout_requires_restart')
        try:
            new = self.replay(s)
            if not -self.limits.future_tolerance <= now-new.t <= self.limits.output_age:
                raise ValueError('stale_or_future_imu')
            if self.state:
                old = self.replay(self.state)
                if abs(old.t-new.t) > 1e-7:
                    raise ValueError('different_comparison_time')
                decay = math.exp(-(old.t-self.offset_t)/self.limits.smoothing_tau)
                dp = old.p+self.p_offset*decay-new.p
                dr = exp_rot(self.r_offset*decay) @ old.r @ new.r.T
                angle = math.acos(float(np.clip((np.trace(dr)-1)/2, -1, 1)))
                raw_angle = math.acos(float(np.clip((np.trace(old.r@new.r.T)-1)/2, -1, 1)))
                if (np.linalg.norm(old.p-new.p) > self.limits.correction_position or
                        raw_angle > self.limits.correction_angle or
                        np.linalg.norm(dp) > self.limits.correction_position or angle > self.limits.correction_angle):
                    return self.fail('correction_jump_requires_restart')
                self.p_offset, self.r_offset = dp, log_rot(dr)
            self.state, self.correction_t, self.offset_t = new, s.t, new.t
            self.pending = None
            self.stats['corrections'] += 1
            self.reason = 'tracking'
            return True
        except ValueError as exc:
            if str(exc) == 'waiting_for_imu_coverage':
                # Independent subscriptions may deliver a LIO snapshot before
                # its last IMU packet. Keep only the newest snapshot, bounded by
                # the SAME correction-age deadline; never extrapolate to it.
                self.pending = s
                self.reason = str(exc)
                return False
            self.stats['corrections_rejected'] += 1
            if self.state:
                return self.fail(str(exc))
            self.reason = str(exc)
            return False

    def output(self, now):
        if not self.clock(now) or self.state is None:
            return None
        if now-self.correction_t > self.limits.correction_age:
            self.fail('correction_timeout_requires_restart')
            return None
        try:
            new = self.replay(self.state)
            if not -self.limits.future_tolerance <= now-new.t <= self.limits.output_age:
                raise ValueError('stale_or_future_imu')
            decay = math.exp(-(new.t-self.offset_t)/self.limits.smoothing_tau)
            offset = self.p_offset*decay
            # This conservative proxy includes smoothing displacement, but does
            # not model correlation between repeated IMU predictions sent to EKF.
            cov = new.cov.copy()
            bias = np.zeros(18)
            bias[:3] = offset
            bias[3:6] = new.r.T @ (self.r_offset*decay)
            cov += np.outer(bias, bias)
            if np.linalg.eigvalsh(cov[:3, :3]).max() > self.limits.position_sigma**2:
                raise ValueError('position_uncertainty_limit')
            smooth = replace(new, p=new.p+offset, r=exp_rot(self.r_offset*decay)@new.r, cov=cov)
            self.state = new
            self.stats['outputs'] += 1
            self.reason = 'tracking'
            # Only pose is exported; smoothed velocity would need the derivative
            # of the decaying offset, not the unmodified inertial velocity.
            return new, smooth
        except ValueError as exc:
            self.fail(str(exc))
            return None
