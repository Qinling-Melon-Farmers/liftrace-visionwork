"""Candidate FC/task boundary: one transform for feedback, geometry and commands.

Pure production component; no ROS or hardware access. An explicit, authoritative
old->new FC reset and independently healthy LIO are required to remap coordinates.
Residuals alone only inhibit motion. Calibration and the task ground are immutable.
"""
from dataclasses import dataclass
import math


class BoundaryRejected(ValueError):
    pass


def vector(values, size):
    result = tuple(float(x) for x in values)
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise BoundaryRejected('invalid_vector')
    return result


def quaternion(values):
    q = vector(values, 4)
    n = math.sqrt(sum(x*x for x in q))
    if abs(n-1.0) > 0.001:
        raise BoundaryRejected('nonunit_quaternion')
    return tuple(x/n for x in q)


def multiply(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)


def rotate(q, p):
    v = multiply(multiply(q, (*p, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return v[:3]


@dataclass(frozen=True)
class Transform:
    xyz: tuple
    xyzw: tuple

    def __post_init__(self):
        object.__setattr__(self, 'xyz', vector(self.xyz, 3))
        object.__setattr__(self, 'xyzw', quaternion(self.xyzw))

    def inverse(self):
        q = (-self.xyzw[0], -self.xyzw[1], -self.xyzw[2], self.xyzw[3])
        return Transform(rotate(q, tuple(-x for x in self.xyz)), q)

    def compose(self, other):
        r = rotate(self.xyzw, other.xyz)
        return Transform(tuple(a+b for a, b in zip(self.xyz, r)),
                         multiply(self.xyzw, other.xyzw))


@dataclass(frozen=True)
class Pose:
    stamp: float
    frame: str
    xyz: tuple
    xyzw: tuple

    def __post_init__(self):
        if not math.isfinite(self.stamp) or self.stamp <= 0 or not self.frame:
            raise BoundaryRejected('invalid_pose_header')
        object.__setattr__(self, 'xyz', vector(self.xyz, 3))
        object.__setattr__(self, 'xyzw', quaternion(self.xyzw))

    def transformed(self, transform, frame):
        result = transform.compose(Transform(self.xyz, self.xyzw))
        return Pose(self.stamp, frame, result.xyz, result.xyzw)


@dataclass(frozen=True)
class LioHealth:
    stamp: float
    epoch: str
    frame: str
    healthy: bool
    body_calibrated: bool


@dataclass(frozen=True)
class FcState:
    stamp: float
    connected: bool
    armed: bool
    mode: str


@dataclass(frozen=True)
class FcReset:
    stamp: float
    epoch: str
    previous_counter: int
    counter: int
    frame: str
    new_from_previous: Transform
    authoritative: bool


@dataclass(frozen=True)
class Limits:
    max_age: float = 0.20
    future_tolerance: float = 0.02
    sync_slop: float = 0.04
    reset_max_age: float = 0.20
    residual_position: float = 0.08
    residual_angle_rad: float = math.radians(3.0)
    sample_count: int = 3
    sample_span: float = 0.10
    sample_gap: float = 0.20

    def __post_init__(self):
        for name in ('max_age', 'sync_slop', 'reset_max_age', 'residual_position',
                     'residual_angle_rad', 'sample_span', 'sample_gap'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise BoundaryRejected('invalid_limit_'+name)
        if (type(self.sample_count) is not int or self.sample_count < 3 or self.future_tolerance < 0 or
                not math.isfinite(self.future_tolerance)):
            raise BoundaryRejected('invalid_window_limits')


class TaskFrameBoundary:
    def __init__(self, *, task_frame, fc_frame, lio_frame, fc_epoch, lio_epoch,
                 task_from_fc, task_from_lio, body_to_camera, ground_z,
                 calibration_verified=False, initial_reset_counter=None,
                 limits=Limits()):
        if (not all((task_frame, fc_frame, lio_frame, fc_epoch, lio_epoch)) or
                task_frame == fc_frame or not math.isfinite(ground_z)):
            raise BoundaryRejected('invalid_reference')
        if (initial_reset_counter is None or isinstance(initial_reset_counter, bool)
                or not isinstance(initial_reset_counter, int) or initial_reset_counter < 0):
            raise BoundaryRejected('explicit_initial_reset_counter_required')
        self.task_frame, self.fc_frame, self.lio_frame = task_frame, fc_frame, lio_frame
        self.fc_epoch, self.lio_epoch = fc_epoch, lio_epoch
        self.task_from_fc, self.task_from_lio = task_from_fc, task_from_lio
        self.body_to_camera, self.ground_z = body_to_camera, float(ground_z)
        if type(calibration_verified) is not bool:
            raise BoundaryRejected('calibration_verified_must_be_boolean')
        self.calibration_verified, self.limits = calibration_verified, limits
        self.reset_counter, self.generation = initial_reset_counter, 0
        self.effective_stamp = 0.0
        self.ready, self.initialized, self.fault = False, False, ''
        self.reason = 'awaiting_calibrated_samples'
        self.samples = []
        self.fc, self.lio, self.lio_health, self.state = None, None, None, None
        self.task_pose, self.hold_task_pose = None, None
        self.last_pair = None
        self.reset_pending = False
        self.last_residual = None
        self.last_reset = None
        self.post_reset_pair_valid = False

    def _block(self, reason, latch=False):
        self.ready, self.reason, self.samples = False, reason, []
        if latch:
            self.fault = reason
        return False

    def _age_ok(self, stamp, now, maximum=None):
        return (math.isfinite(stamp) and stamp > 0 and math.isfinite(now) and
                -self.limits.future_tolerance <= now-stamp <=
                (self.limits.max_age if maximum is None else maximum))

    def update_state(self, state):
        self.state = state
        if not state.connected:
            self._block('fc_disconnected_requires_ground_reinitialization', self.initialized)
        elif state.armed and state.mode not in ('OFFBOARD', 'AUTO.LAND'):
            self._block('manual_takeover_requires_ground_reinitialization', True)

    def _inputs_fresh(self, now):
        if self.fault:
            return self._block(self.fault)
        if not self.calibration_verified:
            return self._block('calibration_not_verified')
        if self.state is None or not self._age_ok(self.state.stamp, now):
            return self._block('fc_state_stale')
        if not self.state.connected:
            return self._block('fc_disconnected')
        if self.fc is None or self.lio is None or self.lio_health is None:
            return self._block('inputs_missing')
        if not all(self._age_ok(x.stamp, now) for x in (self.fc, self.lio, self.lio_health)):
            return self._block('pose_or_lio_health_stale')
        return True

    def observe(self, fc, lio, health, state, now):
        self.update_state(state)
        if self.fault:
            return False
        if (fc.frame != self.fc_frame or lio.frame != self.lio_frame or
                health.frame != self.lio_frame):
            return self._block('source_frame_changed', True)
        if health.epoch != self.lio_epoch:
            return self._block('lio_epoch_changed_requires_ground_reinitialization', True)
        if not health.healthy or not health.body_calibrated:
            return self._block('lio_unhealthy_or_uncalibrated', True)
        pair = (fc.stamp, lio.stamp)
        if self.last_pair is not None and any(a < b for a, b in zip(pair, self.last_pair)):
            return self._block('source_time_regression', True)
        self.fc, self.lio, self.lio_health = fc, lio, health
        if not self._inputs_fresh(now):
            return False
        if (abs(fc.stamp-lio.stamp) > self.limits.sync_slop or
                abs(health.stamp-lio.stamp) > self.limits.sync_slop):
            return self._block('sources_not_synchronized')
        # Do not evaluate pre-reset measurements with a post-reset transform.
        if fc.stamp < self.effective_stamp:
            return self._block('awaiting_post_reset_fc_sample')
        task_fc = fc.transformed(self.task_from_fc, self.task_frame)
        task_lio = lio.transformed(self.task_from_lio, self.task_frame)
        position_error = math.sqrt(sum((a-b)**2 for a, b in zip(task_fc.xyz, task_lio.xyz)))
        angle_error = 2*math.acos(min(1.0, abs(sum(a*b for a, b in zip(task_fc.xyzw, task_lio.xyzw)))))
        self.last_residual = (position_error, angle_error)
        if (position_error > self.limits.residual_position or
                angle_error > self.limits.residual_angle_rad):
            # Missing FC reset metadata and a LIO jump cannot be distinguished
            # from a residual alone. Keep the old reference, inhibit all motion.
            return self._block('unexplained_fc_lio_disagreement')
        if not self.initialized and state.armed:
            return self._block('initialization_requires_disarmed')
        if self.last_pair is not None and any(a == b for a, b in zip(pair, self.last_pair)):
            return self.ready
        self.last_pair = pair
        t = min(pair)
        if self.samples and t-self.samples[-1] > self.limits.sample_gap:
            self.samples = []
            self.ready = False
        self.samples.append(t)
        # Keep a bounded window without erasing its original start prematurely.
        self.samples = self.samples[-max(self.limits.sample_count, 100):]
        self.task_pose = task_fc
        if self.reset_pending:
            self.post_reset_pair_valid = True
        self.ready = (len(self.samples) >= self.limits.sample_count and
                      self.samples[-1]-self.samples[0] >= self.limits.sample_span)
        if self.ready:
            self.initialized = True
            self.reset_pending = False
            self.hold_task_pose = None
            self.reason = 'ready'
        else:
            self.reason = 'verifying_post_reset' if self.reset_pending else 'verifying_sources'
        return self.ready

    def apply_reset(self, event, now):
        if self.fault:
            return False
        if not self.initialized or not self.calibration_verified:
            return self._block('reset_before_reference_initialization', True)
        if (not event.authoritative or event.epoch != self.fc_epoch or
                event.frame != self.fc_frame):
            return self._block('reset_source_or_epoch_unverified', True)
        if (isinstance(event.counter, bool) or isinstance(event.previous_counter, bool) or
                not isinstance(event.counter, int) or not isinstance(event.previous_counter, int)):
            return self._block('invalid_reset_counter', True)
        if event.counter == self.reset_counter and event.previous_counter == self.reset_counter-1:
            # Replays never compose the delta twice or restart the ready window.
            if event != self.last_reset:
                return self._block('conflicting_reset_replay', True)
            return False
        if (event.previous_counter != self.reset_counter or
                event.counter != self.reset_counter+1):
            return self._block('reset_counter_gap_requires_ground_reinitialization', True)
        if (not self._age_ok(event.stamp, now, self.limits.reset_max_age) or
                event.stamp <= self.effective_stamp):
            return self._block('reset_event_stale_or_out_of_order', True)
        if (self.lio_health is None or self.lio is None or self.state is None or
                not all(self._age_ok(x.stamp, now) for x in
                        (self.lio_health, self.lio, self.state)) or
                self.lio_health.epoch != self.lio_epoch or not self.lio_health.healthy):
            return self._block('reset_without_healthy_independent_lio', True)
        # p_new = T_new_old * p_old; T_task_new = T_task_old * inv(T_new_old).
        self.task_from_fc = self.task_from_fc.compose(event.new_from_previous.inverse())
        self.reset_counter, self.effective_stamp = event.counter, event.stamp
        self.generation += 1
        self.hold_task_pose = self.task_pose
        self.reset_pending = True
        self.last_reset = event
        self.post_reset_pair_valid = False
        self._block('verifying_post_reset')
        return True

    def snapshot(self, now):
        if not self._inputs_fresh(now) or not self.ready or self.task_pose is None:
            raise BoundaryRejected(self.reason)
        value = Transform(self.task_pose.xyz, self.task_pose.xyzw).compose(self.body_to_camera)
        camera = Pose(self.task_pose.stamp, self.task_frame, value.xyz, value.xyzw)
        return {'generation': self.generation, 'reset_counter': self.reset_counter,
                'body': self.task_pose, 'camera': camera, 'ground_z': self.ground_z,
                'body_agl': self.task_pose.xyz[2]-self.ground_z,
                'camera_agl': camera.xyz[2]-self.ground_z}

    def command(self, task_setpoint, now):
        if task_setpoint.frame != self.task_frame or not self._age_ok(task_setpoint.stamp, now):
            raise BoundaryRejected('setpoint_frame_or_age_invalid')
        if not self._inputs_fresh(now) or not self.ready:
            raise BoundaryRejected('hold_required:'+self.reason)
        if self.state.mode != 'OFFBOARD' and self.state.armed:
            raise BoundaryRejected('fc_not_offboard')
        return task_setpoint.transformed(self.task_from_fc.inverse(), self.fc_frame)

    def hold_request(self, now):
        """No guessed local hold when coordinates are untrusted.

After an authoritative reset, a revalidated pair can express the previously
trusted physical hold point in the new FC frame while the sample window fills.
This is a request for the final controller, never a mode/arming instruction.
"""
        if (self.reset_pending and self.post_reset_pair_valid and not self.fault
                and self.hold_task_pose is not None
                and self.reason == 'verifying_post_reset' and self._inputs_fresh(now)
                and self.state.armed and self.state.mode == 'OFFBOARD'):
            hold = Pose(now, self.task_frame, self.hold_task_pose.xyz, self.hold_task_pose.xyzw)
            return hold.transformed(self.task_from_fc.inverse(), self.fc_frame)
        return None

    def release_allowed(self, *, permission_stamp, valid_until, now, generation,
                        evidence_stamp=None):
        evidence_stamp = permission_stamp if evidence_stamp is None else evidence_stamp
        return bool(self._inputs_fresh(now) and self.ready and self.state.armed and
                    self.state.mode == 'OFFBOARD' and generation == self.generation and
                    self._age_ok(permission_stamp, now) and
                    permission_stamp >= self.effective_stamp and
                    self._age_ok(evidence_stamp, now) and evidence_stamp >= self.effective_stamp and
                    math.isfinite(valid_until) and valid_until >= now)

    def project_pixel(self, u, v, intrinsics, now):
        """Ground-plane projection uses exactly the same body/ground snapshot."""
        camera = self.snapshot(now)['camera']
        fx, fy, cx, cy = vector(intrinsics, 4)
        if fx <= 0 or fy <= 0 or not all(math.isfinite(x) for x in (u, v)):
            raise BoundaryRejected('invalid_projection_input')
        ray = rotate(camera.xyzw, ((u-cx)/fx, (v-cy)/fy, 1.))
        if ray[2] >= -1e-6:
            raise BoundaryRejected('camera_ray_does_not_intersect_ground')
        distance = (self.ground_z-camera.xyz[2])/ray[2]
        if distance <= 0:
            raise BoundaryRejected('camera_below_ground')
        return tuple(a+distance*b for a, b in zip(camera.xyz, ray))
