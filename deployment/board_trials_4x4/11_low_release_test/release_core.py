"""临时无靶单投：继承低空观察，真实服务只允许一次尝试。此文件不依赖 ROS。"""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

LOW = Path(__file__).resolve().parents[2] / 'low_hover_observation'
sys.path.insert(0, str(LOW))
from observation_core import Observation, route, validate


def validate_trial(c):
    validate(c, 'forward')
    for key in ('descent_agl', 'release_below_agl',
                'descent_speed', 'service_start_timeout', 'release_timeout'):
        value = c[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('positive finite value required: ' + key)
    if not (c['fc_ground_clearance'] < c['descent_agl']
            < c['release_below_agl'] < c['height_agl']):
        raise ValueError('require ground < descent < strict_release_limit < takeoff')
    if c['descent_speed'] > .3:
        raise ValueError('descent exceeds low observation speed')
    points = c['profiles']['forward']
    if len(points) != 1 or any(isinstance(v, bool) for v in points[0]) or points[0][0] <= 0 or points[0][1] != 0:
        raise ValueError('exactly one positive initial-heading forward offset required')
    if c['hover_seconds'] != c['waypoint_seconds']:
        raise ValueError('keep hover_seconds and waypoint_seconds equal')
    value = c['raw_service']
    if not isinstance(value, str) or not value.startswith('/') or any(ch.isspace() for ch in value):
        raise ValueError('absolute raw_service ROS name required')
    if value in ('/Servo', '/mission/servo_action', '/SetServoDuty'):
        raise ValueError('raw_service must be the existing raw Servo backend')


class OnceRelease:
    """先持久锁存再异步调用；失败/超时/取消均消耗本次机会，绝不重发。

    原始 Servo 只有 bool ACK，没有 request_id 去重。真实模式在本轮 run 目录
    保留 attempted 文件，拒绝同run复投；新一轮不锁槽，不自动重发旧轮请求。
    磁盘及服务调用全在工作线程，不阻塞30Hz设定点。
    """
    def __init__(self, slot, real, call, timeout, marker=None, clock=time.monotonic):
        if type(slot) is not int or slot not in (1, 2, 3):
            raise ValueError('slot must be 1/2/3')
        if real and marker is None:
            raise ValueError('persistent attempt marker required for real release')
        self.slot, self.real, self.call = slot, real, call
        self.timeout, self.marker, self.clock = timeout, marker, clock
        self.lock = threading.Lock()
        self.state, self.detail = 'NOT_ATTEMPTED', ''
        self.started = None
        self.thread = None

    def snapshot(self):
        with self.lock:
            return dict(release_mode='REAL' if self.real else 'MOCK', release_slot=self.slot,
                        release_state=self.state, release_detail=self.detail)

    def _finish(self, state, detail):
        with self.lock:
            # 超时后迟到ACK不能把不确定改成成功，也不能重新调度。
            if self.state in ('PENDING', 'CALL_STARTED'):
                self.state, self.detail = state, detail

    def poll(self):
        with self.lock:
            if self.state in ('PENDING', 'CALL_STARTED') and self.clock()-self.started >= self.timeout:
                self.state, self.detail = 'UNKNOWN', 'timeout_no_retry'
        return self.snapshot()['release_state']

    def request(self, live_check):
        with self.lock:
            if self.state != 'NOT_ATTEMPTED':
                return False
            self.state, self.started = 'PENDING', self.clock()
        self.thread = threading.Thread(target=self._run, args=(live_check,), daemon=True)
        self.thread.start()
        return True

    def _run(self, live_check):
        try:
            if self.real:
                # 排他创建且 fsync 成功才允许触达 raw；已有锁存绝不覆盖。
                fd = os.open(str(self.marker), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                    json.dump(dict(slot=self.slot, status='ATTEMPT_RESERVED_NEVER_AUTO_RETRY',
                                   wall_time=time.time()), handle)
                    handle.flush()
                    os.fsync(handle.fileno())
                directory_fd = os.open(str(Path(self.marker).parent), os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            # 写盘之后重新检查最新遥测，不能用排队前的旧许可。
            if not live_check():
                self._finish('CANCELLED', 'dispatch_gate_closed_no_retry')
                return
            with self.lock:
                if self.state != 'PENDING':
                    return
                if self.clock()-self.started >= self.timeout:
                    self.state, self.detail = 'UNKNOWN', 'dispatch_timeout_no_retry'
                    return
                self.state = 'CALL_STARTED'
            if not self.real:
                self._finish('MOCK_COMPLETE', 'no_servo_service_called')
                return
            ok = self.call(self.slot)
            self._finish('PWM_ACKNOWLEDGED' if ok else 'UNKNOWN',
                         'electrical_ack_only' if ok else 'negative_ack_effect_unknown_no_retry')
        except Exception as error:
            self._finish('UNKNOWN', type(error).__name__ + ': ' + str(error))


class LowReleaseObservation(Observation):
    def __init__(self, config, profile, origin, now, release, live_check=None):
        validate_trial(config)
        self.release = release
        self.live_check = live_check
        self.latest_gate = False
        self.route_complete = False
        super().__init__(config, profile, origin, now)

    def make_route(self, origin):
        # 完整复用原 forward 机头方向变换；最后只追加同XY的低位目标。
        points = route(self.c, self.profile, origin)
        x, y, _, yaw = points[-1]
        points.append((x, y, origin[2]-self.c['fc_ground_clearance']+self.c['descent_agl'], yaw))
        return points

    def leg_speed(self):
        return self.c['descent_speed'] if self.index == 2 else super().leg_speed()

    def leg_duration(self):
        # 每段仍须通过父类连续到位稳定窗口；仅前移后额外悬停5秒。
        return self.c['hover_seconds'] if self.index == 1 else 0.

    def release_gate(self, pose, fresh, armed, mode, connected, speed, stop=False):
        if (self.stage not in ('RUN', 'FINISHED_HOVER') or not self.ever_started
                or self.index != 2 or not fresh or not armed or not connected
                or mode != 'OFFBOARD' or stop or pose is None or not math.isfinite(speed)):
            return False
        if not all(math.isfinite(v) for v in pose):
            return False
        agl = pose[2]-self.origin[2]+self.c['fc_ground_clearance']
        # 前两阶段已由父类完成；低位只检查新鲜高度，绝无XYZ到位/低速/停稳门槛。
        return agl < self.c['release_below_agl']

    def step(self, now, pose, stamp, fresh, armed, mode, connected=True, speed=0., stop_requested=False):
        # 父类先处理人工接管、陈旧消息、跳变、限幅、超时与全部动作阶段。
        target = super().step(now, pose, stamp, fresh, armed, mode, connected, speed, stop_requested)
        if self.stage == 'FINISHED_HOVER' and self.index == 2:
            self.route_complete = True
        self.latest_gate = self.release_gate(pose, fresh, armed, mode, connected, speed, stop_requested)
        if self.latest_gate and self.release.poll() == 'NOT_ATTEMPTED':
            self.release.request(self.live_check or (lambda: self.latest_gate))
        outcome = self.release.poll()
        if self.stage == 'RUN':
            self.reason = ('ascending_to_fc_agl', 'forward_then_hover', 'descending_to_release_agl')[self.index]
        elif self.stage == 'FINISHED_HOVER':
            self.reason = 'low_release_' + outcome.lower() + '_hover_for_pilot'
        return target

    def status_details(self):
        return dict(self.release.snapshot(), route_complete=self.route_complete,
                    release_gate_open=self.latest_gate, coordinate_basis='initial_heading_forward')
