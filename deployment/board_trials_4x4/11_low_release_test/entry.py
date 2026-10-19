#!/usr/bin/env python3
"""独立临时低位释放入口；preview/check-config 完全离线，flight 默认机构mock。"""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import sys
import time
import yaml

from release_core import LOW, LowReleaseObservation, OnceRelease, validate_trial
import flight as low_flight
from observation_core import angle_delta

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


class RunLock:
    """只互斥本临时模块的多个入口；不持有/争抢驱动的PWM硬件锁。"""
    def __init__(self, path):
        self.fd = os.open(str(path), os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(self.fd)
            raise RuntimeError('another low_release_test entry is already active')

    def close(self):
        os.close(self.fd)


class ExistingRawServo:
    """复用5b已运行的raw服务。只检查类型/创建客户端，不启动或初始化驱动。"""
    def __init__(self, c):
        import rospy
        import rosservice
        from patrol_control.srv import Servo
        self.name = c['raw_service']
        self.get_type = rosservice.get_service_type
        self.expected_type = Servo._type
        rospy.wait_for_service(self.name, timeout=c['service_start_timeout'])
        self.check_type()
        self.client = rospy.ServiceProxy(self.name, Servo, persistent=False)

    def check_type(self):
        if self.get_type(self.name) != self.expected_type:
            raise RuntimeError('expected patrol_control/Servo at ' + self.name)

    def call(self, slot):
        # 不用 /Servo 代理、不伪造正常任务许可；临时授权只来自本轮阶段/高度门控。
        # 一次call后无论异常/false/超时均不重发；bool只确认电气操作。
        return bool(self.client(req=slot).res)


def live_release_check(controller, c, data, lock, stopping, ros_now, monotonic=time.monotonic):
    """工作线程提交前重读最新消息，防止写盘/调度期间消息过期或飞手接管。"""
    snapshot, wall, now = low_flight.snapshot_clocked(lock, data, monotonic, ros_now)
    state, state_rx = snapshot.get('state', (None, 0.))
    pose_msg, pose_rx = snapshot.get('pose', (None, 0.))
    ev_msg, ev_rx = snapshot.get('ev', (None, 0.))
    odom, odom_rx = snapshot.get('odom', (None, 0.))
    if (stopping.is_set() or state is None or not 0 <= wall-state_rx <= c['state_max_age']
            or not low_flight.valid_pose(pose_msg, c['frame'], now, c['pose_max_age'], wall-pose_rx)
            or not low_flight.valid_pose(ev_msg, c['mission_frame'], now, c['pose_max_age'], wall-ev_rx)):
        return False
    pose = low_flight.xyz_yaw(pose_msg)
    speed = low_flight.valid_speed(odom, c['frame'], now, c['pose_max_age'], wall-odom_rx)
    # 仅复核父类已有的全局范围/跳变保护，不新增低位到位或速度门槛。
    if (math.dist(pose[:2], controller.origin[:2]) > c['max_distance_from_start']
            or pose[2]-controller.origin[2]+c['fc_ground_clearance'] > c['max_height_agl']):
        return False
    stamp = pose_msg.header.stamp.to_sec()
    if controller.last_pose is not None and controller.last_stamp is not None and stamp > controller.last_stamp:
        gap = stamp-controller.last_stamp
        if (math.dist(pose[:3], controller.last_pose[:3]) > c['jump_base_m']+c['jump_speed_mps']*gap
                or abs(angle_delta(pose[3], controller.last_pose[3])) > math.radians(c['yaw_jump_deg'])):
            return False
    return controller.release_gate(pose, True, state.armed, state.mode, state.connected, speed)


def preview(c, slot, real):
    # 构造纯状态机仅用于路线展开，不请求事务，不创建文件/线程/ROS连接。
    action = OnceRelease(slot, False, None, c['release_timeout'])
    controller = LowReleaseObservation(c, 'forward', (0., 0., 0., 0.), 0., action)
    return dict(module='11_low_release_test', release_slot=slot,
                release_mode='REAL' if real else 'MOCK', offline=True,
                control_output=False, servo_output=False, starts_driver=False,
                auto_arm=False, auto_mode=False, auto_land=False,
                coordinate_basis='initial_heading_forward_locked_on_manual_offboard',
                route_at_origin_yaw_zero=controller.goals,
                release_condition='after_forward_hover_during_descent_fresh_AGL_strictly_below_limit',
                end='hold_forward_point_at_descent_agl_for_pilot', config=c)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['preview', 'flight'])
    parser.add_argument('--slot', type=int, choices=[1, 2, 3], default=1)
    parser.add_argument('--real-release', action='store_true', help='explicit confirmation to call the existing REAL raw servo')
    parser.add_argument('--check-config', action='store_true', help='offline only, even with flight/--real-release')
    parser.add_argument('--config', type=Path, default=HERE/'config.yaml')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    config = args.config.resolve()
    c = yaml.safe_load(config.read_text(encoding='utf-8'))
    validate_trial(c)
    if args.mode == 'preview' or args.check_config:
        print(json.dumps(preview(c, args.slot, args.real_release), indent=2))
        return

    mode = 'REAL_EXISTING_RAW_SERVO' if args.real_release else 'MOCK_NO_SERVO_CALL'
    print('LOW_RELEASE_TEST mode={} slot={} flight_control=YES manual_arm_offboard_required'.format(mode, args.slot), flush=True)
    if args.real_release:
        print('REAL_RELEASE_CONFIRMED: existing service {}; type check does not prove physical delivery'.format(c['raw_service']), flush=True)
    logs = ROOT/'logs'
    logs.mkdir(exist_ok=True)
    instance = RunLock(logs/'low_release_test.lock')
    context = {}

    def preflight(runtime_config, master, data, data_lock, topics, stopping, output):
        validate_trial(runtime_config)
        # 这时父入口已确认地面上锁且没有冲突控制发布者；仅真实模式访问服务。
        raw = ExistingRawServo(runtime_config) if args.real_release else None
        context.update(c=runtime_config, data=data, lock=data_lock, stopping=stopping, raw=raw)
        context['release'] = OnceRelease(args.slot, args.real_release,
                                       raw.call if raw else None, runtime_config['release_timeout'],
                                       marker=output/'release_attempted.json' if raw else None)

    def factory(runtime_config, profile, origin, now):
        import rospy

        def check():
            # 服务类型检查也可能耗时，因此最后一步必须再次校验实时遥测。
            if context['raw']:
                context['raw'].check_type()
            return live_release_check(context['controller'], runtime_config, context['data'],
                                      context['lock'], context['stopping'],
                                      lambda: rospy.Time.now().to_sec())

        controller = LowReleaseObservation(runtime_config, profile, origin, now,
                                           context['release'], live_check=check)
        context['controller'] = controller
        return controller

    output = (args.output or logs/('low_release_test_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    try:
        low_flight.main(['flight', '--profile', 'forward', '--config', str(config), '--output', str(output)],
                        controller_factory=factory, preflight=preflight, node_name='low_release_test')
    finally:
        if 'stopping' in context:
            context['stopping'].set()
        if 'release' in context:
            result = context['release'].snapshot()
            # 进程收尾时未返回的调用只能记为不确定，绝不将它当成功或自动重发。
            if result['release_state'] in ('PENDING', 'CALL_STARTED'):
                result.update(release_state='UNKNOWN', release_detail='entry_closed_with_request_pending_no_retry')
            (output/'release_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
            print('LOW_RELEASE_RESULT', json.dumps(result), flush=True)
        instance.close()


if __name__ == '__main__':
    main()
