"""纯离线过程与事务测试，不导入真实ROS、不启动节点或触碰PWM。"""
import copy
import importlib.abc
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
import yaml

from release_core import LowReleaseObservation, OnceRelease, validate_trial
from entry import ExistingRawServo, RunLock, live_release_check

HERE = Path(__file__).resolve().parent


class FakeRelease:
    def __init__(self):
        self.calls = 0
        self.state = 'NOT_ATTEMPTED'

    def poll(self):
        return self.state

    def snapshot(self):
        return dict(release_state=self.state)

    def request(self, check):
        if self.state != 'NOT_ATTEMPTED':
            return False
        self.state = 'MOCK_COMPLETE' if check() else 'CANCELLED'
        self.calls += int(self.state == 'MOCK_COMPLETE')
        return True


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((HERE/'config.yaml').read_text())
        self.action = FakeRelease()
        self.origin = (1., 2., .22, math.pi/2)
        self.o = LowReleaseObservation(self.c, 'forward', self.origin, 0., self.action)
        self.now = 0.
        self.pose = self.origin
        self.speed = 0.

    def launch(self):
        self.o.step(0., self.origin, 10., True, False, 'POSCTL')
        self.now = 2.1
        self.o.step(self.now, self.origin, 10.+self.now, True, True, 'OFFBOARD')
        self.assertEqual(self.o.stage, 'RUN')

    def tick(self, **kwargs):
        self.now += 1/30
        params = dict(fresh=True, armed=True, mode='OFFBOARD', speed=self.speed)
        params.update(kwargs)
        target = self.o.step(self.now, self.pose, 10.+self.now, **params)
        if target is not None:
            self.speed = math.dist(target[:3], self.pose[:3])*30
            self.pose = target
        return target

    def until(self, condition):
        for _ in range(4500):
            if condition():
                return
            self.tick()
        self.fail('state did not progress: '+str((self.o.stage, self.o.index, self.o.reason)))

    def descend(self):
        self.launch()
        self.until(lambda: self.o.index == 2)

    def test_defaults_and_yaw_rotation_match_existing_forward(self):
        self.assertEqual((self.c['arrival_tolerance'], self.c['arrival_speed']), (.08, .12))
        self.assertEqual((self.c['climb_speed'], self.c['horizontal_speed'], self.c['descent_speed']), (.15, .20, .10))
        self.assertAlmostEqual(self.o.goals[0][2], .60)
        self.assertAlmostEqual(self.o.goals[1][0], 1.)
        self.assertAlmostEqual(self.o.goals[1][1], 3.)
        self.assertAlmostEqual(self.o.goals[2][2], .45)
        self.assertEqual(self.o.goals[1][:2], self.o.goals[2][:2])

    def test_arming_alone_does_not_start_and_manual_offboard_starts_automatically(self):
        self.o.step(0., self.origin, 10., True, False, 'POSCTL')
        self.o.step(3., self.origin, 13., True, True, 'POSCTL')
        self.assertEqual(self.o.stage, 'READY')
        self.o.step(3.1, self.origin, 13.1, True, True, 'OFFBOARD')
        self.assertEqual(self.o.stage, 'RUN')
        self.assertEqual(self.action.calls, 0)

    def test_inherited_offboard_needs_new_pilot_transition(self):
        self.o.step(0., self.origin, 10., True, False, 'OFFBOARD')
        self.o.step(3., self.origin, 13., True, True, 'OFFBOARD')
        self.assertEqual(self.o.stage, 'READY')

    def test_heading_locks_at_offboard_not_at_initial_reference(self):
        self.o.step(0., self.origin, 10., True, False, 'POSCTL')
        new_pose = self.origin[:3]+(0.,)
        self.o.step(3., new_pose, 13., True, True, 'OFFBOARD')
        self.assertAlmostEqual(self.o.goals[1][0], 2.)
        self.assertAlmostEqual(self.o.goals[1][1], 2.)

    def test_full_order_hover_then_release_during_descent_then_hold(self):
        self.launch()
        forward_arrival = None
        dwell_start = None
        descent_at = None
        release_pose = None
        for _ in range(1800):
            before_index = self.o.index
            feedback = self.pose
            self.tick()
            if self.o.index == 1 and self.o.arrival_since is not None:
                forward_arrival = self.o.arrival_since
            if self.o.index == 1 and self.o.dwell_since is not None:
                dwell_start = self.o.dwell_since
            if before_index == 1 and self.o.index == 2:
                descent_at = self.now
                self.assertGreaterEqual(descent_at-forward_arrival, self.c['arrival_dwell']+5.)
                self.assertGreaterEqual(descent_at-dwell_start, 5.)
            if self.action.calls and release_pose is None:
                release_pose = feedback
                self.assertEqual(self.o.stage, 'RUN')
                self.assertEqual(self.o.index, 2)
                self.assertGreater(release_pose[2], .45)
                self.assertLess(release_pose[2], .50)
            if self.o.stage == 'FINISHED_HOVER':
                break
        self.assertIsNotNone(descent_at)
        self.assertIsNotNone(release_pose)
        self.assertEqual(self.action.calls, 1)
        self.assertEqual(self.o.stage, 'FINISHED_HOVER')
        self.assertAlmostEqual(self.o.target[0], 1.)
        self.assertAlmostEqual(self.o.target[1], 3.)
        self.assertAlmostEqual(self.o.target[2], .45)
        target = self.o.target
        for _ in range(150):
            self.assertEqual(self.tick(), target)
        self.assertEqual(self.action.calls, 1)

    def test_no_timer_only_motion_when_aircraft_does_not_follow(self):
        self.launch()
        for _ in range(400):
            self.now += 1/30
            self.o.step(self.now, self.origin, 10.+self.now, True, True, 'OFFBOARD')
        self.assertEqual(self.o.index, 0)
        self.assertEqual(self.action.calls, 0)
        self.assertLessEqual(math.dist(self.o.target[:3], self.origin[:3]), self.c['max_setpoint_lead']+1e-9)

    def test_hover_window_resets_after_leaving_forward_point(self):
        self.launch()
        self.until(lambda: self.o.index == 1 and self.o.dwell_since is not None)
        for _ in range(100):
            self.tick()
        self.assertEqual(self.o.index, 1)
        # 仍在全局范围内；只偏离到位半径，不触发父类跳变保护。
        self.pose = (self.pose[0]-.09,)+self.pose[1:]
        self.tick(speed=0.)
        self.assertIsNone(self.o.dwell_since)
        self.assertEqual(self.o.index, 1)
        self.assertEqual(self.action.calls, 0)

    def test_release_gate_strict_boundary_without_low_pose_speed_or_minimum_gate(self):
        self.descend()
        for z, expected in [(.50, False), (.499, True), (.45, True), (.30, True)]:
            # 故意不是低位3D到位，并且速度大于原arrival_speed，仍只按高度放行。
            pose = (self.o.goals[2][0]+.1, self.o.goals[2][1], z, self.origin[3])
            self.assertEqual(self.o.release_gate(pose, True, True, 'OFFBOARD', True, .2), expected)
        self.assertEqual(self.action.calls, 0)

    def test_low_height_before_forward_hover_never_releases(self):
        self.launch()
        low = self.origin[:2]+(.49, self.origin[3])
        self.assertFalse(self.o.release_gate(low, True, True, 'OFFBOARD', True, 0.))
        self.until(lambda: self.o.index == 1)
        self.assertFalse(self.o.release_gate(low, True, True, 'OFFBOARD', True, 0.))

    def test_stop_stale_disconnect_or_pilot_takeover_before_threshold_blocks_release(self):
        for change in [dict(stop_requested=True), dict(fresh=False), dict(connected=False),
                       dict(mode='POSCTL'), dict(armed=False)]:
            with self.subTest(change=change):
                self.setUp()
                self.descend()
                self.tick(**change)
                for _ in range(5):
                    self.tick()
                self.assertEqual(self.action.calls, 0)
                self.assertIn(self.o.stage, ('HOLD_FOR_PILOT', 'TAKEN_OVER'))

    def test_flight_timeout_and_pose_jump_preserve_existing_cancel(self):
        self.descend()
        self.pose = (self.pose[0]+.5,)+self.pose[1:]
        self.tick()
        self.assertEqual(self.o.stage, 'HOLD_FOR_PILOT')
        self.assertEqual(self.action.calls, 0)
        self.setUp()
        self.descend()
        self.now = self.c['mission_timeout']+5
        self.tick()
        self.assertEqual(self.o.reason, 'observation_timeout')
        self.assertEqual(self.action.calls, 0)

    def test_unknown_transaction_does_not_prevent_low_hold_or_retry(self):
        self.descend()
        self.action.state = 'UNKNOWN'
        self.until(lambda: self.o.stage == 'FINISHED_HOVER')
        self.assertAlmostEqual(self.o.target[2], .45)
        self.assertEqual(self.action.calls, 0)
        self.assertIn('unknown', self.o.reason)


class TransactionTests(unittest.TestCase):
    def join(self, action):
        action.thread.join(2.)
        self.assertFalse(action.thread.is_alive())

    def test_each_slot_calls_actual_req_once(self):
        for slot in (1, 2, 3):
            with tempfile.TemporaryDirectory() as directory:
                calls = []
                action = OnceRelease(slot, True, lambda value: calls.append(value) or True, 3., Path(directory)/'attempt.json')
                self.assertTrue(action.request(lambda: True))
                self.join(action)
                for _ in range(10):
                    self.assertFalse(action.request(lambda: True))
                self.assertEqual(calls, [slot])
                self.assertEqual(action.poll(), 'PWM_ACKNOWLEDGED')

    def test_same_run_cannot_repeat_but_new_run_can_use_same_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            calls = []
            def action(marker):
                return OnceRelease(1, True, lambda slot: calls.append(slot) or True, 3., marker)
            marker = Path(directory)/'run1.json'
            first = action(marker); first.request(lambda: True); self.join(first)
            again = action(marker); again.request(lambda: True); self.join(again)
            new_run = action(Path(directory)/'run2.json'); new_run.request(lambda: True); self.join(new_run)
            self.assertEqual(calls, [1, 1])
            self.assertEqual(again.poll(), 'UNKNOWN')
            self.assertEqual(new_run.poll(), 'PWM_ACKNOWLEDGED')

    def test_mock_never_calls_backend_or_writes_attempt(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory)/'attempt.json'
            action = OnceRelease(2, False, lambda _: self.fail('mock called raw'), 3., marker)
            action.request(lambda: True); self.join(action)
            self.assertEqual(action.poll(), 'MOCK_COMPLETE')
            self.assertFalse(marker.exists())

    def test_exception_and_negative_ack_are_unknown_and_never_retried(self):
        for raises in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                calls = []
                def call(slot):
                    calls.append(slot)
                    if raises:
                        raise IOError('effect uncertain')
                    return False
                action = OnceRelease(1, True, call, 3., Path(directory)/'attempt.json')
                action.request(lambda: True); self.join(action)
                self.assertEqual(action.poll(), 'UNKNOWN')
                self.assertFalse(action.request(lambda: True))
                self.assertEqual(calls, [1])

    def test_blocked_call_does_not_block_control_and_late_ack_does_not_clear_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            entered, finish = threading.Event(), threading.Event()
            now = [0.]
            calls = []
            def call(slot):
                calls.append(slot); entered.set(); finish.wait(2.); return True
            action = OnceRelease(1, True, call, 3., Path(directory)/'attempt.json', clock=lambda: now[0])
            try:
                action.request(lambda: True)
                self.assertTrue(entered.wait(1.))
                now[0] = 3.1
                self.assertEqual(action.poll(), 'UNKNOWN')
                self.assertFalse(action.request(lambda: True))
            finally:
                finish.set(); self.join(action)
            self.assertEqual(action.poll(), 'UNKNOWN')
            self.assertEqual(calls, [1])

    def test_dispatch_rechecks_after_marker_and_cancels_without_call(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory)/'attempt.json'
            def check():
                self.assertTrue(marker.exists())
                return False
            action = OnceRelease(1, True, lambda _: self.fail('gate closed'), 3., marker)
            action.request(check); self.join(action)
            self.assertEqual(action.poll(), 'CANCELLED')
            self.assertFalse(action.request(lambda: True))

    def test_delayed_dispatch_timeout_never_calls_even_without_poll(self):
        with tempfile.TemporaryDirectory() as directory:
            now = [0.]
            def check():
                now[0] = 4.; return True
            action = OnceRelease(1, True, lambda _: self.fail('expired'), 3., Path(directory)/'attempt.json', clock=lambda: now[0])
            action.request(check); self.join(action)
            self.assertEqual(action.poll(), 'UNKNOWN')

    def test_invalid_slots_and_missing_marker_rejected(self):
        for slot in (0, 4, True):
            with self.assertRaises(ValueError):
                OnceRelease(slot, False, None, 3.)
        with self.assertRaises(ValueError):
            OnceRelease(1, True, None, 3.)


class RuntimeChecks(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((HERE/'config.yaml').read_text())
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.controller = LowReleaseObservation(self.c, 'forward', (0., 0., .22, 0.), 0., FakeRelease())
        self.controller.stage = 'RUN'; self.controller.index = 2; self.controller.ever_started = True
        def header(frame):
            return N(frame_id=frame, stamp=N(to_sec=lambda: 10.))
        def pose(frame):
            return N(header=header(frame), pose=N(position=N(x=1., y=0., z=.49), orientation=N(x=0., y=0., z=0., w=1.)))
        self.data = dict(state=(N(armed=True, connected=True, mode='OFFBOARD'), 20.),
                         pose=(pose('map'), 20.), ev=(pose('camera_init'), 20.),
                         odom=(N(header=header('map'), twist=N(twist=N(linear=N(x=.2, y=0., z=-.1)))), 20.))

    def check(self, wall=20., now=10.):
        return live_release_check(self.controller, self.c, self.data, self.lock, self.stop, lambda: now, lambda: wall)

    def test_live_gate_allows_moving_descent_and_both_age_clocks_must_be_fresh(self):
        self.assertTrue(self.check())
        self.assertFalse(self.check(wall=20.31))
        self.assertFalse(self.check(now=10.31))
        self.assertFalse(self.check(now=9.99))

    def test_live_gate_rechecks_pilot_stop_mode_and_disarm(self):
        self.stop.set(); self.assertFalse(self.check()); self.stop.clear()
        self.data['state'][0].mode = 'POSCTL'; self.assertFalse(self.check())
        self.data['state'][0].mode = 'OFFBOARD'; self.data['state'][0].armed = False
        self.assertFalse(self.check())

    def test_live_gate_rechecks_invalid_pose_ev_and_speed(self):
        self.data['pose'][0].pose.position.z = math.nan
        self.assertFalse(self.check())
        self.data['pose'][0].pose.position.z = .49
        self.data['ev'][0].header.frame_id = 'wrong'
        self.assertFalse(self.check())
        self.data['ev'][0].header.frame_id = 'camera_init'
        self.data['odom'][0].twist.twist.linear.x = math.nan
        self.assertFalse(self.check())

    def test_raw_uses_existing_wire_type_without_starting_driver(self):
        events = []
        class Servo:
            _type = 'patrol_control/Servo'
        def proxy(name, kind, persistent):
            events.append(('proxy', name, kind._type, persistent))
            return lambda **kwargs: events.append(('call', kwargs)) or N(res=True)
        fake_ros = N(wait_for_service=lambda name, timeout: events.append(('wait', name, timeout)), ServiceProxy=proxy)
        fake_service = N(get_service_type=lambda name: Servo._type)
        with patch.dict(sys.modules, {'rospy': fake_ros, 'rosservice': fake_service,
                                      'patrol_control': N(), 'patrol_control.srv': N(Servo=Servo)}):
            raw = ExistingRawServo(self.c)
            self.assertFalse(any(event[0] == 'call' for event in events))
            self.assertTrue(raw.call(3))
            self.assertIn(('call', dict(req=3)), events)
            fake_service.get_service_type = lambda name: 'std_srvs/Trigger'
            with self.assertRaises(RuntimeError):
                ExistingRawServo(self.c)

    def test_instance_mutex_is_released_and_not_a_permanent_slot_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'entry.lock'
            first = RunLock(path)
            try:
                with self.assertRaises(RuntimeError):
                    RunLock(path)
            finally:
                first.close()
            RunLock(path).close()


class ContractTests(unittest.TestCase):
    def test_preview_and_flight_config_check_are_offline_for_all_slots_and_real_flag(self):
        # 禁止ROS import及所有新进程启动；即使真实flag也必须只打印JSON。
        import entry
        import contextlib
        import io
        class NoROS(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split('.')[0] in ('rospy', 'rosservice', 'rosgraph', 'rosnode', 'mavros_msgs'):
                    raise AssertionError('offline path imported '+fullname)
        blocker = NoROS()
        sys.meta_path.insert(0, blocker)
        try:
            with patch('subprocess.Popen', side_effect=AssertionError('offline path spawned a process')):
                for slot in (1, 2, 3):
                    for mode in ('preview', 'flight'):
                        args = [mode, '--slot', str(slot), '--real-release']
                        if mode == 'flight': args += ['--check-config']
                        out = io.StringIO()
                        with contextlib.redirect_stdout(out):
                            entry.main(args)
                        result = json.loads(out.getvalue())
                        self.assertFalse(result['control_output'])
                        self.assertFalse(result['servo_output'])
                        self.assertEqual(result['release_slot'], slot)
                        self.assertEqual(result['release_mode'], 'REAL')
        finally:
            sys.meta_path.remove(blocker)

    def test_shell_config_check_does_not_source_ros_environment(self):
        result = subprocess.run(['bash', str(HERE/'start.sh'), 'flight', '--real-release', '--check-config'],
                                env=dict(os.environ, BOARD_PYTHON=sys.executable), text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)['offline'])

    def test_invalid_slot_and_invalid_config_rejected_offline(self):
        result = subprocess.run([sys.executable, str(HERE/'entry.py'), 'preview', '--slot', '4'], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        c = yaml.safe_load((HERE/'config.yaml').read_text())
        for key, value in [('descent_agl', .55), ('release_below_agl', math.nan), ('horizontal_speed', 1.)]:
            bad = copy.deepcopy(c); bad[key] = value
            with self.assertRaises(ValueError):
                validate_trial(bad)


if __name__ == '__main__':
    unittest.main()
