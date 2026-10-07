"""Production tail-stage handoff, timer and decision publishing with ROS IO replaced."""
import ast
import math
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as N

from uav_mission.mission_core import MissionPhase

ROOT = Path(__file__).resolve().parents[1]
NS = '/navigation_height_constraint'
CAP = '/external_planner_max_command_z'


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class Clock:
    value = 10.

    @classmethod
    def now(cls):
        return Stamp(cls.value)

    from_sec = Stamp


class Decision:
    SCHEMA_VERSION = 1

    def __init__(self):
        self.header = N()
        self.goal = N(header=N(), pose=N(position=N(), orientation=N()))


class TailStageRetryTests(unittest.TestCase):
    def setUp(self):
        Clock.value = 10.
        self.params = {
            NS + '/enabled': True, NS + '/frame_id': 'map', CAP: 2.78,
            '~mission/post_delivery_parameter_stages': [
                dict(after_completed_waypoints=1, parameters={CAP: .78,
                     '/navigation/planner_bridge/execution/arrival_dwell': .8})],
        }
        self.writes = []

        def put(name, value):
            self.params[name] = value.copy() if isinstance(value, dict) else value
            self.writes.append((Clock.value, name, self.params[name]))

        ros = N(Time=Clock, get_param=lambda name, default=None:
                self.params.get(name, default), set_param=put, loginfo=lambda *a: None)
        source = ROOT / 'scripts/navigation_mission_manager.py'
        tree = ast.parse(source.read_text())
        parent = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == 'NavigationMissionManager')
        parent.body = [n for n in parent.body if isinstance(n, ast.FunctionDef)
                       and n.name in ('_height_stage_ready', '_on_timer', '_publish_action')]
        env = dict(rospy=ros, math=math, MissionPhase=MissionPhase,
                   NavigationDecision=Decision,
                   COMMAND_VALUES={name: name for name in ('RETURN_HOME', 'ABORT', 'HOLD')},
                   TRANSIENT_READINESS_FAILURES={'map_stale', 'pose_stale'})
        exec(compile(ast.fix_missing_locations(ast.Module(body=[parent], type_ignores=[])),
                     str(source), 'exec'), env)
        o = self.manager = env['NavigationMissionManager']()
        o._lock = threading.RLock()
        o._pending_motion_action = None
        o._runtime = N(core=N(mission_id='test', post_delivery_route_index=1,
                              phase=MissionPhase.SEARCH, config=N(mission_frame='map')))
        o._pose = N(header=N(frame_id='map', stamp=Stamp(10.)), pose=N(position=N(z=.77)))
        o._motion_pose_ready = lambda now: True
        o._readiness = lambda now: (True, 'ready')
        o._apply_following_speed = lambda *a, **kw: None
        o._publish_status = lambda **kw: None
        o._handle_callback_exception = lambda where, error: self.fail(str(error))
        self.sent = []
        o._decision_pub = N(publish=self.sent.append)
        self.action = self.make_action()

        def abort(reason, now):
            o._runtime.core.phase = MissionPhase.ABORTED
            self.abort_reason = reason
            return N(reason=reason, action=self.make_action(command='ABORT', reason=reason))

        o._runtime.abort = abort

    @staticmethod
    def make_action(command='RETURN_HOME', reason='post_delivery_route:2'):
        return N(command=command, reason=reason, decision_seq=4, deadline_at=50.,
                 issued_at=10., goal=N(frame_id='map', x=3., y=0., z=.68, yaw=0.),
                 profile_name='test', has_goal=True, has_target=False, target_snapshot=None)

    def at(self, now):
        Clock.value = now
        self.manager._pose.header.stamp = Stamp(now)

    def requests(self):
        return [value for _, name, value in self.writes if name == NS + '/request']

    def ack(self, request=None, **changes):
        self.params[NS + '/ack'] = dict(request or self.requests()[-1], stamp=Clock.value)
        self.params[NS + '/ack'].update(changes)

    def ready(self):
        return self.manager._height_stage_ready(self.action)

    def test_delayed_ack_publishes_real_pending_decision_after_retry(self):
        o = self.manager
        o._publish_action(self.action)
        first = self.requests()[0].copy()
        self.assertIs(o._pending_motion_action, self.action)
        self.at(10.401)
        o._on_timer(None)
        self.assertEqual(self.sent, [])
        self.at(10.45)
        self.ack(first, stamp=10.)
        o._on_timer(None)
        self.assertEqual([m.command for m in self.sent], ['RETURN_HOME'])
        self.assertIsNone(o._pending_motion_action)
        self.assertEqual(self.sent[0].deadline.to_sec(), 50.)
        self.assertEqual(self.sent[0].goal.pose.position.z, .68)
        self.assertEqual([r['id'] for r in self.requests()], [first['id']] * 2)
        self.assertEqual([r['stamp'] for r in self.requests()], [10., 10.])
        self.assertEqual([r['resend_nonce'] for r in self.requests()], [1, 2])

    def test_multiple_retries_use_last_send_time_not_first_stamp(self):
        self.assertFalse(self.ready())
        for now in (10.401, 10.55, 10.802, 10.95, 11.203, 11.35):
            self.at(now)
            self.assertFalse(self.ready())
        requests = self.requests()
        self.assertEqual(len(requests), 4)
        self.assertEqual(len({r['id'] for r in requests}), 1)
        self.assertEqual({r['stamp'] for r in requests}, {10.})
        self.assertEqual([r['resend_nonce'] for r in requests], [1, 2, 3, 4])
        self.ack(requests[0])
        self.assertTrue(self.ready())
        self.assertEqual(self.action.deadline_at, 50.)

    def test_new_action_parameters_frame_namespace_or_stage_require_new_ack(self):
        for change in ('action', 'cap', 'other_parameter', 'frame', 'namespace', 'stage'):
            with self.subTest(change=change):
                self.setUp()
                self.assertFalse(self.ready())
                old = self.requests()[0].copy()
                self.ack(old)
                stage = self.params['~mission/post_delivery_parameter_stages'][0]
                if change == 'action':
                    self.action.decision_seq += 1
                elif change == 'cap':
                    stage['parameters'][CAP] = .80
                elif change == 'other_parameter':
                    stage['parameters']['/navigation/planner_bridge/execution/arrival_dwell'] = .9
                elif change == 'frame':
                    self.params[NS + '/frame_id'] = 'other'
                    self.manager._pose.header.frame_id = 'other'
                elif change == 'namespace':
                    self.manager._height_namespace = '/other'
                    self.params.update({'/other/enabled': True, '/other/frame_id': 'map', '/other/ack': self.params[NS + '/ack']})
                else:
                    self.manager._runtime.core.post_delivery_route_index = 2
                    stage['after_completed_waypoints'] = 2
                self.assertFalse(self.ready())
                ns = getattr(self.manager, '_height_namespace', NS)
                new = self.params[ns + '/request']
                self.assertNotEqual(new['id'], old['id'])
                self.params[ns + '/ack'] = dict(new, stamp=Clock.value)
                self.assertTrue(self.ready())

    def test_reverting_configuration_does_not_reuse_previous_ack(self):
        self.assertFalse(self.ready())
        old = self.requests()[0].copy()
        params = self.params['~mission/post_delivery_parameter_stages'][0]['parameters']
        params[CAP] = .80
        self.assertFalse(self.ready())
        params[CAP] = .78
        self.ack(old)
        self.assertFalse(self.ready())
        self.assertNotEqual(self.requests()[-1]['id'], old['id'])

    def test_ack_validity_checks_still_apply(self):
        self.assertFalse(self.ready())
        for changes in (dict(stamp=9.99), dict(stamp=10.01), dict(frame='wrong'),
                        dict(id='wrong'), dict(max_z=.79), dict(max_z=math.nan)):
            with self.subTest(changes=changes):
                self.ack(**changes)
                self.assertFalse(self.ready())
        self.at(10.601)
        self.ack(stamp=10.1)
        self.assertFalse(self.ready())
        self.ack(stamp=10.2)
        self.assertTrue(self.ready())

    def test_map_stale_blocks_dispatch_and_original_deadline_still_aborts(self):
        o = self.manager
        o._publish_action(self.action)
        self.ack()
        o._readiness = lambda now: (False, 'map_stale')
        o._on_timer(None)
        self.assertEqual(self.sent, [])
        o._motion_pose_ready = lambda now: False
        self.at(50.)
        o._on_timer(None)
        self.assertEqual(self.abort_reason, 'parameter_stage_wait_timeout')
        self.assertEqual([m.command for m in self.sent], ['ABORT'])
        self.assertIsNone(o._pending_motion_action)

    def test_current_pose_above_cap_prevents_parameter_change(self):
        self.manager._pose.pose.position.z = .80
        self.assertFalse(self.ready())
        self.assertEqual(self.params[CAP], 2.78)
        self.assertEqual(self.requests(), [])

    def test_same_limit_next_stage_is_a_new_transaction(self):
        for completed in (0, 1, 4):
            self.manager._runtime.core.post_delivery_route_index = completed
            self.params['~mission/post_delivery_parameter_stages'][0]['after_completed_waypoints'] = completed
            self.action.decision_seq += 1
            self.assertFalse(self.ready())
            self.ack()
            self.assertTrue(self.ready())
        self.assertEqual(len({r['id'] for r in self.requests()}), 3)


if __name__ == '__main__':
    unittest.main()