"""Retry/terminal regressions using production stage and runtime methods."""
import ast
import importlib.util
import math
import os
import subprocess
import tempfile
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch

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


class StageLimitRetryIdentityTests(unittest.TestCase):
    def setUp(self):
        Clock.value = 10.
        self.params = {
            NS + '/enabled': True, CAP: 2.78, '/ceiling': -.1,
            '~high_view_probe/low_stage_parameters': [
                dict(name=CAP, value=1.63), dict(name='/ceiling', value=-.1)],
        }
        self.writes = []
        missing = object()

        def get(name, default=missing):
            if name in self.params:
                return self.params[name]
            if default is missing:
                raise KeyError(name)
            return default

        def put(name, value):
            self.params[name] = value.copy() if isinstance(value, dict) else value
            self.writes.append((Clock.value, name, self.params[name]))

        ros = N(get_param=get, set_param=put, Time=Clock)
        # Import the entire production mixin, substituting only ROS IO/clock.
        source = ROOT / 'src/uav_mission/high_view_stage_limits.py'
        spec = importlib.util.spec_from_file_location(
            'uav_mission.stage_limit_retry_under_test', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', rospy=ros):
            spec.loader.exec_module(module)

        # Keep the actual parent timer's ACK-wait deadline reducer. Importing
        # the ROS shell would otherwise require generated ROS message modules.
        parent_source = ROOT / 'scripts/navigation_mission_manager.py'
        tree = ast.parse(parent_source.read_text())
        parent = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef)
                      and node.name == 'NavigationMissionManager')
        parent.body = [node for node in parent.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_on_timer']
        env = dict(rospy=ros, MissionPhase=MissionPhase)
        exec(compile(ast.fix_missing_locations(ast.Module(
            body=[parent], type_ignores=[])), str(parent_source), 'exec'), env)

        class Base(env['NavigationMissionManager']):
            def _height_stage_ready(self, action):
                return True

            def _motion_pose_ready(self, now):
                return False

        class Manager(module.HighViewStageMixin, Base):
            pass

        self.manager = o = Manager()
        o._runtime = N(stage='REVISIT', core=N(
            config=N(mission_frame='map', mission_timeout=600.),
            mission_id='m', started_at=1., phase=MissionPhase.SEARCH))
        o._pose = N(header=N(frame_id='map', stamp=Stamp(10.)),
                    pose=N(position=N(z=1.18)))
        o._pose_max_age = .5
        o._low_limits_applied = False
        o._high_stage_parameters = None
        o._probe_limit_switch = None
        o._pending_motion_action = None
        import threading
        o._lock = threading.RLock()
        self.published = []
        o._publish_action = self.published.append
        o._publish_status = lambda **kwargs: None

        def abort(reason, now):
            o._runtime.core.phase = MissionPhase.ABORTED
            self.abort_reason = reason
            return N(reason=reason, action=N(command='ABORT'))

        o._runtime.abort = abort
        self.action = N(command='SEARCH', decision_seq=1, deadline_at=50.)

    def at(self, now):
        Clock.value = now
        self.manager._pose.header.stamp = Stamp(now)

    def ready(self):
        return self.manager._height_stage_ready(self.action)

    def ack(self, request=None, stamp=None, **changes):
        value = dict(request or self.params[NS + '/request'])
        value.update(stamp=Clock.value if stamp is None else stamp, **changes)
        self.params[NS + '/ack'] = value

    def requests(self):
        return [value for _, name, value in self.writes
                if name == NS + '/request']

    @staticmethod
    def identity(request):
        return {key: value for key, value in request.items()
                if key != 'resend_nonce'}

    def test_delayed_045_second_ack_matches_after_04_second_retry(self):
        self.assertFalse(self.ready())
        first = self.requests()[0].copy()
        self.at(10.401)
        self.assertFalse(self.ready())
        self.assertEqual(len(self.requests()), 2)
        self.at(10.45)
        self.ack(first)
        self.assertTrue(self.ready())
        self.assertEqual([self.identity(r) for r in self.requests()],
                         [self.identity(first), self.identity(first)])
        self.assertGreater(self.requests()[-1]['resend_nonce'], first['resend_nonce'])
        self.assertEqual(self.action.deadline_at, 50.)

    def test_multiple_retries_keep_identity_first_stamp_and_retry_spacing(self):
        self.assertFalse(self.ready())
        first = self.requests()[0].copy()
        for now in (10.401, 10.55, 10.802, 10.95, 11.203, 11.35):
            self.at(now)
            self.assertFalse(self.ready())
        self.assertEqual(len(self.requests()), 4)
        self.assertTrue(all(self.identity(request) == self.identity(first)
                            for request in self.requests()))
        self.assertEqual([r['resend_nonce'] for r in self.requests()], [1, 2, 3, 4])
        self.assertEqual(self.action.deadline_at, 50.)
        self.ack(first)
        self.assertTrue(self.ready())
        self.assertEqual(first['stamp'], 10.)

    def test_old_ack_does_not_authorize_a_new_action(self):
        self.assertFalse(self.ready())
        old = self.requests()[0].copy()
        self.ack(old)
        self.assertTrue(self.ready())
        self.manager._runtime.stage = 'RESUME_ASCEND'
        self.action.decision_seq = 2
        self.assertFalse(self.ready())
        self.assertNotEqual(self.requests()[-1]['id'], old['id'])
        self.ack(old)
        self.assertFalse(self.ready())
        self.ack()
        self.assertTrue(self.ready())
        self.assertGreater(self.requests()[-1]['resend_nonce'], old['resend_nonce'])

    def test_changed_parameters_same_action_same_clock_get_new_identity(self):
        for name, value in ((CAP, 1.55), ('/ceiling', -.2)):
            with self.subTest(name=name):
                self.setUp()
                self.assertFalse(self.ready())
                old = self.requests()[0].copy()
                self.ack(old)
                limits = self.params['~high_view_probe/low_stage_parameters']
                limits[:] = [dict(item, value=value) if item['name'] == name
                             else item.copy() for item in limits]
                self.assertFalse(self.ready())
                self.assertNotEqual(self.requests()[-1]['id'], old['id'])
                self.assertEqual(self.params[name], value)
                self.ack()
                self.assertTrue(self.ready())

    def test_changed_frame_requires_new_identity(self):
        self.assertFalse(self.ready())
        old = self.requests()[0].copy()
        self.ack(old)
        self.manager._runtime.core.config.mission_frame = 'new-map'
        self.manager._pose.header.frame_id = 'new-map'
        self.assertFalse(self.ready())
        self.assertNotEqual(self.requests()[-1]['id'], old['id'])
        self.ack()
        self.assertTrue(self.ready())

    def test_ack_age_first_stamp_frame_limit_and_future_are_checked(self):
        self.assertFalse(self.ready())
        first = self.requests()[0].copy()
        for changes in (dict(stamp=9.99), dict(stamp=10.01),
                        dict(frame='wrong'), dict(max_z=1.6),
                        dict(max_z=math.nan), dict(id='old-id')):
            with self.subTest(changes=changes):
                self.ack(first, **changes)
                self.assertFalse(self.ready())
        self.at(10.601)
        self.ack(first, stamp=10.1)
        self.assertFalse(self.ready())
        self.ack(first, stamp=10.2)
        self.assertTrue(self.ready())

    def test_expired_ack_on_same_identity_stays_blocked_until_fresh_ack(self):
        self.assertFalse(self.ready())
        first = self.requests()[0].copy()
        self.ack(first, stamp=10.05)
        self.at(10.601)
        self.assertFalse(self.ready())
        self.assertEqual(self.identity(self.requests()[-1]), self.identity(first))
        self.assertGreater(self.requests()[-1]['resend_nonce'], first['resend_nonce'])
        self.ack(first)
        self.assertTrue(self.ready())

    def test_deadline_refuses_ack_and_parent_timer_aborts_wait(self):
        for mission_deadline in (False, True):
            with self.subTest(mission_deadline=mission_deadline):
                self.setUp()
                if mission_deadline:
                    self.manager._runtime.core.config.mission_timeout = 9.45
                else:
                    self.action.deadline_at = 10.45
                self.assertFalse(self.ready())
                self.at(10.45)
                self.ack()
                self.assertFalse(self.ready())
                self.assertEqual(len(self.requests()), 1)
                if not mission_deadline:
                    self.manager._pending_motion_action = self.action
                    self.manager._on_timer(None)
                    self.assertEqual(self.abort_reason, 'parameter_stage_wait_timeout')
                    self.assertEqual([a.command for a in self.published], ['ABORT'])
                    self.assertIsNone(self.manager._pending_motion_action)



class PlannerHeightAckRetryTests(unittest.TestCase):
    """Run the actual FSM ACK block with the actual Python stage method."""
    @classmethod
    def setUpClass(cls):
        source = (ROOT.parent / 'Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp').read_text()
        start = source.index('  static ros::Time height_request_polled;',
                             source.index('void KinoReplanFSM::execFSMCallback'))
        end = source.index('  if (exec_state_==EXEC_TRAJ', start)
        program = r'''
#include <XmlRpcValue.h>
#include <string>
#include <cmath>
#include <limits>
#include <iostream>
#include <iomanip>
XmlRpc::XmlRpcValue request_data,ack_data;
int writes=0,reads=0;
namespace ros {
struct Duration {double v;double toSec()const{return v;}};
struct Time {double v=0;Time(){} Time(double x):v(x){} bool isZero()const{return v==0;} double toSec()const{return v;}};
Duration operator-(Time a,Time b){return Duration{a.v-b.v};}
bool operator<(Time a,Time b){return a.v<b.v;}
namespace param {
bool getCached(const std::string&,XmlRpc::XmlRpcValue& out){++reads;out=request_data;return true;}
void set(const std::string&,const XmlRpc::XmlRpcValue& out){++writes;ack_data=out;}
}}
std::string heightConstraintNamespace(){return "/test";}
struct Height {bool enabled=true,valid=true;double max_z=1.63;std::string frame="map";} height;
void tick(double t) {
ros::Time now(t);
BLOCK
}
int main(){
 double t,cap;std::string id,frame;int kind,nonce;
 std::cout << std::setprecision(17);
 while(std::cin >> t >> id >> cap >> frame >> kind >> nonce){
  request_data.clear();
  request_data["id"]=id;request_data["max_z"]=cap;request_data["frame"]=frame;
  if(kind==1)request_data["resend_nonce"]=nonce;
  if(kind==2)request_data["resend_nonce"]=std::string("invalid");
  if(kind==3)request_data["resend_nonce"]=static_cast<double>(nonce);
  tick(t);
  std::cout << writes << " " << reads;
  if(writes){
   std::cout << " " << static_cast<std::string>(ack_data["id"])
    << " " << static_cast<double>(ack_data["max_z"])
    << " " << static_cast<std::string>(ack_data["frame"])
    << " " << static_cast<double>(ack_data["stamp"]);
   if(ack_data.hasMember("resend_nonce"))
    std::cout << " " << static_cast<int>(ack_data["resend_nonce"]);
  }
  std::cout << std::endl;
 }
}
'''.replace('BLOCK', source[start:end])
        cls.tmp = tempfile.TemporaryDirectory(prefix='stage-limit-ack-')
        cls.addClassCleanup(cls.tmp.cleanup)
        cpp = Path(cls.tmp.name) / 'ack.cpp'
        cpp.write_text(program)
        cls.executable = Path(cls.tmp.name) / 'ack'
        subprocess.run([
            'g++', '-std=c++14', '-I/opt/ros/noetic/include/xmlrpcpp',
            '-I/opt/ros/noetic/include', str(cpp), '-L/opt/ros/noetic/lib',
            '-Wl,-rpath,/opt/ros/noetic/lib', '-lxmlrpcpp',
            '-o', str(cls.executable)], check=True)

    def setUp(self):
        self.fixture = StageLimitRetryIdentityTests()
        self.fixture.setUp()
        self.process = subprocess.Popen([str(self.executable)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True,
                                        env=dict(os.environ, LD_LIBRARY_PATH=
                                            '/opt/ros/noetic/lib' + (':' + os.environ['LD_LIBRARY_PATH']
                                            if os.environ.get('LD_LIBRARY_PATH') else '')))
        self.addCleanup(self.close)

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=5.)
        self.process.stdout.close()

    def poll(self, now, request=None, kind=None):
        request = request or self.fixture.params[NS + '/request']
        has_nonce = 'resend_nonce' in request
        kind = int(has_nonce) if kind is None else kind
        line = '{} {} {} {} {} {}\n'.format(
            now, request['id'], request['max_z'], request['frame'],
            kind, request.get('resend_nonce', 0))
        self.process.stdin.write(line)
        self.process.stdin.flush()
        parts = self.process.stdout.readline().split()
        self.assertTrue(parts, 'planner ACK harness exited unexpectedly')
        writes, reads = map(int, parts[:2])
        ack = None
        if writes:
            ack = dict(id=parts[2], max_z=float(parts[3]), frame=parts[4],
                       stamp=float(parts[5]))
            if len(parts) > 6:
                ack['resend_nonce'] = int(parts[6])
        return writes, reads, ack

    def test_ack_older_than_05_transport_delay_regenerates_after_resend(self):
        f = self.fixture
        self.assertFalse(f.ready())
        first = f.requests()[0].copy()
        writes, _, old_ack = self.poll(10.)
        self.assertEqual(writes, 1)
        f.at(10.601)
        f.params[NS + '/ack'] = old_ack
        self.assertFalse(f.ready())
        self.assertEqual(f.identity(f.requests()[-1]), f.identity(first))
        writes, _, fresh_ack = self.poll(10.601)
        self.assertEqual(writes, 2)
        self.assertGreater(fresh_ack['resend_nonce'], old_ack['resend_nonce'])
        f.params[NS + '/ack'] = fresh_ack
        self.assertTrue(f.ready())
        self.assertEqual(f.action.deadline_at, 50.)
        for now in (10.802, 11.003, 11.204, 11.405, 12.):
            writes, _, ack = self.poll(now)
            self.assertEqual(writes, 2)
            self.assertEqual(ack, fresh_ack)

    def test_045_delayed_initial_ack_is_valid_even_after_nonce_retry(self):
        f = self.fixture
        self.assertFalse(f.ready())
        _, _, ack = self.poll(10.)
        f.at(10.401)
        self.assertFalse(f.ready())
        self.assertGreater(f.requests()[-1]['resend_nonce'], ack['resend_nonce'])
        f.at(10.45)
        f.params[NS + '/ack'] = ack
        self.assertTrue(f.ready())
        self.assertEqual(f.action.deadline_at, 50.)

    def test_repeated_nonce_and_legacy_requests_never_write_periodically(self):
        request = dict(id='legacy', max_z=1.63, frame='map')
        for i in range(100):
            writes, reads, ack = self.poll(10. + i * .01, request)
            self.assertEqual(writes, 1)
            self.assertNotIn('resend_nonce', ack)
        self.assertLessEqual(reads, 10)
        request.update(id='retry', resend_nonce=1)
        writes, _, ack = self.poll(11.2, request)
        self.assertEqual(writes, 2)
        self.assertEqual(ack['resend_nonce'], 1)
        for i in range(100):
            writes, _, _ = self.poll(11.21 + i * .01, request)
            self.assertEqual(writes, 2)
        request['resend_nonce'] = 2
        writes, _, ack = self.poll(12.4, request)
        self.assertEqual(writes, 3)
        self.assertEqual(ack['resend_nonce'], 2)
        self.assertEqual(self.poll(13., request)[0], 3)

    def test_invalid_nonce_cap_and_frame_cannot_refresh_ack(self):
        request = dict(id='retry', max_z=1.63, frame='map', resend_nonce=1)
        self.assertEqual(self.poll(10., request)[0], 1)
        self.assertEqual(self.poll(10.2, request, kind=2)[0], 1)
        self.assertEqual(self.poll(10.4, request, kind=3)[0], 1)
        self.assertEqual(self.poll(10.6, dict(request, resend_nonce=-1))[0], 1)
        self.assertEqual(self.poll(10.8, dict(request, resend_nonce=2, max_z=1.5))[0], 1)
        self.assertEqual(self.poll(11., dict(request, resend_nonce=2, frame='other'))[0], 1)
        self.assertEqual(self.poll(11.2, dict(request, resend_nonce=2))[0], 2)

    def test_retries_and_regenerated_ack_do_not_extend_deadline(self):
        f = self.fixture
        f.action.deadline_at = 10.9
        self.assertFalse(f.ready())
        _, _, ack = self.poll(10.)
        f.params[NS + '/ack'] = ack
        for now in (10.601, 10.899):
            f.at(now)
            self.assertFalse(f.ready())
        _, _, ack = self.poll(10.9)
        self.assertGreater(ack['stamp'], 10.)
        f.at(10.9)
        f.params[NS + '/ack'] = ack
        self.assertFalse(f.ready())
        self.assertEqual(f.action.deadline_at, 10.9)
        self.assertEqual(len(f.requests()), 2)
        f.manager._pending_motion_action = f.action
        f.manager._on_timer(None)
        self.assertEqual(f.abort_reason, 'parameter_stage_wait_timeout')
        self.assertEqual([a.command for a in f.published], ['ABORT'])

if __name__ == '__main__':
    unittest.main()
