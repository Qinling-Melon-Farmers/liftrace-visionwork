#!/usr/bin/env python3
"""Call production proxy/Bridge/core methods with transport doubles, no ROS nodes."""
import ast
import copy
from dataclasses import dataclass, replace
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as N
import unittest

from uav_mission.release_transactions import (
    EXECUTION_UNKNOWN, NOT_STARTED, RAW_CALL_STARTED, COMPLETED, action_identity, execution_fact,
    result_terminal, SemanticContradictionWindow,
)
from uav_mission.mission_core import MissionCore, MissionConfig, SlotStatus, ResultEvent
from uav_mission.planner_execution import (
    MotionDecision, MotionGoal, TargetIdentity, PlannerMotionExecutor,
    PlannerMotionConfig, OdomSample,
)
from uav_mission.position_settle import PositionSettleWindow
from test_mission_core import candidate, competition_profile, result_for

P = Path(__file__).resolve().parents[1]
NS = 1_000_000_000

class Stamp:
    def __init__(self, value=0):
        self.value = float(value)
        self.secs = int(self.value)
        self.nsecs = round((self.value - self.secs) * NS)
    def to_sec(self): return self.value
    def to_nsec(self): return round(self.value * NS)
    def __lt__(self, other): return self.value < other.value
    def __le__(self, other): return self.value <= other.value
    def __gt__(self, other): return self.value > other.value
    def __sub__(self, other): return Stamp(self.value - other.value)
    def __add__(self, other): return Stamp(self.value + other.value)

class Clock:
    value = 100.10
    @classmethod
    def now(cls): return Stamp(cls.value)

class Message:
    def __init__(self):
        self.header = N(stamp=Stamp(0))
        self.execution_id = self.execution_state = 0
        self.terminal = self.success = self.permitted = False
        self.payload_slot = self.target_id = self.decision_seq = self.attempt = 0
        self.align_mode = self.target_class = self.mission_id = self.reason = ''
        self.target_first_seen = self.evidence_stamp = self.valid_until = Stamp(0)

class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))

ROS = N(Time=Clock, ROSException=TimeoutError, ServiceException=RuntimeError,
        loginfo=lambda *a: None, logwarn=lambda *a: None, logerr=lambda *a: None)
EXT = N(LANDED_STATE_ON_GROUND=1, LANDED_STATE_IN_AIR=2)


def classes(filename, names, namespace):
    tree = ast.parse((P / 'scripts' / filename).read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name in names]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, 'exec'), namespace)
    return namespace

COMMON = dict(EXECUTION_UNKNOWN=EXECUTION_UNKNOWN, NOT_STARTED=NOT_STARTED, RAW_CALL_STARTED=RAW_CALL_STARTED,
              COMPLETED=COMPLETED, action_identity=action_identity,
              execution_fact=execution_fact, result_terminal=result_terminal,
              SemanticContradictionWindow=SemanticContradictionWindow,
              threading=threading, time=time, math=math, dataclass=dataclass, rospy=ROS)
PROXY = classes('guarded_servo_proxy.py', ['GuardedServoProxy'], dict(COMMON,
    ReleaseResult=Message, ReleasePermission=Message, ServoResponse=lambda **v: N(**v),
    ServoActionResponse=lambda **v: N(**v)))['GuardedServoProxy']
BR = classes('navigation_planner_bridge.py',
    ['SemanticTargetPose','TargetTransaction','LandingTransaction','NavigationPlannerBridge'],
    dict(COMMON, MotionDecision=MotionDecision, ExtendedState=EXT,
         _stamp_to_ns=lambda s: s.to_nsec()))
ARB = classes('release_permission_arbiter.py', ['ReleasePermissionArbiter'],
    dict(COMMON, Bool=lambda **v:N(**v),ReleasePermission=Message,ReleaseAuthorization=Message))['ReleasePermissionArbiter']


def permission(seq=1, **fields):
    m = Message()
    m.header.stamp = Stamp(100.1)
    m.valid_until = Stamp(101.)
    m.evidence_stamp = Stamp(100.04)
    m.permitted = True
    m.permission_epoch = 'arbiter-test'
    m.permission_revision = seq
    m.payload_slot, m.target_id = 1, 7
    m.align_mode, m.target_class = 'drop_circle', 'bridge'
    m.mission_id, m.decision_seq, m.attempt = 'mission-1', seq, 1
    m.target_first_seen = Stamp(99.)
    for k, v in fields.items(): setattr(m,k,v)
    return m


def context(m, active=False):
    return N(active=active, payload_slot=m.payload_slot, align_mode=m.align_mode,
        semantic_target_id=m.target_id, semantic_target_class=m.target_class,
        mission_id=m.mission_id, decision_seq=m.decision_seq, attempt=m.attempt,
        semantic_target_first_seen=m.target_first_seen)


def result(m, fact=COMPLETED, terminal=True, execution_id=1, stamp=100.04):
    msg = copy.deepcopy(m)
    msg.header.stamp = Stamp(stamp)
    msg.success, msg.execution_state, msg.terminal = fact == COMPLETED, fact, terminal
    msg.execution_id = execution_id
    return msg

class Raw:
    def __init__(self, success=True, wait_fail=False, rpc_fail=False):
        self.calls = 0
        self.success, self.wait_fail, self.rpc_fail = success, wait_fail, rpc_fail
    def wait_for_service(self, timeout):
        if self.wait_fail: raise TimeoutError('unavailable')
    def __call__(self, slot):
        self.calls += 1
        if self.rpc_fail: raise RuntimeError('closed after PWM')
        return N(res=self.success)


def proxy(raw=None):
    o = PROXY.__new__(PROXY)
    o._permission = permission()
    o._consumed_permission_stamp = Stamp(0)
    o._completed_slots, o._locked_slots, o._locked_actions = set(), set(), set()
    o._inflight_slots, o._revoked_actions = set(), set()
    o._execution_id = 0
    o._lock, o._result_pub = threading.RLock(), Publisher()
    o._permission_changed = threading.Condition(o._lock)
    o._permission_refresh_wait = .1
    o._raw_wait_timeout, o._permission_max_age = .25, .5
    o._raw_client = raw or Raw()
    return o


def core():
    o = MissionCore(competition_profile(), MissionConfig(early_return_enabled=False))
    o.start('mission-1', 100.)
    return o


def bridge():
    o = BR['NavigationPlannerBridge'].__new__(BR['NavigationPlannerBridge'])
    o._lock = threading.RLock()
    o._output_enabled = True
    o._now_ns = lambda: round(Clock.value * NS)
    o._mission_frame = 'camera_init'
    o._capture_max_age_ns = 500_000_000
    o._executor = PlannerMotionExecutor(PlannerMotionConfig(executor_id='test-bridge'))
    o.core = core()
    c = candidate(target_id=7, now=100.)
    a = o.core.choose_confirmed(c,100.,(0,0))
    t = TargetIdentity(c.target_id,c.first_seen_ns,c.last_seen_ns,c.class_name,a.attempt,a.payload_slot)
    d = MotionDecision('mission-1', a.decision_seq, round(a.issued_at * NS),
        round(a.deadline_at * NS), 'APPROACH','r2026',MotionGoal('camera_init',1.,0.,1.2),t)
    o._executor.submit_decision(d,round(100. * NS))
    o._executor._active.handed_off = True  # Fixture: APPROACH has reached CAPTURE.
    o._transaction = BR['TargetTransaction'](decision=d, phase='ALIGNMENT',
        target_pose=BR['SemanticTargetPose']('camera_init',1.,0.,0.,round(99.9 * NS)),
        align_mode='drop_circle',strict_evidence_stamp_ns=round(100.04 * NS))
    o._pending_release = o._landing = None
    o._flight_state = None
    o._flight_state_receipt_ns = o._flight_state_source_ns = 0
    o._landing_handoff_mode = 'AUTO.LAND'
    o._landing_handoff_wait_ns = 500_000_000
    o._landing_mode_transition_timeout_ns = 2_500_000_000
    o._landing_state_max_age_ns = 2_500_000_000
    o._recovery_settle = PositionSettleWindow(500_000_000,.15,500_000_000)
    o._landing_settle = PositionSettleWindow(200_000_000,.15,500_000_000)
    o._landing_height, o._landing_radius = .05, .25
    o._latest_candidates, o.context_events, o.events, o.errors = (), [], [], []
    o._publish_alignment_context = lambda active,now=None: o.context_events.append(active)
    o._handle_callback_exception = lambda where,error:o.errors.append((where,error))
    def apply(out):
        o.events.extend(out.events)
        for e in out.events:
            fields = {name:getattr(e,name) for name in ResultEvent.__dataclass_fields__}
            accepted, why, _ = o.core.apply_result(ResultEvent(**fields),Clock.value)
            if not accepted: o.errors.append(('core',why))
    o._apply_outcome = apply
    return o

class ProxyTests(unittest.TestCase):
    def setUp(self): Clock.value = 100.10
    def test_preflight_denial_never_calls_raw_or_locks_slot(self):
        for changes in (dict(valid_until=Stamp(99.)),dict(permitted=False,reason='release_altitude_invalid')):
            o=proxy();o._permission=permission(**changes)
            self.assertFalse(o._on_servo_request(N(req=1)).res)
            self.assertEqual(o._raw_client.calls,0)
            self.assertEqual(o._locked_slots,set())
            self.assertEqual(o._result_pub.messages[-1].execution_state,NOT_STARTED)
    def test_service_discovery_failure_is_not_started_and_fresh_retry_can_complete(self):
        o=proxy(Raw(wait_fail=True));o._on_servo_request(N(req=1))
        self.assertEqual(o._result_pub.messages[-1].execution_state,NOT_STARTED)
        o._raw_client=Raw();m=permission(permission_revision=2);m.header.stamp=Stamp(100.11)
        Clock.value=100.11;o._on_permission(m)
        self.assertTrue(o._on_servo_request(N(req=1)).res)
        self.assertEqual(o._raw_client.calls,1)
    def test_false_or_rpc_error_locks_slot_before_success_and_cannot_reenter(self):
        for raw in (Raw(success=False),Raw(rpc_fail=True)):
            o=proxy(raw);self.assertFalse(o._on_servo_request(N(req=1)).res)
            self.assertEqual(raw.calls,1)
            states=[m.execution_state for m in o._result_pub.messages]
            self.assertEqual(states,[RAW_CALL_STARTED,RAW_CALL_STARTED])
            self.assertFalse(o._result_pub.messages[0].terminal)
            ids=[m.execution_id for m in o._result_pub.messages];self.assertEqual(ids[0],ids[1])
            o._on_permission(permission(seq=2))
            self.assertFalse(o._on_servo_request(N(req=1)).res)
            self.assertEqual(raw.calls,1)
            self.assertEqual(len(o._result_pub.messages),2)
    def test_revocation_before_call_proves_not_started_and_blocks_future_fresh_permit(self):
        o=proxy();o._on_alignment_context(context(o._permission))
        self.assertEqual(o._result_pub.messages[-1].execution_state,NOT_STARTED)
        o._on_permission(permission())
        self.assertFalse(o._on_servo_request(N(req=1)).res)
        self.assertEqual(o._raw_client.calls,0)
    def test_cancel_after_call_entry_cannot_claim_not_started(self):
        started,finish=threading.Event(),threading.Event()
        class Blocking(Raw):
            def __call__(self,slot):
                self.calls+=1;started.set();finish.wait(2.);return N(res=True)
        o=proxy(Blocking());thread=threading.Thread(target=lambda:o._on_servo_request(N(req=1)))
        thread.start();self.assertTrue(started.wait(1.))
        o._on_alignment_context(context(o._permission))
        o._on_permission(permission());self.assertFalse(o._on_servo_request(N(req=1)).res)
        finish.set();thread.join(2.)
        self.assertEqual([m.execution_state for m in o._result_pub.messages],[RAW_CALL_STARTED,COMPLETED])
        self.assertEqual(o._raw_client.calls,1)
    def test_cancel_during_discovery_is_rechecked_before_raw_call(self):
        o=proxy()
        o._raw_client.wait_for_service=lambda timeout:o._on_alignment_context(context(o._permission))
        self.assertFalse(o._on_servo_request(N(req=1)).res)
        self.assertEqual(o._raw_client.calls,0)
    def test_proxy_releases_shared_lock_during_normal_one_second_raw_rpc(self):
        started=threading.Event()
        class Slow(Raw):
            def __call__(self,slot):
                self.calls+=1;started.set();time.sleep(1.);return N(res=True)
        o=proxy(Slow());thread=threading.Thread(target=lambda:o._on_servo_request(N(req=1)))
        thread.start();self.assertTrue(started.wait(1.))
        before=time.monotonic();o._on_permission(permission());elapsed=time.monotonic()-before
        thread.join(2.);self.assertLess(elapsed,.1)
        self.assertEqual(o._raw_client.calls,1)
    def test_fenced_companion_returns_exact_request_fact_and_rejects_foreign_identity(self):
        o=proxy();req=permission();req.request_id=41
        response=o._on_servo_action_request(req)
        self.assertEqual((response.request_id,response.payload_slot,response.res,
            response.execution_state,response.terminal),(41,1,True,COMPLETED,True))
        self.assertEqual(o._raw_client.calls,1)
        for field,value in (('mission_id','old'),('decision_seq',2),('attempt',2),
                            ('target_first_seen',Stamp(98.)),('target_id',9),
                            ('target_class','panzer'),('align_mode','drop_cross')):
            o=proxy();req=permission();req.request_id=42;setattr(req,field,value)
            response=o._on_servo_action_request(req)
            self.assertEqual(response.execution_state,NOT_STARTED,field)
            self.assertTrue(response.terminal);self.assertFalse(response.res)
            self.assertEqual(o._raw_client.calls,0)
    def test_fenced_unknown_result_cannot_unlock_same_slot_or_become_no_start(self):
        o=proxy(Raw(success=False));req=permission();req.request_id=43
        response=o._on_servo_action_request(req)
        self.assertEqual(response.execution_state,RAW_CALL_STARTED)
        o._on_permission(permission(seq=2));req=permission(seq=2);req.request_id=44
        response=o._on_servo_action_request(req)
        self.assertEqual(response.execution_state,EXECUTION_UNKNOWN)
        self.assertEqual(o._raw_client.calls,1)
    def test_old_revocation_does_not_cancel_new_action_on_same_slot(self):
        o=proxy();old=permission();o._on_alignment_context(context(old))
        current=permission(seq=2,header=N(stamp=Stamp(100.11)))
        Clock.value=100.11;o._on_permission(current)
        o._on_alignment_context(context(old))
        self.assertTrue(o._on_servo_request(N(req=1)).res)
        self.assertEqual(o._raw_client.calls,1)

    @staticmethod
    def waiting_request(o):
        waiting = threading.Event()
        original = o._permission_changed.wait
        def observe_wait(timeout):
            waiting.set()
            return original(timeout)
        o._permission_changed.wait = observe_wait
        req = copy.deepcopy(o._permission)
        req.request_id = 71
        answers = []
        thread = threading.Thread(target=lambda: answers.append(o._on_servo_action_request(req)))
        thread.start()
        if not waiting.wait(1.):
            thread.join(1.)
            raise AssertionError("request did not wait for clock/permission")
        return thread, answers

    def test_one_ms_future_permit_waits_without_revoking_confirmed_action(self):
        o = proxy()
        o._permission.header.stamp = Stamp(Clock.value + .001)
        thread, answers = self.waiting_request(o)
        self.assertEqual(o._raw_client.calls, 0)
        self.assertEqual(o._result_pub.messages, [])
        Clock.value += .001
        o._on_permission(copy.deepcopy(o._permission))
        thread.join(1.)
        self.assertFalse(thread.is_alive())
        self.assertTrue(answers[0].res)
        self.assertEqual(o._raw_client.calls, 1)
        self.assertEqual([m.execution_state for m in o._result_pub.messages],
                         [RAW_CALL_STARTED, COMPLETED])
        self.assertEqual(o._revoked_actions, set())

    def test_expired_same_action_can_refresh_without_reacquiring_target(self):
        o = proxy()
        o._permission.valid_until = Stamp(100.05)
        thread, answers = self.waiting_request(o)
        o._on_permission(permission(permission_revision=2))
        thread.join(1.)
        self.assertTrue(answers[0].res)
        self.assertEqual(o._raw_client.calls, 1)
        self.assertEqual([m.execution_state for m in o._result_pub.messages],
                         [RAW_CALL_STARTED, COMPLETED])

    def test_clock_never_catches_up_is_bounded_not_started(self):
        o = proxy()
        o._permission_refresh_wait = .02
        o._permission.header.stamp = Stamp(100.2)
        req = copy.deepcopy(o._permission); req.request_id = 72
        begin = time.monotonic()
        response = o._on_servo_action_request(req)
        self.assertLess(time.monotonic() - begin, .5)
        self.assertEqual(response.execution_state, NOT_STARTED)
        self.assertEqual(response.reason, "permission_clock_ahead")
        self.assertEqual(o._raw_client.calls, 0)
        self.assertEqual(o._locked_slots, set())

    def test_revoke_during_clock_wait_does_not_start(self):
        o = proxy()
        o._permission.header.stamp = Stamp(100.101)
        thread, answers = self.waiting_request(o)
        o._on_alignment_context(context(o._permission))
        thread.join(1.)
        self.assertFalse(thread.is_alive())
        self.assertFalse(answers[0].res)
        self.assertEqual(o._raw_client.calls, 0)
        self.assertTrue(all(m.execution_state == NOT_STARTED for m in o._result_pub.messages))

    def test_changed_identity_or_geometry_during_wait_is_rejected(self):
        for replacement in (permission(seq=2),
                            permission(permitted=False, reason="release_altitude_invalid")):
            o = proxy()
            o._permission.header.stamp = Stamp(100.101)
            thread, answers = self.waiting_request(o)
            o._on_permission(replacement)
            thread.join(1.)
            self.assertFalse(thread.is_alive())
            self.assertFalse(answers[0].res)
            self.assertEqual(o._raw_client.calls, 0)

    def test_discovery_uses_renewed_same_action_permission(self):
        o = proxy()
        o._permission.valid_until = Stamp(100.2)
        req = copy.deepcopy(o._permission); req.request_id = 73
        def discovery(timeout):
            Clock.value = 100.3
            new = permission(permission_revision=2)
            new.header.stamp = Stamp(100.3)
            o._on_permission(new)
        o._raw_client.wait_for_service = discovery
        response = o._on_servo_action_request(req)
        self.assertTrue(response.res)
        self.assertEqual(o._consumed_permission_stamp.to_sec(), 100.3)
        self.assertEqual(o._raw_client.calls, 1)

    def test_fresh_refresh_cannot_change_target_after_discovery(self):
        o = proxy()
        req = copy.deepcopy(o._permission); req.request_id = 74
        o._raw_client.wait_for_service = lambda timeout: o._on_permission(permission(seq=2))
        response = o._on_servo_action_request(req)
        self.assertFalse(response.res)
        self.assertEqual(o._raw_client.calls, 0)

    def test_legacy_false_means_unknown_and_true_means_completed(self):
        m=N(success=False);self.assertEqual(execution_fact(m),0)
        m.success=True;self.assertEqual(execution_fact(m),COMPLETED)

class BridgeReleaseTests(unittest.TestCase):
    def setUp(self): Clock.value=100.1
    def test_fixed_ack_accepts_older_image_after_new_context_and_commits_once(self):
        o=bridge();o._strict_context_matches=lambda m,now:True
        ctx=N(evidence=N(header=N(stamp=Stamp(100.045))))
        o._on_release_evidence_context(ctx)
        self.assertEqual(o._transaction.strict_evidence_stamp_ns,round(100.04*NS))
        msg=result(permission());o._on_release_result(msg);o._on_release_result(msg)
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.committed_slots,1)
        self.assertEqual(len([e for e in o.events if e.payload_committed]),1)
    def test_fixed_ack_rejects_foreign_mission_generation_slot_attempt_and_firstseen(self):
        for changes in (dict(mission_id='old'),dict(decision_seq=2),dict(payload_slot=2),
                        dict(attempt=2),dict(target_first_seen=Stamp(98.)),dict(target_id=8)):
            o=bridge();o._on_release_result(result(permission(**changes)))
            self.assertEqual(o.core.committed_slots,0)
            self.assertEqual(o.events,[])
    def test_incomplete_identity_never_falls_back_in_default_formal_chain(self):
        for fields in (dict(mission_id=''),dict(decision_seq=0),
                       dict(target_first_seen=Stamp(0)),dict(target_first_seen=None)):
            for fact in (NOT_STARTED,COMPLETED):
                o=bridge();o._on_release_result(result(permission(**fields),fact))
                self.assertEqual(o.events,[],(fields,fact))
                self.assertEqual(o.core.slots[0].status,SlotStatus.RESERVED)
                self.assertEqual(o.errors,[])
    def test_explicit_legacy_switch_only_accepts_unknown_old_format(self):
        for fact in (NOT_STARTED,COMPLETED):
            o=bridge();o._allow_legacy_release_results=True
            o._on_release_result(result(permission(mission_id=''),fact))
            self.assertEqual(o.events,[]);self.assertEqual(o.core.committed_slots,0)
        old=result(permission(mission_id=''));old.execution_state=EXECUTION_UNKNOWN
        old.terminal=False
        strict=bridge();strict._on_release_result(old)
        self.assertEqual(strict.events,[])
        compatible=bridge();compatible._allow_legacy_release_results=True
        compatible._on_release_result(old)
        self.assertEqual(compatible.errors,[]);self.assertEqual(compatible.core.committed_slots,1)

    def test_result_before_action_or_from_future_cannot_change_release_facts(self):
        for stamp in (99.9,101.0):
            o=bridge();o._on_release_result(result(permission(),stamp=stamp))
            self.assertEqual(o.events,[]);self.assertEqual(o.core.committed_slots,0)
    def test_late_no_start_for_old_action_cannot_unlock_new_generation(self):
        o=bridge();old=permission()
        o._on_release_result(result(old,NOT_STARTED))
        fresh=candidate(target_id=8,now=100.2)
        Clock.value=100.2;new=o.core.choose_confirmed(fresh,100.2,(0,0))
        self.assertIsNotNone(new)
        target=TargetIdentity(fresh.target_id,fresh.first_seen_ns,fresh.last_seen_ns,
                             fresh.class_name,new.attempt,new.payload_slot)
        d=MotionDecision('mission-1',new.decision_seq,round(new.issued_at*NS),
            round(new.deadline_at*NS),'APPROACH','r2026',MotionGoal('camera_init',1.,0.,1.2),target)
        o._executor.submit_decision(d,round(100.2*NS));o._executor._active.handed_off=True
        o._transaction=BR['TargetTransaction'](decision=d,phase='ALIGNMENT',align_mode='drop_circle')
        count=len(o.events);o._on_release_result(result(old,NOT_STARTED))
        self.assertEqual(o.core.slots[0].status,SlotStatus.RESERVED)
        self.assertEqual(o.core.active_action.decision_seq,new.decision_seq)
        self.assertEqual(len(o.events),count)

    def test_not_started_denial_returns_reserved_slot_without_quarantine(self):
        o=bridge();o._on_release_result(result(permission(),NOT_STARTED))
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.slots[0].status,SlotStatus.FREE)
        self.assertEqual(o._transaction.phase,'TERMINAL')
    def test_started_then_terminal_unknown_quarantines_and_stale_notstarted_cannot_clear(self):
        o=bridge();o._on_release_result(result(permission(),RAW_CALL_STARTED,False))
        o._on_release_result(result(permission(),RAW_CALL_STARTED))
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.slots[0].status,SlotStatus.QUARANTINED)
        o._on_release_result(result(permission(),NOT_STARTED))
        self.assertEqual(o.core.slots[0].status,SlotStatus.QUARANTINED)
    def test_delayed_success_after_timeout_reconciles_same_fixed_action(self):
        o=bridge();o._on_release_result(result(permission(),RAW_CALL_STARTED,False))
        Clock.value=191.;o.core.expire_active(Clock.value)
        o._expire_handoff_if_due(round(Clock.value*NS))
        o._on_release_result(result(permission(),stamp=100.2))
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.committed_slots,1)
    def test_timeout_then_proxy_cancel_proof_reclaims_uncalled_slot(self):
        o=bridge();o.core._observe_target_stage(result_for(o.core.active_action,1,
            stage='ALIGNMENT',reason='strict_alignment_context_valid'))
        Clock.value=191.;o.core.expire_active(Clock.value)
        o._expire_handoff_if_due(round(Clock.value*NS))
        o._on_release_result(result(permission(),NOT_STARTED))
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.slots[0].status,SlotStatus.FREE)

class ArbiterTests(unittest.TestCase):
    def fixture(self):
        o=ARB.__new__(ARB);o._lock=threading.RLock()
        m=permission();o._commitment=N(mission_id=m.mission_id,decision_seq=m.decision_seq,
            attempt=m.attempt,payload_slot=m.payload_slot,target_id=m.target_id,
            target_first_seen_nsec=m.target_first_seen.to_nsec(),target_class=m.target_class)
        o._blocked_slots,o._completed_slots,o._released_targets,o._revoked_actions=set(),set(),set(),set()
        o._called_actions={};o._authorized_actions={};o._next_slot=1;o._permission_state_pub=Publisher()
        o._remember_authorization(m)
        return o
    def test_unknown_locks_slot_without_stealing_positive_execution_identity(self):
        o=self.fixture();m=permission()
        o._on_result(result(m,EXECUTION_UNKNOWN,execution_id=10))
        self.assertEqual(o._blocked_slots,{1})
        self.assertIsNone(o._authorized_actions[action_identity(m)]["execution_id"])
        o._on_result(result(m,RAW_CALL_STARTED,False,execution_id=11))
        o._on_result(result(m,NOT_STARTED,execution_id=12))
        self.assertEqual(o._authorized_actions[action_identity(m)]["execution_id"],11)
        o._on_result(result(m,execution_id=11))
        self.assertEqual(o._next_slot,2)

    def test_negative_result_id_cannot_mask_late_real_execution(self):
        o=self.fixture();m=permission()
        o._on_result(result(m,NOT_STARTED,execution_id=10))
        self.assertIn(action_identity(m),o._revoked_actions)
        self.assertIsNone(o._authorized_actions[action_identity(m)]["execution_id"])
        o._on_result(result(m,RAW_CALL_STARTED,False,execution_id=11))
        o._on_alignment_context(context(m))
        o._on_result(result(m,execution_id=11))
        self.assertEqual(o._next_slot,2)

    def test_real_publish_path_retains_authorization_after_end(self):
        o=self.fixture();o._authorized_actions.clear();m=permission()
        o._permission_epoch='arbiter-test';o._permission_revision=0
        o._authorization_pub=Publisher()
        o._require_evidence_context=False;o._permission_lifetime=.25
        o._align_mode=m.align_mode;o._permission_pub=Publisher()
        o._evaluate=lambda now:(True,"permission_granted",dict(
            target_id=m.target_id,target_class=m.target_class,
            evidence_stamp=m.evidence_stamp,mission_id=m.mission_id,
            decision_seq=m.decision_seq,attempt=m.attempt,target_first_seen=m.target_first_seen))
        old_duration=getattr(ROS,"Duration",None);ROS.Duration=Stamp
        try:o._publish_permission(None)
        finally:
            if old_duration is None:del ROS.Duration
            else:ROS.Duration=old_duration
        self.assertTrue(o._permission_pub.messages[-1].permitted)
        o._on_alignment_context(context(m))
        o._on_result(result(m,RAW_CALL_STARTED,False,execution_id=30))
        o._on_result(result(m,execution_id=30))
        self.assertEqual(o._next_slot,2)

    def test_all_start_end_success_orders_advance_exactly_once(self):
        from itertools import permutations
        for order in permutations(("start", "end", "success")):
            with self.subTest(order=order):
                o=self.fixture();m=permission()
                for event in order:
                    if event=="end":o._on_alignment_context(context(m))
                    else:o._on_result(result(m,RAW_CALL_STARTED if event=="start" else COMPLETED,
                                             terminal=event=="success",execution_id=19))
                o._on_result(result(m,execution_id=19))
                self.assertEqual(o._next_slot,2)
                self.assertEqual(o._completed_slots,{1})
                self.assertIn(action_identity(m),o._revoked_actions)
                self.assertFalse(o._permission_state_pub.messages[-1].data)

    def test_three_slots_with_end_before_results(self):
        o=self.fixture()
        for slot in (1,2,3):
            m=permission(seq=slot,payload_slot=slot,target_id=slot+20)
            o._remember_authorization(m)
            o._on_alignment_context(context(m))
            o._on_result(result(m,RAW_CALL_STARTED,False,execution_id=100+slot))
            o._on_result(result(m,execution_id=100+slot))
            o._on_result(result(m,execution_id=100+slot))
            self.assertEqual(o._next_slot,slot+1)
        self.assertEqual(o._completed_slots,{1,2,3})

    def test_commitment_without_published_permission_cannot_authorize_result(self):
        o=self.fixture();o._authorized_actions.clear()
        o._on_result(result(permission()))
        self.assertEqual(o._next_slot,1)
        self.assertEqual(o._blocked_slots,set())

    def test_denied_permission_is_not_historical_authorization(self):
        o=self.fixture();o._authorized_actions.clear()
        m=permission(permitted=False);o._remember_authorization(m)
        o._on_alignment_context(context(m));o._on_result(result(m))
        self.assertEqual(o._next_slot,1)

    def test_execution_id_and_mode_must_match_authorized_action(self):
        for change in (dict(execution_id=21),dict(align_mode="drop_red_cross")):
            o=self.fixture();m=permission()
            o._on_alignment_context(context(m))
            o._on_result(result(m,RAW_CALL_STARTED,False,execution_id=20))
            msg=result(m,execution_id=20)
            for name,value in change.items():setattr(msg,name,value)
            o._on_result(msg)
            self.assertEqual(o._next_slot,1)
            o._on_result(result(m,execution_id=20))
            self.assertEqual(o._next_slot,2)

    def test_late_completion_does_not_delete_different_current_commitment(self):
        o=self.fixture();old=permission()
        o._on_alignment_context(context(old))
        new=N(mission_id="new",decision_seq=9,attempt=1,payload_slot=2,
              target_id=8,target_first_seen_nsec=99*NS,target_class="panzer")
        o._commitment=new
        o._on_result(result(old))
        self.assertIs(o._commitment,new)
        self.assertEqual(o._next_slot,2)

    def test_uncertain_failure_blocks_permission_and_does_not_advance_slot(self):
        o=self.fixture();o._on_result(result(permission(),RAW_CALL_STARTED,False))
        self.assertEqual(o._blocked_slots,{1});self.assertEqual(o._next_slot,1)
    def test_not_started_denial_neither_locks_nor_consumes_slot(self):
        o=self.fixture();o._on_result(result(permission(),NOT_STARTED))
        self.assertEqual(o._blocked_slots,set());self.assertEqual(o._next_slot,1)
    def test_completed_call_survives_context_revocation_and_duplicate_does_not_advance(self):
        o=self.fixture();o._on_result(result(permission(),RAW_CALL_STARTED,False))
        o._on_alignment_context(context(permission()))
        o._on_result(result(permission()));o._on_result(result(permission()))
        self.assertEqual(o._next_slot,2);self.assertEqual(o._completed_slots,{1})
    def test_nonterminal_completed_is_not_a_completion_fact(self):
        o=self.fixture();o._on_result(result(permission(),COMPLETED,False))
        self.assertEqual(o._next_slot,1);self.assertEqual(o._completed_slots,set())
    def test_late_no_start_after_start_cannot_unblock_permission(self):
        o=self.fixture();o._on_result(result(permission(),RAW_CALL_STARTED,False))
        o._on_result(result(permission(),NOT_STARTED))
        self.assertEqual(o._blocked_slots,{1});self.assertEqual(o._next_slot,1)

    def test_strict_arbiter_rejects_incomplete_fence(self):
        for fields in (dict(mission_id=''),dict(decision_seq=0),dict(target_first_seen=Stamp(0))):
            o=self.fixture();o._require_evidence_context=True
            o._on_result(result(permission(**fields)))
            self.assertEqual(o._blocked_slots,set());self.assertEqual(o._next_slot,1)

    def test_foreign_result_does_not_lock_current_slot(self):
        o=self.fixture();o._on_result(result(permission(seq=3),RAW_CALL_STARTED))
        self.assertEqual(o._blocked_slots,set())

class CoreFollowupTests(unittest.TestCase):
    def test_fixed_reacquired_identity_cannot_be_preempted_by_stale_higher_weight(self):
        o=core();red=candidate(1,'red_cross',now=100.)
        o.ingest([red],100.)
        pan=candidate(7,'panzer',now=128.)
        a=o.choose_confirmed(pan,128.,(0,0))
        self.assertEqual(a.candidate_key,pan.key);self.assertEqual(a.target_class,'panzer')
    def test_fixed_reacquire_rejects_stale_cooldown_and_invalidated_candidates(self):
        for mode in ('stale','invalidated','cooldown'):
            o=core();c=candidate(7,now=100.);o.ingest([c],100.)
            if mode=='invalidated':o.queue.invalidate(c.key,'class_disproved')
            if mode=='cooldown':
                a=o.choose_confirmed(c,100.,(0,0))
                o.apply_result(result_for(a,1,status='FAILED',stage='CAPTURE',terminal=True,
                    retryable=True),100.1)
            self.assertIsNone(o.choose_confirmed(c,101. if mode=='stale' else 100.2,(0,0)))
    def test_capture_deadline_and_abort_without_execution_do_not_quarantine(self):
        for operation in ('deadline','abort'):
            o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
            if operation=='deadline':o.expire_active(a.deadline_at)
            else:o.abort('map_stale',101.)
            self.assertEqual(o.slots[0].status,SlotStatus.FREE)
    def test_timeout_without_signal_only_frees_before_strict_permission(self):
        for permitted in (False,True):
            o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
            if permitted:
                accepted,reason,_=o.apply_result(result_for(a,1,status='STARTED',
                    stage='ALIGNMENT',reason='strict_alignment_context_valid'),100.1)
                self.assertTrue(accepted,reason)
            o.expire_active(a.deadline_at)
            self.assertEqual(o.slots[0].status,
                SlotStatus.QUARANTINED if permitted else SlotStatus.FREE)
    def test_negative_alignment_timeout_after_permit_is_not_no_start_proof(self):
        o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
        o.apply_result(result_for(a,1,status='STARTED',stage='ALIGNMENT',
            reason='strict_alignment_context_valid'),100.1)
        accepted,reason,_=o.apply_result(result_for(a,2,status='TIMED_OUT',
            stage='ALIGNMENT',terminal=True,retryable=True),100.2)
        self.assertTrue(accepted,reason)
        self.assertEqual(o.slots[0].status,SlotStatus.QUARANTINED)
    def test_late_start_fact_after_timeout_blocks_late_negative_proof(self):
        o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
        o.apply_result(result_for(a,1,status='STARTED',stage='ALIGNMENT',
            reason='strict_alignment_context_valid'),100.1)
        o.expire_active(a.deadline_at)
        admitted=result_for(a,2,status='STARTED',stage='RELEASE',
                            reason='raw_actuator_call_started')
        self.assertTrue(o.apply_result(admitted,a.deadline_at+.1)[0])
        proof=replace(result_for(a,3,status='FAILED',stage='RELEASE',terminal=True),
                      evidence_source='guarded_servo_proxy:1:NOT_STARTED')
        self.assertTrue(o.apply_result(proof,a.deadline_at+.2)[0])
        self.assertEqual(o.slots[0].status,SlotStatus.QUARANTINED)
    def test_nonterminal_no_start_label_does_not_free_quarantined_slot(self):
        o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
        o.apply_result(result_for(a,1,status='STARTED',stage='ALIGNMENT',
            reason='strict_alignment_context_valid'),100.1);o.expire_active(a.deadline_at)
        fake=replace(result_for(a,2,status='PROGRESS',stage='ALIGNMENT'),
                     evidence_source='guarded_servo_proxy:1:NOT_STARTED')
        o.apply_result(fake,a.deadline_at+.1)
        self.assertEqual(o.slots[0].status,SlotStatus.QUARANTINED)

    def test_abort_after_call_started_is_uncertain_and_rejects_late_no_start(self):
        o=core();a=o.choose_confirmed(candidate(7,now=100.),100.,(0,0))
        o.apply_result(result_for(a,1,status='STARTED',stage='RELEASE'),100.1)
        o.abort('map_stale',100.2)
        proof=replace(result_for(a,2,status='FAILED',stage='RELEASE',terminal=True),
                      evidence_source='guarded_servo_proxy:1:NOT_STARTED')
        o.apply_result(proof,100.3)
        self.assertEqual(o.slots[0].status,SlotStatus.QUARANTINED)
    def test_alignment_timeout_exactly_at_mission_deadline_does_not_waste_slot(self):
        o=core();c=candidate(7,now=699.);a=o.choose_confirmed(c,699.,(0,0))
        o.apply_result(result_for(a,1,status='TIMED_OUT',stage='ALIGNMENT',terminal=True,
            retryable=True,event_time=700.),700.)
        self.assertEqual(o.slots[0].status,SlotStatus.FREE)

class ContradictionTests(unittest.TestCase):
    def setUp(self): Clock.value=100.1
    def target(self,t,label='pillbox',x=1.,confidence=.95,valid=True):
        return N(last_seen=Stamp(t),map_point=N(x=x,y=0.,z=0.),map_valid=valid,
            association_valid=True,reject_reason='',state=2,map_frame='camera_init',
            class_confidence=confidence,class_name=label)
    def test_persistent_same_place_conflict_revokes_and_waits_for_atomic_proxy_proof(self):
        o=bridge()
        for t in (100.1,100.2,100.3):
            Clock.value=t;o._latest_candidates=(self.target(t),);o._check_alignment_contradiction(round(t*NS))
        self.assertEqual(o._transaction.phase,'CANCEL_PENDING')
        self.assertEqual(o.context_events,[False]);self.assertEqual(o.core.slots[0].status,SlotStatus.RESERVED)
        o._on_release_result(result(permission(),NOT_STARTED))
        self.assertEqual(o.errors,[]);self.assertEqual(o.core.slots[0].status,SlotStatus.FREE)
    def test_single_repeat_stale_neighbor_low_confidence_dropout_never_revokes(self):
        for mode in ('repeat','stale','neighbor','low_confidence','dropout','unconfirmed','foreign_frame'):
            o=bridge()
            for t in (100.1,100.2,100.3,100.4):
                Clock.value=t
                item=self.target(100.1 if mode=='repeat' else 98. if mode=='stale' else t,
                    x=1.7 if mode=='neighbor' else 1.,confidence=.8 if mode=='low_confidence' else .95)
                if mode=='unconfirmed':item.state=1
                if mode=='foreign_frame':item.map_frame='other'
                o._latest_candidates=() if mode=='dropout' else (item,)
                o._check_alignment_contradiction(round(t*NS))
            self.assertEqual(o._transaction.phase,'ALIGNMENT',mode)
    def test_strong_new_support_resets_conflicting_streak(self):
        o=bridge()
        for t,label in ((100.1,'pillbox'),(100.2,'bridge'),(100.3,'pillbox'),(100.4,'pillbox')):
            Clock.value=t;o._latest_candidates=(self.target(t,label),)
            o._check_alignment_contradiction(round(t*NS))
        self.assertEqual(o._transaction.phase,'ALIGNMENT')
    def test_call_started_never_triggers_notstarted_revocation(self):
        o=bridge();o._transaction.raw_call_observed=True
        for t in (100.1,100.2,100.3):
            o._latest_candidates=(self.target(t),);o._check_alignment_contradiction(round(t*NS))
        self.assertEqual(o.context_events,[])

class LandingTests(unittest.TestCase):
    def setUp(self): Clock.value=100.1
    def landing(self):
        o=bridge();o._transaction=None
        d=MotionDecision('land',8,100*NS,110*NS,'LAND','r2026')
        o._executor=PlannerMotionExecutor();o._executor.submit_decision(d,100*NS)
        o._landing=BR['LandingTransaction'](d,BR['SemanticTargetPose']('camera_init',1.,0.,0.,100*NS),100*NS,True)
        o._flight_state=N(connected=True,armed=False,mode='OFFBOARD',header=N(stamp=Stamp(100.1)))
        o._flight_state_source_ns=o._landed_state_source_ns=round(100.1*NS)
        o._flight_state_receipt_ns=round(100.1*NS)
        o._control_state=0;o._control_state_receipt_ns=round(100.1*NS)
        o._landed_state=EXT.LANDED_STATE_ON_GROUND;o._landed_state_receipt_ns=round(100.1*NS)
        o._apply_outcome=lambda out:o.events.extend(out.events)
        return o
    def sample(self,t,z=.7,x=1.):return OdomSample(round(t*NS),'camera_init',x,0.,z,0.,0.,0.)
    def test_fresh_disarmed_on_ground_settles_after_controller_exits_land(self):
        o=self.landing()
        for t in (100.1,100.2,100.3):
            o._flight_state_receipt_ns=o._landed_state_receipt_ns=round(t*NS)
            o._update_landing(self.sample(t),round(t*NS))
        self.assertIsNone(o._landing)
        self.assertEqual(o.events[-1].status,'SUCCEEDED')
    def test_disconnect_stale_ground_airborne_unaccepted_land_or_motion_never_completes(self):
        for mode in ('disconnected','stale_ground','airborne','unaccepted','moving','stale_state','stale_ground_source','stale_state_source'):
            o=self.landing()
            if mode=='disconnected':o._flight_state.connected=False
            if mode=='airborne':o._landed_state=EXT.LANDED_STATE_IN_AIR
            if mode=='unaccepted':o._landing.started=False
            if mode=='stale_ground':o._landed_state_receipt_ns=97*NS
            if mode=='stale_state':o._flight_state_receipt_ns=97*NS
            if mode=='stale_ground_source':o._landed_state_source_ns=97*NS
            if mode=='stale_state_source':o._flight_state_source_ns=97*NS
            for i,t in enumerate((100.1,100.2,100.3,100.4)):
                o._update_landing(self.sample(t,x=1.+i*.1 if mode=='moving' else 1.),round(t*NS))
            self.assertIsNotNone(o._landing,mode)
            self.assertFalse(any(e.status=='SUCCEEDED' for e in o.events),mode)
    def test_airborne_manual_takeover_cannot_become_success_after_later_disarm(self):
        o=self.landing();o._landed_state=EXT.LANDED_STATE_IN_AIR
        o._on_flight_state(N(connected=True,armed=True,mode='POSCTL',header=N(stamp=Stamp(Clock.value))))
        self.assertIsNone(o._landing);self.assertEqual(o.events[-1].status,'CANCELLED')
        o._on_flight_state(N(connected=True,armed=False,mode='OFFBOARD',header=N(stamp=Stamp(Clock.value))))
        o._landed_state=EXT.LANDED_STATE_ON_GROUND
        o._update_landing(self.sample(100.3),round(100.3*NS))
        self.assertFalse(any(e.status=='SUCCEEDED' for e in o.events))
    def test_armed_on_ground_still_requires_land_control_and_height(self):
        o=self.landing();o._flight_state.armed=True;o._control_state=3
        for t in (100.1,100.2,100.3):o._update_landing(self.sample(t),round(t*NS))
        self.assertIsNotNone(o._landing)

if __name__=='__main__':unittest.main()
