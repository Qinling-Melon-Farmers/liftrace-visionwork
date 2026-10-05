"""Real proxy admission methods under reordered permission transport; no hardware."""
import copy
import threading
import time
import unittest
import test_release_transaction_followups as t

class RevisionTests(unittest.TestCase):
    def setUp(self): t.Clock.value=100.1
    def request(self, **changes):
        m=t.permission(permission_revision=18, **changes);m.request_id=900;return m
    def wait_request(self, proxy, req):
        answers=[]
        worker=threading.Thread(target=lambda: answers.append(proxy._on_servo_action_request(req)))
        worker.start();time.sleep(.01)
        self.assertEqual(proxy._raw_client.calls,0)
        self.assertTrue(worker.is_alive())
        return worker,answers
    def test_arbiter_publishes_same_explicit_revision_to_both_consumers(self):
        p=t.ArbiterTests().fixture();m=self.request()
        p._permission_epoch='one-instance';p._permission_revision=0
        p._authorization_pub=t.Publisher();p._permission_pub=t.Publisher()
        p._require_evidence_context=False;p._permission_lifetime=.25;p._align_mode=m.align_mode
        p._evaluate=lambda now:(True,'permission_granted',dict(
            target_id=m.target_id,target_class=m.target_class,evidence_stamp=m.evidence_stamp,
            mission_id=m.mission_id,decision_seq=m.decision_seq,attempt=m.attempt,
            target_first_seen=m.target_first_seen))
        old=getattr(t.ROS,'Duration',None);t.ROS.Duration=t.Stamp
        try:
            p._publish_permission(None);p._publish_permission(None)
        finally:
            if old is None:del t.ROS.Duration
            else:t.ROS.Duration=old
        self.assertEqual([x.permission_revision for x in p._permission_pub.messages],[1,2])
        for full,control in zip(p._permission_pub.messages,p._authorization_pub.messages):
            self.assertEqual(full.permission_epoch,control.permission_epoch)
            self.assertEqual(full.permission_revision,control.permission_revision)
            self.assertEqual(t.action_identity(full),t.action_identity(control))
            self.assertEqual(full.valid_until.to_nsec(),control.valid_until.to_nsec())
            self.assertEqual(full.header.stamp.to_nsec(),control.header.stamp.to_nsec())
            self.assertEqual(full.permitted,control.permitted)

    def test_older_denial_then_requested_grant_completes_without_exiting_action(self):
        p=t.proxy();p._permission=t.permission(permitted=False,reason='no_release_commitment',permission_revision=17)
        req=self.request();worker,answers=self.wait_request(p,req)
        self.assertEqual(p._result_pub.messages,[])
        p._on_permission(copy.deepcopy(req));worker.join(1)
        self.assertFalse(worker.is_alive());self.assertTrue(answers[0].res)
        self.assertEqual(p._raw_client.calls,1)
        self.assertEqual([m.execution_state for m in p._result_pub.messages],[t.RAW_CALL_STARTED,t.COMPLETED])
        self.assertFalse(p._on_servo_action_request(req).res)
        self.assertEqual(p._raw_client.calls,1)
    def test_missing_permission_waits_for_requested_or_newer_same_action(self):
        for revision in (18,19):
            p=t.proxy();p._permission=None;worker,answers=self.wait_request(p,self.request())
            p._on_permission(t.permission(permission_revision=revision));worker.join(1)
            self.assertTrue(answers[0].res);self.assertEqual(p._raw_client.calls,1)
    def test_current_and_newer_denial_are_not_treated_as_transport_delay(self):
        for revision in (18,19):
            p=t.proxy();p._permission=t.permission(permission_revision=revision,permitted=False,reason='no_release_commitment')
            before=time.monotonic();answer=p._on_servo_action_request(self.request())
            self.assertLess(time.monotonic()-before,.05)
            self.assertEqual(answer.reason,'no_release_commitment')
            self.assertEqual(answer.execution_state,t.NOT_STARTED)
            self.assertEqual(p._raw_client.calls,0)
    def test_new_denial_while_waiting_wins(self):
        p=t.proxy();worker,answers=self.wait_request(p,self.request())
        p._on_permission(t.permission(permission_revision=19,permitted=False,reason='release_altitude_invalid'))
        worker.join(1);self.assertEqual(answers[0].reason,'release_altitude_invalid')
        self.assertEqual(p._raw_client.calls,0)
    def test_old_same_epoch_messages_cannot_replace_newer_denial(self):
        p=t.proxy();denial=t.permission(permission_revision=19,permitted=False,reason='release_altitude_invalid')
        p._on_permission(denial);p._on_permission(self.request())
        self.assertIs(p._permission,denial)
        self.assertEqual(p._on_servo_action_request(self.request()).reason,'release_altitude_invalid')
        self.assertEqual(p._raw_client.calls,0)
    def test_bounded_timeout_has_positive_no_start_fact_not_a_raw_call(self):
        p=t.proxy();p._permission_refresh_wait=.025
        p._permission=t.permission(permission_revision=17,permitted=False,reason='no_release_commitment')
        begin=time.monotonic();answer=p._on_servo_action_request(self.request())
        self.assertLess(time.monotonic()-begin,.15)
        self.assertEqual(answer.reason,'permission_revision_not_received')
        self.assertEqual(answer.execution_state,t.NOT_STARTED);self.assertEqual(p._locked_slots,set())
        self.assertEqual(p._raw_client.calls,0)
    def test_restart_epoch_cannot_authorize_an_old_request(self):
        p=t.proxy();worker,answers=self.wait_request(p,self.request())
        p._on_permission(t.permission(permission_revision=1,permission_epoch='restarted'))
        worker.join(1);self.assertEqual(answers[0].reason,'permission_epoch_changed')
        self.assertEqual(p._raw_client.calls,0)
    def test_revocation_during_revision_wait_remains_immediate(self):
        p=t.proxy();req=self.request();worker,answers=self.wait_request(p,req)
        p._on_alignment_context(t.context(req));worker.join(1)
        self.assertEqual(answers[0].reason,'alignment_action_revoked');self.assertEqual(p._raw_client.calls,0)
    def test_missing_token_or_wrong_target_never_actuates(self):
        for changes in ({'permission_epoch':''},{'permission_revision':0},{'target_id':90}):
            p=t.proxy();p._permission=self.request();req=copy.deepcopy(p._permission)
            for k,v in changes.items():setattr(req,k,v)
            answer=p._on_servo_action_request(req)
            self.assertFalse(answer.res);self.assertEqual(p._raw_client.calls,0)
    def test_new_denial_during_service_discovery_cannot_be_bypassed(self):
        p=t.proxy();p._permission=self.request()
        def discovery(timeout):p._on_permission(t.permission(permission_revision=19,permitted=False,reason='release_altitude_invalid'))
        p._raw_client.wait_for_service=discovery
        answer=p._on_servo_action_request(self.request())
        self.assertFalse(answer.res);self.assertEqual(p._raw_client.calls,0)

if __name__=='__main__':unittest.main()