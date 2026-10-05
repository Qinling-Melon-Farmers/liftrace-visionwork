"""Shared stage parameter handoff for simulation and board high-view managers."""
import math
import rospy
from .mission_core import MissionPhase


class HighViewStageMixin:
    def _height_stage_ready(self,action):
        if not super()._height_stage_ready(action):return False
        if self._runtime is None or not hasattr(self._runtime,'stage') or action.command in ('ABORT','HOLD'):return True
        stage=self._runtime.stage
        if stage in ('REVISIT','REACQUIRE','DELIVERY','LOW_COVERAGE','LOCAL_WALL_VERIFY'):
            low=True
        elif stage in ('SURVEY','RESUME_ASCEND','RESUME_JOIN','RETURN_COLUMN','LOCAL_DESCENT_TRANSIT','DESCEND'):
            low=False
        else:return True
        now=rospy.Time.now().to_sec()
        if now>=min(action.deadline_at,self._runtime.core.started_at+self._runtime.core.config.mission_timeout):
            self._last_reason='probe_parameter_stage_deadline';return False
        if low==self._low_limits_applied and self._probe_limit_switch is None and stage not in ('RESUME_ASCEND','RESUME_JOIN'):return True
        limits=rospy.get_param('~high_view_probe/low_stage_parameters',[])
        if not limits:return not getattr(self._runtime,'resume_attempted',False)
        if self._high_stage_parameters is None:
            # Capture before the first reduction, including a disabled virtual
            # ceiling. Do not invent a high cap or re-enable a field ceiling.
            self._high_stage_parameters=[dict(name=x['name'],value=rospy.get_param(x['name'])) for x in limits]
        parameters=limits if low else self._high_stage_parameters
        ns=getattr(self,'_height_namespace','/navigation_height_constraint')
        enabled=rospy.get_param(ns+'/enabled',False)
        if not enabled and stage in ('RESUME_ASCEND','RESUME_JOIN'):
            self._last_reason='resume_requires_planner_height_constraint'
            return False
        cap_name=rospy.get_param(ns+'/limit_parameter','/external_planner_max_command_z')
        cap=next((float(x['value']) for x in parameters if x['name']==cap_name),None)
        if enabled and cap is None:
            self._last_reason='probe_height_limit_parameter_missing'
            return False
        if cap is not None:
            if not math.isfinite(cap):raise ValueError('nonfinite probe height cap')
            if (self._pose is None or self._pose.header.frame_id!=self._runtime.core.config.mission_frame
                    or not 0<=now-self._pose.header.stamp.to_sec()<=self._pose_max_age
                    or self._pose.pose.position.z>cap+1e-9):
                self._last_reason='waiting_below_probe_height_limit';return False
        # A retry belongs to this transition only while its full parameter
        # set and frame remain unchanged, including non-height parameters.
        key=(self._runtime.core.mission_id,action.decision_seq,low,ns,
             self._runtime.core.config.mission_frame,
             tuple((x['name'],x['value']) for x in parameters))
        switch=self._probe_limit_switch
        if switch is None or switch['key']!=key:
            for x in parameters:rospy.set_param(x['name'],x['value'])
            if any(rospy.get_param(x['name'])!=x['value'] for x in parameters):
                raise RuntimeError('probe stage parameter readback failed')
            switch=dict(key=key,request=None,last_sent=None);self._probe_limit_switch=switch
        if enabled and cap is not None:
            request=switch['request']
            if request is None:
                # Retain the first-request timestamp: a delayed ACK must not
                # be invalidated by retransmitting the same transition.
                serial=getattr(self,'_probe_limit_request_serial',0)+1
                self._probe_limit_request_serial=serial
                request=dict(id='%s:probe:%s:%s:%.9f'%(key[0],key[1],serial,now),
                             max_z=cap,frame=self._runtime.core.config.mission_frame,stamp=now)
                switch['request']=request
            if switch['last_sent'] is None or not 0<=now-switch['last_sent']<=.4:
                # Only an explicit send advances this nonce. The planner can
                # refresh an aged/lost ACK without periodic parameter writes.
                nonce=getattr(self,'_probe_limit_resend_nonce',0)+1
                self._probe_limit_resend_nonce=nonce
                request=dict(request,resend_nonce=nonce);switch['request']=request
                rospy.set_param(ns+'/request',request);switch['last_sent']=now
            ack=rospy.get_param(ns+'/ack',{})
            stamp=float(ack.get('stamp',-1.));applied=float(ack.get('max_z',float('nan')))
            if (ack.get('id')!=request['id'] or ack.get('frame')!=request['frame']
                    or not math.isfinite(applied) or abs(applied-cap)>1e-9
                    or stamp<request['stamp'] or not 0<=now-stamp<=.5):
                self._last_reason='waiting_probe_height_constraint_ack';return False
        self._low_limits_applied=low;self._probe_limit_switch=None
        return True

    def _on_timer(self,event):
        # A map grace period may still be executing the previous leg. Its stop
        # deadline must run outside general map/pose readiness and ACK waits.
        with self._lock:
            runtime=self._runtime
            live=runtime is not None and runtime.core.phase not in (MissionPhase.COMPLETE,MissionPhase.ABORTED)
            pending=getattr(self,'_pending_motion_action',None)
            motion_until=getattr(runtime,'descent_motion_until',None)
            if live and motion_until is not None:
                active=getattr(runtime.core,'active_action',None)
                continuing=active is not None and active.decision_seq==runtime.descent_motion_seq
                if not continuing and pending is None:
                    # The old leg finished, or a successor was actually published.
                    runtime.descent_motion_until=None;runtime.descent_motion_seq=None
                elif rospy.Time.now().to_sec()>=motion_until:
                    now=rospy.Time.now().to_sec()
                    # Fresh-map fallback gets one immediate attempt to publish
                    # its replacement. A blocked handoff must stop the old leg.
                    out=(runtime._wait_for_descent(now,runtime.descent_wait_mode) if continuing
                         else runtime._finish(False,'descent_motion_handoff_timeout',now))
                    if out.action is not None and out.action.command not in ('ABORT','HOLD') and not self._readiness(now)[0]:
                        out=runtime._finish(False,'descent_motion_handoff_timeout',now)
                    self._publish_action(out.action)
                    if getattr(self,'_pending_motion_action',None) is not None:
                        out=runtime._finish(False,'descent_motion_handoff_timeout',now)
                        self._publish_action(out.action)
                    if getattr(self,'_pending_motion_action',None) is None:
                        runtime.descent_motion_until=None;runtime.descent_motion_seq=None
                    self._publish_status(force=True);return
            if (live and pending is None and getattr(runtime,'resume_attempted',False)
                    and not runtime.resume_completed and runtime.resume_until is not None
                    and runtime.stage in ('RESUME_ASCEND','RESUME_JOIN','SURVEY')
                    and getattr(runtime.core,'active_action',None) is not None
                    and rospy.Time.now().to_sec()>=runtime.resume_until):
                out=runtime._resume_failed(rospy.Time.now().to_sec(),'resume_budget_exhausted')
                if out.action is not None and out.action.command in ('ABORT','HOLD'):
                    self._publish_action(out.action);self._publish_status(force=True);return
                # Retain the existing pose/map/ACK admission path; the motion
                # watchdog above bounds a replacement that cannot be published.
                self._pending_motion_action=out.action
            if (runtime is not None and runtime.core.phase not in (MissionPhase.COMPLETE,MissionPhase.ABORTED)
                    and getattr(runtime,'stage',None)=='DESCENT_WAIT'):
                now=rospy.Time.now().to_sec()
                if now>=runtime.core.started_at+runtime.core.config.mission_timeout:
                    out=runtime.abort('descent_wait_mission_deadline',now)
                    self._publish_action(out.action);self._publish_status(force=True);return
                if runtime.descent_wait_until is not None and now>=runtime.descent_wait_until:
                    out=runtime._wait_for_descent(now,runtime.descent_wait_mode)
                    if out.action is not None:
                        if out.action.command in ('ABORT','HOLD'):
                            self._publish_action(out.action);self._publish_status(force=True);return
                        # Do not bypass fresh map/frame readiness on fallback.
                        self._pending_motion_action=out.action
            pending=getattr(self,'_pending_motion_action',None)
            if (runtime is not None and getattr(runtime,'stage',None) in ('RESUME_ASCEND','RESUME_JOIN')
                    and pending is not None and runtime.resume_until is not None
                    and rospy.Time.now().to_sec()>=min(runtime.resume_until,pending.deadline_at)):
                now=rospy.Time.now().to_sec()
                if runtime._route_binding_matches(pending):
                    runtime.route.interrupt(pending.decision_seq);runtime.core.active_action=None
                    self._pending_motion_action=None;self._probe_limit_switch=None
                    self._low_limits_applied=None  # Re-apply low limits after an unacknowledged raise.
                    out=runtime._resume_failed(now,'resume_constraint_wait_timeout')
                    if out.action is not None:
                        if out.action.command in ('ABORT','HOLD'):
                            self._publish_action(out.action);self._publish_status(force=True);return
                        self._pending_motion_action=out.action
        super()._on_timer(event)
