"""Opt-in full mission: survey all/early top3, sensed-grid ordering, fresh delivery."""
from dataclasses import asdict,replace
from uav_high_view.core import Catalog
from uav_high_view.grid_cost import GridCost
from uav_high_view.local_descent import propose
from uav_high_view.survey_policy import SurveyPolicy
from .high_view_probe import HighViewProbe
from .mission_runtime import MissionRuntime
from .mission_core import MissionPhase
from .search_types import Waypoint


class HighViewFull(HighViewProbe):
    def __init__(self,core,config,policy=None):
        super().__init__(core,config)
        self.policy=policy or SurveyPolicy()
        # Fixed targets are navigation knowledge for this mission; last_seen
        # remains untouched and never substitutes for fresh release evidence.
        self.catalog=Catalog(self.policy.catalog_config(core.config.mission_frame,core.config.mission_timeout),core.profile.weights)
        self.required=set(core.profile.interrupt_classes)
        self.top_hints={}
        self.grid=GridCost()
        self.orders=[];self.revisit_counts={};self.fresh_candidate=None
        self.completed_reacquisitions=[]
        self.descent_proposal=None
        self.first_hint_ready={}

    def _candidate_validation_config(self):
        if self.stage=='SURVEY':
            return replace(self.core.config,min_streak=self.policy.candidate_min_streak)
        return self.core.config

    def start(self,mission_id,now,current_xy):
        result=super().start(mission_id,now,current_xy)
        self.survey_until=now+self.core.config.mission_timeout
        return result

    def _dispatch_route(self,command,reason,now,route_outcome=None):
        # No 45s research-probe cut-off in the full strategy.
        result=MissionRuntime._dispatch_route(self,command,'high_view_full:'+self.stage,now,route_outcome)
        if result.action and self.stage=='REVISIT':self.revisit_started=now
        return result

    def _all_top(self,now):
        groups={}
        for h in self.catalog.hints(int(round(now*1e9))):groups.setdefault(h.class_name,[]).append(h)
        return {c:groups[c][0] for c in self.required if len(groups.get(c,[]))==1}

    def _consider_search_replacement(self,now):
        if self.stage=='SURVEY' and self.ascent_verified and set(self._all_top(now))==self.required:
            active=self.core.active_action
            if not self._route_binding_matches(active):return self._fail_closed('survey_binding_mismatch',now)
            retired=self.route.interrupt(active.decision_seq)
            if not retired.accepted:return self._fail_closed('survey_interrupt_failed',now)
            # Same serialized replacement boundary as the original search
            # interruption: targetless motion retires before the next sequence.
            self.core.active_action=None
            self.events.append(dict(stage='SURVEY_INTERRUPTED_TOP3',time=now,retired_seq=active.decision_seq,original_deadline=active.deadline_at))
            return self._retreat(now)
        return self._outcome(True,'full_motion_pending')

    def _retreat(self,now):
        self.top_hints=self._all_top(now)
        if set(self.top_hints)!=self.required:return self._finish(False,'survey_complete_missing_top3',now)
        if self.policy.direct_descent:
            exit_goal=self.core.config.post_delivery_route[0]
            plan=propose(self.grid,self._current_xy,{c:h.xy for c,h in self.top_hints.items()},
                         (exit_goal.x,exit_goal.y),now,self.policy.descent_radius_m,self.policy.descent_max_candidates)
            if plan is None:return self._finish(False,'no_local_descent_route',now)
            self.descent_proposal=dict(plan,time=now,from_xy=tuple(self._current_xy),map_stamp=self.grid.stamp,
                                       scope='SENSED_OCCUPANCY_PROPOSAL_REQUIRES_3D_PLANNER')
            self.orders.append(dict(time=now,classes=list(plan['classes']),grid_length_m=plan['cost_m'],
                                    map_stamp=self.grid.stamp,scope='INTERRUPTION_POINT_ROUTE'))
            self.selected=None
            direct=plan['kind']=='CURRENT_COLUMN'
            stage='DESCEND' if direct else 'LOCAL_DESCENT_TRANSIT'
            z=self.probe_config.ground_z+(self.probe_config.low_agl if direct else self.probe_config.high_agl)
            self._change_route(stage,[Waypoint(*plan['xy'],z)],now)
            return self._dispatch_route('SEARCH','local_descent',now)
        result=super()._retreat(now)
        self.selected=None
        return result

    def _next_target(self,now):
        remaining={c:h for c,h in self.top_hints.items() if c not in self.core.queue.delivered_classes}
        if not remaining:return self._fail_closed('no_remaining_target_before_tail',now)
        exit_goal=self.core.config.post_delivery_route[0]
        if len(remaining)==1:
            # There is no visit-order optimization with one target. Leave
            # reachability to the original full 3-D flight planner, rather than
            # aborting on a coarse 2-D ranking-grid endpoint/age rejection.
            cost,names=None,tuple(remaining)
            scope='SINGLE_REMAINING_TARGET_REQUIRES_3D_PLANNER'
        else:
            order=self.grid.order(self._current_xy,{c:h.xy for c,h in remaining.items()},(exit_goal.x,exit_goal.y),now)
            if order is None:return self._finish(False,'no_fresh_grid_route_to_required_targets',now)
            cost,names=order
            scope='COARSE_OCCUPANCY_COST_NOT_FLIGHT_APPROVAL'
        self.orders.append(dict(time=now,classes=list(names),grid_length_m=cost,map_stamp=self.grid.stamp,scope=scope))
        self.selected=remaining[names[0]]
        cls=self.selected.class_name;self.revisit_counts[cls]=self.revisit_counts.get(cls,0)+1
        if self.revisit_counts[cls]>2:return self._finish(False,'revisit_budget_exhausted',now)
        self.reacquired=None;self.fresh_candidate=None
        self._change_route('REVISIT',[Waypoint(*self.selected.xy,self.probe_config.ground_z+self.probe_config.low_agl)],now)
        return self._dispatch_route('SEARCH','ordered_revisit',now)

    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        if self.stage=='LOCAL_DESCENT_TRANSIT' and self.route.is_complete:
            self._change_route('DESCEND',[Waypoint(*self.descent_proposal['xy'],self.probe_config.ground_z+self.probe_config.low_agl)],now)
            return self._dispatch_route('SEARCH','local_descent',now)
        if self.stage=='DESCEND' and self.route.is_complete:return self._next_target(now)
        if self.stage=='DELIVERY':return self._next_target(now)
        return super()._schedule_from_search(now,prefer_resume,route_outcome)

    def ingest(self,candidates,now):
        outcome=super().ingest(candidates,now)
        if self.stage=='SURVEY':
            for name,hint in self._all_top(now).items():
                self.first_hint_ready.setdefault(name,dict(time=now,last_seen_ns=hint.last_seen_ns,xy=hint.xy,
                                                           uncertainty_m=hint.uncertainty_m,evidence_count=hint.evidence_count))
        if self.stage=='REACQUIRE' and self.reacquired is not None:
            fresh=[c for c in candidates if c.target_id==self.reacquired['target_id'] and c.last_seen_ns==self.reacquired['last_seen_ns']]
            if len(fresh)==1:self.fresh_candidate=fresh[0]
        return outcome

    def tick(self,now,current_xy):
        with self._lock:
            if self.stage=='REACQUIRE':
                now,failed=self._operation_time(now)
                if failed is not None:return failed
                self._set_current_xy(current_xy)
                if now>=self.wait_until:return self._finish(False,'reacquisition_timeout',now)
                if self.reacquired is None or self.fresh_candidate is None:return self._outcome(True,'waiting_for_fresh_delivery_candidate')
                validation=self.core.ingest([self.fresh_candidate],now)
                if not validation[0].accepted:
                    self.reacquired=None;self.fresh_candidate=None
                    return self._outcome(True,'waiting_for_fresh_delivery_candidate')
                action=self.core.choose(now,current_xy,False)
                if action is not None:
                    if action.command!='APPROACH':return self._fail_closed('unexpected_delivery_dispatch',now)
                    self.completed_reacquisitions.append(dict(self.reacquired))
                    self.stage='DELIVERY';self.events.append(dict(stage='DELIVERY',time=now,target=action.target_class))
                    return self._outcome(True,'fresh_ordered_delivery',action)
                self.reacquired=None;self.fresh_candidate=None
                return self._outcome(True,'waiting_for_fresh_delivery_candidate')
            # Original runtime executes APPROACH/release/recovery and tail.
            result=super().tick(now,current_xy)
            if self.core.phase in (MissionPhase.POST_DELIVERY_ROUTE,MissionPhase.RETURN_HOME,MissionPhase.LAND):self.stage='TAIL'
            if self.core.phase==MissionPhase.COMPLETE:self.done=True;self.succeeded=True
            return result

    def probe_status(self):
        value=super().probe_status()
        value.update(scope='HIGH_VIEW_FULL_MISSION',survey_policy='COMPLETE_ROUTE_OR_CONFIRMED_TOP3',
                     required_classes=sorted(self.required),top_hints={c:asdict(h) for c,h in self.top_hints.items()},
                     orders=list(self.orders),reacquisitions=list(self.completed_reacquisitions))
        value.update(survey_policy=asdict(self.policy),descent_proposal=self.descent_proposal,
                     first_hint_ready=dict(self.first_hint_ready))
        return value
