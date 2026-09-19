"""Opt-in full mission: survey all/early top3, sensed-grid ordering, fresh delivery."""
from dataclasses import asdict,replace
import math
from uav_high_view.navigation_memory import NavigationMemory
from uav_high_view.core import Catalog
from uav_high_view.grid_cost import GridCost
from uav_high_view.local_descent import propose,propose_column
from uav_high_view.survey_policy import SurveyPolicy
from .high_view_probe import HighViewProbe
from .mission_runtime import MissionRuntime
from .mission_core import MissionPhase
from .search_types import Waypoint
from .coverage_route import CoverageRoute
from .boundary_revisit import BoundaryRevisit


class HighViewFull(HighViewProbe):
    def __init__(self,core,config,policy=None,fallback_route=None,boundary_policy=None):
        super().__init__(core,config)
        self.policy=policy or SurveyPolicy()
        self.boundary_policy=boundary_policy or BoundaryRevisit()
        self.revisit_viewpoints={}
        self.boundary_rejections=0
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
        self.memory=NavigationMemory(self.required,int(core.config.mission_timeout*1e9),config.association_radius)
        self.fallback_route=fallback_route
        self._progress=None;self._alternative=False
        self.fallback_started=None
        self.descent_debug=None
        self.conflict_checked=set()
        self.conflict_active=False
        self.conflict_check_started=None

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
        ns=int(round(now*1e9))
        return self.memory.update(self.catalog.hints(ns),self.catalog.epoch,ns)

    def _consider_search_replacement(self,now):
        if self.stage=='LOW_COVERAGE':return MissionRuntime._consider_search_replacement(self,now)
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
        if self.stage=='SURVEY' and self.ascent_verified:
            active=self.core.active_action
            distance=math.dist(self._current_xy,(active.goal.x,active.goal.y))
            if self._progress is None or self._progress[0]!=active.decision_seq:
                self._progress=(active.decision_seq,distance,now)
            elif distance<self._progress[1]-self.policy.survey_progress_m:
                self._progress=(active.decision_seq,distance,now)
            elif now-self._progress[2]>=self.policy.survey_stall_seconds:
                self.events.append(dict(stage='SURVEY_NO_PROGRESS',time=now,goal=(active.goal.x,active.goal.y),decision_seq=active.decision_seq))
                # Use the original timeout reducer, not a fabricated executor result.
                self.core.active_action=replace(active,deadline_at=now)
                return MissionRuntime.tick(self,now,self._current_xy)
        return self._outcome(True,'full_motion_pending')

    def _finish_route(self,action,succeeded,now):
        if self.stage=='REVISIT' and self.conflict_active and not succeeded:
            outcome,failed=MissionRuntime._finish_route(self,action,succeeded,now)
            if failed is not None:return outcome,failed
            self.events.append(dict(stage='CONFLICT_LOCATION_UNREACHABLE',time=now,xy=self.selected.xy))
            return outcome,self._start_fallback(now,'conflict_location_unreachable')
        if self.stage=='LOW_COVERAGE':return MissionRuntime._finish_route(self,action,succeeded,now)
        if self.stage=='SURVEY' and self.ascent_verified:
            outcome,failed=MissionRuntime._finish_route(self,action,succeeded,now)
            if failed is not None:return outcome,failed
            if succeeded:self._alternative=False
            else:
                candidates=[]
                if not self._alternative and self.grid.stamp is not None and 0<=now-self.grid.stamp<=2.:
                    costs=self.grid.distances(self._current_xy)
                    for i in range(16):
                        angle=i*math.pi/8;r=self.policy.survey_alternative_radius_m
                        xy=(action.goal.x+r*math.cos(angle),action.goal.y+r*math.sin(angle))
                        cost=costs.get(self.grid.cell(xy),math.inf)
                        if math.isfinite(cost):candidates.append((cost,xy))
                remaining=list(self.route.waypoints[self.route.current_index:])
                if candidates:
                    xy=min(candidates)[1]
                    self._change_route('SURVEY',[Waypoint(*xy,action.goal.z)]+remaining,now)
                    self._alternative=True
                    self.events.append(dict(stage='SURVEY_ALTERNATIVE',time=now,xy=xy,scope='COARSE_PROPOSAL_REQUIRES_3D_PLANNER'))
                else:
                    self._alternative=False
                    self.events.append(dict(stage='SURVEY_SKIPPED',time=now,goal=(action.goal.x,action.goal.y)))
            return outcome,None
        return super()._finish_route(action,succeeded,now)

    def _retreat(self,now):
        self.top_hints=self._all_top(now)
        if not self.ascent_verified:return self._finish(False,'ascent_not_verified',now)
        if set(self.top_hints)!=self.required:
            self.events.append(dict(stage='PARTIAL_HINT_FALLBACK',time=now,known=sorted(self.top_hints)))
        if self.policy.direct_descent:
            exit_goal=self.core.config.post_delivery_route[0]
            def blocked(xy):
                cell=self.grid.cell(xy)
                return None if cell is None else bool(self.grid.blocked[cell])
            self.descent_debug=dict(time=now,map_stamp=self.grid.stamp,current_xy=tuple(self._current_xy),
                                    current_blocked=blocked(self._current_xy),exit_blocked=blocked((exit_goal.x,exit_goal.y)),
                                    target_blocked={c:blocked(h.xy) for c,h in self.top_hints.items()})
            plan=propose(self.grid,self._current_xy,{c:h.xy for c,h in self.top_hints.items()},
                         (exit_goal.x,exit_goal.y),now,self.policy.descent_radius_m,self.policy.descent_max_candidates)
            if plan is None:
                plan=propose_column(self.grid,self._current_xy,now,self.policy.descent_radius_m,self.policy.descent_max_candidates)
                if plan is not None:self.events.append(dict(stage='DESCENT_COLUMN_WITHOUT_FULL_TOUR',time=now,xy=plan['xy']))
            if plan is None and set(self.top_hints)!=self.required:
                # Return along the already verified ascent column if the local
                # coarse proposal is unavailable. Actual 3-D planner still owns motion.
                return super()._retreat(now)
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
        self.conflict_active=False
        remaining={c:h for c,h in self.top_hints.items() if c not in self.core.queue.delivered_classes}
        if not remaining:return self._start_fallback(now,'known_hints_exhausted')
        self.revisit_viewpoints={c:self.boundary_policy.viewpoint(h.xy,h.uncertainty_m) for c,h in remaining.items()}
        remaining={c:h for c,h in remaining.items() if self.revisit_viewpoints[c] is not None}
        if not remaining:return self._start_fallback(now,'no_admissible_revisit_viewpoint')
        points={c:self.revisit_viewpoints[c] for c in remaining}
        exit_goal=self.core.config.post_delivery_route[0]
        if len(remaining)==1:
            # There is no visit-order optimization with one target. Leave
            # reachability to the original full 3-D flight planner, rather than
            # aborting on a coarse 2-D ranking-grid endpoint/age rejection.
            cost,names=None,tuple(remaining)
            scope='SINGLE_REMAINING_TARGET_REQUIRES_3D_PLANNER'
        else:
            order=self.grid.order(self._current_xy,points,(exit_goal.x,exit_goal.y),now)
            if order is None:
                distances=self.grid.distances(self._current_xy) if self.grid.stamp is not None and 0<=now-self.grid.stamp<=2. else {}
                reachable=[(distances.get(self.grid.cell(points[c]),math.inf),c) for c,h in remaining.items()]
                reachable=[item for item in reachable if math.isfinite(item[0])]
                if not reachable:return self._start_fallback(now,'hint_route_unavailable')
                cost,name=min(reachable);names=(name,)
                scope='ONE_REACHABLE_HINT_FULL_TOUR_UNAVAILABLE_REQUIRES_3D_PLANNER'
            else:
                cost,names=order
                scope='COARSE_OCCUPANCY_COST_NOT_FLIGHT_APPROVAL'
        self.orders.append(dict(time=now,classes=list(names),grid_length_m=cost,map_stamp=self.grid.stamp,scope=scope))
        self.selected=remaining[names[0]]
        cls=self.selected.class_name;self.revisit_counts[cls]=self.revisit_counts.get(cls,0)+1
        if self.revisit_counts[cls]>2:return self._start_fallback(now,'revisit_budget_exhausted')
        self.reacquired=None;self.fresh_candidate=None
        view=self.revisit_viewpoints[cls]
        self.events.append(dict(stage='REVISIT_VIEWPOINT',time=now,xy=view,hint_xy=self.selected.xy,
                                uncertainty_m=self.selected.uncertainty_m,boundary_margin_m=self.boundary_policy.margin))
        self._change_route('REVISIT',[Waypoint(*view,self.probe_config.ground_z+self.probe_config.low_agl)],now)
        return self._dispatch_route('SEARCH','ordered_revisit',now)

    def _next_conflict_location(self,now):
        # These are competing visual hypotheses, not confirmed class coordinates.
        # A new physical low-view observation must still pass the original chain.
        if self.conflict_check_started is not None and now-self.conflict_check_started>=75.:
            return None
        proposals=[]
        costs=self.grid.distances(self._current_xy) if self.grid.stamp is not None and 0<=now-self.grid.stamp<=2. else {}
        for cls,hints in self.memory.verification_hints(int(round(now*1e9))).items():
            if cls in self.core.queue.delivered_classes:continue
            for index,h in enumerate(hints):
                key=(cls,index)
                if key in self.conflict_checked:continue
                view=self.boundary_policy.viewpoint(h.xy,h.uncertainty_m)
                if view is None:
                    self.conflict_checked.add(key)
                    self.events.append(dict(stage='CONFLICT_LOCATION_INADMISSIBLE',time=now,xy=h.xy))
                    continue
                cost=costs.get(self.grid.cell(view),math.inf)
                proposals.append((cost,math.dist(self._current_xy,view),cls,index,h,view))
        if not proposals:return None
        _,_,cls,index,h,view=min(proposals,key=lambda p:p[:4])
        if self.conflict_check_started is None:self.conflict_check_started=now
        self.conflict_checked.add((cls,index));self.conflict_active=True
        self.selected=h;self.reacquired=None;self.fresh_candidate=None
        self.events.append(dict(stage='CONFLICT_LOW_VERIFY',time=now,class_name=cls,xy=h.xy,
                                viewpoint=view,hypothesis=index,scope='LOW_VIEW_RECHECK_NOT_RELEASE_AUTHORIZATION'))
        self._change_route('REVISIT',[Waypoint(*view,self.probe_config.ground_z+self.probe_config.low_agl)],now)
        out=self._dispatch_route('SEARCH','conflict_low_verify',now)
        if out.action:
            action=replace(out.action,deadline_at=min(out.action.deadline_at,now+25.,self.conflict_check_started+75.))
            self.core.active_action=action
            return self._outcome(True,out.reason,action)
        return out

    def _start_fallback(self,now,reason):
        self.conflict_active=False
        check=self._next_conflict_location(now)
        if check is not None:return check
        if self.fallback_route is None:return self._finish(False,'fallback_route_unavailable',now)
        points=list(self.fallback_route.waypoints)
        costs=self.grid.distances(self._current_xy) if self.grid.stamp is not None and 0<=now-self.grid.stamp<=2. else {}
        index=min(range(len(points)),key=lambda i:(costs.get(self.grid.cell((points[i].x,points[i].y)),math.inf),math.dist(self._current_xy,(points[i].x,points[i].y))))
        points=points[index:]+points[:index]
        self._change_route('LOW_COVERAGE',points,now)
        self.route.max_failures_per_waypoint=self.fallback_route.max_failures_per_waypoint
        self.fallback_started=now
        self.events.append(dict(stage='LOW_COVERAGE_HANDOFF',time=now,reason=reason,entry_index=index,committed_slots=self.core.committed_slots,original_deadline=self.core.started_at+self.core.config.mission_timeout))
        return MissionRuntime._schedule_from_search(self,now,False)

    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        if self.stage=='LOW_COVERAGE':return MissionRuntime._schedule_from_search(self,now,prefer_resume,route_outcome)
        if self.stage=='LOCAL_DESCENT_TRANSIT' and self.route.is_complete:
            self._change_route('DESCEND',[Waypoint(*self.descent_proposal['xy'],self.probe_config.ground_z+self.probe_config.low_agl)],now)
            return self._dispatch_route('SEARCH','local_descent',now)
        if self.stage=='DESCEND' and self.route.is_complete:return self._next_target(now)
        if self.stage=='DELIVERY':return self._next_target(now)
        return super()._schedule_from_search(now,prefer_resume,route_outcome)

    def ingest(self,candidates,now):
        if self.stage=='LOW_COVERAGE':return MissionRuntime.ingest(self,candidates,now)
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
                if now>=self.wait_until:return self._start_fallback(now,'reacquisition_timeout')
                if self.reacquired is None or self.fresh_candidate is None:return self._outcome(True,'waiting_for_fresh_delivery_candidate')
                if self.boundary_policy.enabled and not self.boundary_policy.admissible((self.fresh_candidate.x,self.fresh_candidate.y)):
                    self.boundary_rejections+=1
                    self.reacquired=None;self.fresh_candidate=None
                    return self._outcome(True,'fresh_target_outside_boundary_approach_region')
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
        value.update(boundary_policy=asdict(self.boundary_policy),boundary_rejections=self.boundary_rejections,revisit_viewpoints=self.revisit_viewpoints,
                     survey_policy=asdict(self.policy),descent_proposal=self.descent_proposal,
                     first_hint_ready=dict(self.first_hint_ready),navigation_memory_events=list(self.memory.events),
                     fallback_started=self.fallback_started,descent_debug=self.descent_debug,
                     conflict_active=self.conflict_active,conflict_checked=sorted(self.conflict_checked),
                     conflict_locations={c:[asdict(h) for h in hs] for c,hs in self.memory.verification_hints(self.memory.last_now or 0).items()})
        return value
