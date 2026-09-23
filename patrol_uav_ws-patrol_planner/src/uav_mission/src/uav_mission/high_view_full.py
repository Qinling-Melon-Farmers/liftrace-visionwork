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
from .mission_core import GoalSnapshot,MissionPhase
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
        self.core.approach_admission=self._approach_allowed
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
        self.memory=NavigationMemory(core.profile.weights,int(core.config.mission_timeout*1e9),config.association_radius)
        self.fallback_route=fallback_route
        self._progress=None;self._alternative=False
        self._survey_original=None;self.skipped_survey_xy=[]
        self.fallback_started=None
        self.descent_debug=None
        self.conflict_checked=set()
        self.conflict_active=False
        self.conflict_check_started=None
        self.local_wall_verify_used=False;self.local_wall_verify_started=None
        self.local_wall_target=None
        self.unreachable_classes=set()
        self.degraded_from=None

    def _candidate_validation_config(self):
        if self.stage=='SURVEY':
            return replace(self.core.config,min_streak=self.policy.candidate_min_streak)
        return self.core.config

    def _approach_allowed(self,candidate):
        xy=(candidate.x,candidate.y)
        allowed=(not self.boundary_policy.enabled or
                 self.boundary_policy.admissible(xy))
        if candidate.class_name in self.unreachable_classes:allowed=False
        elif (not allowed and self.stage in ('REACQUIRE','LOCAL_WALL_VERIFY') and
              self.selected is not None and candidate.class_name==self.selected.class_name and
              math.dist(xy,self.selected.xy)<=max(.65,self.selected.uncertainty_m+.25)):
            allowed=self.boundary_policy.approach_center(xy) is not None
        if not allowed:self.boundary_rejections+=1
        return allowed

    def _bounded_approach(self,action,now):
        if action is None or action.command!='APPROACH':return action
        xy=(action.target_snapshot.x,action.target_snapshot.y)
        center=self.boundary_policy.approach_center(xy)
        if center is None:raise RuntimeError('admitted_target_has_no_safe_approach_center')
        if math.dist(center,xy)>1e-6:
            action=replace(action,reason='near_wall_bounded_approach',
                           goal=GoalSnapshot(action.goal.frame_id,*center,action.goal.z,action.goal.yaw))
            self.core.active_action=action
            self.events.append(dict(stage='NEAR_WALL_BOUNDED_APPROACH',time=now,
                                    target=action.target_class,target_xy=xy,center_xy=center))
        return action

    def _defer_selected(self,now,reason):
        self.events.append(dict(stage='TARGET_DEFERRED',time=now,reason=reason,
                                target=self.selected.class_name,
                                visits=self.revisit_counts.get(self.selected.class_name,0)))
        self.reacquired=None;self.fresh_candidate=None
        return self._next_target(now)

    def start(self,mission_id,now,current_xy):
        result=super().start(mission_id,now,current_xy)
        self.survey_until=now+self.core.config.mission_timeout
        return result

    def _dispatch_route(self,command,reason,now,route_outcome=None):
        # No 45s research-probe cut-off in the full strategy.
        result=MissionRuntime._dispatch_route(self,command,'high_view_full:'+self.stage,now,route_outcome)
        if result.action and self.stage=='REVISIT':self.revisit_started=now
        if result.action and self.stage=='LOCAL_WALL_VERIFY':
            action=replace(result.action,deadline_at=min(result.action.deadline_at,now+20.,self.local_wall_verify_started+40.))
            self.core.active_action=action
            return self._outcome(True,result.reason,action,route_outcome)
        return result

    def _all_hints(self,now):
        ns=int(round(now*1e9))
        return self.memory.update(self.catalog.hints(ns),self.catalog.epoch,ns)

    def _all_top(self,now):
        return {c:h for c,h in self._all_hints(now).items() if c in self.required}

    def _consider_search_replacement(self,now):
        if self.stage in ('LOW_COVERAGE','LOCAL_WALL_VERIFY'):
            outcome=MissionRuntime._consider_search_replacement(self,now)
            if outcome.action is not None and outcome.action.command=='APPROACH':
                action=self._bounded_approach(outcome.action,now)
                outcome=replace(outcome,action=action,snapshot=self._snapshot())
            if (self.stage=='LOCAL_WALL_VERIFY' and outcome.action is not None and
                    outcome.action.command=='APPROACH'):
                self.stage='DELIVERY'
                self.events.append(dict(stage='DELIVERY',time=now,target=outcome.action.target_class,
                                        reason='local_wall_visual_confirmation'))
            return outcome
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
        if self.stage=='REVISIT' and not succeeded:
            outcome,failed=MissionRuntime._finish_route(self,action,succeeded,now)
            if failed is not None:return outcome,failed
            return outcome,self._defer_selected(now,'revisit_unreachable')
        if self.stage in ('LOW_COVERAGE','LOCAL_WALL_VERIFY'):
            return MissionRuntime._finish_route(self,action,succeeded,now)
        if self.stage=='SURVEY' and self.ascent_verified:
            outcome,failed=MissionRuntime._finish_route(self,action,succeeded,now)
            if failed is not None:return outcome,failed
            if succeeded:
                self._alternative=False;self._survey_original=None
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
                    self._survey_original=(action.goal.x,action.goal.y)
                    self.events.append(dict(stage='SURVEY_ALTERNATIVE',time=now,xy=xy,scope='COARSE_PROPOSAL_REQUIRES_3D_PLANNER'))
                else:
                    self._alternative=False
                    skipped=self._survey_original or (action.goal.x,action.goal.y)
                    self._survey_original=None
                    if skipped not in self.skipped_survey_xy:self.skipped_survey_xy.append(skipped)
                    self.events.append(dict(stage='SURVEY_SKIPPED',time=now,goal=(action.goal.x,action.goal.y),region=skipped))
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
        remaining={c:h for c,h in self.top_hints.items()
                   if c not in self.core.queue.delivered_classes and c not in self.unreachable_classes}
        if not remaining and self.degraded_from is not None:
            remaining={c:h for c,h in self._all_hints(now).items()
                       if c not in self.required and c not in self.core.queue.delivered_classes and
                       c not in self.unreachable_classes}
            if remaining:
                best=max(self.core.profile.weight(c) for c in remaining)
                remaining={c:h for c,h in remaining.items() if self.core.profile.weight(c)==best}
                self.events.append(dict(stage='LOWER_WEIGHT_HINT_SELECTED',time=now,
                                        classes=sorted(remaining),after=self.degraded_from))
        if not remaining:return self._start_fallback(now,'known_hints_exhausted')
        remaining={c:h for c,h in remaining.items() if self.revisit_counts.get(c,0)<2}
        if not remaining:return self._start_fallback(now,'revisit_budget_exhausted')
        self.revisit_viewpoints={c:self.boundary_policy.viewpoint(h.xy,h.uncertainty_m) for c,h in remaining.items()}
        remaining={c:h for c,h in remaining.items() if self.revisit_viewpoints[c] is not None}
        if not remaining:return self._start_fallback(now,'no_admissible_revisit_viewpoint')
        # Complete a first pass over valid hints before retrying a deferred one.
        # The existing two-visits-per-class limit remains the only revisit budget.
        least_visits=min(self.revisit_counts.get(c,0) for c in remaining)
        remaining={c:h for c,h in remaining.items() if self.revisit_counts.get(c,0)==least_visits}
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
                if reachable:
                    cost,name=min(reachable);names=(name,)
                    scope='ONE_REACHABLE_HINT_FULL_TOUR_UNAVAILABLE_REQUIRES_3D_PLANNER'
                else:
                    cost=None
                    names=(min(remaining,key=lambda c:(math.dist(self._current_xy,points[c]),c)),)
                    scope='COARSE_ORDER_UNAVAILABLE_REQUIRES_3D_PLANNER'
            else:
                cost,names=order
                scope='COARSE_OCCUPANCY_COST_NOT_FLIGHT_APPROVAL'
        self.orders.append(dict(time=now,classes=list(names),grid_length_m=cost,map_stamp=self.grid.stamp,scope=scope))
        self.selected=remaining[names[0]]
        cls=self.selected.class_name;self.revisit_counts[cls]=self.revisit_counts.get(cls,0)+1
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
            if cls in self.core.queue.delivered_classes or cls in self.unreachable_classes:continue
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

    def _prioritized_fallback(self,points):
        # Each pair of consecutive sweep endpoints spans one full low lane.
        # A skipped high region chooses its nearest lane, not merely its nearest
        # endpoint; visit both endpoints before resuming the other lanes.
        lanes=[]
        for i in range(0,len(points)-1,2):
            a,b=points[i:i+2]
            if abs(a.y-b.y)>1e-6 or abs(a.x-b.x)<1e-6:
                return None
            lanes.append((i,(a,b)))
        if len(points)%2 or not lanes:return None
        # Several skipped high points can describe the same unseen sector.
        # Map each point against all lanes before removing any lane, so that
        # one sector is searched once rather than consuming adjacent lanes.
        priority_indices=set()
        for region in self.skipped_survey_xy:
            def gap(lane):
                index,(a,b)=lane
                segment_gap=max(min(a.x,b.x)-region[0],0.,region[0]-max(a.x,b.x))
                return (abs(a.y-region[1])+segment_gap,index)
            priority_indices.add(min(lanes,key=gap)[0])
        chosen=[];remaining=list(lanes);origin=self._current_xy
        while priority_indices:
            def entry_cost(lane):
                index,pair=lane
                return (min(math.dist(origin,(p.x,p.y)) for p in pair),index)
            index,pair=min((lane for lane in remaining if lane[0] in priority_indices),key=entry_cost)
            entry,exit_=sorted(pair,key=lambda p:math.dist(origin,(p.x,p.y)))
            chosen.extend((entry,exit_))
            origin=(exit_.x,exit_.y)
            remaining=[lane for lane in remaining if lane[0]!=index]
            priority_indices.remove(index)
        if not chosen:return None
        # Continue the usual boustrophedon order from the next untouched lane.
        first_index=min(range(len(remaining)),key=lambda i:math.dist(origin,(remaining[i][1][0].x,remaining[i][1][0].y))) if remaining else 0
        remaining=remaining[first_index:]+remaining[:first_index]
        for _,pair in remaining:chosen.extend(pair)
        return chosen

    def _local_wall_recheck(self,now):
        if self.local_wall_verify_used or not self.boundary_policy.enabled:
            return None
        remaining=[h for c,h in self.top_hints.items()
                   if c not in self.core.queue.delivered_classes and c not in self.unreachable_classes]
        if len(remaining)!=1 or len(self.top_hints)!=len(self.required):
            return None
        hint=remaining[0]
        if not self.boundary_policy.near(hint.xy):
            return None
        view=self.boundary_policy.viewpoint(hint.xy,hint.uncertainty_m)
        if view is None:return None
        a,b,c,d=self.boundary_policy.bounds
        margin=self.boundary_policy.margin
        left=(max(a+margin,view[0]-.30),view[1])
        right=(min(b-margin,view[0]+.30),view[1])
        points=[xy for xy in (left,right) if math.dist(xy,view)>.05 and
                self.boundary_policy.admissible(xy)]
        if not points:return None
        points.sort(key=lambda xy:math.dist(self._current_xy,xy))
        self.local_wall_verify_used=True;self.local_wall_verify_started=now
        self.local_wall_target=hint.class_name
        self.selected=hint
        self._change_route('LOCAL_WALL_VERIFY',[
            Waypoint(x,y,self.probe_config.ground_z+self.probe_config.low_agl)
            for x,y in points],now)
        self.route.max_failures_per_waypoint=1
        self.events.append(dict(stage='LOCAL_WALL_VERIFY',time=now,target=hint.class_name,
                                hint_xy=hint.xy,waypoints=points,
                                scope='FRESH_VISUAL_CANDIDATE_AND_BOUNDARY_ADMISSION_REQUIRED'))
        return MissionRuntime._schedule_from_search(self,now,False)

    def _degrade_near_wall(self,now):
        target=self.local_wall_target
        if target is None:
            remaining=[h for c,h in self.top_hints.items()
                       if c not in self.core.queue.delivered_classes and c not in self.unreachable_classes]
            if len(remaining)!=1 or not self.boundary_policy.near(remaining[0].xy):return None
            target=remaining[0].class_name
        self.unreachable_classes.add(target)
        self.degraded_from=target
        lower=tuple(c for c in self.core.profile.weights if c not in self.required)
        self.core.interrupt_class_override=tuple(c for c in self.core.profile.weights
                                                  if c not in self.unreachable_classes)
        self.events.append(dict(stage='NEAR_WALL_TARGET_UNREACHABLE',time=now,
                                target=target,next_classes=sorted(lower,key=lambda c:-self.core.profile.weight(c))))
        return self._next_target(now)

    def apply_result(self,event,now,current_xy):
        # A confirmed inaccessible delivery point is not a reason to search
        # for the same target again.  Retire only failures before release;
        # uncertain/committed releases keep the original fail-closed path.
        with self._lock:
            active=self.core.active_action
            inaccessible=(active is not None and active.command=='APPROACH' and
                          event.decision_seq==active.decision_seq and
                          event.terminal and event.status=='FAILED' and
                          event.reason in ('near_wall_visual_alignment_unreachable',
                                           'initial_plan_timeout') and
                          not self.core.active_release_started)
            if not inaccessible:
                return super().apply_result(event,now,current_xy)
            old=(set(self.unreachable_classes),self.degraded_from,
                 self.local_wall_target,self.core.interrupt_class_override)
            target=active.target_class
            self.unreachable_classes.add(target)
            self.degraded_from=target
            self.local_wall_target=target
            self.core.interrupt_class_override=tuple(
                c for c in self.core.profile.weights if c not in self.unreachable_classes)
            outcome=super().apply_result(event,now,current_xy)
            if not outcome.accepted:
                (self.unreachable_classes,self.degraded_from,self.local_wall_target,
                 self.core.interrupt_class_override)=old
            else:
                self.events.append(dict(stage='DELIVERY_POINT_UNREACHABLE',time=now,
                                        target=target,reason=event.reason,
                                        next_classes=sorted(self.core.interrupt_class_override)))
            return outcome

    def _start_fallback(self,now,reason):
        self.conflict_active=False
        check=self._next_conflict_location(now)
        if check is not None:return check
        local=self._local_wall_recheck(now)
        if local is not None:return local
        if self.degraded_from is None and (self.local_wall_verify_used or
                (len(self.top_hints)==len(self.required) and
                 len([c for c in self.top_hints if c not in self.core.queue.delivered_classes])==1)):
            downgraded=self._degrade_near_wall(now)
            if downgraded is not None:return downgraded
        if self.fallback_route is None:return self._finish(False,'fallback_route_unavailable',now)
        points=list(self.fallback_route.waypoints)
        costs=self.grid.distances(self._current_xy) if self.grid.stamp is not None and 0<=now-self.grid.stamp<=2. else {}
        prioritized=self._prioritized_fallback(points) if self.skipped_survey_xy else None
        if prioritized is None:
            index=min(range(len(points)),key=lambda i:(costs.get(self.grid.cell((points[i].x,points[i].y)),math.inf),math.dist(self._current_xy,(points[i].x,points[i].y))))
            points=points[index:]+points[:index]
        else:
            points=prioritized
            index=next(i for i,p in enumerate(self.fallback_route.waypoints) if p==points[0])
            self.events.append(dict(stage='LOW_COVERAGE_SKIPPED_HIGH_PRIORITY',time=now,
                                    regions=list(self.skipped_survey_xy),
                                    first_lane_y=points[0].y))
        self._change_route('LOW_COVERAGE',points,now)
        self.route.max_failures_per_waypoint=self.fallback_route.max_failures_per_waypoint
        self.fallback_started=now
        self.events.append(dict(stage='LOW_COVERAGE_HANDOFF',time=now,reason=reason,entry_index=index,committed_slots=self.core.committed_slots,original_deadline=self.core.started_at+self.core.config.mission_timeout))
        return MissionRuntime._schedule_from_search(self,now,False)

    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        if self.stage=='LOCAL_WALL_VERIFY' and self.route.is_complete:
            return self._degrade_near_wall(now) or self._finish(False,'near_wall_local_verify_exhausted',now)
        if self.stage in ('LOW_COVERAGE','LOCAL_WALL_VERIFY'):
            outcome=MissionRuntime._schedule_from_search(self,now,prefer_resume,route_outcome)
            if outcome.action is not None and outcome.action.command=='APPROACH':
                action=self._bounded_approach(outcome.action,now)
                return replace(outcome,action=action,snapshot=self._snapshot())
            return outcome
        if self.stage=='LOCAL_DESCENT_TRANSIT' and self.route.is_complete:
            self._change_route('DESCEND',[Waypoint(*self.descent_proposal['xy'],self.probe_config.ground_z+self.probe_config.low_agl)],now)
            return self._dispatch_route('SEARCH','local_descent',now)
        if self.stage=='DESCEND' and self.route.is_complete:return self._next_target(now)
        if self.stage=='DELIVERY':return self._next_target(now)
        return super()._schedule_from_search(now,prefer_resume,route_outcome)

    def ingest(self,candidates,now):
        if self.stage in ('LOW_COVERAGE','LOCAL_WALL_VERIFY'):
            return MissionRuntime.ingest(self,candidates,now)
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
                if now>=self.wait_until:return self._defer_selected(now,'reacquisition_timeout')
                if self.reacquired is None or self.fresh_candidate is None:return self._outcome(True,'waiting_for_fresh_delivery_candidate')
                if not self._approach_allowed(self.fresh_candidate):
                    self.reacquired=None;self.fresh_candidate=None
                    return self._outcome(True,'fresh_target_outside_boundary_approach_region')
                validation=self.core.ingest([self.fresh_candidate],now)
                if not validation[0].accepted:
                    self.reacquired=None;self.fresh_candidate=None
                    return self._outcome(True,'waiting_for_fresh_delivery_candidate')
                action=self.core.choose(now,current_xy,False)
                if action is not None:
                    if action.command!='APPROACH':return self._fail_closed('unexpected_delivery_dispatch',now)
                    action=self._bounded_approach(action,now)
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
                     skipped_survey_xy=list(self.skipped_survey_xy),
                     local_wall_verify_used=self.local_wall_verify_used,
                     degraded_from=self.degraded_from,
                     unreachable_classes=sorted(self.unreachable_classes),
                     conflict_active=self.conflict_active,conflict_checked=sorted(self.conflict_checked),
                     conflict_locations={c:[asdict(h) for h in hs] for c,hs in self.memory.verification_hints(self.memory.last_now or 0).items()})
        return value
