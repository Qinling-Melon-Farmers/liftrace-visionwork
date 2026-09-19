"""Local trial endings on the unmodified mission/release transaction core."""
from dataclasses import replace,asdict
import math,itertools
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.high_view_full import HighViewFull
from uav_mission.mission_core import GoalSnapshot,MissionPhase
from uav_high_view.core import Epoch
from uav_high_view.grid_cost import GridCost

def landing_here(runtime,xy):
    cfg=runtime.core.config;xy=tuple(xy)
    runtime.core.config=replace(cfg,home_xy=xy,landing_xy=xy,
        post_delivery_route=(GoalSnapshot(cfg.mission_frame,*xy,cfg.return_altitude),))

class LocalLandingMixin:
    def apply_result(self,event,now,current_xy):
        active=self.core.active_action
        if active and active.command=='APPROACH' and event.decision_seq==active.decision_seq:
            landing_here(self,current_xy)
        result=super().apply_result(event,now,current_xy)
        expected=len(self.trial_manifest or {}) if hasattr(self,'trial_manifest') else 1
        if (result.accepted and event.status=='SUCCEEDED' and event.stage=='RECOVERY' and expected>0 and self.core.committed_slots>=expected and result.action is not None and result.action.command=='RETURN_HOME'):
            # Replace an as-yet unpublished competition return with the local test LAND.
            self.core.active_action=None
            return self.end_here(now,'board_mock_deliveries_complete')
        return result
    def end_here(self,now,reason):
        landing_here(self,self._current_xy)
        self.core.phase=MissionPhase.LAND
        action=self.core._new_action('LAND',reason,now,timeout=self.core.config.mission_timeout)
        if hasattr(self,'stage'):self.stage='TAIL'
        return self._outcome(True,reason,action)

class SingleDeliveryRuntime(LocalLandingMixin,MissionRuntime):
    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        if self.core.committed_slots>=1:return self.end_here(now,'board_one_mock_delivery_complete')
        if self.route.is_complete:return self._fail_closed('board_line_finished_without_delivery',now)
        return super()._schedule_from_search(now,prefer_resume,route_outcome)

class OpenTourGrid(GridCost):
    """At most three destinations, no fictitious return-to-start edge."""
    def order(self,start,points,end,now):
        if self.stamp is None or not 0<=now-self.stamp<=2. or not 1<=len(points)<=3:return None
        origins={'START':start,**points};dist={k:self.distances(v) for k,v in origins.items()}
        tours=[]
        for names in itertools.permutations(points):
            prev='START';cost=0.
            for name in names:
                cost+=dist[prev].get(self.cell(points[name]),math.inf);prev=name
            if math.isfinite(cost):tours.append((cost,names))
        return min(tours) if tours else None

class FullCircleRuntime(LocalLandingMixin,HighViewFull):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.trial_manifest=None
    def start(self,mission_id,now,current_xy):
        self.catalog.reset(Epoch(mission_id,'fixed-board-session',self.probe_config.source_key))
        self.survey_until=now+self.core.config.mission_timeout
        return MissionRuntime.start(self,mission_id,now,current_xy)
    def _all_top(self,now):
        current=super()._all_top(now)
        return current if self.trial_manifest is None else dict(self.trial_manifest)
    def _consider_search_replacement(self,now):
        # Deliberately retain the full survey even when all three hints are ready.
        if self.stage=='SURVEY' and self.ascent_verified:
            active=self.core.active_action;distance=math.dist(self._current_xy,(active.goal.x,active.goal.y))
            if self._progress is None or self._progress[0]!=active.decision_seq:self._progress=(active.decision_seq,distance,now)
            elif distance<self._progress[1]-self.policy.survey_progress_m:self._progress=(active.decision_seq,distance,now)
            elif now-self._progress[2]>=self.policy.survey_stall_seconds:
                self.events.append(dict(stage='SURVEY_NO_PROGRESS',time=now,decision_seq=active.decision_seq))
                self.core.active_action=replace(active,deadline_at=now)
                return MissionRuntime.tick(self,now,self._current_xy)
        return self._outcome(True,'board_complete_full_circle')
    def _retreat(self,now):
        if self.trial_manifest is None:
            self.trial_manifest=dict(super()._all_top(now))
            self.events.append(dict(stage='BOARD_MEMORY_FROZEN',time=now,classes=sorted(self.trial_manifest)))
        if not self.trial_manifest:return self._finish(False,'board_no_valid_target_recorded',now)
        return super()._retreat(now)
    def _finish_route(self,action,succeeded,now):
        if self.stage=='SURVEY' and not succeeded:
            outcome,failure=MissionRuntime._finish_route(self,action,succeeded,now)
            return outcome,failure or self._finish(False,'board_full_circle_incomplete',now)
        return super()._finish_route(action,succeeded,now)
    def _start_fallback(self,now,reason):
        remaining=set(self.trial_manifest or {})-self.core.queue.delivered_classes
        if not remaining:
            return self.end_here(now,'board_memorized_targets_complete' if self.core.committed_slots else 'board_no_valid_target_recorded')
        # Do not add a new lawnmower search when a memorized target cannot be completed.
        return self._finish(False,'board_memorized_target_incomplete:'+reason,now)
    def probe_status(self):
        value=super().probe_status();value.update(scope='BOARD_FULL_CIRCLE_MOCK_DELIVERY',
            trial_manifest={k:asdict(v) for k,v in (self.trial_manifest or {}).items()},
            trial_memory_count=len(self.trial_manifest or {}),early_top3_interrupt_enabled=False)
        return value
