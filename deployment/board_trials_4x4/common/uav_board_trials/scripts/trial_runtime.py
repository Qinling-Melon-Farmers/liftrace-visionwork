"""Local trial endings on the unmodified mission/release transaction core."""
from dataclasses import replace,asdict
import math,itertools
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.high_view_full import HighViewFull
from uav_mission.high_view_probe import HighViewProbe
from uav_mission.mission_core import GoalSnapshot,MissionPhase
from uav_high_view.core import Epoch
from uav_high_view.grid_cost import GridCost

def landing_here(runtime,xy):
    cfg=runtime.core.config;xy=tuple(xy)
    runtime.core.config=replace(cfg,home_xy=xy,landing_xy=xy,
        post_delivery_route=(GoalSnapshot(cfg.mission_frame,*xy,cfg.return_altitude),))

class LocalLandingMixin:
    # Runtimes that must fly back to the original takeoff point before descending
    # set this True; the default keeps the legacy "land in place" trial ending.
    return_to_takeoff_before_land=False
    def _local_landing_xy(self):
        """Local LAND anchor: the takeoff column when the trial asks to return."""
        if self.return_to_takeoff_before_land:
            return tuple(self.home)
        return self._current_xy
    def apply_result(self,event,now,current_xy):
        active=self.core.active_action
        if active and active.command=='APPROACH' and event.decision_seq==active.decision_seq:
            landing_here(self,self._local_landing_xy() if self.return_to_takeoff_before_land else current_xy)
        result=super().apply_result(event,now,current_xy)
        expected=len(self.trial_manifest or {}) if hasattr(self,'trial_manifest') else getattr(self,'delivery_count',1)
        if (result.accepted and event.status=='SUCCEEDED' and event.stage=='RECOVERY' and expected>0 and self.core.committed_slots>=expected and result.action is not None and result.action.command=='RETURN_HOME'):
            if self.return_to_takeoff_before_land:
                # Keep the return transit: fly to the takeoff anchor first, then the
                # mission core issues LAND there and the terminal 30cm hover
                # (trial_terminal_hover) waits for the manual landing.
                return result
            # Replace an as-yet unpublished competition return with the local test LAND.
            self.core.active_action=None
            return self.end_here(now,'board_mock_deliveries_complete')
        return result
    def end_here(self,now,reason):
        landing_here(self,self._local_landing_xy())
        if self.return_to_takeoff_before_land:
            action=self.core._return_action(reason,now)
        else:
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
    def order(self,start,points,end,now,max_age=2.):
        if self.stamp is None or not 0<=now-self.stamp<=max_age or not 1<=len(points)<=3:return None
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
        return current if self.trial_manifest is None else {k:v for k,v in current.items() if k in self.trial_manifest}
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
        check=self._next_conflict_location(now,allowed_classes=remaining)
        if check is not None:return check
        # Do not add a new lawnmower search when a memorized target cannot be completed.
        return self._finish(False,'board_memorized_target_incomplete:'+reason,now)
    def probe_status(self):
        value=super().probe_status();value.update(scope='BOARD_FULL_CIRCLE_MOCK_DELIVERY',
            trial_manifest={k:asdict(v) for k,v in (self.trial_manifest or {}).items()},
            trial_memory_count=len(self.trial_manifest or {}),early_top3_interrupt_enabled=False)
        return value


class MultiDeliveryRuntime(LocalLandingMixin,MissionRuntime):
    def __init__(self,*args,delivery_count=2,**kwargs):
        if type(delivery_count) is not int or not 1<=delivery_count<=3:
            raise ValueError('delivery_count must be 1..3')
        self.delivery_count=delivery_count
        super().__init__(*args,**kwargs)
    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        if self.core.committed_slots>=self.delivery_count:
            return self.end_here(now,'board_multi_deliveries_complete')
        if self.route.is_complete:
            return self._fail_closed('board_line_finished_before_delivery_count',now)
        return super()._schedule_from_search(now,prefer_resume,route_outcome)


class PriorityRevisitRuntime(FullCircleRuntime):
    """Production early exit/recovery policy, with a local trial endpoint."""
    # Group 5: finish by returning to the original takeoff point, descend to the
    # 30cm terminal hover there and wait for the pilot to land manually.
    return_to_takeoff_before_land=True
    def _consider_search_replacement(self,now):
        return HighViewFull._consider_search_replacement(self,now)
    def _finish_route(self,action,succeeded,now):
        return HighViewFull._finish_route(self,action,succeeded,now)
    def _start_fallback(self,now,reason):
        remaining=set(self.trial_manifest or {})-self.core.queue.delivered_classes
        if remaining:
            check=self._next_conflict_location(now,allowed_classes=remaining)
            if check is not None:return check
            local=self._local_wall_recheck(now)
            if local is not None:return local
            resume=self._try_resume_survey(now)
            if resume is not None:return resume
        return super()._start_fallback(now,reason)
    def probe_status(self):
        value=super().probe_status()
        value.update(scope='BOARD_PRIORITY_REVISIT',early_top3_interrupt_enabled=True)
        return value


class MemoryOnlyRuntime(FullCircleRuntime):
    """Complete survey, descend through the planner, then land; never APPROACH."""
    def _next_target(self,now):
        return self.end_here(now,'board_memory_only_complete')
    def probe_status(self):
        value=super().probe_status()
        value.update(scope='BOARD_MEMORY_ONLY',memory_only=True)
        return value


class FullMissionTrialRuntime(HighViewFull):
    """Same full strategy as research, with measured hardware configuration."""
    def start(self,mission_id,now,current_xy):
        self.catalog.reset(Epoch(mission_id,'fixed-board-session',self.probe_config.source_key))
        self.survey_until=now+self.core.config.mission_timeout
        return MissionRuntime.start(self,mission_id,now,current_xy)


class HighSpeedCaptureRuntime(MemoryOnlyRuntime):
    """Survey capture; return home without target revisit or delivery."""
    return_to_takeoff_before_land=True
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.capture_complete=False
    def _retreat(self,now):
        if not self.ascent_verified or not self.route.is_complete:
            return self._finish(False,'board_capture_route_incomplete',now)
        self.trial_manifest=dict(self._all_hints(now))
        self.events.append(dict(stage='BOARD_CAPTURE_PASSES_COMPLETE',time=now,classes=sorted(self.trial_manifest)))
        # Return to the verified ascent location and descend with the same 3-D
        # planner. Neither an empty catalogue nor a category conflict starts a revisit.
        return HighViewProbe._retreat(self,now)
    def _next_target(self,now):
        self.capture_complete=True
        return self.end_here(now,'board_high_speed_capture_complete')
    def probe_status(self):
        value=super().probe_status()
        value.update(scope='BOARD_HIGH_SPEED_CAPTURE',capture_complete=self.capture_complete,
                     detection_success_evaluated=False)
        return value
