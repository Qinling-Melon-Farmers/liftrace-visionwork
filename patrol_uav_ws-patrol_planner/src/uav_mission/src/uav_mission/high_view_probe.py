"""Bounded research motion/reacquisition runtime, sharing original transactions.

No target coordinates from scene truth; no payload actions. The nominal full
mission runtime is never instantiated as an active competitor in this probe.
"""
from dataclasses import dataclass, replace, asdict
import math
from uav_high_view.core import Catalog, Config, Epoch, Key, Observation
from .coverage_route import CoverageRoute
from .mission_core import MissionPhase, validate_candidate
from .mission_runtime import MissionRuntime
from .search_types import Waypoint


@dataclass(frozen=True)
class ProbeConfig:
    ground_z: float
    survey_xy: tuple
    high_agl: float = 2.6
    low_agl: float = 1.4
    survey_budget: float = 45.
    reacquire_budget: float = 15.
    hint_radius: float = .20
    association_radius: float = .60
    pose_max_age: float = .5
    source_key: str = 'installed-camera-1280-v1'

    def __post_init__(self):
        values=(self.ground_z,self.high_agl,self.low_agl,self.survey_budget,
                self.reacquire_budget,self.hint_radius,self.association_radius,self.pose_max_age)
        if (not all(math.isfinite(v) for v in values)
                or not 2.0<=self.high_agl<=2.8 or not 0<self.low_agl<self.high_agl
                or self.ground_z+self.low_agl<=0 or not 0<self.survey_budget<=60
                or not 0<self.reacquire_budget<=30 or not 0<self.hint_radius<=.25
                or not self.hint_radius<self.association_radius<=.8
                or not 0<self.pose_max_age<=.5 or not self.source_key
                or not 1<=len(self.survey_xy)<=8):
            raise ValueError('invalid probe configuration')
        if any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in self.survey_xy):
            raise ValueError('invalid survey points')


class HighViewProbe(MissionRuntime):
    def __init__(self, core, config):
        self.probe_config=config
        self.home=tuple(core.config.home_xy)
        high=config.ground_z+config.high_agl
        points=[Waypoint(*self.home,high)]+[Waypoint(x,y,high) for x,y in config.survey_xy]
        super().__init__(core,CoverageRoute(points,'high-view-probe:SURVEY',1))
        self.catalog=Catalog(Config(frame=core.config.mission_frame),core.profile.weights)
        self.stage='SURVEY'
        self.pose=None
        self.pose_stamp=None
        self.survey_until=None
        self.ascent_verified=False
        self.selected=None
        self.reacquired=None
        self.revisit_started=None
        self.wait_until=None
        self.done=False
        self.succeeded=False
        self.failure=''
        self.events=[]
        self.observation_counts={}

    def update_pose(self, xyz, stamp, frame):
        with self._lock:
            if frame!=self.core.config.mission_frame or not all(math.isfinite(v) for v in tuple(xyz)+(stamp,)):
                raise ValueError('probe pose invalid')
            if self.pose_stamp is not None:
                if stamp<self.pose_stamp:raise ValueError('probe pose clock rewind')
                dt=stamp-self.pose_stamp
                if dt>0 and math.dist(tuple(xyz),self.pose)>3.*dt+.25:
                    raise ValueError('probe pose discontinuity')
            self.pose=tuple(xyz);self.pose_stamp=stamp

    def start(self, mission_id, now, current_xy):
        self.catalog.reset(Epoch(mission_id,'static-sitl-world',self.probe_config.source_key))
        self.survey_until=now+self.probe_config.survey_budget
        return super().start(mission_id,now,current_xy)

    def _dispatch_route(self, command, reason, now, route_outcome=None):
        result=super()._dispatch_route(command,'high_view_probe:'+self.stage,now,route_outcome)
        if result.action and self.stage=='SURVEY':
            action=replace(result.action,deadline_at=min(result.action.deadline_at,self.survey_until))
            self.core.active_action=action
            return self._outcome(True,result.reason,action,route_outcome)
        if result.action and self.stage=='REVISIT':
            self.revisit_started=now
        return result

    def _change_route(self, stage, points, now):
        if self.core.active_action is not None or self.route.active is not None:
            raise RuntimeError('cannot replace a live motion transaction')
        self.stage=stage
        self.route=CoverageRoute(tuple(points),'high-view-probe:'+stage,1)
        self.events.append(dict(stage=stage,time=now))

    def _retreat(self,now):
        if not self.ascent_verified:
            return self._finish(False,'ascent_not_verified',now)
        hints=self.catalog.hints(int(round(now*1e9)))
        if hints:
            self.selected=min(hints,key=lambda h:(-self.core.profile.weight(h.class_name),h.key))
        high=self.probe_config.ground_z+self.probe_config.high_agl
        self._change_route('RETURN_COLUMN',[Waypoint(*self.home,high)],now)
        return self._dispatch_route('SEARCH','return_column',now)

    def _finish(self,success,reason,now):
        self.done=True;self.succeeded=success;self.failure='' if success else reason
        self.events.append(dict(stage='DONE',success=success,reason=reason,time=now))
        # End this segment via the original ABORT/hold contract, never by
        # forging a completed three-delivery mission or a landing result.
        return super().abort('research_segment_end:'+reason,now)

    def _finish_route(self, action, succeeded, now):
        index=self.route.current_index
        route_outcome,failed=super()._finish_route(action,succeeded,now)
        if failed is not None:return route_outcome,failed
        if self.stage=='SURVEY' and index==0 and succeeded:
            self.ascent_verified=True
        if not succeeded:
            if self.stage=='SURVEY' and index>0 and now>=self.survey_until:
                return route_outcome,self._retreat(now)
            return route_outcome,self._finish(False,'motion_failed:'+self.stage,now)
        return route_outcome,None

    def _consider_search_replacement(self,now):
        # Targets remain survey hints, never interrupt into APPROACH/ALIGN.
        return self._outcome(True,'probe_motion_pending')

    def _schedule_from_search(self,now,prefer_resume,route_outcome=None):
        c=self.probe_config
        if self.stage=='SURVEY' and (self.route.is_complete or now>=self.survey_until):
            return self._retreat(now)
        if self.route.is_complete:
            if self.stage=='RETURN_COLUMN':
                # Static-scene retrace of the confirmed ascent column. The
                # original planner must still accept and complete this segment.
                self._change_route('DESCEND',[Waypoint(*self.home,c.ground_z+c.low_agl)],now)
            elif self.stage=='DESCEND':
                if self.selected is None:return self._finish(False,'no_high_view_hint',now)
                if now-self.selected.last_seen_ns/1e9>self.catalog.config.hint_ttl_ns/1e9:
                    return self._finish(False,'hint_expired_before_revisit',now)
                self._change_route('REVISIT',[Waypoint(*self.selected.xy,c.ground_z+c.low_agl)],now)
            elif self.stage=='REVISIT':
                self.stage='REACQUIRE';self.wait_until=now+c.reacquire_budget
                self.events.append(dict(stage='REACQUIRE',time=now))
                return self._outcome(True,'waiting_for_fresh_low_view')
        return self._dispatch_route('SEARCH','probe_waypoint',now,route_outcome)

    def _candidate_validation_config(self):
        return self.core.config

    def ingest(self,candidates,now):
        with self._lock:
            self._require_started()
            now,failed=self._operation_time(now)
            if failed is not None:return failed
            if self.done:return self._outcome(False,'probe_finished')
            if self.pose is None or not 0<=now-self.pose_stamp<=self.probe_config.pose_max_age:
                return self._outcome(False,'probe_pose_unavailable')
            validations=[]
            good=[]
            for candidate in candidates:
                validation=validate_candidate(candidate,now,self.core.profile,self._candidate_validation_config())
                validations.append(validation)
                if validation.accepted:good.append(candidate)
                else:
                    key='candidate:'+validation.reason
                    self.observation_counts[key]=self.observation_counts.get(key,0)+1
            if self.stage=='SURVEY':
                for v in good:
                    observation=Observation(self.catalog.epoch,Key(v.target_id,v.first_seen_ns),
                        v.last_seen_ns,v.map_frame,v.class_name,(v.x,v.y),v.class_confidence,
                        v.map_quality,self.probe_config.hint_radius,self.pose[2]-self.probe_config.ground_z,
                        transform_age_ns=int(round(v.transform_age_sec*1e9)))
                    reason=self.catalog.observe(observation,int(round(now*1e9)))
                    self.observation_counts[reason]=self.observation_counts.get(reason,0)+1
            elif self.stage=='REACQUIRE':
                matches=[v for v in good if v.class_name==self.selected.class_name
                    and v.last_seen_ns/1e9>self.wait_until-self.probe_config.reacquire_budget
                    and abs(self.pose[2]-(self.probe_config.ground_z+self.probe_config.low_agl))<=.2
                    and math.hypot(v.x-self.selected.xy[0],v.y-self.selected.xy[1])<=self.probe_config.association_radius]
                if len(matches)==1:
                    v=matches[0]
                    self.reacquired=dict(target_id=v.target_id,class_name=v.class_name,
                        last_seen_ns=v.last_seen_ns,xy=[v.x,v.y],time=now,
                        hint_delta=math.hypot(v.x-self.selected.xy[0],v.y-self.selected.xy[1]))
            return self._outcome(True,'probe_observations',candidate_validations=validations)

    def tick(self,now,current_xy):
        with self._lock:
            if self.stage=='REACQUIRE' and not self.done:
                now,failed=self._operation_time(now)
                if failed is not None:return failed
                if self.reacquired is not None:return self._finish(True,'fresh_low_reacquisition',now)
                if now>=self.wait_until:return self._finish(False,'reacquisition_timeout',now)
                return self._outcome(True,'waiting_for_fresh_low_view')
            result=super().tick(now,current_xy)
            if self.core.phase==MissionPhase.ABORTED and not self.done:
                self.done=True;self.failure=result.reason
            return result

    def probe_status(self):
        with self._lock:
            return dict(scope='HIGH_VIEW_SINGLE_REVISIT_NO_DELIVERY',stage=self.stage,
                done=self.done,succeeded=self.succeeded,failure=self.failure,
                ascent_verified=self.ascent_verified,selected=asdict(self.selected) if self.selected else None,
                reacquired=self.reacquired,events=list(self.events),slots_committed=self.core.committed_slots,
                catalog_entries=len(self.catalog.entries),observation_counts=dict(self.observation_counts))
