"""Bounded local SEARCH/RESUME advice. No route cursor or execution authority."""
from dataclasses import dataclass
import math
import numpy as np
from .queries import Region


@dataclass(frozen=True)
class ProposalConfig:
    flight_bounds: tuple  # known allowed XY search bounds, not obstacle truth
    lookaheads: tuple = (1.2, 2.4)
    lateral_offsets: tuple = (-.8, 0., .8)
    leg_length: float = .6
    view_half_x: float = .45
    view_half_y: float = .40
    speed: float = .5
    min_progress: float = .3
    max_leg_length: float = 4.
    max_candidates: int = 12
    max_snapshot_age: float = 1.
    max_pose_age: float = .5
    max_result_age: float = .5
    max_pose_drift: float = .25
    max_vertical_change: float = .25
    min_gain_m2: float = .05
    switch_gain_ratio: float = 1.2

    def __post_init__(self):
        if len(self.flight_bounds)!=4 or not np.isfinite(self.flight_bounds).all():
            raise ValueError('invalid flight bounds')
        a,b,c,d=self.flight_bounds
        if a>=b or c>=d:raise ValueError('invalid flight bounds')
        for name in ('leg_length','view_half_x','view_half_y','speed','min_progress','max_leg_length',
                     'max_snapshot_age','max_pose_age','max_result_age','max_pose_drift','max_vertical_change'):
            v=getattr(self,name)
            if not math.isfinite(v) or v<=0:raise ValueError('invalid proposal limit: '+name)
        if not 1<=self.max_candidates<=16 or int(self.max_candidates)!=self.max_candidates:
            raise ValueError('candidate limit must be in [1,16]')
        if (not self.lookaheads or not self.lateral_offsets or
                len(self.lookaheads)*len(self.lateral_offsets)+1>self.max_candidates or
                any(not math.isfinite(v) or v<=0 for v in self.lookaheads) or
                not np.isfinite(self.lateral_offsets).all() or
                len(set(self.lookaheads))!=len(self.lookaheads) or len(set(self.lateral_offsets))!=len(self.lateral_offsets)):
            raise ValueError('invalid or excessive proposal stencil')
        if not math.isfinite(self.min_gain_m2) or self.min_gain_m2<0 or not math.isfinite(self.switch_gain_ratio) or self.switch_gain_ratio<1:
            raise ValueError('invalid gain thresholds')


@dataclass(frozen=True)
class Context:
    mission_id: str
    decision_seq: int
    command: str
    frame_id: str
    pose: tuple
    pose_stamp: float
    nominal_goal: tuple
    deadline: float

    def validate(self, now, config):
        if not self.mission_id or not self.frame_id or self.command not in ('SEARCH','RESUME'):
            raise ValueError('not_active_search_or_resume')
        if self.decision_seq<=0 or int(self.decision_seq)!=self.decision_seq:
            raise ValueError('invalid_decision_identity')
        if (len(self.pose)!=3 or len(self.nominal_goal)!=3 or
                not np.isfinite((*self.pose,*self.nominal_goal,self.pose_stamp,self.deadline,now)).all() or
                self.pose_stamp<=0 or not 0<=now-self.pose_stamp<=config.max_pose_age or self.deadline<=now):
            raise ValueError('invalid_or_stale_motion_context')
        if abs(self.pose[2]-self.nominal_goal[2])>config.max_vertical_change:
            raise ValueError('search_height_transition')

    @property
    def identity(self):
        return (self.mission_id,self.decision_seq,self.command,self.frame_id,self.nominal_goal)


@dataclass(frozen=True)
class Candidate:
    name: str
    entry: tuple
    exit: tuple
    segment_indices: tuple
    observation: dict


@dataclass(frozen=True)
class Batch:
    request_id: int
    created: float
    snapshot_stamp: float
    generation: int
    context: Context
    candidates: tuple
    segments: tuple


class LocalProposer:
    def __init__(self, config):
        self.config=config
        self.sequence=0
        self.last_key=None
        self.last_name=None

    def prepare(self, snapshot, context, now):
        c=self.config
        context.validate(now,c)
        reason=snapshot.check(now,context.frame_id,context.mission_id,snapshot.generation,c.max_snapshot_age)
        if reason:raise ValueError(reason)
        a,b,y0,y1=c.flight_bounds
        inside=lambda p:a<=p[0]<=b and y0<=p[1]<=y1
        if not inside(context.pose) or not inside(context.nominal_goal):
            raise ValueError('outside_search_bounds')
        start=np.asarray(context.pose,dtype=float);goal=np.asarray(context.nominal_goal,dtype=float)
        delta=goal[:2]-start[:2];distance=float(np.linalg.norm(delta))
        if distance<c.min_progress:raise ValueError('nominal_goal_already_near')
        forward=delta/distance;side=np.array([-forward[1],forward[0]])
        specs=[]
        if float(np.linalg.norm(goal-start))<=c.max_leg_length:
            specs.append(('nominal',goal,goal))
        for i,ahead in enumerate(c.lookaheads):
            if ahead>=distance-c.min_progress:continue
            for j,offset in enumerate(c.lateral_offsets):
                entry=goal.copy();entry[:2]=start[:2]+forward*ahead+side*offset
                end=entry.copy();end[:2]+=forward*min(c.leg_length,distance-ahead)
                if not inside(entry) or not inside(end):continue
                if np.linalg.norm(entry[:2]-goal[:2])>distance-c.min_progress:continue
                if np.linalg.norm(entry-start)>c.max_leg_length:continue
                specs.append(('local_%d_%d'%(i,j),entry,end))
        regions=[]
        for name,entry,end in specs:
            bounds=(min(entry[0],end[0])-c.view_half_x,max(entry[0],end[0])+c.view_half_x,
                    min(entry[1],end[1])-c.view_half_y,max(entry[1],end[1])+c.view_half_y)
            travel=(float(np.linalg.norm(entry-start))+float(np.linalg.norm(end-entry)))/c.speed
            regions.append(Region(name,bounds,travel))
        ranked=snapshot.rank(regions,now=now,frame_id=context.frame_id,epoch=context.mission_id,
                             generation=snapshot.generation,max_age=c.max_snapshot_age,max_regions=c.max_candidates)
        rows={row['name']:row for row in ranked}
        candidates=[];segments=[]
        for name,entry,end in specs:
            if not rows[name]['eligible_for_ranking']:continue
            indices=[len(segments)];segments.append((tuple(start),tuple(entry)))
            if np.linalg.norm(end-entry)>1e-9:
                indices.append(len(segments));segments.append((tuple(entry),tuple(end)))
            candidates.append(Candidate(name,tuple(entry),tuple(end),tuple(indices),rows[name]))
        if not candidates:raise ValueError('no_local_candidate_within_bounds')
        self.sequence+=1
        return Batch(self.sequence,now,snapshot.stamp,snapshot.generation,context,tuple(candidates),tuple(segments))

    def finish(self, batch, reply, context, generation, now):
        c=self.config
        result=dict(accepted=False,reason='',request_id=batch.request_id,selected=None,candidates=[],
                    source_pose=list(batch.context.pose),pose_stamp=batch.context.pose_stamp,created=batch.created,
                    source_snapshot_stamp=batch.snapshot_stamp,
                    nominal_goal=list(batch.context.nominal_goal),mission_id=batch.context.mission_id,
                    decision_seq=batch.context.decision_seq,generation=batch.generation,
                    scope='LOCAL_OBSERVATION_ADVICE_ONLY',flight_authorized=False,
                    requires_trajectory_validation=True)
        def reject(reason):
            result['reason']=reason;return result
        try:context.validate(now,c)
        except ValueError as e:return reject(str(e))
        if context.identity!=batch.context.identity or generation!=batch.generation:
            return reject('decision_or_memory_changed')
        if not 0<=now-batch.created<=c.max_result_age or now-batch.snapshot_stamp>c.max_snapshot_age:
            return reject('proposal_expired')
        if np.linalg.norm(np.asarray(context.pose)-batch.context.pose)>c.max_pose_drift:
            return reject('pose_moved_since_query')
        if (reply.get('request_id')!=batch.request_id or reply.get('frame_id')!=context.frame_id or
                not reply.get('planner_session') or not isinstance(reply.get('map_revision'),int) or reply['map_revision']<=0):
            return reject('probe_identity_mismatch')
        for key,limit in [('stamp',c.max_result_age),('map_stamp',c.max_snapshot_age)]:
            value=reply.get(key)
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0 or not 0<=now-value<=limit:
                return reject('probe_'+key+'_stale_or_future')
        if reply.get('known_free_proven') is not False or reply.get('requires_trajectory_validation') is not True:
            return reject('unsupported_probe_semantics')
        codes=reply.get('status',[])
        if not reply.get('accepted'):
            return reject('probe_rejected:'+str(reply.get('reason','')))
        if len(codes)!=len(batch.segments) or any(type(v) is not int or v not in range(5) for v in codes):
            return reject('probe_status_mismatch')
        eligible=[]
        for item in batch.candidates:
            clear=all(codes[i]==0 for i in item.segment_indices)
            row=dict(name=item.name,entry=list(item.entry),exit=list(item.exit),
                     segment_status=[codes[i] for i in item.segment_indices],geometry_clear=clear,
                     estimated_gain_m2=item.observation['potential_observation_area_m2'],
                     score=item.observation['observation_score'])
            result['candidates'].append(row)
            if clear and row['estimated_gain_m2']>=c.min_gain_m2:eligible.append(row)
        if not eligible:return reject('no_clear_candidate_with_observation_gain')
        eligible.sort(key=lambda row:(-row['score'],row['name']))
        best=eligible[0]
        key=(context.identity,generation,reply['planner_session'])
        previous=next((r for r in eligible if r['name']==self.last_name),None)
        if self.last_key==key and previous and best['score']<previous['score']*c.switch_gain_ratio:
            best=previous
        self.last_key=key;self.last_name=best['name']
        result.update(accepted=True,reason='local_hint_requires_actual_planner',selected=best,
                      planner_session=reply['planner_session'],map_revision=reply['map_revision'],
                      map_stamp=reply['map_stamp'],snapshot_stamp=batch.snapshot_stamp,
                      geometry_scope='INFLATED_CLOUD_CENTERLINE_ONLY',expires_at=min(batch.created+c.max_result_age,context.deadline))
        return result
