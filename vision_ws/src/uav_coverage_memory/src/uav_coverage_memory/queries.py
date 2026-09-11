"""Read-only coverage queries for future local rescan selection.

Scores concern observation value, not route feasibility. No waypoints are
generated, and even an entirely estimated-seen box does not authorize skipping.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Region:
    name: str
    bounds: tuple  # min_x, max_x, min_y, max_y; observation region, NOT a goal
    estimated_travel_s: float

    def __post_init__(self):
        if len(self.bounds) != 4 or not all(math.isfinite(v) for v in self.bounds):
            raise ValueError('invalid region bounds')
        a,b,c,d=self.bounds
        if not self.name or a >= b or c >= d or not math.isfinite(self.estimated_travel_s) or self.estimated_travel_s < 0:
            raise ValueError('invalid observation region')


class Snapshot:
    def __init__(self, config, state_grid, stamp, epoch, generation, historical_seen=None):
        self.config=config
        self.stamp=float(stamp)
        self.epoch=str(epoch)
        self.generation=int(generation)
        nx=int(math.ceil((config.max_x-config.min_x)/config.resolution-1e-9))
        ny=int(math.ceil((config.max_y-config.min_y)/config.resolution-1e-9))
        if nx*ny > config.max_cells:
            raise ValueError('snapshot exceeds cell bound')
        states=np.asarray(state_grid)
        if states.shape != (ny,nx) or not np.isin(states,[0,20,40,60,80,100]).all():
            raise ValueError('invalid memory state grid')
        if not math.isfinite(self.stamp) or self.stamp <= 0 or generation <= 0 or int(generation) != generation:
            raise ValueError('invalid snapshot identity/time')
        self.states=states.astype(np.int8,copy=True)
        self.states.flags.writeable=False
        self.historical_seen=np.zeros((ny,nx),dtype=bool) if historical_seen is None else np.asarray(historical_seen,dtype=bool).copy()
        if self.historical_seen.shape != self.states.shape:
            raise ValueError('history shape mismatch')
        self.historical_seen.flags.writeable=False
        self.xe=np.minimum(config.min_x+np.arange(nx+1)*config.resolution,config.max_x)
        self.ye=np.minimum(config.min_y+np.arange(ny+1)*config.resolution,config.max_y)

    def check(self, now, frame_id, epoch, generation, max_age):
        if not math.isfinite(now) or not math.isfinite(max_age) or max_age <= 0:
            return 'invalid_query_time'
        if frame_id != self.config.frame_id:
            return 'frame_mismatch'
        if not self.epoch or epoch != self.epoch or generation != self.generation:
            return 'epoch_or_generation_mismatch'
        if not 0 <= now-self.stamp <= max_age:
            return 'snapshot_stale_or_future'
        return ''

    def query(self, region, *, now, frame_id, epoch, generation, max_age=1.0):
        reason=self.check(now,frame_id,epoch,generation,max_age)
        base={'name':region.name,'accepted':not reason,'reason':reason,
              'scope':'OBSERVATION_HINT_ONLY_NOT_A_WAYPOINT_OR_SKIP_PERMISSION',
              'requires_reachability_validation':True,'epoch':self.epoch,'generation':self.generation}
        if reason:return base
        a,b,c,d=region.bounds
        dx=np.maximum(0,np.minimum(self.xe[1:],b)-np.maximum(self.xe[:-1],a))
        dy=np.maximum(0,np.minimum(self.ye[1:],d)-np.maximum(self.ye[:-1],c))
        area=dy[:,None]*dx[None,:]
        requested=(b-a)*(d-c)
        values={str(code):float(area[self.states==code].sum()) for code in [0,20,40,60,80,100]}
        historical={str(code):float(area[(self.states==code)&self.historical_seen].sum()) for code in [0,20,40,60,80,100]}
        # Exact clipped cell-area accounting, not a center-hit pixel count.
        base.update(region_area_m2=requested,in_grid_area_m2=float(area.sum()),
                    outside_grid_area_m2=max(0.,requested-float(area.sum())),state_area_m2=values,
                    seen_estimate_fraction=values['100']/requested,
                    historical_estimate_area_m2=float(area[self.historical_seen].sum()),
                    historical_state_area_m2=historical,
                    needs_view_change=values['60']>0,
                    estimated_travel_s=region.estimated_travel_s)
        return base

    def rank(self, regions, *, now, frame_id, epoch, generation, max_age=1.,
             quality_retry_weight=.5, occlusion_retry_weight=.25, pending_weight=.2,
             fixed_view_cost_s=3., history_revisit_weight=.2):
        weights=[quality_retry_weight,occlusion_retry_weight,pending_weight,history_revisit_weight]
        if any(not math.isfinite(w) or not 0 <= w <= 1 for w in weights) or not math.isfinite(fixed_view_cost_s) or fixed_view_cost_s <= 0:
            raise ValueError('invalid observation score weights')
        if len({r.name for r in regions}) != len(regions):
            raise ValueError('duplicate region identity')
        rows=[self.query(r,now=now,frame_id=frame_id,epoch=epoch,generation=generation,max_age=max_age) for r in regions]
        for row in rows:
            row['eligible_for_ranking']=row['accepted'] and row['outside_grid_area_m2']<=1e-8
            if not row['eligible_for_ranking']:
                row['observation_score']=None
                continue
            v=row['state_area_m2']
            h=row['historical_state_area_m2']
            gain=sum(weight*(v[code]-(1-history_revisit_weight)*h[code])
                     for code,weight in [('0',1),('20',1),('40',quality_retry_weight),('60',occlusion_retry_weight),('80',pending_weight)])
            row.update(potential_observation_area_m2=gain,
                       observation_score=gain/(row['estimated_travel_s']+fixed_view_cost_s))
        return sorted(rows,key=lambda r:(not r['eligible_for_ranking'],-(r['observation_score'] or 0),r['name']))


class MissionLedger:
    """Past visibility estimates for one consumer/session, never free-space proof.

    Active TTL expiry does not erase the fact of an earlier observation.
    Changed mission/generation/geometry and clock regression reset history.
    Occlusion from a new viewpoint does not erase a past observation; scene or
    localization changes must advance the epoch/generation instead.
    """
    def __init__(self):
        self.identity=None
        self.history=None
        self.last_stamp=-np.inf
        self.last_now=-np.inf

    def update(self, snapshot, now, max_age=1.):
        reason=snapshot.check(now,snapshot.config.frame_id,snapshot.epoch,snapshot.generation,max_age)
        if reason:raise ValueError(reason)
        c=snapshot.config
        identity=(snapshot.epoch,snapshot.generation,c.frame_id,c.min_x,c.max_x,c.min_y,c.max_y,c.resolution,c.ground_z)
        if identity!=self.identity or now<self.last_now:
            self.identity=identity
            self.history=np.zeros_like(snapshot.states,dtype=bool)
        elif snapshot.stamp<self.last_stamp:
            raise ValueError('out_of_order_snapshot')
        elif snapshot.stamp==self.last_stamp:
            raise ValueError('duplicate_snapshot')
        self.history[snapshot.states==100]=True
        self.last_stamp=snapshot.stamp
        self.last_now=now
        return Snapshot(c,snapshot.states,snapshot.stamp,snapshot.epoch,snapshot.generation,self.history)
