"""Validation for optional research entry goals; no ROS or actuator authority."""
from dataclasses import dataclass
import math
from numbers import Real
from .mission_core import GoalSnapshot


def finite(value, name):
    if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value):
        raise ValueError('local_invalid_'+name)
    return float(value)


def vector(value,name):
    if not isinstance(value,(list,tuple)) or len(value)!=3:
        raise ValueError('local_invalid_'+name)
    return tuple(finite(v,name) for v in value)


def integer(value,name):
    if type(value) is not int or value<=0:raise ValueError('local_invalid_'+name)
    return value


def distance(a,b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


@dataclass(frozen=True)
class LocalMotionConfig:
    enabled: bool = False
    bounds: tuple = ()
    max_per_mission: int = 4
    timeout: float = 12.
    resume_reserve: float = 3.
    min_action_age: float = .5
    max_advice_age: float = .5
    max_memory_age: float = 1.
    max_map_age: float = 1.
    max_pose_age: float = .5
    max_pose_drift: float = .25
    max_vertical_change: float = .25
    min_leg: float = .5
    max_leg: float = 4.
    max_lateral: float = 1.
    max_extra_distance: float = 1.5
    min_progress: float = .3
    min_entry_gain: float = .1
    entry_arrival_xy: float = .35

    def __post_init__(self):
        if type(self.enabled) is not bool:raise ValueError('local_enabled_must_be_bool')
        if type(self.max_per_mission) is not int or not 1<=self.max_per_mission<=8:
            raise ValueError('local_mission_budget_invalid')
        for name,value in vars(self).items():
            if name not in ('enabled','bounds','max_per_mission') and finite(value,name)<=0:
                raise ValueError('local_nonpositive_'+name)
        if self.timeout>30 or self.max_advice_age>1 or self.max_leg>4 or self.min_leg>=self.max_leg:
            raise ValueError('local_limits_excessive')
        if self.enabled or self.bounds:
            if len(self.bounds)!=4:raise ValueError('local_bounds_required')
            a,b,c,d=[finite(v,'bounds') for v in self.bounds]
            if a>=b or c>=d:raise ValueError('local_bounds_invalid')


def validate_mode(enabled,use_sim_time,start_mode):
    if type(enabled) is not bool:raise ValueError('local_enabled_must_be_bool')
    if enabled and (use_sim_time is not True or start_mode!='full'):
        raise ValueError('local_motion_requires_simulation_full_mode')


@dataclass(frozen=True)
class LocalEntry:
    request_id: int
    generation: int
    goal: GoalSnapshot
    estimated_gain: float


def validate_advice(data, memory, active, nominal, mission_id, current_xyz,
                    pose_stamp, now, config):
    """Recheck the task/observation/geometry boundary at the actual adoption time."""
    if not config.enabled:raise ValueError('local_motion_disabled')
    if not isinstance(data,dict) or not isinstance(memory,dict):raise ValueError('local_advice_missing')
    if type(data.get('schema_version')) is not int or data['schema_version']!=1 or data.get('accepted') is not True:
        raise ValueError('local_advice_not_accepted')
    if (data.get('scope')!='LOCAL_OBSERVATION_ADVICE_ONLY' or data.get('flight_authorized') is not False or
            data.get('requires_trajectory_validation') is not True or data.get('known_free_proven') is not False or
            data.get('geometry_scope')!='INFLATED_CLOUD_CENTERLINE_ONLY' or data.get('gain_scope')!='ENTRY_VIEW_PROXY'):
        raise ValueError('local_advice_semantics')
    if active is None or active.has_target or active.command not in ('SEARCH','RESUME'):
        raise ValueError('local_not_search_motion')
    integer(data.get('decision_seq'),'decision_seq')
    if (data.get('mission_id')!=mission_id or data.get('decision_seq')!=active.decision_seq or
            data.get('command')!=active.command or data.get('frame_id')!=active.goal.frame_id):
        raise ValueError('local_decision_mismatch')
    now=finite(now,'now');pose=vector(current_xyz,'pose')
    if now-active.issued_at<config.min_action_age:raise ValueError('local_action_too_new')
    if active.deadline_at-now<config.timeout+config.resume_reserve:
        raise ValueError('local_insufficient_original_lease')
    def age(value,limit,name):
        value=finite(value,name)
        if value<=0 or not 0<=now-value<=limit:raise ValueError('local_stale_or_future_'+name)
        return value
    created=age(data.get('created'),config.max_advice_age,'created')
    receipt=age(data.get('receipt_ros'),config.max_advice_age,'receipt')
    expires=finite(data.get('expires_at'),'expiry')
    if receipt<created or not now<expires<=min(created+config.max_advice_age,active.deadline_at)+1e-6:
        raise ValueError('local_advice_expired')
    age(pose_stamp,config.max_pose_age,'current_pose')
    age(data.get('pose_stamp'),config.max_pose_age+config.max_advice_age,'source_pose')
    if distance(pose,vector(data.get('source_pose'),'source_pose'))>config.max_pose_drift:
        raise ValueError('local_pose_drift')
    generation=integer(data.get('generation'),'generation')
    if (memory.get('epoch')!=mission_id or memory.get('generation')!=generation or
            memory.get('frame_id')!=active.goal.frame_id):
        raise ValueError('local_memory_identity_changed')
    memory_stamp=age(memory.get('receipt_ros'),config.max_memory_age,'memory')
    observed=age(data.get('source_snapshot_stamp'),config.max_memory_age,'source_snapshot')
    if memory_stamp+1e-6<observed:raise ValueError('local_source_snapshot_not_received')
    age(data.get('map_stamp'),config.max_map_age,'map')
    integer(data.get('map_revision'),'map_revision')
    if not isinstance(data.get('planner_session'),str) or not data['planner_session']:
        raise ValueError('local_planner_session_missing')
    if (distance(vector(data.get('nominal_goal'),'nominal'),nominal)>1e-6 or
            distance((active.goal.x,active.goal.y,active.goal.z),nominal)>1e-6):
        raise ValueError('local_nominal_goal_mismatch')
    selected=data.get('selected')
    if not isinstance(selected,dict) or selected.get('name')=='nominal' or selected.get('geometry_clear') is not True:
        raise ValueError('local_entry_not_selected')
    statuses=selected.get('segment_status')
    if not isinstance(statuses,list) or not 1<=len(statuses)<=2 or any(type(s) is not int or s!=0 for s in statuses):
        raise ValueError('local_entry_geometry_rejected')
    entry=vector(selected.get('entry'),'entry')
    gain=finite(selected.get('entry_gain_m2'),'entry_gain')
    if gain<config.min_entry_gain:raise ValueError('local_entry_gain_insufficient')
    a,b,c,d=config.bounds
    for p in (pose,entry,nominal):
        if not a<=p[0]<=b or not c<=p[1]<=d:raise ValueError('local_search_bounds')
    if abs(entry[2]-nominal[2])>1e-6 or not 0<=entry[2]<=4:
        raise ValueError('local_entry_changes_altitude')
    if abs(pose[2]-entry[2])>config.max_vertical_change:
        raise ValueError('local_search_height_transition')
    leg=distance(pose,entry)
    remaining=distance(pose[:2],nominal[:2])
    if not config.min_leg<=leg<=config.max_leg or remaining<config.min_leg:
        raise ValueError('local_entry_length')
    dx,dy=nominal[0]-pose[0],nominal[1]-pose[1]
    ex,ey=entry[0]-pose[0],entry[1]-pose[1]
    lateral=abs(ex*dy-ey*dx)/remaining
    progress=remaining-distance(entry[:2],nominal[:2])
    extra=distance(pose[:2],entry[:2])+distance(entry[:2],nominal[:2])-remaining
    if lateral>config.max_lateral or progress<config.min_progress or extra>config.max_extra_distance:
        raise ValueError('local_entry_not_bounded_progress')
    return LocalEntry(integer(data.get('request_id'),'request_id'),generation,
        GoalSnapshot(active.goal.frame_id,*entry,yaw=active.goal.yaw),gain)
