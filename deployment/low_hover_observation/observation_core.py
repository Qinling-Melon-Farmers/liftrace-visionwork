"""Bounded diagnostic flight in the initial FC frame; no mission/servo/planner."""
import math


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def route(config, profile, origin):
    x, y, z, yaw = origin
    target_z = z - config['fc_ground_clearance'] + config['height_agl']
    result = [(x, y, target_z, yaw)]
    for forward, left in config['profiles'][profile]:
        result.append((x+math.cos(yaw)*forward-math.sin(yaw)*left,
                       y+math.sin(yaw)*forward+math.cos(yaw)*left, target_z, yaw))
    return result


def validate(config, profile):
    if profile not in ('hover', 'forward', 'square'):
        raise ValueError('unknown profile')
    keys = ('fc_ground_clearance','height_agl','climb_speed','horizontal_speed',
            'max_setpoint_lead','arrival_tolerance','arrival_speed','arrival_dwell',
            'hover_seconds','waypoint_seconds','pose_max_age','state_max_age',
            'warmup_seconds','mission_timeout','max_distance_from_start',
            'max_height_agl','jump_base_m','jump_speed_mps','yaw_jump_deg')
    if any(not isinstance(config[k], (float,int)) or isinstance(config[k],bool)
           or not math.isfinite(config[k]) or config[k]<=0 for k in keys):
        raise ValueError('positive finite configuration required')
    for key,default in [('ground_stable_seconds',2.0)]:
        value=config.get(key,default)
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid ground stability configuration')
    if not isinstance(config.get('require_fc_ev_yaw_agreement',False),bool):
        raise ValueError('yaw agreement switch must be boolean')
    if not config['fc_ground_clearance'] < config['height_agl'] <= config['max_height_agl'] <= 1.2:
        raise ValueError('invalid low-flight FC height')
    if config['horizontal_speed']>.4 or config['climb_speed']>.3 or config['max_setpoint_lead']>.25:
        raise ValueError('not a low-speed observation profile')
    points=config['profiles'][profile]
    if len(points)>8:raise ValueError('too many observation legs')
    for point in points:
        if len(point)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in point):
            raise ValueError('invalid route point')
        if math.hypot(*point)>config['max_distance_from_start']:
            raise ValueError('route outside observation range')
    if not config['frame'] or not config['mission_frame']:raise ValueError('frame required')


class Observation:
    def __init__(self, config, profile, origin, now):
        validate(config,profile)
        self.c=config;self.profile=profile;self.origin=origin;self.goals=route(config,profile,origin)
        self.target=tuple(origin);self.stage='READY';self.reason='manual_arm_then_offboard'
        self.index=0;self.ready_at=now;self.started=None;self.previous_tick=now
        self.last_pose=None;self.last_stamp=None;self.arrival_since=None;self.dwell_since=None
        self.prev_mode=None;self.ever_started=False

    def hold(self, reason, pose=None):
        if self.stage in ('TAKEN_OVER','HOLD_FOR_PILOT'):return
        self.stage='HOLD_FOR_PILOT';self.reason=reason
        if pose is not None:self.target=tuple(pose)

    def step(self, now, pose, stamp, fresh, armed, mode, connected=True,
             speed=0., stop_requested=False):
        dt=min(max(now-self.previous_tick,0.),.1);self.previous_tick=now
        entered=(mode=='OFFBOARD' and self.prev_mode is not None and self.prev_mode!='OFFBOARD')
        self.prev_mode=mode
        if self.ever_started and (not armed or mode!='OFFBOARD'):
            self.stage='TAKEN_OVER';self.reason='manual_mode_or_disarm';return None
        if self.stage=='TAKEN_OVER':return None
        if stop_requested:self.hold('operator_stop_requested',pose if fresh else None)
        if self.stage=='READY':
            if not connected or not fresh or not math.isfinite(speed):
                self.hold('ground_reference_invalid_restart_on_ground');return self.target
            # FC yaw may still converge on the ground. Only the position/ground
            # reference is frozen at READY; lock heading on fresh pilot entry.
            if math.dist(pose[:3],self.origin[:3])>.10:
                self.hold('start_reference_changed',pose);return self.target
            self.target=self.target[:3]+(pose[3],)
            if not armed:self.target=tuple(pose)
            if (armed and entered and now-self.ready_at>=self.c['warmup_seconds']):
                self.origin=tuple(self.origin[:3])+(pose[3],)
                self.goals=route(self.c,self.profile,self.origin)
                self.target=tuple(pose)
                self.stage='RUN';self.started=now;self.ever_started=True
                self.last_pose=tuple(pose);self.last_stamp=stamp
                self.reason='ascending_to_fc_agl'
            return self.target
        if self.stage=='HOLD_FOR_PILOT':return self.target
        if not connected or not fresh or not math.isfinite(speed):
            self.hold('telemetry_stale');return self.target
        if self.last_stamp is not None and stamp>self.last_stamp:
            gap=stamp-self.last_stamp
            if (math.dist(pose[:3],self.last_pose[:3])>self.c['jump_base_m']+self.c['jump_speed_mps']*gap
                    or abs(angle_delta(pose[3],self.last_pose[3]))>math.radians(self.c['yaw_jump_deg'])):
                # Zero instantaneous position/yaw error in the current estimate;
                # this is a pilot handoff, not a proven FC-reset recovery.
                self.hold('pose_discontinuity_take_over',pose);return self.target
        self.last_pose=tuple(pose);self.last_stamp=stamp
        agl=pose[2]-self.origin[2]+self.c['fc_ground_clearance']
        if (math.dist(pose[:2],self.origin[:2])>self.c['max_distance_from_start']
                or agl>self.c['max_height_agl']):
            self.hold('observation_bounds_take_over',pose);return self.target
        if now-self.started>=self.c['mission_timeout']:
            self.hold('observation_timeout',pose);return self.target
        if self.stage=='FINISHED_HOVER':return self.target
        goal=self.goals[self.index]
        # Ramp a setpoint, capped near feedback: no blind time-based march.
        delta=[goal[i]-self.target[i] for i in range(3)]
        distance=math.sqrt(sum(v*v for v in delta))
        pace=self.c['climb_speed'] if self.index==0 else self.c['horizontal_speed']
        step=min(distance,pace*dt)
        next_xyz=[self.target[i]+(delta[i]*step/distance if distance else 0.) for i in range(3)]
        lead=math.dist(next_xyz,pose[:3])
        if lead>self.c['max_setpoint_lead']:
            next_xyz=[pose[i]+(next_xyz[i]-pose[i])*self.c['max_setpoint_lead']/lead for i in range(3)]
        if math.dist(next_xyz,self.target[:3])>pace*dt+1e-9:
            self.hold('tracking_lead_rate_conflict');return self.target
        self.target=tuple(next_xyz)+(self.origin[3],)
        arrived=math.dist(pose[:3],goal[:3])<=self.c['arrival_tolerance'] and speed<=self.c['arrival_speed']
        if not arrived:self.arrival_since=None;self.dwell_since=None
        elif self.arrival_since is None:self.arrival_since=now
        elif now-self.arrival_since>=self.c['arrival_dwell']:
            if self.dwell_since is None:self.dwell_since=now
            duration=self.c['hover_seconds'] if self.index==0 else self.c['waypoint_seconds']
            if now-self.dwell_since>=duration:
                if self.index+1==len(self.goals):
                    self.stage='FINISHED_HOVER';self.reason='complete_manual_landing'
                else:
                    self.index+=1;self.reason='observation_leg_'+str(self.index)
                    self.arrival_since=None;self.dwell_since=None
        return self.target
