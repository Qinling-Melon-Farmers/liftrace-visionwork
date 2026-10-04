"""Opt-in motion policies. No ROS, actuator calls or replacement mission FSM."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class MotionOptimization:
    enabled: bool = False
    dynamic_boundary: bool = True
    moving_recovery: bool = True
    diagonal_entry: bool = True
    merge_collinear_relays: bool = True
    survey_line_weight: float = 2.0
    braking_speed_mps: float = 1.2
    braking_accel_mps2: float = .6
    reaction_seconds: float = .3
    boundary_reserve_m: float = .15
    recovery_handoff_agl: float = .9
    recovery_min_ack_seconds: float = .6
    recovery_max_vz: float = .6
    corridor_max_agl: float = 1.0

    def __post_init__(self):
        for name in ('enabled','dynamic_boundary','moving_recovery','diagonal_entry','merge_collinear_relays'):
            if type(getattr(self,name)) is not bool: raise ValueError(name+' must be boolean')
        names=('survey_line_weight','braking_speed_mps','braking_accel_mps2','reaction_seconds',
               'boundary_reserve_m','recovery_handoff_agl','recovery_min_ack_seconds','recovery_max_vz','corridor_max_agl')
        if any(isinstance(getattr(self,k),bool) or not math.isfinite(getattr(self,k)) for k in names):
            raise ValueError('non-finite motion optimization')
        if not (0<=self.survey_line_weight<=10 and .1<=self.braking_speed_mps<=1.5
                and .1<=self.braking_accel_mps2<=1. and .1<=self.reaction_seconds<=1.
                and .05<=self.boundary_reserve_m<=.5 and .75<=self.recovery_handoff_agl<=1.4
                and .3<=self.recovery_min_ack_seconds<=3. and .1<=self.recovery_max_vz<=.8
                and .8<=self.corridor_max_agl<=1.2):
            raise ValueError('motion optimization outside validated design ranges')

    def boundary_slow(self, boundary, xy, goal, fresh, was_slow=False):
        if not fresh or xy is None or not all(math.isfinite(v) for v in (*xy,*goal)): return True
        v=self.braking_speed_mps
        stop=v*v/(2*self.braking_accel_mps2)+v*self.reaction_seconds+self.boundary_reserve_m
        hysteresis=.2 if was_slow else 0.
        # Straight distance is a lower bound on path length: it slows early on a detour,
        # never presumes that an unseen bend is a short safe path.
        return (boundary.clearance(xy)<=boundary.margin+stop+hysteresis or
                (boundary.near(goal) and math.dist(xy,goal)<=stop+1.+hysteresis))

class MovingRecoveryWindow:
    """Fresh post-ACK evidence at the minimum handoff plane, without stopping ascent."""
    def __init__(self):
        self.first=None
        self.last=None
    def reset(self):
        self.first=None; self.last=None
    def update(self, sample, ack_ns, now_ns, min_ack_seconds, max_vz):
        valid=(sample.stamp_ns>=ack_ns and now_ns-ack_ns>=int(min_ack_seconds*1e9)
               and math.hypot(sample.vx,sample.vy)<=.25 and -.03<=sample.vz<=max_vz)
        if not valid:
            self.reset();return False
        stamp=sample.stamp_ns
        if self.last is not None and stamp<=self.last: return False
        if self.last is None or stamp-self.last>200_000_000: self.first=stamp
        self.last=stamp
        return stamp-self.first>=150_000_000

def optimize_post_route(points, options, corridor_points_count, wall_axis, wall_coordinates):
    """Collapse only explicitly low, collinear corridor relays; retain turns and H suffix.
    Removed points remain in metadata. The 3-D collision planner owns the whole segment.
    """
    route=[list(p) for p in points]
    original=[p[:] for p in route]
    if not options.enabled: return route,dict(original=original,diagonal_entry=False,removed=[])
    if not route or not 1<=corridor_points_count<=len(route): raise ValueError('invalid corridor prefix')
    entry=False; removed=[]
    if (options.diagonal_entry and corridor_points_count>=2 and
            math.dist(route[0][:2],route[1][:2])<1e-6 and route[0][2]>route[1][2]+.05):
        removed.append(route.pop(0));corridor_points_count-=1;entry=True
    # First corridor waypoint must explicitly be a low entrance before any gate wall.
    if wall_axis not in (0,1) or not wall_coordinates: raise ValueError('known door planes required')
    if any(not math.isfinite(w) for w in wall_coordinates):raise ValueError('invalid wall plane')
    if options.merge_collinear_relays:
        i=1
        while i<corridor_points_count-1:
            a,b,c=route[i-1:i+2]
            u=[b[j]-a[j] for j in range(3)];v=[c[j]-b[j] for j in range(3)]
            cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            # No vertical segments, no reversal and no corners may disappear.
            if (max(abs(b[2]-a[2]),abs(c[2]-b[2]))<1e-6 and
                    sum(t*t for t in cross)<1e-10 and sum(x*y for x,y in zip(u,v))>0):
                removed.append(route.pop(i));corridor_points_count-=1
            else:i+=1
    return route,dict(original=original,diagonal_entry=entry,removed=removed,
                      corridor_points_count=corridor_points_count)
