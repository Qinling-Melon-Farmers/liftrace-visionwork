"""Geometry-only route hypotheses. Unknown/occupied segments need a planner."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SurveyRoute:
    family: str
    xy: tuple
    yaw: float
    fc_agl: float
    role: str = 'GEOMETRY_ONLY_NOT_COLLISION_CHECKED'

    @property
    def length(self):
        return sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.xy,self.xy[1:]))


def route_hypotheses(bounds, inset_x, inset_y, lane_spacing, fc_agl,
                     loop_yaw=0., lanes_yaw=math.pi/2):
    if (len(bounds)!=4 or not all(math.isfinite(v) for v in
            tuple(bounds)+(inset_x,inset_y,lane_spacing,fc_agl,loop_yaw,lanes_yaw))):
        raise ValueError('finite geometry required')
    x0,x1,y0,y1=bounds
    if not (0<2*inset_x<x1-x0 and 0<2*inset_y<y1-y0
            and lane_spacing>0 and 2.0<=fc_agl<=3.0):
        raise ValueError('invalid survey bounds')
    a,b,c,d=x0+inset_x,x1-inset_x,y0+inset_y,y1-inset_y
    count=math.ceil((d-c)/lane_spacing)+1
    if count>16:
        raise ValueError('at most sixteen lanes')
    loop=((a,c),(b,c),(b,d),(a,d),(a,c))
    lanes=[]
    # Evenly distribute endpoints; avoid the legacy tiny final lane gap.
    for index in range(count):
        y=c+(d-c)*index/(count-1)
        lanes.extend(((a,y),(b,y)) if index%2==0 else ((b,y),(a,y)))
    return (SurveyRoute('inner_loop',loop,loop_yaw,fc_agl),
            SurveyRoute('loop_plus_center',loop+(((a+b)/2,c),((a+b)/2,d)),loop_yaw,fc_agl),
            SurveyRoute('sparse_lanes',tuple(lanes),lanes_yaw,fc_agl))
