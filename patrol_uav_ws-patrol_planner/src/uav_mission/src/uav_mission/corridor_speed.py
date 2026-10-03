"""Pose-driven following-distance schedule; fixed wall planes, no opening truth."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class CorridorSpeedConfig:
    axis: int = 1
    wall_coordinates: tuple = (-1.6, 1.6)
    enter_distance_m: float = .75
    exit_distance_m: float = .95
    open_lead_m: float = .4
    door_lead_m: float = .15
    landing_radius_m: float = .8

    def __post_init__(self):
        if (self.axis not in (0,1) or not self.wall_coordinates or
            not all(math.isfinite(v) for v in (*self.wall_coordinates,self.enter_distance_m,self.exit_distance_m,self.open_lead_m,self.door_lead_m,self.landing_radius_m)) or
            not 0<self.enter_distance_m<self.exit_distance_m or
            not .05<=self.door_lead_m<=self.open_lead_m<=.6 or self.landing_radius_m<.5):
            raise ValueError('invalid corridor speed configuration')

class CorridorSpeed:
    def __init__(self,config):self.config=config;self.slow=True
    def select(self,xy,landing_xy,completed):
        c=self.config
        if xy is None or not all(math.isfinite(v) for v in xy):return 'CORRIDOR_UNCERTAIN',c.door_lead_m
        # completed==1 is the deliberate staging descent, not fast transit.
        if completed<2:return 'CORRIDOR_DESCENT',c.door_lead_m
        if math.dist(xy,landing_xy)<=c.landing_radius_m:return 'H_APPROACH',c.door_lead_m
        distance=min(abs(xy[c.axis]-v) for v in c.wall_coordinates)
        if distance<=c.enter_distance_m:self.slow=True
        elif distance>=c.exit_distance_m:self.slow=False
        return ('DOOR',c.door_lead_m) if self.slow else ('CORRIDOR_OPEN',c.open_lead_m)
