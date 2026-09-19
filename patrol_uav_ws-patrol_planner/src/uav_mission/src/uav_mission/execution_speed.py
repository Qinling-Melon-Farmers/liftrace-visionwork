"""Opt-in following-distance phases; distances are not promised airspeeds."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowingSpeed:
    cruise_lead_m: float = 1.0
    precision_lead_m: float = .4
    corridor_lead_m: float = .15
    corridor_after_waypoints: int = 1
    boundary_lead_m: float = .20

    def __post_init__(self):
        values=(self.cruise_lead_m,self.precision_lead_m,self.corridor_lead_m)
        if (not all(math.isfinite(v) for v in values)
                or not .05<=self.corridor_lead_m<=self.precision_lead_m<=self.cruise_lead_m<=1.5
                or not math.isfinite(self.boundary_lead_m) or not .05<=self.boundary_lead_m<=self.precision_lead_m
                or type(self.corridor_after_waypoints) is not int
                or self.corridor_after_waypoints!=1):
            raise ValueError('invalid following speed profile')

    def select(self,command,reason,completed,near_boundary=False):
        if near_boundary and command in ('SEARCH','RESUME','APPROACH'):
            return 'BOUNDARY_REVISIT',self.boundary_lead_m
        if command in ('SEARCH','RESUME'):
            return 'CRUISE',self.cruise_lead_m
        if command=='RETURN_HOME' and reason.startswith('post_delivery_route:'):
            if completed<self.corridor_after_waypoints:return 'TRANSIT_TO_CORRIDOR',self.cruise_lead_m
            return 'CORRIDOR',self.corridor_lead_m
        if command in ('LAND','HOLD','ABORT'):return 'TERMINAL',self.corridor_lead_m
        return 'PRECISION',self.precision_lead_m
