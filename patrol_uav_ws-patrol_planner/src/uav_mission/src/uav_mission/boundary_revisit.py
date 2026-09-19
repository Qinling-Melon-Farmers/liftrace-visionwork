"""Interior revisit proposals; high cues are never release authorization."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BoundaryRevisit:
    enabled: bool = False
    bounds: tuple = (-4.8, 4.8, -.5, 7.4)
    guard_side_m: float = .55  # already inflated engineering envelope
    yaw_budget_deg: float = 10.
    tracking_reserve_m: float = .03  # explicit additional tracking allowance
    max_view_offset_m: float = .65

    def __post_init__(self):
        if (type(self.enabled) is not bool or len(self.bounds) != 4 or
            not all(math.isfinite(v) for v in tuple(self.bounds)+(self.guard_side_m,
                self.yaw_budget_deg,self.tracking_reserve_m,self.max_view_offset_m)) or
            not 0 < self.guard_side_m <= 1 or not 0 <= self.yaw_budget_deg <= 45 or
            not 0 <= self.tracking_reserve_m <= .1 or not 0 < self.max_view_offset_m <= 1 or
            self.bounds[1]-self.bounds[0] <= 2*self.margin or
            self.bounds[3]-self.bounds[2] <= 2*self.margin):
            raise ValueError('invalid boundary revisit configuration')

    @property
    def margin(self):
        yaw=math.radians(self.yaw_budget_deg)
        return .5*self.guard_side_m*(math.cos(yaw)+math.sin(yaw))+self.tracking_reserve_m

    def clearance(self, xy):
        x,y=xy; a,b,c,d=self.bounds
        return min(x-a,b-x,y-c,d-y)

    def admissible(self, xy):
        return all(math.isfinite(v) for v in xy) and self.clearance(xy)>=self.margin

    def viewpoint(self, xy, uncertainty):
        if not all(math.isfinite(v) for v in (*xy,uncertainty)) or not 0<=uncertainty<=.5:
            return None
        if not self.enabled:return tuple(xy)
        a,b,c,d=self.bounds; margin=self.margin+uncertainty
        point=(min(b-margin,max(a+margin,xy[0])),min(d-margin,max(c+margin,xy[1])))
        return point if math.dist(point,xy)<=self.max_view_offset_m and self.admissible(point) else None

    def near(self, xy):
        return self.enabled and self.clearance(xy)<self.margin+self.max_view_offset_m

    def slow_coverage(self, xy, goal):
        """Brake near a field edge without slowing an entire coverage strip."""
        return self.enabled and (self.clearance(xy)<.85 or
                                 (self.near(goal) and math.dist(xy,goal)<1.0))
