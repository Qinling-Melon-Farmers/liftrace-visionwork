"""Rendering-only camera geometry; never a flight goal or navigation input."""
from dataclasses import dataclass
import math


def look_at(camera, target, vertical_yaw=0.):
    dx,dy,dz=(b-a for a,b in zip(camera,target))
    if math.sqrt(dx*dx+dy*dy+dz*dz)<1e-6:raise ValueError('coincident camera/target')
    yaw=math.atan2(dy,dx) if math.hypot(dx,dy)>1e-8 else vertical_yaw
    pitch=-math.atan2(dz,math.hypot(dx,dy))
    sy,cy=math.sin(yaw/2),math.cos(yaw/2);sp,cp=math.sin(pitch/2),math.cos(pitch/2)
    # Gazebo camera views along local +X, with a level horizon (no aircraft roll).
    return (-sy*sp,cy*sp,sy*cp,cy*cp)


def intersects(a,b,box):
    """Segment intersects an axis-aligned visual wall (six min/max values)."""
    tmin,tmax=0.,1.
    for i in range(3):
        lo,hi=box[2*i:2*i+2];delta=b[i]-a[i]
        if abs(delta)<1e-10:
            if a[i]<lo or a[i]>hi:return False
        else:
            near,far=sorted(((lo-a[i])/delta,(hi-a[i])/delta))
            tmin=max(tmin,near);tmax=min(tmax,far)
            if tmin>tmax:return False
    return True


@dataclass(frozen=True)
class FollowView:
    distance: float = 1.8
    lift: float = 1.0
    min_z: float = 1.2
    max_z: float = 4.6
    bounds: tuple = (-4.55,4.55,-.25,8.85)
    smoothing_s: float = .35
    def __post_init__(self):
        if (len(self.bounds)!=4 or not all(math.isfinite(v) for v in (*self.bounds,self.distance,self.lift,self.min_z,self.max_z,self.smoothing_s))
                or not .5<=self.distance<=4 or not .2<=self.lift<=3 or not 0<self.min_z<self.max_z
                or not .05<=self.smoothing_s<=2 or self.bounds[0]>=self.bounds[1] or self.bounds[2]>=self.bounds[3]):
            raise ValueError('invalid render camera configuration')

    def propose(self,body,yaw,previous,dt,walls=()):
        if not all(math.isfinite(v) for v in (*body,yaw,dt)) or dt<0:raise ValueError('invalid camera state')
        x,y,z=body;left,right,bottom,top=self.bounds
        zc=min(self.max_z,max(self.min_z,z+self.lift));options=[]
        for offset in (0,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,3*math.pi/4,-3*math.pi/4,math.pi):
            angle=yaw+math.pi+offset
            point=(min(right,max(left,x+self.distance*math.cos(angle))),
                   min(top,max(bottom,y+self.distance*math.sin(angle))),zc)
            if math.dist(point,body)<.5 or any(intersects(point,body,w) for w in walls):continue
            cost=abs(offset)+(.4*math.dist(point,previous) if previous else 0)
            options.append((cost,point))
        if not options:
            point=(min(right,max(left,x)),min(top,max(bottom,y)),self.max_z)
            mode='OVERHEAD_FALLBACK'
        else:point=min(options)[1];mode='FOLLOW'
        if previous is not None and dt<=1.0:
            alpha=1-math.exp(-dt/self.smoothing_s)
            blended=tuple(a+alpha*(b-a) for a,b in zip(previous,point))
            if not any(intersects(blended,body,w) for w in walls):point=blended
        visible=not any(intersects(point,body,w) for w in walls)
        return dict(xyz=point,xyzw=look_at(point,body),mode=mode,wall_los_clear=visible)
