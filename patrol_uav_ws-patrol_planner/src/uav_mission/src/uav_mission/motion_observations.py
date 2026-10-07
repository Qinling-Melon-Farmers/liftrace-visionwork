"""Observation policies; no timestamp rewriting and no inferred twist convention."""
import math

def odom_world_velocity(message, convention):
    q=message.pose.pose.orientation;v=message.twist.twist.linear
    values=(q.x,q.y,q.z,q.w,v.x,v.y,v.z)
    if not all(math.isfinite(float(x)) for x in values):raise ValueError('non-finite odometry')
    n=sum(float(x)**2 for x in values[:4])
    if abs(n-1.)>1e-3:raise ValueError('invalid odometry attitude')
    if not message.header.frame_id:raise ValueError('missing odometry world frame')
    if convention=='header':return v.x,v.y,v.z
    if convention!='child' or not message.child_frame_id:raise ValueError('ambiguous odometry twist frame')
    x,y,z,w=(float(a)/math.sqrt(n) for a in values[:4])
    # Full child->header rotation, not yaw-only.
    return ((1-2*(y*y+z*z))*v.x+2*(x*y-z*w)*v.y+2*(x*z+y*w)*v.z,
            2*(x*y+z*w)*v.x+(1-2*(x*x+z*z))*v.y+2*(y*z-x*w)*v.z,
            2*(x*z-y*w)*v.x+2*(y*z+x*w)*v.y+(1-2*(x*x+y*y))*v.z)

class FreshPoseWindow:
    def __init__(self):self.blocked=False;self.reset()
    def reset(self):self.first=None;self.last=None;self.count=0
    def update(self,stamp,now,max_age):
        if not all(math.isfinite(x) for x in (stamp,now)) or stamp<=0 or not 0<=now-stamp<=max_age:
            self.blocked=True;self.reset();return False
        if not self.blocked:return True
        if self.last is not None and stamp<self.last:self.reset()
        if self.last==stamp:return False
        if self.last is None or stamp-self.last>max_age:
            self.first=stamp;self.count=0
        self.last=stamp;self.count+=1
        if self.count>=3 and stamp-self.first>=.2-1e-9:self.blocked=False;return True
        return False
