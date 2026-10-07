"""Constant-size quaternion box projection for independent flight validation."""
import math
def projected_bounds(xyz,quat,size=(.55,.55,.4),offset_z=-.02):
    if not all(math.isfinite(v) for v in (*xyz,*quat,*size,offset_z)) or min(size)<=0:
        raise ValueError('invalid envelope input')
    norm=math.sqrt(sum(v*v for v in quat))
    if abs(norm-1.)>.05:raise ValueError('invalid quaternion')
    x,y,z,w=(v/norm for v in quat)
    rows=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
          (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
          (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    out=[]
    for coordinate,row in zip(xyz,rows):
        center=coordinate+offset_z*row[2];extent=sum(abs(v)*s*.5 for v,s in zip(row,size))
        out.extend((center-extent,center+extent))
    return tuple(out)
def within_xy(projected,region,tolerance=1e-4):
    return (projected[0]>=region['min_x']-tolerance and projected[1]<=region['max_x']+tolerance and
            projected[2]>=region['min_y']-tolerance and projected[3]<=region['max_y']+tolerance)
