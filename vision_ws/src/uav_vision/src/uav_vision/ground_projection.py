"""Camera ray / horizontal plane intersection shared with the H projector."""
import math


def rotate_vector(vector, quaternion):
    x, y, z, w = quaternion
    values = tuple(vector) + tuple(quaternion)
    norm = x*x + y*y + z*z + w*w
    if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 1e-3:
        raise ValueError("projection_invalid_quaternion")
    # Same rigid transform as tf2_geometry_msgs.do_transform_point, without
    # translating the ray direction or depending on ROS in the math helper.
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def intersect_ground(ray, origin, quaternion, ground_z, epsilon=1e-5):
    direction = rotate_vector(ray, quaternion)
    if not all(math.isfinite(v) for v in tuple(origin)+(ground_z, epsilon)):
        raise ValueError("projection_nonfinite")
    if epsilon <= 0 or abs(direction[2]) < epsilon:
        raise ValueError("ray_parallel_ground")
    scale = (ground_z-origin[2])/direction[2]
    if scale <= 0:
        raise ValueError("intersection_behind_camera")
    point = (origin[0]+scale*direction[0], origin[1]+scale*direction[1], ground_z)
    if not all(math.isfinite(v) for v in point):
        raise ValueError("projection_nonfinite")
    return point
