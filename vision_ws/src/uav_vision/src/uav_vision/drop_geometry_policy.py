"""Pure admission and measured-slot conventions for experimental drop geometry."""
import math
from uav_vision.ground_projection import rotate_vector


def observation_age(stamp, now):
    if not all(math.isfinite(v) for v in (stamp, now)) or stamp <= 0 or now < stamp:
        return float("inf")
    return now-stamp


class ObservationFence:
    """Only newer images count; duplicate/older packets neither revoke nor renew."""
    def __init__(self):
        self.latest = 0
        self.identity = None

    def accept(self, stamp_ns, identity):
        if stamp_ns < self.latest:
            return "out_of_order_observation"
        if stamp_ns == self.latest:
            return "duplicate_observation"
        changed = identity != self.identity
        self.latest, self.identity = stamp_ns, identity
        return "new_identity" if changed else "new_observation"


def calibrated_camera_info(info, source_frame):
    if not source_frame or info.header.frame_id != source_frame:
        return False
    values = tuple(info.K)+tuple(info.P)+tuple(info.R)+tuple(info.D)
    return (info.width > 0 and info.height > 0 and len(info.K) == 9 and
            len(info.P) == 12 and len(info.R) == 9 and
            all(math.isfinite(v) for v in values) and
            info.K[0] > 0 and info.K[4] > 0 and info.P[0] > 0 and info.P[5] > 0 and
            abs(info.K[8]-1) < 1e-6 and abs(info.P[10]-1) < 1e-6 and
            info.distortion_model in ("plumb_bob", "rational_polynomial"))


def match_source_frame(target, frame):
    """Memory loses source frame: recover it only from the exact observed geometry."""
    if frame.header.stamp != target.last_seen or not frame.header.frame_id:
        raise ValueError("source_observation_missing")
    matches = [d for d in frame.detections if
               d.class_name == target.class_name and d.center_refined and
               d.association_valid and not d.reject_reason and
               all(abs(getattr(d.center_px, a)-getattr(target.center_px, a)) < 1e-4
                   for a in ("x", "y", "z"))]
    if len(matches) != 1:
        raise ValueError("source_geometry_ambiguous_or_missing")
    return frame.header.frame_id


def slot_goal(point, body_origin, orientation, slot, offsets, mode):
    """physical_body_position means measured FLU slot position relative to FC.

    FC goal = target minus rotated slot position. The legacy opt-in preserves
    the old signed target-shift convention only for explicit comparisons.
    """
    if mode not in ("zero", "physical_body_position", "legacy_body_target_shift"):
        raise ValueError("slot_compensation_mode_invalid")
    if not 1 <= slot <= 3:
        raise ValueError("payload_slot_invalid")
    dx, dy = (0., 0.) if mode == "zero" else offsets[slot-1]
    if mode == "physical_body_position":
        dx, dy = -dx, -dy
    if not all(math.isfinite(v) for v in tuple(point)+tuple(body_origin)+(dx, dy)):
        raise ValueError("slot_geometry_nonfinite")
    if isinstance(orientation, (int, float)):
        orientation = (0., 0., math.sin(orientation/2), math.cos(orientation/2))
    world = rotate_vector((dx, dy, 0.), orientation)
    goal = (point[0]+world[0], point[1]+world[1], point[2])
    return goal, math.hypot(goal[0]-body_origin[0], goal[1]-body_origin[1])


def pixel_equivalent_limit(jacobian, error_xy, max_pixels):
    """Convert the original pixel circle along the present metric-error direction.

    J is a central one-pixel finite difference of the full exposure projection,
    including distortion and attitude. This is a local tolerance conversion,
    not a new absolute metre threshold or a pixel-main-point release condition.
    """
    a, b, c, d = jacobian
    x, y = error_xy
    if not all(math.isfinite(v) for v in (a,b,c,d,x,y,max_pixels)) or max_pixels <= 0:
        raise ValueError("alignment_tolerance_invalid")
    det = a*d-b*c
    if abs(det) <= 1e-12:
        raise ValueError("alignment_projection_singular")
    equivalent = math.hypot((d*x-b*y)/det, (-c*x+a*y)/det)
    error = math.hypot(x,y)
    if error <= 1e-12:
        return max_pixels*min(math.hypot(a,c), math.hypot(b,d))
    return max_pixels*error/equivalent
