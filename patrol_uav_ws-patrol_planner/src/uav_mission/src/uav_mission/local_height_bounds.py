"""Task-local coordinate floor; ordinary missions retain their original limits."""
import math
import os

def constant_height_profile():
    return os.environ.get('UAV_BOARD_VISUAL_CONSTANT_AGL') == '0.4'

def min_local_z():
    if not constant_height_profile():
        return 0.0
    value=float(os.environ['UAV_BOARD_VISUAL_MIN_LOCAL_Z'])
    if not math.isfinite(value) or not -.55 <= value <= .25:
        raise ValueError('invalid board visual local-Z floor')
    return value
