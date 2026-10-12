"""Opt-in H mode 4 configuration; no ROS or flight operations."""
import copy
import math


DEFAULTS = dict(descent_speed=0.30, touchdown_overtravel=0.04,
                ground_confirm_duration=0.3, state_max_age=2.5,
                disarm_retry=1.0)
DEFAULT_TOPIC = '/navigation/landing_setpoint_touchdown'
OVERRIDE_KEYS = frozenset(('landing_handoff_mode', 'landing_alignment_mode',
                           'landing_descent_mode', 'landing_descent_speed_mps',
                           'landing_offboard'))


def validate_landing_offboard(settings):
    handoff = settings.get('landing_handoff_mode', 'AUTO.LAND')
    if handoff not in ('AUTO.LAND', 'POSCTL', 'OFFBOARD'):
        raise ValueError('landing_handoff_mode must be AUTO.LAND, POSCTL or OFFBOARD')
    mode = settings.get('landing_descent_mode', 1)
    if (mode == 4) != (handoff == 'OFFBOARD'):
        raise ValueError('landing_descent_mode 4 requires landing_handoff_mode OFFBOARD and vice versa')
    supplied = settings.get('landing_offboard', {})
    if not isinstance(supplied, dict) or set(supplied) - (set(DEFAULTS) | {'setpoint_topic'}):
        raise ValueError('invalid landing_offboard fields')
    values = dict(DEFAULTS, **{key: value for key, value in supplied.items() if key != 'setpoint_topic'})
    for key, value in values.items():
        if (type(value) not in (int, float) or not math.isfinite(value) or value <= 0):
            raise ValueError('invalid landing_offboard ' + key)
    if values['descent_speed'] > 1.0:
        raise ValueError('landing_offboard descent_speed must be in (0, 1]')
    topic = supplied.get('setpoint_topic', DEFAULT_TOPIC)
    if (not isinstance(topic, str) or not topic.startswith('/') or len(topic) < 2
            or any(not (ch.isascii() and (ch.isalnum() or ch in '/_')) for ch in topic)):
        raise ValueError('invalid landing_offboard setpoint_topic')
    return values


def landing_offboard_parameters(settings, ground_z, fc_ground_clearance):
    values = validate_landing_offboard(settings)
    if settings.get('landing_handoff_mode', 'AUTO.LAND') != 'OFFBOARD':
        return {}
    if (type(ground_z) not in (int, float) or not math.isfinite(ground_z)
            or type(fc_ground_clearance) not in (int, float)
            or not math.isfinite(fc_ground_clearance) or fc_ground_clearance <= 0):
        raise ValueError('OFFBOARD requires finite measured ground and positive FC clearance')
    if values['touchdown_overtravel'] >= fc_ground_clearance:
        raise ValueError('landing_offboard touchdown_overtravel must be below FC clearance')
    if fc_ground_clearance >= 0.30:
        raise ValueError('OFFBOARD staging AGL 0.30 must clear stationary FC height')
    capture = settings.get('landing_capture_agl', 0)
    if type(capture) not in (int, float) or not math.isfinite(capture) or capture <= 0.37:
        raise ValueError('OFFBOARD staging must be below H capture height')
    return dict(offboard_ground_z=float(ground_z),
                offboard_fc_clearance=float(fc_ground_clearance),
                offboard_setpoint_topic=settings.get('landing_offboard', {}).get('setpoint_topic', DEFAULT_TOPIC),
                **{'offboard_' + key: float(value) for key, value in values.items()})


def apply_landing_override(settings, override):
    """Merge a small candidate patch, preserving measured field and fallback policy."""
    if not isinstance(override, dict) or not override or set(override) - OVERRIDE_KEYS:
        raise ValueError('landing override only accepts H mode/alignment/descent/OFFBOARD settings')
    result = copy.deepcopy(settings)
    result.update(copy.deepcopy(override))
    # Shared validator also checks descent types, speed and alignment choices.
    from uav_mission.landing_posctl_config import validate_landing_posctl
    validate_landing_posctl(result)
    return result
