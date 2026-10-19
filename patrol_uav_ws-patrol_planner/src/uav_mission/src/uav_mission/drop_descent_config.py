"""Delivery-only descent settings; no H, actuator, route or PX4 parameter changes."""
import math


def drop_horizontal_compensation_mode(settings):
    mode = settings.get('drop_horizontal_compensation_mode', 2)
    if type(mode) is not int or mode not in (1, 2):
        raise ValueError('drop_horizontal_compensation_mode must be 1 or 2')
    return mode


def drop_descent_parameters(settings):
    mode = settings.get('drop_descent_mode', 1)
    speed = settings.get('drop_descent_speed_mps', 0.20)
    if type(mode) is not int or mode not in (1, 2, 3, 4):
        raise ValueError('drop_descent_mode must be 1, 2, 3 or 4')
    max_speed = 3.0 if mode == 2 else 1.0
    if (isinstance(speed, bool) or not isinstance(speed, (int, float)) or
            not math.isfinite(speed) or not 0.0 < speed <= max_speed):
        raise ValueError('drop_descent_speed_mps must be in (0, %g] for mode %d' % (max_speed, mode))
    result = dict(mode=mode, speed_mps=float(speed), max_step_dt_sec=0.20,
                  zero_velocity_setpoint_topic='/navigation/drop_setpoint_vertical',
                  velocity_setpoint_topic='/navigation/drop_setpoint_velocity')
    for source, key, default, upper in (
            ('drop_descent_mode4_speed_mps', 'velocity_speed_mps', .60, 1.0),
            ('drop_descent_acceleration_mps2', 'acceleration_mps2', .60, 2.0),
            ('drop_descent_braking_mps2', 'braking_mps2', .60, 2.0),
            ('drop_descent_hold_tolerance_m', 'hold_tolerance_m', .015, .03)):
        value = settings.get(source, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or not 0.0 < value <= upper):
            raise ValueError('%s must be in (0, %s]' % (source, upper))
        result[key] = float(value)
    return result
