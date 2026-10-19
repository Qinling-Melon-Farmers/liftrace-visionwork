"""Opt-in measured-H fallback for the hardware external LAND path only.

The controller starts wait_sec on arrival at preset H (XY/Z), then freezes
that preset center and checks descent XY/speed stability. Cancellation resets
that runtime state; this module only validates and translates configuration.
"""
import math

DEFAULTS = dict(enabled=False, wait_sec=15.0,
                arrival_xy_tolerance_m=0.20, arrival_z_tolerance_m=0.15,
                descend_xy_tolerance_m=0.08, max_horizontal_speed_mps=0.10,
                stable_duration_sec=0.30)


def landing_coordinate_fallback_parameters(settings, *, external_land=False):
    supplied = settings.get('landing_coordinate_fallback', {})
    if not isinstance(supplied, dict) or set(supplied) - set(DEFAULTS):
        raise ValueError('invalid landing_coordinate_fallback fields')
    values = dict(DEFAULTS, **supplied)
    if type(values['enabled']) is not bool:
        raise ValueError('landing_coordinate_fallback enabled must be boolean')
    for key, value in values.items():
        if key == 'enabled':
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid landing_coordinate_fallback ' + key)
        values[key] = float(value)
    if values['wait_sec'] > 60.0:
        raise ValueError('landing_coordinate_fallback wait_sec must be in (0, 60]')
    if values['arrival_xy_tolerance_m'] < values['descend_xy_tolerance_m']:
        raise ValueError('landing_coordinate_fallback arrival XY must be >= descent XY')
    if values['enabled']:
        if not external_land or settings.get('landing_handoff_mode') != 'POSCTL':
            raise ValueError('landing_coordinate_fallback requires external LAND with POSCTL')
        xy = settings.get('landing_xy')
        if (not isinstance(xy, (list, tuple)) or len(xy) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in xy)):
            raise ValueError('landing_coordinate_fallback requires finite measured landing_xy')
    return values
