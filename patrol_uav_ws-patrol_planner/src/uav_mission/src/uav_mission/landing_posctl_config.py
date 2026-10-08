"""硬件配置生成器共用的 POSCTL 低位 H 交接参数。

H 捕获保持原 8cm、10 帧新鲜图像与位置冻结，不增加高位运动窗口；
独立高位高度容差仍保留，低位以安全高度带和连续新鲜反馈允许受控缓降。
不使用下降中的残缺图形重定位。AUTO.LAND 保持原 0.40/0.55m 离地高度。
"""
import math


DEFAULTS = dict(
    target_agl=0.35,
    trigger_agl=0.37,
    xy_tolerance_m=0.05,
    height_tolerance_m=0.02,
    capture_height_tolerance_m=0.10,
    max_horizontal_speed_mps=0.08,
    max_vertical_speed_mps=0.10,
    stable_duration_sec=0.15,
    max_odom_age_sec=0.2,
    max_sample_gap_sec=0.2,
    min_samples=3,
)


def validate_landing_posctl(settings):
    supplied = settings.get('landing_posctl', {})
    if not isinstance(supplied, dict) or set(supplied) - set(DEFAULTS):
        raise ValueError('invalid landing_posctl fields')
    values = dict(DEFAULTS, **supplied)
    for key, value in values.items():
        if key == 'min_samples':
            if type(value) is not int or value < 3:
                raise ValueError('landing_posctl min_samples must be an integer >= 3')
        elif (isinstance(value, bool) or not isinstance(value, (int, float))
              or not math.isfinite(value) or value <= 0):
            raise ValueError('invalid landing_posctl ' + key)
    if values['trigger_agl'] < values['target_agl']:
        raise ValueError('landing_posctl trigger_agl must be >= target_agl')
    if values['xy_tolerance_m'] > 0.08:
        raise ValueError('landing_posctl XY tolerance must not exceed H acquisition 0.08 m')
    if (settings.get('landing_handoff_mode', 'AUTO.LAND') == 'POSCTL'
            and values['trigger_agl'] >= settings['landing_capture_agl']):
        raise ValueError('landing_posctl trigger must be below H capture height')
    return values


def landing_control_parameters(settings, ground_z, fc_ground_clearance):
    values = validate_landing_posctl(settings)
    if settings.get('landing_handoff_mode', 'AUTO.LAND') != 'POSCTL':
        return ground_z + 0.40, ground_z + 0.55, None
    if (not math.isfinite(fc_ground_clearance) or fc_ground_clearance <= 0
            or values['target_agl'] - values['height_tolerance_m'] <= fc_ground_clearance):
        raise ValueError('landing_posctl height band must clear stationary FC height')
    stability = {key: value for key, value in values.items()
                 if key not in ('target_agl', 'trigger_agl')}
    return ground_z + values['target_agl'], ground_z + values['trigger_agl'], stability
