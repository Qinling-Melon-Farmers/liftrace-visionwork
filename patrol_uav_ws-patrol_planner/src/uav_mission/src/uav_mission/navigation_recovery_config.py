"""Parameter wiring for the existing bounded navigation recovery candidate.

Opt-in only: adapting the implementation is not whole-aircraft acceptance.
No EV prediction, reset attestation, or new mission authority is connected.
"""
import math


def recovery_parameters(settings, ground_z):
    supplied = settings.get('navigation_recovery', {})
    if not isinstance(supplied, dict) or set(supplied) - {'enabled'}:
        raise ValueError('navigation_recovery accepts only enabled')
    enabled = supplied.get('enabled', False)
    if type(enabled) is not bool or not math.isfinite(ground_z):
        raise ValueError('invalid navigation_recovery enabled or ground reference')
    common = dict(enabled=enabled, max_speed=.30, max_seconds=8.,
                  hard_min_z=ground_z+.05, hard_max_z=ground_z+3.50,
                  max_height_excess=.25)
    params = {}
    for prefix in ('/navigation_recovery', '/traj_server/navigation_recovery',
                   '/fast_planner_node/navigation_recovery'):
        params.update({prefix+'/'+key: value for key, value in common.items()})
    params['/navigation/planner_bridge/navigation_recovery/enabled'] = enabled
    params.update({'/fast_planner_node/navigation_recovery/'+key: value
                   for key, value in dict(radius=.8, odom_twist_frame='child', max_acceleration=.5,
                       max_input_age=.25, max_map_age=.5, tracking_error=.08).items()})
    params.update({'/fast_planner_node/sdf_map/'+key: value for key, value in
                   dict(recovery_layers_enabled=enabled, recovery_body_xy=.39,
                        recovery_body_up=.20, recovery_body_down=.20).items()})
    return params
