"""Experimental profile shared with research; hardware ceilings and release gates unchanged."""
from dataclasses import asdict
import math,struct
from uav_mission.motion_optimization import MotionOptimization,optimize_post_route


def motion_options(settings):
    raw=settings.get('motion_optimization',{})
    if not isinstance(raw,dict):raise ValueError('motion_optimization must be an object')
    values=dict(braking_speed_mps=float(settings['cruise_speed']),braking_accel_mps2=float(settings['cruise_acceleration']))
    values.update(raw)
    if settings['mode'] in ('landing','memory_only','high_speed_capture'):values['moving_recovery']=False
    options=MotionOptimization(**values)
    if options.recovery_handoff_agl>=settings['low_agl']:raise ValueError('Recovery handoff must be below revisit altitude')
    if options.braking_speed_mps<settings['cruise_speed']:raise ValueError('Braking reserve must cover configured cruise speed')
    if options.braking_accel_mps2>settings['cruise_acceleration']:raise ValueError('Braking reserve cannot assume stronger acceleration')
    return options


def apply_generated_motion(runtime,control,overrides,settings,ground):
    options=motion_options(settings)
    report=dict(options=asdict(options),survey_pattern=settings.get('survey_pattern','site_waypoints'),scope='RESEARCH_ONLY_NOT_DEPLOYED',post_route='unchanged')
    runtime['motion_optimization']=asdict(options)
    weight=float(settings.get('planner_line_preference_weight',2.0))
    if not math.isfinite(weight) or not 0<=weight<=10:raise ValueError('invalid global line weight')
    runtime['planner_line_preference']={'weight':weight}
    overrides['/fast_planner_node/search/line_deviation_weight']=weight
    overrides['/navigation/planner_bridge/motion_optimization']=asdict(options)
    overrides['/navigation/planner_bridge/execution/odom_twist_frame']='child'
    report['global_line_preference_weight']=weight
    if not options.enabled:return report
    if options.moving_recovery:
        height=struct.unpack('f',struct.pack('f',ground+options.recovery_handoff_agl))[0]
        control['uav_vision']['recovery_height']=height
        overrides['/navigation/planner_bridge/target/recovery_height']=height
        report['recovery_handoff_local_z']=height
    if settings.get('trial_kind') in ('corridor_landing','full_mission'):
        geometry=settings.get('corridor_geometry')
        if geometry is None:
            report['post_route']='Retained: provide measured corridor_geometry before merging relays/diagonal entry'
            return report
        if not isinstance(geometry,dict) or set(geometry)!={'wall_axis','wall_coordinates','entry_waypoints'}:raise ValueError('Measured corridor_geometry requires wall_axis, wall_coordinates and entry_waypoints')
        original=runtime['mission']['post_delivery_route'];count=len(settings['corridor_waypoints'])
        entry=geometry['entry_waypoints']
        if type(entry) is not int or not 1<=entry<=count:raise ValueError('Invalid corridor entry waypoint count')
        if any(p[2]>ground+options.corridor_max_agl+1e-6 for p in original[entry-1:count]):raise ValueError('Corridor waypoints exceed configured height cap')
        # Prevent removal of mandatory descent/turn/landing points. The helper only
        # removes strictly collinear horizontal relays and an explicit high/low pair.
        route,detail=optimize_post_route(original,options,count,geometry['wall_axis'],geometry['wall_coordinates'])
        anchor=original[entry-1]
        # Entry transition must remain a waypoint; retain original route otherwise.
        if anchor not in route:
            report['post_route']='Retained: entry transition would be removed'
            return report
        new_entry=route.index(anchor)+1
        m=runtime['mission'];m['post_delivery_route']=route
        m['post_delivery_route_revision']+='-motion-opt'
        m['post_delivery_parameter_stages'].append(dict(after_completed_waypoints=new_entry,parameters={
            '/external_planner_max_command_z':ground+options.corridor_max_agl,
            '/navigation/planner_bridge/execution/max_goal_z':ground+options.corridor_max_agl}))
        # H scan/capture is a different stage. Restore its original limits after
        # the explicit corridor prefix, before the preserved landing suffix.
        m['post_delivery_parameter_stages'].append(dict(after_completed_waypoints=detail['corridor_points_count'],parameters={
            '/external_planner_max_command_z':ground+settings.get('max_agl',2.9),
            '/navigation/planner_bridge/execution/max_goal_z':ground+settings.get('max_agl',2.9)}))
        runtime['corridor_speed_schedule']=dict(axis=geometry['wall_axis'],wall_coordinates=geometry['wall_coordinates'],entry_waypoints=new_entry,open_lead_m=min(.6,runtime['following_speed_profile']['cruise_lead_m']),door_lead_m=min(.15,runtime['following_speed_profile']['corridor_lead_m']))
        from uav_mission.corridor_speed import CorridorSpeedConfig
        CorridorSpeedConfig(**runtime['corridor_speed_schedule'])
        report['post_route']=detail
    return report
