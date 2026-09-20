"""Generate all local-Z parameters from one stationary FC reference."""
from pathlib import Path
import math,copy,json,yaml,struct

def validate_settings(settings):
    if settings.get('alignment_mode','measured') not in ('measured','legacy_static'):
        raise ValueError('Unknown alignment_mode')
    if settings.get('trial_kind') != 'corridor_landing':
        return
    points=settings.get('corridor_waypoints')
    if not isinstance(points,list) or len(points)<2:
        raise ValueError('Fill corridor_waypoints with at least two measured waypoints; no default route is permitted')
    landing=settings.get('landing_xy')
    if not isinstance(landing,list) or len(landing)!=2:
        raise ValueError('Fill landing_xy with the measured H center')
    def finite(value):
        return not isinstance(value,bool) and isinstance(value,(int,float)) and math.isfinite(value)
    def check_xy(x,y):
        if not finite(x) or not finite(y) or not .35<=x<=3.4 or not -1.4<=y<=1.4:
            raise ValueError('Waypoint/H outside configured 4x4 center bounds: X [.35,3.4], Y [-1.4,1.4]')
    check_xy(*landing)
    for point in points:
        if not isinstance(point,dict) or set(point)-{'x','y','agl'} or 'x' not in point or 'y' not in point:
            raise ValueError('Each corridor waypoint requires x/y and optional agl')
        check_xy(point['x'],point['y'])
        height=point.get('agl',settings['low_agl'])
        if not finite(height) or not .5<=height<=1.8:
            raise ValueError('Corridor waypoint agl must be within [0.5,1.8] m')

def generate(root,out,settings,fc_xyz,rig):
    validate_settings(settings)
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    x,y,z=map(float,fc_xyz);ground=z-float(rig['fc_ground_clearance']);mode=settings['mode']
    ceiling_enabled=bool(settings.get('virtual_ceiling_enabled',False))
    if not all(math.isfinite(v) for v in (x,y,z,ground)) or abs(x)>.3 or abs(y)>.3 or abs(z)>.3:raise ValueError('Unexpected camera_init origin; inspect localization before flight')
    low=ground+float(settings['low_agl']);high=ground+float(settings['high_agl']);drop=ground+float(settings['drop_agl']);capture=ground+float(settings['landing_capture_agl'])
    # Legacy align_height is float32, recovery_height is double. Use an
    # exactly representable shared value so equality cannot fail its guard.
    low=struct.unpack('f',struct.pack('f',low))[0]
    if not .45<=settings['drop_agl']<=1.0 or min(drop,ground+.40)<=.05:raise ValueError('Legacy positive local-Z bounds not met')
    point=lambda a,b,h:[x+a,y+b,h]
    runtime=yaml.safe_load((root/'docs/verification/fov_landing_inner_20260919/seed_2672/fast_runtime.yaml').read_text())
    m=runtime['mission'];m.update(home_xy=[x,y],landing_xy=[x+.6,y],approach_altitude=low,return_altitude=low,timeout=300.,forced_return_at=240.,post_delivery_route_revision='board-'+mode,
        post_delivery_route=[point(.6,0,low)],post_delivery_parameter_stages=[],early_return_enabled=False,delivery_reserve_per_slot=25.,return_land_reserve=45.,nominal_speed=float(settings['cruise_speed']),motion_action_timeout=30.,target_action_timeout=60.)
    runtime.pop('corridor_speed_schedule',None);runtime.pop('fixed_search_region',None)
    runtime['search'].update(min_x=x+.6,max_x=x+3.2,min_y=y-1.2,max_y=y+1.2,lane_spacing=1.2,altitude=low)
    runtime['runtime'].update(start_mode='post_delivery' if mode=='landing' else 'full',mission_id_prefix='board-'+mode)
    line=[point(.6,0,low),point(3.0,0,low)]
    runtime['trial']=dict(mode=mode,waypoints=line,camera_info_topic=settings.get('camera_info_topic','/camera/camera_info'))
    if mode=='landing':
        hx,hy=settings['landing_xy'];transit=ground+settings['landing_transit_agl']
        m.update(landing_xy=[x+hx,y+hy],return_altitude=capture,post_delivery_route=[point(max(.6,hx-.7),hy,transit),point(hx,hy,transit),point(hx,hy,capture)])
    if settings.get('trial_kind')=='corridor_landing':
        hx,hy=settings['landing_xy']
        route=[point(w['x'],w['y'],ground+float(w.get('agl',settings['low_agl']))) for w in settings['corridor_waypoints']]
        for final in [point(hx,hy,ground+settings['landing_transit_agl']),point(hx,hy,capture)]:
            if any(abs(a-b)>1e-6 for a,b in zip(route[-1],final)):route.append(final)
        m.update(post_delivery_route=route,post_delivery_route_revision='board-corridor-landing',landing_xy=[x+hx,y+hy])
    bounds=[x-.35,x+3.4,y-1.4,y+1.4]
    runtime['high_view_probe']=dict(config=dict(ground_z=ground,high_agl=settings['high_agl'],low_agl=settings['low_agl'],survey_xy=[point(a,b,0)[:2] for a,b in [(1,-1.0),(3,-1.0),(3,1.0),(1,1.0),(1,-1.0)]],source_key='board-inherited-camera-static-start'),camera_info_topic=settings.get('camera_info_topic','/camera/camera_info'))
    runtime['high_view_probe']['low_stage_parameters']=[dict(name=key,value=value) for key,value in {'/external_planner_max_command_z':ground+1.85,'/fast_planner_node/sdf_map/virtual_ceil_height':(ground+2. if ceiling_enabled else -.1),'/fast_planner_node/sdf_map/horizontal_avoidance/tracking_margin':0.,'/fast_planner_node/fsm/goal_adjustment_radius':.15}.items()]
    runtime['high_view_full']=dict(policy=dict(high_max_agl=3.0,candidate_min_streak=1,min_interval_ns=100000000,min_span_ns=200000000,max_uncertainty_m=.45,direct_descent=True,descent_radius_m=1.,descent_max_candidates=9,survey_stall_seconds=8.,survey_progress_m=.15,survey_alternative_radius_m=.3),grid=dict(bounds=bounds,resolution=.10),boundary_policy=dict(enabled=True,bounds=[x-.35,x+4.,y-2.,y+2.]))
    control=yaml.safe_load((root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_horizontal_control.yaml').read_text())
    control.update(waypoints=[dict(x=x,y=y,z=(ground+settings['landing_transit_agl'] if mode=='landing' else low),yaw=0.,pointmode='Takeoff_point',hover_time=0.)],align_height=low,land_height=ground+.40,px4_max_distance=.15)
    control['switch']['auto_land']=mode=='landing';control['drop_system'].update(enable_drop=mode!='landing',release_setpoint_height=drop,height_threshold=drop+.10)
    control['switch']['flag_landing_detect']=1 if mode=='landing' else 0
    control['uav_vision'].update(recovery_height=low,pixel_to_body_matrix=list(rig['pixel_to_body_matrix']),max_movement_distance=.15)
    control['external_landing'].update(frame='camera_init',capture_height=capture if mode=='landing' else low,auto_land_height=ground+.55,detections_topic='/uav_vision/detections_mapped' if mode=='landing' else '/board_trials/h_disabled')
    overrides={
        '/fast_planner_node/sdf_map/resolution':.10,'/fast_planner_node/sdf_map/map_size_x':10.,'/fast_planner_node/sdf_map/map_size_y':6.,'/fast_planner_node/sdf_map/map_size_z':3.8,
        '/fast_planner_node/sdf_map/visualization_rate':2.,
        '/fast_planner_node/sdf_map/local_update_range_x':4.5,'/fast_planner_node/sdf_map/local_update_range_y':3.,'/fast_planner_node/sdf_map/local_update_range_z':3.,
        '/fast_planner_node/sdf_map/ground_height':ground-.1,'/fast_planner_node/sdf_map/virtual_ceil_height':(ground+3.0 if ceiling_enabled else -.1),
        '/fast_planner_node/sdf_map/horizontal_avoidance/min_x':bounds[0],'/fast_planner_node/sdf_map/horizontal_avoidance/max_x':bounds[1],'/fast_planner_node/sdf_map/horizontal_avoidance/min_y':bounds[2],'/fast_planner_node/sdf_map/horizontal_avoidance/max_y':bounds[3],
        '/fast_planner_node/sdf_map/horizontal_avoidance/floor_z':ground+.1,'/fast_planner_node/sdf_map/horizontal_avoidance/obstacle_min_z':ground+.1,
        '/fast_planner_node/sdf_map/search_region/enabled':True,'/fast_planner_node/sdf_map/search_region/min_x':bounds[0],'/fast_planner_node/sdf_map/search_region/max_x':bounds[1],'/fast_planner_node/sdf_map/search_region/min_y':bounds[2],'/fast_planner_node/sdf_map/search_region/max_y':bounds[3],
        '/fast_planner_node/sdf_map/horizontal_avoidance/tracking_margin':.1 if mode=='high_view' else 0.,
        '/external_planner_max_command_z':ground+2.9,'/navigation/planner_bridge/execution/max_goal_z':ground+2.9,
        '/navigation/planner_bridge/execution/arrival_position_tolerance':.12,'/navigation/planner_bridge/execution/arrival_dwell':.8,
        '/navigation/planner_bridge/execution/search_initial_plan_timeout':12.,
        '/navigation/planner_bridge/target/recovery_height':low,
        '/release_permission_arbiter/pose_topic':'/navigation/local_pose','/release_permission_arbiter/min_release_altitude':drop-.08,'/release_permission_arbiter/max_release_altitude':drop+.12,
        '/board_trials/ground_z':ground,'/board_trials/fc_on_ground_z':z,'/board_trials/mock_only':True,
    }
    for name,data in [('runtime.yaml',runtime),('control.yaml',control),('overrides.yaml',overrides),('auto_land.yaml',dict(frame='camera_init',landing_xy=[x+.6,y],cruise_z=low,route_revision='board-'+mode))]:
        (out/name).write_text(yaml.safe_dump(data,sort_keys=False))
    reference=dict(mode=mode,fc_xyz=[x,y,z],ground_z=ground,low_z=low,high_z=high,drop_z=drop,known_rig=rig,settings=settings)
    (out/'ground_reference.json').write_text(json.dumps(reference,indent=2));return reference
