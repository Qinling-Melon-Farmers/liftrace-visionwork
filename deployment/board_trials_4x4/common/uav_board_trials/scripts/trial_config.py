"""Generate all local-Z parameters from one stationary FC reference."""
from pathlib import Path
import math,copy,json,yaml,struct

TRIAL_FOLDERS = {
    'visual_interrupt': '01_visual_interrupt', 'high_view': '02_high_view_revisit',
    'landing': '03_h_landing', 'corridor_landing': '04_corridor_landing',
    'low_multi': '05_low_multi', 'high_priority': '06_high_priority',
    'memory_only': '07_memory_only', 'full_mission': '08_full_mission',
    'high_speed_capture': '09_high_speed_capture',
}
HIGH_MODES = ('high_view', 'high_priority', 'memory_only', 'high_view_full', 'high_speed_capture')
H_MODES = ('landing', 'high_view_full')
NO_DROP_MODES = ('landing', 'memory_only', 'high_speed_capture')

def apply_site_profile(settings, profile):
    """Merge measured site geometry without changing the module's mission kind."""
    allowed={'flight_area','search_line_x','compressed_image_topic','high_agl','max_agl',
             'terminal_hover_agl','auto_start_after_arm','initialization_timeout','obstacle_columns_enabled',
             'bag_image_hz','bag_image_topic','record_inflated_cloud','record_map_clouds'}
    if settings['mode'] in H_MODES:
        allowed.update(('landing_xy', 'landing_handoff_mode', 'landing_handoff_status_topic'))
    if settings.get('trial_kind') in ('corridor_landing','full_mission'):
        allowed.update(('corridor_waypoints','corridor_geometry'))
    if not isinstance(profile,dict) or set(profile)-allowed:
        raise ValueError('Unsupported site profile key')
    if settings['mode'] in H_MODES and 'terminal_hover_agl' in profile:
        raise ValueError('H landing cannot use terminal_hover_agl; preserve visual landing handoff')
    settings.update(profile)
    return settings

def mapping_profile(settings):
    if settings['mode'] in HIGH_MODES:
        return 'high'
    return 'corridor' if settings.get('trial_kind') == 'corridor_landing' else 'low'

def flight_geometry(settings):
    defaults=dict(center_bounds=[-.35,3.4,-1.4,1.4],target_bounds=[-.35,4.,-2.,2.],
                  search_bounds=[.6,3.2,-1.2,1.2],staging_xy=[.6,.05],
                  survey_xy=[[1.,-1.],[3.,-1.],[3.,1.],[1.,1.],[1.,-1.]],
                  map_size=[10.,6.,3.8])
    custom=settings.get("flight_area",{})
    if not isinstance(custom,dict) or set(custom)-set(defaults):raise ValueError("Unknown flight_area fields")
    area=dict(defaults,**custom)
    if settings.get('mode') == 'high_speed_capture' and any(
            key in settings for key in ('capture_line_xy','capture_round_trips')):
        raise ValueError('Capture now uses flight_area staging_xy/survey_xy; remove obsolete line settings')
    def finite(v):return not isinstance(v,bool) and isinstance(v,(int,float)) and math.isfinite(v)
    for key in ("center_bounds","target_bounds","search_bounds"):
        v=area[key]
        if not isinstance(v,list) or len(v)!=4 or not all(finite(t) for t in v) or v[0]>=v[1] or v[2]>=v[3]:
            raise ValueError("Invalid flight_area "+key)
    bounds=area["center_bounds"];target=area["target_bounds"];search=area["search_bounds"]
    def inside(point,box=bounds):
        return isinstance(point,(list,tuple)) and len(point)==2 and all(finite(t) for t in point) and box[0]<=point[0]<=box[1] and box[2]<=point[1]<=box[3]
    if not inside([0.,0.]) or not inside(area["staging_xy"]):raise ValueError("Origin/staging outside flight_area")
    if not all(inside(v,target) for v in ([bounds[0],bounds[2]],[bounds[1],bounds[3]])):
        raise ValueError("target_bounds must contain center_bounds")
    if not all(inside(v) for v in ([search[0],search[2]],[search[1],search[3]])):raise ValueError("Search bounds outside flight_area")
    if 'survey_pattern' in settings:
        if settings['mode'] not in HIGH_MODES:raise ValueError('Survey pattern only applies to high-view modules')
        from uav_mission.survey_routes import survey_route
        old=area['survey_xy'];xs=[min(v[0] for v in old),max(v[0] for v in old)]
        if settings['survey_pattern']=='snake3':xs.insert(1,sum(xs)/2)
        area['survey_xy']=survey_route(settings['survey_pattern'],xs,min(v[1] for v in old),max(v[1] for v in old),start_high=old[0][1]>sum(v[1] for v in old)/len(old))
    points=area["survey_xy"]
    if not isinstance(points,list) or len(points)<(2 if settings.get('mode')=='high_speed_capture' else 3) or any(not inside(v) for v in points):raise ValueError("Survey point outside flight_area")
    if settings.get('mode')=='high_speed_capture':
        if any(not (bounds[0]+.3<=p[0]<=bounds[1]-.3 and bounds[2]+.3<=p[1]<=bounds[3]-.3)
               for p in [area['staging_xy'],*points]):
            raise ValueError('Capture points require 0.30m inset inside center bounds')
        # Short survey legs are intentional. Speed acceptance uses measured
        # moving windows; completing the route does not prove cruise speed.
    size=area["map_size"]
    if not isinstance(size,list) or len(size)!=3 or not all(finite(v) and v>0 for v in size):raise ValueError("Invalid map_size")
    if max(abs(bounds[0]),abs(bounds[1]))+.3>=size[0]/2 or max(abs(bounds[2]),abs(bounds[3]))+.3>=size[1]/2:
        raise ValueError("Planner map too small for flight_area and initial pose tolerance")
    return area

def validate_settings(settings):
    if settings.get('mode') not in ('visual_interrupt','low_multi',*HIGH_MODES,'landing'):
        raise ValueError('Unknown trial mode')
    if settings.get('landing_handoff_mode', 'AUTO.LAND') not in ('AUTO.LAND', 'POSCTL'):
        raise ValueError('landing_handoff_mode must be AUTO.LAND or POSCTL')
    if 'landing_handoff_mode' in settings and settings['mode'] not in H_MODES:
        raise ValueError('landing_handoff_mode only applies to visual H landing')
    if 'landing_handoff_status_topic' in settings:
        topic = settings['landing_handoff_status_topic']
        if settings['mode'] not in H_MODES:
            raise ValueError('landing_handoff_status_topic only applies to visual H landing')
        if (not isinstance(topic, str) or not topic.startswith('/') or len(topic) < 2
                or any(not (ch.isascii() and (ch.isalnum() or ch in '/_')) for ch in topic)):
            raise ValueError('Invalid landing_handoff_status_topic')
    if settings.get('actuator_mode','mock') not in ('mock','real','none'):
        raise ValueError('Unknown actuator_mode')
    if settings.get('actuator_mode')=='real' and settings['mode'] in NO_DROP_MODES:
        raise ValueError('This module does not permit release')
    raw=settings.get('raw_servo_service','/legacy/Servo_raw')
    if not isinstance(raw,str) or not raw.startswith('/') or raw in ('/Servo','/board_trials/Servo','/board_trials/mock_servo'):
        raise ValueError('raw_servo_service must be an independent absolute hardware service')
    for key,lo,hi in [('low_agl',1.,1.8),('high_agl',2.,2.8),('landing_transit_agl',.5,1.8),('landing_capture_agl',.5,2.0),('cruise_speed',.1,1.2 if settings['mode']=='high_speed_capture' else .5),('cruise_acceleration',.1,1.0 if settings['mode']=='high_speed_capture' else .5)]:
        v=settings.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid '+key)
    if settings['mode']=='high_speed_capture':
        if settings['cruise_speed'] not in (.5,1.,1.2) or settings.get('high_agl')!=2. or settings.get('max_agl')!=2.:
            raise ValueError('Capture uses 0.5/1.0/1.2m/s and 2.0m high/cap')
        if settings.get('terminal_hover_agl')!=.3:
            raise ValueError('Capture requires 0.30m terminal hover')
        if settings.get('capture_lighting','unspecified') not in ('normal','dim','unspecified'):
            raise ValueError('Invalid capture lighting label')
    if type(settings.get('delivery_count',2)) is not int or not 1<=settings.get('delivery_count',2)<=3:
        raise ValueError('delivery_count must be 1..3')
    if type(settings.get('virtual_ceiling_enabled',False)) is not bool:
        raise ValueError('virtual_ceiling_enabled must be boolean')

    if settings.get('alignment_mode','measured') not in ('measured','legacy_static'):
        raise ValueError('Unknown alignment_mode')
    if not 30<=float(settings.get('initialization_timeout',90))<=300:raise ValueError('invalid initialization timeout')
    if type(settings.get('auto_start_after_arm',False)) is not bool:raise ValueError('invalid auto start')
    max_agl=settings.get('max_agl',2.9)
    if not isinstance(max_agl,(int,float)) or not math.isfinite(max_agl) or not 2.<=max_agl<=3.:raise ValueError('invalid max_agl')
    if settings['high_agl']>max_agl:raise ValueError('high altitude exceeds cap')
    if 'terminal_hover_agl' in settings and not .25<=settings['terminal_hover_agl']<=.5:raise ValueError('invalid terminal hover')
    if type(settings.get('obstacle_columns_enabled',True)) is not bool:raise ValueError('invalid obstacle column flag')
    from trial_motion import motion_options
    motion_options(settings)
    area=flight_geometry(settings);cb=area['center_bounds']
    if settings['mode'] in H_MODES and settings.get('trial_kind') not in ('corridor_landing','full_mission'):
        landing=settings.get('landing_xy')
        if (not isinstance(landing,list) or len(landing)!=2 or
                any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in landing)):
            raise ValueError('Fill landing_xy with the measured H center')
        if not max(.35,cb[0])<=landing[0]<=cb[1] or not cb[2]<=landing[1]<=cb[3]:
            raise ValueError('Waypoint/H outside configured flight_area center bounds')
    if settings['mode'] in HIGH_MODES:
        from uav_mission.high_view_probe import ProbeConfig
        ProbeConfig(ground_z=0.,high_agl=settings['high_agl'],low_agl=settings['low_agl'],
                    survey_xy=tuple(tuple(v) for v in area['survey_xy']),
                    staging_xy=tuple(area['staging_xy']))

    line=settings.get('search_line_x',[.6,1.2,1.8,2.4,3.0])
    if (not isinstance(line,list) or len(line)<2 or
            any(isinstance(v,bool) or not isinstance(v,(int,float)) or
                not math.isfinite(v) or not max(.35,cb[0])<=v<=cb[1] for v in line) or
            any(a>=b for a,b in zip(line,line[1:]))):
        raise ValueError('search_line_x must advance inside flight_area center bounds')
    if settings.get('trial_kind') not in ('corridor_landing','full_mission'):
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
        if not finite(x) or not finite(y) or not max(.35,cb[0])<=x<=cb[1] or not cb[2]<=y<=cb[3]:
            raise ValueError('Waypoint/H outside configured flight_area center bounds')
    check_xy(*landing)
    for point in points:
        if not isinstance(point,dict) or set(point)-{'x','y','agl'} or 'x' not in point or 'y' not in point:
            raise ValueError('Each corridor waypoint requires x/y and optional agl')
        check_xy(point['x'],point['y'])
        height=point.get('agl',settings['low_agl'])
        if not finite(height) or not .5<=height<=1.8:
            raise ValueError('Corridor waypoint agl must be within [0.5,1.8] m')

def generate(root,out,settings,fc_xyz,rig):
    validate_settings(settings);area=flight_geometry(settings)
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    x,y,z=map(float,fc_xyz);ground=z-float(rig['fc_ground_clearance']);mode=settings['mode'];high_mode=mode in HIGH_MODES;h_landing=mode in H_MODES;drop_enabled=mode not in NO_DROP_MODES
    actuator=settings.get('actuator_mode','mock') if drop_enabled else 'none'
    ceiling_enabled=bool(settings.get('virtual_ceiling_enabled',False))
    if not all(math.isfinite(v) for v in (x,y,z,ground)) or abs(x)>.3 or abs(y)>.3 or abs(z)>.3:raise ValueError('Unexpected camera_init origin; inspect localization before flight')
    low=ground+float(settings['low_agl']);high=ground+float(settings['high_agl']);drop=ground+float(settings['drop_agl']);capture=ground+float(settings['landing_capture_agl'])
    # Legacy align_height is float32, recovery_height is double. Use an
    # exactly representable shared value so equality cannot fail its guard.
    low=struct.unpack('f',struct.pack('f',low))[0]
    if not .45<=settings['drop_agl']<=1.0 or min(drop,ground+.40)<=.05:raise ValueError('Legacy positive local-Z bounds not met')
    point=lambda a,b,h:[x+a,y+b,h]
    runtime=yaml.safe_load((root/'docs/verification/fov_landing_inner_20260919/seed_2672/fast_runtime.yaml').read_text())
    m=runtime['mission'];m.update(home_xy=[x,y],landing_xy=[x+.6,y],approach_altitude=low,return_altitude=low,timeout=600. if mode=='high_view_full' else 300.,forced_return_at=510. if mode=='high_view_full' else 240.,post_delivery_route_revision='board-'+mode,
        post_delivery_route=[point(.6,0,low)],post_delivery_parameter_stages=[],early_return_enabled=False,delivery_reserve_per_slot=25.,return_land_reserve=45.,nominal_speed=float(settings['cruise_speed']),motion_action_timeout=30.,target_action_timeout=60.)
    runtime.pop('corridor_speed_schedule',None);runtime.pop('fixed_search_region',None)
    sx0,sx1,sy0,sy1=area['search_bounds']
    runtime['search'].update(min_x=x+sx0,max_x=x+sx1,min_y=y+sy0,max_y=y+sy1,lane_spacing=1.2,altitude=low)
    runtime['runtime'].update(start_mode='post_delivery' if mode=='landing' else 'full',mission_id_prefix='board-'+mode)
    # Do not make reaching a distant, possibly occupied endpoint a prerequisite
    # for seeing the target in front. Every shorter leg still uses Fast-Planner.
    line=[point(v,0,low) for v in settings.get('search_line_x',[.6,1.2,1.8,2.4,3.0])]
    runtime['trial']=dict(mode=mode,waypoints=line,camera_info_topic=settings.get('camera_info_topic','/camera/camera_info'),delivery_count=settings.get('delivery_count',2),actuator_mode=actuator)
    if mode=='high_speed_capture':
        runtime['trial']['capture']=dict(speed=settings['cruise_speed'],lighting=settings.get('capture_lighting','unspecified'),
            route='flight_area.survey_xy',survey_xy=copy.deepcopy(area['survey_xy']))
    runtime['following_speed_profile']=(dict(cruise_lead_m=min(float(settings['cruise_speed']),1.0),precision_lead_m=.4,corridor_lead_m=.15)
        if mode=='high_speed_capture' else dict(cruise_lead_m=.50,precision_lead_m=.25,corridor_lead_m=.25))
    if mode=='landing':
        hx,hy=settings['landing_xy'];transit=ground+settings['landing_transit_agl']
        m.update(landing_xy=[x+hx,y+hy],return_altitude=capture,post_delivery_route=[point(max(.6,hx-.7),hy,transit),point(hx,hy,transit),point(hx,hy,capture)])
    if settings.get('trial_kind') in ('corridor_landing','full_mission'):
        hx,hy=settings['landing_xy']
        route=[point(w['x'],w['y'],ground+float(w.get('agl',settings['low_agl']))) for w in settings['corridor_waypoints']]
        for final in [point(hx,hy,ground+settings['landing_transit_agl']),point(hx,hy,capture)]:
            if any(abs(a-b)>1e-6 for a,b in zip(route[-1],final)):route.append(final)
        m.update(post_delivery_route=route,post_delivery_route_revision='board-'+settings['trial_kind'],landing_xy=[x+hx,y+hy])
    if mode=='high_view_full':
        m['post_delivery_parameter_stages']=[dict(after_completed_waypoints=0,parameters={
            '/fast_planner_node/sdf_map/virtual_ceil_height':(ground+3. if ceiling_enabled else -.1),
            '/navigation/planner_bridge/execution/arrival_position_tolerance':.12,
            '/navigation/planner_bridge/execution/arrival_dwell':.8})]
    bx0,bx1,by0,by1=area['center_bounds'];bounds=[x+bx0,x+bx1,y+by0,y+by1]
    tx0,tx1,ty0,ty1=area['target_bounds'];target_bounds=[x+tx0,x+tx1,y+ty0,y+ty1]
    runtime['high_view_probe']=dict(config=dict(ground_z=ground,high_agl=settings['high_agl'],low_agl=settings['low_agl'],staging_xy=[x+area['staging_xy'][0],y+area['staging_xy'][1]],survey_xy=[point(a,b,0)[:2] for a,b in area['survey_xy']],source_key='board-inherited-camera-static-start'),camera_info_topic=settings.get('camera_info_topic','/camera/camera_info'))
    runtime['high_view_probe']['low_stage_parameters']=[dict(name=key,value=value) for key,value in {'/external_planner_max_command_z':max(ground+1.85,capture+.1) if h_landing else ground+1.85,'/fast_planner_node/sdf_map/virtual_ceil_height':(ground+2. if ceiling_enabled else -.1),'/fast_planner_node/fsm/goal_adjustment_radius':.15}.items()]
    runtime['high_view_full']=dict(policy=dict(high_min_agl=max(1.8,settings['high_agl']-.2),high_max_agl=settings['high_agl']+.2,coarse_enabled=True,coarse_min_confidence=.60,coarse_interrupt_min_interval_ns=100000000,coarse_interrupt_max_gap_ns=1000000000,coarse_interrupt_consistency_m=.5,interrupt_refined_classes=[],recheck_observe_seconds=5.,recheck_shift_after_seconds=1.,recheck_shift_radius_m=.5,candidate_min_streak=1,min_interval_ns=100000000,min_span_ns=200000000,max_uncertainty_m=.45,direct_descent=True,descent_radius_m=1.,descent_max_candidates=9,survey_stall_seconds=8.,survey_progress_m=.15,survey_alternative_radius_m=.3),grid=dict(bounds=bounds,resolution=.10,inflation=.25),boundary_policy=dict(enabled=True,bounds=target_bounds))
    resume_enabled=settings.get('resume_survey_enabled',False)
    if type(resume_enabled) is not bool:
        raise ValueError('resume_survey_enabled must be boolean')
    if resume_enabled and mode not in ('high_priority','high_view_full'):
        raise ValueError('survey resume is only available for priority/full mission trials')
    runtime['high_view_full']['policy']['resume_survey_enabled']=resume_enabled
    if high_mode:
        from uav_mission.high_view_probe import ProbeConfig
        from uav_high_view.survey_policy import SurveyPolicy
        ProbeConfig(**runtime['high_view_probe']['config'])
        SurveyPolicy(**runtime['high_view_full']['policy'])
    control=yaml.safe_load((root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_horizontal_control.yaml').read_text())
    control.update(waypoints=[dict(x=x,y=y,z=(ground+settings['landing_transit_agl'] if mode=='landing' else low),yaw=0.,pointmode='Takeoff_point',hover_time=0.)],align_height=low,land_height=ground+.40,px4_max_distance=.25)
    control['switch']['auto_land']=h_landing;control['drop_system'].update(enable_drop=drop_enabled,release_setpoint_height=drop,height_threshold=drop+.10)
    control['switch']['flag_landing_detect']=1 if h_landing else 0
    control['uav_vision'].update(recovery_height=low,
        standard_recovery_setpoint_height=low+.10,cross_recovery_setpoint_height=low+.10,
        pixel_to_body_matrix=list(rig['pixel_to_body_matrix']),max_movement_distance=.15,
        drop_metric_scale_enabled=True,drop_ground_z=ground,
        drop_map_frame=rig['mission_frame'],
        drop_camera_info_topic=settings.get('camera_info_topic','/camera/camera_info'))
    # Preserve the flight team's existing fixed-frame offsets; not a new calibration.
    for key in ('slot_offsets','dynamic_slot_offsets'):
        if key in rig:control['drop_system'][key]=copy.deepcopy(rig[key])
    control['external_landing'].update(frame='camera_init',capture_height=capture if h_landing else low,auto_land_height=ground+.55,detections_topic='/uav_vision/detections_mapped' if h_landing else '/board_trials/h_disabled',handoff_mode=settings.get('landing_handoff_mode', 'AUTO.LAND'),handoff_status_topic=settings.get('landing_handoff_status_topic', '/patrol_control/external_landing_handoff'))
    overrides={
        '/fast_planner_node/sdf_map/resolution':.10,'/fast_planner_node/sdf_map/map_size_x':area['map_size'][0],'/fast_planner_node/sdf_map/map_size_y':area['map_size'][1],'/fast_planner_node/sdf_map/map_size_z':area['map_size'][2],
        '/fast_planner_node/sdf_map/visualization_rate':2.,
        '/fast_planner_node/sdf_map/local_update_range_x':4.5,'/fast_planner_node/sdf_map/local_update_range_y':3.,'/fast_planner_node/sdf_map/local_update_range_z':3.,
        '/fast_planner_node/sdf_map/ground_height':ground-.1,'/fast_planner_node/sdf_map/virtual_ceil_height':(ground+3.0 if ceiling_enabled else -.1),
        '/fast_planner_node/sdf_map/horizontal_avoidance/min_x':bounds[0],'/fast_planner_node/sdf_map/horizontal_avoidance/max_x':bounds[1],'/fast_planner_node/sdf_map/horizontal_avoidance/min_y':bounds[2],'/fast_planner_node/sdf_map/horizontal_avoidance/max_y':bounds[3],
        '/fast_planner_node/sdf_map/horizontal_avoidance/floor_z':ground+.1,'/fast_planner_node/sdf_map/horizontal_avoidance/obstacle_min_z':ground+.4,
        # Low visual/H/corridor trials use the successful board's real 3-D map.
        # Only the tree/high survey trial additionally enforces no-overflight.
        '/fast_planner_node/sdf_map/horizontal_avoidance/enabled':high_mode and settings.get('obstacle_columns_enabled',True),
        '/fast_planner_node/sdf_map/search_region/enabled':True,'/fast_planner_node/sdf_map/search_region/min_x':bounds[0],'/fast_planner_node/sdf_map/search_region/max_x':bounds[1],'/fast_planner_node/sdf_map/search_region/min_y':bounds[2],'/fast_planner_node/sdf_map/search_region/max_y':bounds[3],
        '/fast_planner_node/sdf_map/obstacles_inflation':.25,
        '/fast_planner_node/sdf_map/obstacles_inflation_up':.20,
        '/fast_planner_node/sdf_map/obstacles_inflation_down':.10,
        '/traj_server/traj_server/target_dist':.25,
        '/external_planner_start_max_distance':(max(.75,min(float(settings['cruise_speed']),1.0)+.25) if mode=='high_speed_capture' else .75),
        '/external_planner_max_command_z':ground+settings.get('max_agl',2.9),'/navigation/planner_bridge/execution/max_goal_z':ground+settings.get('max_agl',2.9),
        '/navigation/planner_bridge/execution/arrival_position_tolerance':.12,'/navigation/planner_bridge/execution/arrival_dwell':.8,
        '/navigation/planner_bridge/execution/initial_plan_timeout':12.,
        '/navigation/planner_bridge/execution/search_initial_plan_timeout':12.,
        '/fast_planner_node/fsm/liveness_enabled':True,'/fast_planner_node/fsm/server_hold_replan_enabled':True,
        '/fast_planner_node/fsm/server_hold_seconds':.25,'/fast_planner_node/fsm/server_progress_max_age':.5,
        '/fast_planner_node/progress/enabled':True,'/traj_server/progress/enabled':True,
        '/traj_server/traj_server/require_goal_identity':True,
        '/navigation/planner_bridge/target/recovery_height':low,
        '/release_permission_arbiter/pose_topic':'/navigation/local_pose','/release_permission_arbiter/min_release_altitude':drop-.08,'/release_permission_arbiter/max_release_altitude':drop+.12,
        '/board_trials/ground_z':ground,'/board_trials/fc_on_ground_z':z,'/board_trials/mock_only':actuator=='mock',
        '/board_trials/actuator_mode':actuator,
        '/target_map_projector/coarse_navigation_enabled':high_mode,
        '/target_map_projector/coarse_min_confidence':.60,
        # Loaded after Phase D's detector YAML in the common application launch.
        # Standalone H, corridor+H and full-mission LAND use the same fallback.
        '/landing_detector/landing_enable_h_stroke_fallback':h_landing,
        '/target_memory/search_confirmation_max_gap_sec':1.0,
        '/drop_aligner/stable_frames':5,
    }
    from trial_motion import apply_generated_motion
    motion_report=apply_generated_motion(runtime,control,overrides,settings,ground)
    (out/'motion_profile.json').write_text(json.dumps(motion_report,indent=2))
    (out/'terminal_hover.yaml').write_text(yaml.safe_dump(dict(frame='camera_init',ground_z=ground,hover_agl=settings.get('terminal_hover_agl',.3),max_agl=settings.get('max_agl',2.9),descent_speed=.15)))
    for name,data in [('runtime.yaml',runtime),('control.yaml',control),('overrides.yaml',overrides),('auto_land.yaml',dict(frame='camera_init',landing_xy=[x+.6,y],cruise_z=low,route_revision='board-'+mode))]:
        (out/name).write_text(yaml.safe_dump(data,sort_keys=False))
    reference=dict(mode=mode,mapping_profile=mapping_profile(settings),fc_xyz=[x,y,z],ground_z=ground,low_z=low,high_z=high,drop_z=drop,takeoff_z=control['waypoints'][0]['z'],known_rig=rig,settings=settings)
    (out/'ground_reference.json').write_text(json.dumps(reference,indent=2));return reference
