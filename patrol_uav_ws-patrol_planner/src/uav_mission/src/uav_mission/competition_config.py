"""Field configuration for the full mission. Coordinates are fixed at takeoff.

No target positions are supplied: survey and fallback cover the search bounds.
Corridor anchors/H must be measured; all local-Z values share one ground datum.
"""
from pathlib import Path
import copy,json,math,struct
import yaml
from uav_mission.high_view_probe import ProbeConfig
from uav_high_view.survey_policy import SurveyPolicy
from uav_mission.execution_speed import FollowingSpeed
from uav_mission.corridor_speed import CorridorSpeedConfig
from uav_mission.motion_optimization import MotionOptimization, optimize_post_route


def validate(settings, flight=False):
    motion=MotionOptimization(**settings.get('motion_optimization',{}))
    if motion.enabled:
        if motion.braking_speed_mps<settings['cruise_speed'] or motion.braking_accel_mps2>settings['cruise_acceleration']:
            raise ValueError('braking model must not overstate commanded dynamics')
        if motion.moving_recovery and not settings['drop_agl']+.15<=motion.recovery_handoff_agl<=settings['low_agl']:
            raise ValueError('recovery handoff must clear release height before low transit')
        if not settings.get('corridor_speed_schedule'):
            raise ValueError('motion optimization needs measured corridor wall planes')
        if settings['landing_transit_agl']>motion.corridor_max_agl:
            raise ValueError('H transit exceeds corridor cap')

    if settings.get('mode')!='high_view_full' or settings.get('actuator_mode')!='real':
        raise ValueError('competition uses the full mission and permission-guarded real release')
    FollowingSpeed(**settings['following_speed_profile'])
    if settings.get('corridor_speed_schedule'):CorridorSpeedConfig(**settings['corridor_speed_schedule'])
    SurveyPolicy(**settings['survey_policy'])
    def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
    def bounds(name):
        v=settings[name]
        if len(v)!=4 or not all(finite(t) for t in v) or v[0]>=v[1] or v[2]>=v[3]:raise ValueError('invalid '+name)
        return v
    area=bounds('flight_bounds');search=bounds('search_center_bounds');targets=bounds('target_bounds');cover=bounds('coverage_bounds')
    def inside(p,box=area):
        return len(p)==2 and all(finite(v) for v in p) and box[0]<=p[0]<=box[1] and box[2]<=p[1]<=box[3]
    for box in (search,targets,cover):
        if not inside([box[0],box[2]]) or not inside([box[1],box[3]]):raise ValueError('region outside flight bounds')
    if not inside([0.,0.]) or not inside(settings['staging_xy'],search):raise ValueError('invalid origin/staging')
    if not 1<=len(settings['survey_xy'])<=8 or any(not inside(p,search) for p in settings['survey_xy']):raise ValueError('survey outside search interior')
    if not inside([cover[0],cover[2]],search) or not inside([cover[1],cover[3]],search):raise ValueError('coverage outside search interior')
    for key,lo,hi in [('low_agl',1.,1.8),('high_agl',2.,3.),('max_agl',2.,3.5),('drop_agl',.45,1.),('landing_transit_agl',.5,1.8),('landing_capture_agl',.8,2.),('cruise_speed',.1,1.2),('cruise_acceleration',.1,1.),('lane_spacing',.2,1.2)]:
        if not finite(settings[key]) or not lo<=settings[key]<=hi:raise ValueError('invalid '+key)
    if not settings['low_agl']<settings['high_agl']<=settings['max_agl']:raise ValueError('altitude order')
    for k in ('site_confirmed','auto_start_after_arm','obstacle_columns_enabled'):
        if type(settings[k]) is not bool:raise ValueError('invalid '+k)
    if type(settings.get('landing_enable_h_stroke_fallback')) is not bool:
        raise ValueError('landing_enable_h_stroke_fallback must be an explicit boolean')
    if settings['alignment_mode'] not in ('legacy_static','measured'):raise ValueError('alignment mode')
    if settings.get('virtual_ceiling_enabled',False):raise ValueError('frozen field profile keeps virtual ceiling off')
    raw=settings['raw_servo_service']
    if not isinstance(raw,str) or not raw.startswith('/') or raw=='/Servo':raise ValueError('independent raw service required')
    size=settings['map_size']
    if len(size)!=3 or not all(finite(v) and v>0 for v in size):raise ValueError('map size')
    if max(abs(area[0]),abs(area[1]))+.3>=size[0]/2 or max(abs(area[2]),abs(area[3]))+.3>=size[1]/2 or size[2]<settings['max_agl']+.3:raise ValueError('map too small')
    route=settings['corridor_waypoints'];landing=settings['landing_xy']
    if not isinstance(route,list):raise ValueError('corridor list')
    for p in route:
        if set(p)!={'x','y','agl'} or not inside([p['x'],p['y']]) or not finite(p['agl']) or not .5<=p['agl']<=1.8:raise ValueError('invalid measured corridor waypoint')
    if landing is not None and not inside(landing):raise ValueError('H outside flight bounds')
    if flight and (not settings['site_confirmed'] or len(route)<2 or landing is None):raise ValueError('flight requires confirmed field, measured corridor and H')
    if settings['following_speed_profile']['cruise_lead_m']>1.0:raise ValueError('lead exceeds frozen controller admission')
    if settings.get('record_map_clouds',False):raise ValueError('competition profile records only the local inflated cloud')
    return settings


def generate(root,out,settings,fc_xyz,rig):
    validate(settings,flight=True)
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    cfg=root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition'
    x,y,z=map(float,fc_xyz);ground=z-float(rig['fc_ground_clearance'])
    if not all(math.isfinite(v) for v in (x,y,z,ground)) or max(abs(x),abs(y),abs(z))>.3:raise ValueError('unexpected initial reference')
    low=struct.unpack('f',struct.pack('f',ground+settings['low_agl']))[0]
    camera_offset=-(float(rig['body_to_imu_xyz'][2])+float(rig['imu_to_camera_xyz'][2]))
    if 'survey_camera_agl' in settings:
        requested=float(settings['survey_camera_agl'])
        if not math.isfinite(requested) or requested<=0 or abs(settings['high_agl']-(requested+camera_offset))>1e-6:
            raise ValueError('survey camera AGL and FC high_agl disagree with rig')
    motion=MotionOptimization(**settings.get('motion_optimization',{}))
    high=ground+settings['high_agl'];drop=ground+settings['drop_agl'];capture=ground+settings['landing_capture_agl'];cap=ground+settings['max_agl']
    if min(drop,ground+.4)<=.05:raise ValueError('legacy positive local-Z bounds not met')
    point=lambda px,py,h:[x+px,y+py,h]
    shift=lambda box:[box[0]+x,box[1]+x,box[2]+y,box[3]+y]
    area=shift(settings['flight_bounds']);search=shift(settings['search_center_bounds']);target=shift(settings['target_bounds']);cover=shift(settings['coverage_bounds'])
    hx,hy=settings['landing_xy'];landing=[x+hx,y+hy]
    route=[point(p['x'],p['y'],ground+p['agl']) for p in settings['corridor_waypoints']]
    corridor_count=len(route)
    for p in (point(hx,hy,ground+settings['landing_transit_agl']),point(hx,hy,capture)):
        if math.dist(route[-1],p)>1e-6:route.append(p)
    motion_meta={}
    if motion.enabled:
        schedule=settings['corridor_speed_schedule']
        axis=schedule.get('axis',1)
        walls=[v+(x if axis==0 else y) for v in schedule['wall_coordinates']]
        # Only a same-XY high-to-low entrance pair may be combined. All remaining
        # corridor points must already satisfy the low-plane constraint.
        low_prefix=route[1:corridor_count] if (corridor_count>=2 and
            math.dist(route[0][:2],route[1][:2])<1e-6 and route[0][2]>route[1][2]+.05) else route[:corridor_count]
        if any(p[2]>ground+motion.corridor_max_agl+1e-6 for p in low_prefix):
            raise ValueError('corridor relay above low cap')
        route,motion_meta=optimize_post_route(route,motion,corridor_count,axis,walls)
        entry_index=(1 if not motion_meta['diagonal_entry'] and len(route)>1 and
            math.dist(route[0][:2],route[1][:2])<1e-6 and route[0][2]>route[1][2]+.05 else 0)
        motion_meta['entry_completed_waypoints']=entry_index+1
        entrance=route[entry_index]
        if entrance[2]>ground+motion.corridor_max_agl+1e-6 or min(abs(entrance[axis]-w) for w in walls)<schedule.get('exit_distance_m',.95):
            raise ValueError('low entrance must be reached before gate braking zone')
    runtime=yaml.safe_load((cfg/'runtime_base.yaml').read_text())
    runtime['mission'].update(home_xy=[x,y],landing_xy=landing,approach_altitude=low,return_altitude=low,
        nominal_speed=.5,post_delivery_route=route,post_delivery_route_revision='competition-measured',
        post_delivery_parameter_stages=[dict(after_completed_waypoints=0,parameters={
            '/external_planner_max_command_z':max(ground+1.85,capture+.1),
            '/fast_planner_node/sdf_map/search_region/enabled':False,
            '/fast_planner_node/sdf_map/virtual_ceil_height':-.1,
            '/navigation/planner_bridge/execution/arrival_position_tolerance':.12,
            '/navigation/planner_bridge/execution/arrival_dwell':.8})])
    if motion.enabled:
        runtime['motion_optimization']=copy.deepcopy(settings['motion_optimization'])
        runtime['motion_optimization_metadata']=motion_meta
        parameters=runtime['mission']['post_delivery_parameter_stages'][0]['parameters']
        parameters['/external_planner_max_command_z']=cap
        # Enter the corridor only AFTER the low entrance point has settled.
        runtime['mission']['post_delivery_parameter_stages'].append(dict(after_completed_waypoints=motion_meta['entry_completed_waypoints'],
            parameters={'/external_planner_max_command_z':ground+motion.corridor_max_agl}))
        runtime['mission']['post_delivery_parameter_stages'].append(dict(
            after_completed_waypoints=motion_meta['corridor_points_count'],
            parameters={'/external_planner_max_command_z':max(ground+motion.corridor_max_agl,capture+.1)}))
    runtime['search'].update(min_x=cover[0],max_x=cover[1],min_y=cover[2],max_y=cover[3],altitude=low,lane_spacing=settings['lane_spacing'],route_revision='competition-coverage')
    runtime['runtime'].update(start_mode='full',mission_id_prefix='competition')
    runtime['following_speed_profile']=copy.deepcopy(settings['following_speed_profile'])
    if settings.get('corridor_speed_schedule'):
        schedule=copy.deepcopy(settings['corridor_speed_schedule'])
        schedule['wall_coordinates']=[v+(x if schedule['axis']==0 else y) for v in schedule['wall_coordinates']]
        if motion.enabled: schedule['entry_waypoints']=motion_meta['entry_completed_waypoints']
        runtime['corridor_speed_schedule']=schedule
    runtime['high_view_probe']=dict(config=dict(ground_z=ground,high_agl=settings['high_agl'],low_agl=settings['low_agl'],staging_xy=point(*settings['staging_xy'],0)[:2],survey_xy=[point(*p,0)[:2] for p in settings['survey_xy']],source_key='competition-field-rig'),camera_info_topic=settings['camera_info_topic'],low_stage_parameters=[
        dict(name='/external_planner_max_command_z',value=max(ground+1.85,capture+.1)),
        dict(name='/fast_planner_node/sdf_map/virtual_ceil_height',value=-.1),
        dict(name='/fast_planner_node/fsm/goal_adjustment_radius',value=.15)])
    policy=copy.deepcopy(settings['survey_policy']);policy.update(high_min_agl=max(1.8,settings['high_agl']-.2),high_max_agl=settings['high_agl']+.2)
    runtime['high_view_full']=dict(policy=policy,grid=dict(bounds=search,resolution=.10,inflation=.25),boundary_policy=dict(enabled=True,bounds=target))
    ProbeConfig(**runtime['high_view_probe']['config']);SurveyPolicy(**policy)
    control=yaml.safe_load((cfg/'control_base.yaml').read_text())
    control.update(waypoints=[dict(x=x,y=y,z=low,yaw=0.,pointmode='Takeoff_point',hover_time=0.)],align_height=low,land_height=ground+.4,px4_max_distance=.4)
    control['switch'].update(auto_land=True,flag_landing_detect=1)
    control['drop_system'].update(enable_drop=True,release_setpoint_height=drop,height_threshold=drop+.1)
    for key in ('slot_offsets','dynamic_slot_offsets'):control['drop_system'][key]=copy.deepcopy(rig[key])
    control['uav_vision'].update(recovery_height=low,standard_recovery_setpoint_height=low+.1,cross_recovery_setpoint_height=low+.1,pixel_to_body_matrix=rig['pixel_to_body_matrix'],max_movement_distance=.15,
        drop_metric_scale_enabled=True,drop_ground_z=ground,
        drop_map_frame=rig['mission_frame'],drop_camera_info_topic=settings['camera_info_topic'])
    control['external_landing'].update(frame=rig['mission_frame'],capture_height=capture,auto_land_height=ground+.55,detections_topic='/uav_vision/detections_mapped')
    overrides={
        '/landing_detector/landing_enable_h_stroke_fallback':settings['landing_enable_h_stroke_fallback'],
        '/fast_planner_node/sdf_map/resolution':.10,
        '/fast_planner_node/sdf_map/map_size_x':settings['map_size'][0],'/fast_planner_node/sdf_map/map_size_y':settings['map_size'][1],'/fast_planner_node/sdf_map/map_size_z':settings['map_size'][2],
        '/fast_planner_node/sdf_map/visualization_rate':2.,
        '/fast_planner_node/sdf_map/local_update_range_x':4.5,'/fast_planner_node/sdf_map/local_update_range_y':3.,'/fast_planner_node/sdf_map/local_update_range_z':3.,
        '/fast_planner_node/sdf_map/ground_height':ground-.1,'/fast_planner_node/sdf_map/virtual_ceil_height':-.1,
        '/fast_planner_node/sdf_map/horizontal_avoidance/enabled':settings['obstacle_columns_enabled'],
        '/fast_planner_node/sdf_map/horizontal_avoidance/floor_z':ground+.1,'/fast_planner_node/sdf_map/horizontal_avoidance/obstacle_min_z':ground+.4,
        '/fast_planner_node/sdf_map/obstacles_inflation':.25,'/fast_planner_node/sdf_map/obstacles_inflation_up':.20,'/fast_planner_node/sdf_map/obstacles_inflation_down':.10,
        '/fast_planner_node/sdf_map/search_region/enabled':True,
        '/fast_planner_node/fsm/goal_adjustment_radius':.3,
        '/traj_server/traj_server/target_dist':.4,'/external_planner_start_max_distance':1.2,
        '/external_planner_max_command_z':cap,'/navigation/planner_bridge/execution/max_goal_z':cap,
        '/navigation/planner_bridge/execution/initial_plan_timeout':12.,'/navigation/planner_bridge/execution/search_initial_plan_timeout':12.,
        '/fast_planner_node/fsm/liveness_enabled':True,'/fast_planner_node/fsm/server_hold_replan_enabled':True,
        '/fast_planner_node/fsm/server_hold_seconds':.25,'/fast_planner_node/fsm/server_progress_max_age':.5,
        '/fast_planner_node/progress/enabled':True,'/traj_server/progress/enabled':True,'/traj_server/traj_server/require_goal_identity':True,
        '/navigation/planner_bridge/target/recovery_height':low,
        '/release_permission_arbiter/pose_topic':'/navigation/local_pose','/release_permission_arbiter/min_release_altitude':drop-.08,'/release_permission_arbiter/max_release_altitude':drop+.12,
        '/target_map_projector/coarse_navigation_enabled':True,'/target_map_projector/coarse_min_confidence':policy['coarse_min_confidence'],
        '/target_memory/search_confirmation_max_gap_sec':1.,'/drop_aligner/stable_frames':5,
    }
    if motion.enabled:
        overrides['/navigation/planner_bridge/motion_optimization']=copy.deepcopy(settings['motion_optimization'])
        overrides['/fast_planner_node/search/line_deviation_weight']=0.
        if motion.moving_recovery:
            handoff=struct.unpack('f',struct.pack('f',ground+motion.recovery_handoff_agl))[0]
            control['uav_vision']['recovery_height']=handoff
            overrides['/navigation/planner_bridge/target/recovery_height']=handoff
            # Legacy climb setpoints stay high until a fresh, collision-checked
            # planner transaction takes over. No horizontal direct-control shortcut.
    for region,box in [('horizontal_avoidance',search),('search_region',search)]:
        for key,val in zip(('min_x','max_x','min_y','max_y'),box):overrides['/fast_planner_node/sdf_map/'+region+'/'+key]=val
    for name,data in [('runtime.yaml',runtime),('control.yaml',control),('overrides.yaml',overrides)]:
        (out/name).write_text(yaml.safe_dump(data,sort_keys=False))
    reference=dict(mode='high_view_full',mapping_profile='high',fc_xyz=[x,y,z],ground_z=ground,low_z=low,high_z=high,drop_z=drop,takeoff_z=low,known_rig=rig,settings=settings)
    (out/'ground_reference.json').write_text(json.dumps(reference,indent=2))
    return reference
