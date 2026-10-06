from pathlib import Path
import sys,json,tempfile
import yaml,roslaunch,rospkg
from trial_config import generate,TRIAL_FOLDERS,HIGH_MODES,H_MODES,NO_DROP_MODES,mapping_profile
from trial_manager import BoardManager
from unittest.mock import patch
R=Path(__file__).resolve().parents[5];P=R/'deployment/board_trials_4x4/common/uav_board_trials'
roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(R/'vision_ws/src'),str(R/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
rows=[]
for folder in TRIAL_FOLDERS.values():
    s=yaml.safe_load((R/'deployment/board_trials_4x4'/folder/'settings.yaml').read_text());rig=yaml.safe_load((P/'config/known_rig.yaml').read_text())
    if folder in ('04_corridor_landing','08_full_mission'):
        # Test fixture only; the shipped settings file remains empty.
        s['corridor_waypoints']=[dict(x=.6,y=0),dict(x=1.5,y=.4)]
        s['landing_xy']=[2.5,0]
    with tempfile.TemporaryDirectory() as tmp:
        ref=generate(R,tmp,s,(0.,0.,-.05),rig)
        for enabled in ('false','true'):
            cfg=roslaunch.config.load_config_default([(str(P/'launch/application.launch'),[f'enable_control_output:={enabled}',f'mode:={s["mode"]}','model_path:=/test/model.rknn',f'generated_dir:={tmp}',f'ground_z:={ref["ground_z"]}',f'low_z:={ref["low_z"]}',f'cruise_speed:={s["cruise_speed"]}',f'cruise_acceleration:={s["cruise_acceleration"]}',f'terminal_hover_enabled:={str("terminal_hover_agl" in s).lower()}',f'max_command_z:={ref["ground_z"]+s.get("max_agl",2.9)}'])],11311,verbose=False)
            values={k:v.value for k,v in cfg.params.items()};nodes={n.name:n for n in cfg.nodes}
            assert values['/landing_detector/landing_enable_h_stroke_fallback']==(s['mode'] in H_MODES)
            assert values['/landing_detector/landing_enable_h_structure_check'] is True
            assert values['/landing_detector/default_align_mode']=='disabled'
            assert values['/landing_detector/process_only_in_landing_mode'] is True
            if s['mode'] in H_MODES and enabled=='true':
                assert values['/external_landing/handoff_mode']==s.get('landing_handoff_mode','AUTO.LAND')
                assert values['/external_landing/detections_topic']=='/uav_vision/detections_mapped'
                assert values['/external_landing/alignment_tolerance']==.08
                assert values['/external_landing/stable_frames']==10
                assert values['/external_landing/mark_max_age_sec']==.5
                assert 'trial_auto_land' not in nodes and 'trial_terminal_hover' not in nodes
            assert values['/fast_planner_node/sdf_map/visualization_rate']==2.
            assert values['/fast_planner_node/manager/max_vel']==s['cruise_speed']
            assert s['cruise_speed']==(1. if s['mode']=='high_speed_capture' else .5)
            assert values['/fast_planner_node/search/max_vel']==s['cruise_speed']
            assert values['/fast_planner_node/manager/max_acc']==s['cruise_acceleration']==0.35
            assert values['/fast_planner_node/sdf_map/virtual_ceil_height']==-.1
            assert [values['/fast_planner_node/sdf_map/'+key] for key in ('obstacles_inflation','obstacles_inflation_up','obstacles_inflation_down')]==[.25,.2,.1]
            assert values['/fast_planner_node/sdf_map/horizontal_avoidance/enabled']==(s['mode'] in HIGH_MODES and s.get('obstacle_columns_enabled',True))
            assert values['/fast_planner_node/sdf_map/horizontal_avoidance/column_middle_enabled']
            assert values['/fast_planner_node/sdf_map/horizontal_avoidance/column_band_low_ratio']==.4
            assert values['/fast_planner_node/sdf_map/horizontal_avoidance/column_band_high_ratio']==.6
            assert values['/fast_planner_node/sdf_map/local_update_range_x']>=3.4
            assert values['/fast_planner_node/sdf_map/local_update_range_y']>=2.
            assert values['/navigation/planner_bridge/execution/initial_plan_timeout']==12.
            assert values['/fast_planner_node/fsm/server_hold_replan_enabled']
            assert values['/fast_planner_node/progress/enabled']
            assert values['/traj_server/progress/enabled']
            assert values['/traj_server/traj_server/require_goal_identity']
            if s['mode'] in HIGH_MODES:assert values['/navigation/mission_manager/high_view_probe/config/staging_xy']==([.8,0.] if s['mode']=='high_speed_capture' else [.6,.05])
            assert not any(n.package in ('gazebo_ros','actuator_pwm') for n in cfg.nodes)
            assert 'trial_journal' in nodes and 'trial_recorder' not in nodes and 'target_detector_rknn' in nodes
            metadata=Path(values['/target_detector_rknn/metadata_path'])
            contract=yaml.safe_load(metadata.read_text())
            assert list(contract['names'].values())==['bridge','panzer','pillbox','tent','red_cross']
            assert contract['output_channels']==9 and contract['box_format']=='xywh'
            assert ('patrol_control' in nodes)==(enabled=='true')
            assert ('board_mock_servo' in nodes)==(enabled=='true' and s['mode'] not in NO_DROP_MODES)
            assert ('trial_auto_land' in nodes)==(enabled=='true' and s['mode'] not in H_MODES and 'terminal_hover_agl' not in s)
            if enabled=='true' and s['mode'] not in NO_DROP_MODES:
                assert values['/guarded_servo_proxy/raw_service_name']=='/board_trials/mock_servo';assert values['/release_permission_arbiter/pose_topic']=='/navigation/local_pose'
                assert values['/guarded_servo_proxy/service_name']=='/board_trials/Servo'
                assert ('/Servo','/board_trials/Servo') in [tuple(v) for v in nodes['patrol_control'].remap_args],nodes['patrol_control'].remap_args
                assert values['/external_landing/detections_topic']==('/uav_vision/detections_mapped' if s['mode'] in H_MODES else '/board_trials/h_disabled')
            runtime=yaml.safe_load((Path(tmp)/'runtime.yaml').read_text())
            assert values['/navigation/mission_manager/high_view_full/policy/interrupt_refined_classes']==[]
            # Exercise the actual adapter's mission/runtime construction with expanded parameters.
            def get_param(key,default=None):
                path='/navigation/mission_manager/'+key[1:] if key.startswith('~') else key
                if path in values:return values[path]
                prefix=path.rstrip('/')+'/'
                selected={k[len(prefix):]:v for k,v in values.items() if k.startswith(prefix)}
                if selected:
                    result={}
                    for name,value in selected.items():
                        parts=name.split('/');target=result
                        for part in parts[:-1]:target=target.setdefault(part,{})
                        target[parts[-1]]=value
                    return result
                return default
            manager=BoardManager.__new__(BoardManager);manager.mode=s['mode'];manager._profile_name='r2026';manager._profile_path=str(R/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_profiles.yaml')
            manager._high_stage_parameters=None
            with patch('rospy.get_param',side_effect=get_param):actual=manager._new_runtime()
            rows.append(dict(folder=folder,trial=s['mode'],control_output=enabled,nodes=len(nodes),runtime=type(actual).__name__,passed=True))
        for enabled in ('false','true'):
            cfg=roslaunch.config.load_config_default([(str(P/'launch/localization.launch'),[f'enable_control_output:={enabled}',"alignment_mode:="+s.get('alignment_mode','measured'),'mapping_profile:='+mapping_profile(s)])],11311,verbose=False)
            alignment=[n for n in cfg.nodes if n.name=='map_camera_alignment']
            assert len(alignment)==1
            assert alignment[0].type==('static_transform_publisher' if s.get('alignment_mode')=='legacy_static' else 'map_camera_alignment.py')
            params={k:v.value for k,v in cfg.params.items()};assert params['/navigation_frame_adapter/mission_frame']=='camera_init';assert params['/navigation_frame_adapter/local_frame']=='map';assert params['/navigation_frame_adapter/enable_setpoints']==(enabled=='true')
            assert params['/feature_extract_enable'] is True
            assert params['/cube_side_length']==20.0 and params['/mapping/det_range']==6.0
            # FAST-LIO must not relocate its cube while stationary at its center.
            assert params['/cube_side_length']/2 > 1.5*params['/mapping/det_range']
            assert not any(n.name=='freedom' for n in cfg.nodes)
            mapping=roslaunch.config.load_config_default([(str(P/'launch/mapping.launch'),['mapping_profile:='+mapping_profile(s)])],11311,verbose=False)
            params.update({k:v.value for k,v in mapping.params.items()})
            assert [n.name for n in mapping.nodes]==['freedom']
            assert params['/freedom/sensor_tf_frame']=='mapping_imu'
            assert params['/freedom/map_tf_frame']=='camera_init'
            assert params['/freedom/map/voxel_depth']==2
            assert params['/freedom/map/sub_voxel_size']==.1
            assert params['/freedom/map/counts_to_free']==6 and params['/freedom/map/counts_to_revert']==20
            name=mapping_profile(s);distance,top={'low':(6.,2.),'high':(6.,3.2),'corridor':(5.,1.5)}[name]
            for prefix in ('/freedom/sensor/','/freedom/map/raycast_'):
                assert params[prefix+'max_range']==distance
                assert params[prefix+'min_z']==-1.0 and params[prefix+'max_z']==top
            # Loading resource profiles must preserve real sensor type and extrinsics.
            assert params['/preprocess/lidar_type']==1
            assert params['/mapping/extrinsic_T']==[-.011,-.02329,.04412]

(R/'logs/high_speed_capture_20260929').mkdir(parents=True,exist_ok=True)
(R/'logs/high_speed_capture_20260929/validation_static.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
