from pathlib import Path
import sys,json,tempfile
import yaml,roslaunch,rospkg
from trial_config import generate
from trial_manager import BoardManager
from unittest.mock import patch
R=Path(__file__).resolve().parents[5];P=R/'deployment/board_trials_4x4/common/uav_board_trials'
roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(R/'vision_ws/src'),str(R/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
rows=[]
for folder in ('01_visual_interrupt','02_high_view_revisit','03_h_landing'):
    s=yaml.safe_load((R/'deployment/board_trials_4x4'/folder/'settings.yaml').read_text());rig=yaml.safe_load((P/'config/known_rig.yaml').read_text())
    with tempfile.TemporaryDirectory() as tmp:
        ref=generate(R,tmp,s,(0.,0.,-.05),rig)
        for enabled in ('false','true'):
            cfg=roslaunch.config.load_config_default([(str(P/'launch/application.launch'),[f'enable_control_output:={enabled}',f'mode:={s["mode"]}','model_path:=/test/model.rknn',f'generated_dir:={tmp}',f'ground_z:={ref["ground_z"]}',f'low_z:={ref["low_z"]}'])],11311,verbose=False)
            values={k:v.value for k,v in cfg.params.items()};nodes={n.name:n for n in cfg.nodes}
            assert not any(n.package in ('gazebo_ros','actuator_pwm') for n in cfg.nodes)
            assert 'trial_recorder' in nodes and 'target_detector_rknn' in nodes
            assert ('patrol_control' in nodes)==(enabled=='true')
            assert ('board_mock_servo' in nodes)==(enabled=='true' and s['mode']!='landing')
            assert ('trial_auto_land' in nodes)==(enabled=='true' and s['mode']!='landing')
            if enabled=='true' and s['mode']!='landing':
                assert values['/guarded_servo_proxy/raw_service_name']=='/board_trials/mock_servo';assert values['/release_permission_arbiter/pose_topic']=='/navigation/local_pose'
                assert values['/guarded_servo_proxy/service_name']=='/board_trials/Servo'
                assert ('/Servo','/board_trials/Servo') in [tuple(v) for v in nodes['patrol_control'].remap_args],nodes['patrol_control'].remap_args
                assert values['/external_landing/detections_topic']=='/board_trials/h_disabled'
            runtime=yaml.safe_load((Path(tmp)/'runtime.yaml').read_text())
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
            with patch('rospy.get_param',side_effect=get_param):actual=manager._new_runtime()
            rows.append(dict(trial=s['mode'],control_output=enabled,nodes=len(nodes),runtime=type(actual).__name__,passed=True))
        for enabled in ('false','true'):
            cfg=roslaunch.config.load_config_default([(str(P/'launch/localization.launch'),[f'enable_control_output:={enabled}'])],11311,verbose=False)
            params={k:v.value for k,v in cfg.params.items()};assert params['/navigation_frame_adapter/mission_frame']=='camera_init';assert params['/navigation_frame_adapter/local_frame']=='map';assert params['/navigation_frame_adapter/enable_setpoints']==(enabled=='true')
(R/'deployment/board_trials_4x4/validation_static.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
