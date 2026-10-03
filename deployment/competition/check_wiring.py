#!/usr/bin/env python3
"""Offline launch expansion and real runtime construction. Starts no ROS nodes."""
from pathlib import Path
import importlib.util,json,tempfile
from unittest.mock import patch
import yaml,roslaunch,rospkg
from uav_mission.competition_config import generate
R=Path(__file__).resolve().parents[2];M=R/'patrol_uav_ws-patrol_planner/src/uav_mission'
roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(R/'vision_ws/src'),str(R/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
spec=importlib.util.spec_from_file_location('checked_full_manager',str(M/'scripts/navigation_high_view_full.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
s=yaml.safe_load((R/'deployment/competition/field.example.yaml').read_text())
# Offline fixture only; shipped field profile stays unconfirmed/without gate coordinates.
s.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
rig=yaml.safe_load((M/'config/competition/known_rig.yaml').read_text());rows=[]
def load(name,args):return roslaunch.config.load_config_default([(str(M/'launch'/('competition_'+name+'.launch')),args)],11311,verbose=False)
with tempfile.TemporaryDirectory() as tmp:
 ref=generate(R,tmp,s,(0.,0.,0.),rig)
 for enabled in ('false','true'):
  cfg=load('application',['enable_control_output:='+enabled,'model_path:=/not-loaded.rknn','generated_dir:='+tmp,'ground_z:='+str(ref['ground_z']),'low_z:='+str(ref['low_z']),'cruise_speed:='+str(s['cruise_speed']),'cruise_acceleration:='+str(s['cruise_acceleration'])])
  v={k:p.value for k,p in cfg.params.items()};nodes={n.name:n for n in cfg.nodes}
  assert not any(n.package in ('uav_board_trials','gazebo_ros','actuator_pwm') for n in cfg.nodes)
  assert nodes['mission_manager'].type=='navigation_competition_manager.py'
  assert ('patrol_control' in nodes)==(enabled=='true')
  assert ('guarded_servo_proxy' in nodes)==(enabled=='true')
  assert v['/navigation/planner_bridge/execution/enabled']==(enabled=='true')
  assert v['/fast_planner_node/manager/max_vel']==s['cruise_speed']
  assert v['/fast_planner_node/manager/max_acc']==s['cruise_acceleration']
  assert v['/fast_planner_node/sdf_map/virtual_ceil_height']==-.1
  assert [v['/fast_planner_node/sdf_map/'+k] for k in ('obstacles_inflation','obstacles_inflation_up','obstacles_inflation_down')]==[.25,.2,.1]
  assert v['/navigation/planner_bridge/target/recovery_height']==v['/navigation/mission_manager/mission/approach_altitude']
  if enabled=='true':
   assert v['/guarded_servo_proxy/raw_service_name']=='/legacy/Servo_raw'
   assert v['/guarded_servo_proxy/service_name']=='/Servo'
   assert v['/external_landing/auto_land_height']<v['/external_landing/capture_height']
  metadata=yaml.safe_load(Path(v['/target_detector_rknn/metadata_path']).read_text())
  assert list(metadata['names'].values())==['bridge','panzer','pillbox','tent','red_cross']
  def get_param(key,default=None):
   key='/navigation/mission_manager/'+key[1:] if key.startswith('~') else key
   if key in v:return v[key]
   prefix=key.rstrip('/')+'/'
   children={k[len(prefix):]:val for k,val in v.items() if k.startswith(prefix)}
   if not children:return default
   result={}
   for name,val in children.items():
    parts=name.split('/');target=result
    for part in parts[:-1]:target=target.setdefault(part,{})
    target[parts[-1]]=val
   return result
  manager=module.FullManager.__new__(module.FullManager);manager._profile_name='r2026';manager._profile_path=str(M/'config/competition_profiles.yaml')
  with patch('rospy.get_param',side_effect=get_param):runtime=manager._new_runtime()
  assert type(runtime).__name__=='HighViewFull'
  assert runtime.descent_grid is not runtime.grid
  rows.append(dict(control_output=enabled,runtime=type(runtime).__name__,nodes=len(cfg.nodes)))
 for mode in ('legacy_static','measured'):
  cfg=load('localization',['alignment_mode:='+mode,'enable_control_output:=false'])
  v={k:p.value for k,p in cfg.params.items()}
  assert not any(n.name=='freedom' for n in cfg.nodes)
  assert v['/feature_extract_enable'] and v['/cube_side_length']==20.
  assert v['/pcd_save/pcd_save_en']==False
  assert not v['/navigation_frame_adapter/enable_setpoints']
  assert len([n for n in cfg.nodes if n.name=='map_camera_alignment'])==1
 cfg=load('mapping',['mapping_profile:=high'])
 assert [n.name for n in cfg.nodes]==['freedom']
print(json.dumps(dict(status='PASS',applications=rows,localization_modes=2,mapping=1,ros_started=False),indent=2))
