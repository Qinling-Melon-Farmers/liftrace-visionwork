from pathlib import Path
import os,json,math,rospkg
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader
R=Path(__file__).resolve().parents[3]
D=R/'docs/verification/integrated_two_20261004'
packages=rospkg.RosPack()
for name,expected in [('uav_mission',R/'patrol_uav_ws-patrol_planner/src/uav_mission'),('uav_vision',R/'vision_ws/src/uav_vision'),('uav_high_view',R/'vision_ws/src/uav_high_view')]:
    assert Path(packages.get_path(name)).resolve()==expected.resolve(),(name,packages.get_path(name))
for seed in (31,38):
    os.environ['SIM_RUN_DIR']=str(R/'logs'/f'preflight_balanced_seed{seed}')
    c=ROSLaunchConfig()
    XmlLoader().load(str(D/'replay.launch'),c,verbose=False,argv=[f'scene_dir:={R}/docs/verification/integrated_two_20261004/seed_{seed}',f'field_seed:={seed}','target_model_path:=/home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt'])
    params={k:v.value for k,v in c.params.items()}
    expected={
      '/fast_planner_node/sdf_map/obstacles_inflation':.25,
      '/fast_planner_node/sdf_map/obstacles_inflation_up':.2,
      '/fast_planner_node/sdf_map/obstacles_inflation_down':.1,
      '/fast_planner_node/sdf_map/horizontal_avoidance/enabled':True,
      '/fast_planner_node/sdf_map/horizontal_avoidance/column_max_hull_span':1.6,
      '/fast_planner_node/sdf_map/horizontal_avoidance/column_max_fill_distance':.35,
      '/fast_planner_node/sdf_map/resolution':.05,
      '/navigation/mission_manager/high_view_full/grid/inflation':.25,
      '/navigation/mission_manager/high_view_full/policy/coarse_enabled':True,
      '/navigation/mission_manager/high_view_full/policy/coarse_interrupt_min_interval_ns':100000000,
      '/navigation/mission_manager/high_view_full/policy/recheck_observe_seconds':5.,
      '/navigation/mission_manager/high_view_probe/config/high_agl':2.6,
    }
    expected.update({'/landing_detector/landing_enable_h_stroke_fallback':True,'/external_landing/capture_height':.68,'/uav_vision/drop_metric_scale_enabled':True,'/target_memory/drop_circle_geometry_confidence':.75,'/target_detector/pause_in_landing_mode':True,'/landing_detector/landing_adaptive_block_size':81,'/fast_planner_node/sdf_map/virtual_ceil_height':-.1})
    for key,value in expected.items():assert params.get(key)==value,(key,params.get(key),value)
    selected=params.copy()
    selected['offline_checks']={'status':'PASS','effective_xy_dilation_m':math.ceil(.25/.05)*.05,'nodes_launched':False}
    (D/f'preflight_seed{seed}.json').write_text(json.dumps(selected,indent=2))
    print('seed',seed,'XML parameters PASS; no ROS nodes launched')
