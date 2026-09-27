from pathlib import Path
import os,json,math,rospkg
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader
R=Path(__file__).resolve().parents[3]
D=R/'docs/verification/balanced_columns_20260927'
packages=rospkg.RosPack()
for name,expected in [('uav_mission',R/'patrol_uav_ws-patrol_planner/src/uav_mission'),('uav_vision',R/'vision_ws/src/uav_vision'),('uav_high_view',R/'vision_ws/src/uav_high_view')]:
    assert Path(packages.get_path(name)).resolve()==expected.resolve(),(name,packages.get_path(name))
for seed in (31,38):
    os.environ['SIM_RUN_DIR']=str(R/'logs'/f'preflight_balanced_seed{seed}')
    c=ROSLaunchConfig()
    XmlLoader().load(str(D/'replay.launch'),c,verbose=False,argv=[f'scene_dir:={R}/docs/verification/history_31_40_20260920/seed_{seed}',f'field_seed:={seed}','target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'])
    params={k:v.value for k,v in c.params.items()}
    expected={
      '/fast_planner_node/sdf_map/obstacles_inflation':.275,
      '/fast_planner_node/sdf_map/obstacles_inflation_up':.2,
      '/fast_planner_node/sdf_map/obstacles_inflation_down':.1,
      '/fast_planner_node/sdf_map/horizontal_avoidance/enabled':True,
      '/fast_planner_node/sdf_map/horizontal_avoidance/column_max_hull_span':1.6,
      '/fast_planner_node/sdf_map/horizontal_avoidance/column_max_fill_distance':.35,
      '/fast_planner_node/sdf_map/resolution':.05,
      '/navigation/mission_manager/high_view_full/grid/inflation':.275,
      '/navigation/mission_manager/high_view_full/policy/coarse_enabled':True,
      '/navigation/mission_manager/high_view_full/policy/coarse_interrupt_min_interval_ns':100000000,
      '/navigation/mission_manager/high_view_full/policy/recheck_observe_seconds':5.,
      '/navigation/mission_manager/high_view_probe/config/high_agl':2.6,
    }
    for key,value in expected.items():assert params.get(key)==value,(key,params.get(key),value)
    selected={k:v for k,v in params.items() if any(word in k for word in ('sdf_map/','high_view_full/','high_view_probe/','presentation_'))}
    selected['offline_checks']={'status':'PASS','effective_xy_dilation_m':math.ceil(.275/.05)*.05,'nodes_launched':False}
    (D/f'preflight_seed{seed}.json').write_text(json.dumps(selected,indent=2))
    print('seed',seed,'XML parameters PASS; no ROS nodes launched')
