from pathlib import Path
import json,math,roslaunch
root=Path(__file__).resolve().parents[3]
out=root/'docs/verification/shared_boundary_20260921';out.mkdir(exist_ok=True)
launch=root/'vision_ws/src/uav_high_view/launch/fov_inner_repair.launch'
cfg=roslaunch.config.load_config_default([(str(launch),[
 'scene_dir:='+str(root/'docs/verification/history_31_40_20260920/seed_32'),
 'field_seed:=32','target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'])],11311,verbose=False)
v={k:x.value for k,x in cfg.params.items()}
key=v['/fast_planner_node/sdf_map/search_region/boundary_policy_param']
assert v[key+'/enabled'] is True
b=v[key+'/bounds'];side=v.get(key+'/guard_side_m',.55);yaw=math.radians(v.get(key+'/yaw_budget_deg',10));reserve=v.get(key+'/tracking_reserve_m',.03)
clearance=v['/fast_planner_node/manager/clearance_threshold'];assert reserve>clearance>=0
half=.5*side*(math.cos(yaw)+math.sin(yaw))
hard=[b[0]+half,b[1]-half,b[2]+half,b[3]-half]
result={'passed':True,'shared_source':key,'field':b,'hard_center_bounds':hard,'tracking_reserve':reserve,'planner_clearance':clearance,'nodes':len(cfg.nodes),'scope':'offline launch expansion; not dynamic flight'}
(out/'launch_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
