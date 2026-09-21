"""ROS launch expansion only; no nodes or simulator started."""
from pathlib import Path
import ast,json,roslaunch
D=Path(__file__).resolve().parent;R=D.parents[2]
ast.parse((D/'run_matrix.py').read_text());results=[]
for seed in (31,32,34,37,38,40):
    scene=R/f'docs/verification/history_31_40_20260920/seed_{seed}'
    cfg=roslaunch.config.load_config_default([(str(D/'replay.launch'),[
      'scene_dir:='+str(scene),'field_seed:='+str(seed),
      'target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'])],11311,verbose=False)
    v={k:x.value for k,x in cfg.params.items()}
    expected={'/navigation_vcl06_assertion/wall_timeout':6000.0,
              '/navigation_vcl06_assertion/startup_wall_timeout':600.0,
              '/navigation/planner_bridge/execution/initial_plan_timeout':12.0,
              '/navigation/planner_bridge/execution/search_initial_plan_timeout':12.0,
              '/navigation/mission_manager/high_view_probe/config/high_agl':2.6,
              '/fast_planner_node/sdf_map/search_region/boundary_policy_param':'/navigation/mission_manager/high_view_full/boundary_policy'}
    for k,x in expected.items():assert v.get(k)==x,(seed,k,v.get(k),x)
    results.append(dict(seed=seed,scene=str(scene),nodes=len(cfg.nodes),passed=True,expected=expected))
(D/'preflight.json').write_text(json.dumps(results,indent=2)+'\n');print('Six launch expansions passed')
