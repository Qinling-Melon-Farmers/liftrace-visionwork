import json,os,sys
from pathlib import Path
import roslaunch,yaml
D=Path(__file__).resolve().parent;R=D.parents[2]
model='/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'
assert Path(model).is_file();rows=[]
for case in json.loads((D/'cases.json').read_text()):
    cfg=roslaunch.ROSLaunchConfig()
    args=['strategy:=true',f'field_seed:={case["seed"]}',f'target_model_path:={model}']+[f'{k}:={case[k]}' for k in ('world','field_config','runtime_config','gate_geometry_config')]
    roslaunch.xmlloader.XmlLoader().load(str(R/'vision_ws/src/uav_high_view/launch/fast_comparison.launch'),cfg,argv=args,verbose=False)
    value=lambda p:cfg.params[p].value
    assert value('/external_planner_start_max_distance')==1.2
    assert value('/fast_planner_node/fsm/goal_adjustment_radius')==.3
    assert value('/fast_planner_node/sdf_map/horizontal_avoidance/enabled') is True
    assert '/fast_planner_node/sdf_map/horizontal_avoidance/column_top_z' not in cfg.params
    assert value('/navigation/mission_manager/following_speed_profile/cruise_lead_m')==1.
    assert value('/navigation/mission_manager/following_speed_profile/corridor_after_waypoints')==1
    source=R/f'docs/verification/full_random_five_20260910/seed_{case["seed"]}/scenario_inputs'
    assert Path(case['world']).read_bytes()==(source/'field.world').read_bytes()
    old=yaml.safe_load((source/'runtime.yaml').read_text());new=yaml.safe_load(Path(case['runtime_config']).read_text())
    for data in (old,new):data['mission'].pop('post_delivery_parameter_stages')
    assert old==new
    rows.append(dict(seed=case['seed'],same_historical_world=True,mission_unchanged_except_speed_stage=True,full_height_columns=True))
(D/'preflight.json').write_text(json.dumps(dict(python=sys.executable,cases=rows,scope='Static launch expansion only'),indent=2))
print('PASS three historical scenes and high-fast parameter contracts')
