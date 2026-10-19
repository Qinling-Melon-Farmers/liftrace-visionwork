from pathlib import Path
import json,xml.etree.ElementTree as ET
import roslaunch,yaml
D=Path(__file__).resolve().parent;R=D.parents[2];rows=[]
for case in json.loads((D/'cases.json').read_text()):
    seed=case['seed'];scene=Path(case['scene_dir'])
    config=roslaunch.config.load_config_default([(str(R/'vision_ws/src/uav_high_view/launch/fov_inner_repair.launch'),[f'scene_dir:={scene}',f'field_seed:={seed}','corridor_fast:=true','high_agl:=2.6','target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'])],11311,verbose=False)
    values={k:v.value for k,v in config.params.items()}
    assert values['/uav_vision/pixel_to_body_matrix']==[-1.,0.,0.,1.]
    assert values['/fast_planner_node/sdf_map/search_region/enabled']
    assert values['/navigation_vcl06_assertion/post_delivery_gate/low_height_region/max_height']==1.2
    assert values['/simulation/landing_phase_active'] is False
    field=next(v for v in ET.parse(scene/'field.world').iter('model') if v.get('name')=='toudi2')
    for name in ('Wall_1','Wall_9','Wall_11','Wall_12'):
        link=next(v for v in field.findall('link') if v.get('name')==name)
        assert float(link.findtext('collision/geometry/box/size').split()[2])==4.
    assert len(yaml.safe_load((scene/'field_config.yaml').read_text())['spawn']['frozen_layout'])==5
    rows.append(dict(seed=seed,parameter_count=len(values),nodes=len(config.nodes),passed=True))
(D/'preflight.json').write_text(json.dumps(rows,indent=2));print('Ten launch expansions passed')
