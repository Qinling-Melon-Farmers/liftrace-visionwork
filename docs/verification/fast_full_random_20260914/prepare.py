"""Export fixed pilot and five random layouts; no simulator or flight commands."""
from pathlib import Path
import importlib.util,json,sys
import yaml

ROOT=Path(__file__).resolve().parents[3]
D=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'simulation_tools'))
from r2026_scene import scene_layout
spec=importlib.util.spec_from_file_location('scene_export',str(ROOT/'simulation_tools/tools/export_r2026_scene.py'))
export=importlib.util.module_from_spec(spec);spec.loader.exec_module(export)


def fast_runtime(source,destination):
    cfg=yaml.safe_load(Path(source).read_text())
    stages=cfg['mission']['post_delivery_parameter_stages']
    before=next(s for s in stages if s['after_completed_waypoints']==0)
    arrival=next(s for s in stages if s['after_completed_waypoints']==1)
    for key in ('/traj_server/traj_server/target_dist','/px4_max_distance'):
        arrival['parameters'][key]=before['parameters'].pop(key)
    assert not any('target_dist' in k or 'px4_max_distance' in k for k in before['parameters'])
    assert cfg['mission']['post_delivery_route'][0][:2]==cfg['mission']['post_delivery_route'][1][:2]
    Path(destination).write_text(yaml.safe_dump(cfg,sort_keys=False))


if __name__=='__main__':
    mission=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config'
    pilot=D/'pilot';pilot.mkdir(exist_ok=True)
    fast_runtime(ROOT/'docs/verification/high_view_full_20260914/seed_32/runtime.yaml',pilot/'runtime.yaml')
    index={}
    for seed in range(31,36):
        folder=D/f'seed_{seed}'
        layout=scene_layout(seed)
        export.export_scene(ROOT/'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world',
                            mission/'coverage_r2026_horizontal.yaml',mission/'vcl06_full_low_corridor_runtime.yaml',folder,layout)
        fast_runtime(folder/'experimental_runtime.yaml',folder/'runtime.yaml')
        index[str(seed)]=dict(seed=seed,world=str(folder/'field.world'),field_config=str(folder/'field_config.yaml'),
                              runtime_config=str(folder/'runtime.yaml'),gate_geometry_config=str(folder/'gate_geometry.yaml'),
                              door_pattern=layout['door_pattern'],trees=layout['trees'])
    (D/'scenarios.json').write_text(json.dumps(index,indent=2))
    print(json.dumps({s:v['door_pattern'] for s,v in index.items()}))
