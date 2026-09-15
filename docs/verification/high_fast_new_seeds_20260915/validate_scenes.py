"""Static validation of the five new scenes: files, geometry, runtime, launch expansion.

No simulator, no ROS master: roslaunch XML expansion only (same method as
high_fast_five_20260915/preflight.py). Run under rl_drone with ROS sourced.
"""
from pathlib import Path
import json, sys
import yaml

D = Path(__file__).resolve().parent
R = D.parents[2]
MODEL = '/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'
assert Path(MODEL).is_file(), MODEL

cases = json.loads((D / 'cases.json').read_text())
scene = json.loads((D / 'scenarios.json').read_text())
assert len(cases) == 5 and [c['seed'] for c in cases] == [36, 37, 38, 39, 40]

import roslaunch  # noqa: E402

rows = []
for case in cases:
    seed = case['seed']
    meta = scene[str(seed)]
    for key in ('world', 'field_config', 'runtime_config', 'gate_geometry_config'):
        assert Path(case[key]).is_file(), case[key]
    fld = yaml.safe_load(Path(case['field_config']).read_text())
    gate = yaml.safe_load(Path(case['gate_geometry_config']).read_text())
    run = yaml.safe_load(Path(case['runtime_config']).read_text())
    scn = json.loads((D / f'seed_{seed}' / 'scene.json').read_text())

    # declared pattern must match the gate geometry openings (lateral_min > 8.0 => left/+Y)
    pattern = meta['door_pattern']
    got = ''.join('L' if d['lateral_min'] > 8.0 else 'R'
                  for d in gate['post_delivery_gate']['doors'][1:3])
    assert got == pattern, f'seed {seed}: pattern {pattern} != geometry {got}'
    assert len(gate['post_delivery_gate']['doors']) == 3
    region = gate['post_delivery_gate']['low_height_region']
    assert region['max_height'] == 0.7 and region['min_y'] == 7.6 and region['max_y'] == 9.1

    # trees in scene.json must equal the field_config static exclusions
    trees = sorted((round(t['x'], 6), round(t['y'], 6)) for t in scn['trees'])
    excl = sorted((round(e['world_x'], 6), round(e['world_y'], 6))
                  for e in fld['static_exclusions'] if e['name'].startswith('combined_tree_'))
    assert trees == excl, f'seed {seed}: tree mismatch'
    assert len(trees) == 4

    # high-fast runtime adjustment must be present
    stages = run['mission']['post_delivery_parameter_stages']
    first = next(s for s in stages if s['after_completed_waypoints'] == 0)
    second = next(s for s in stages if s['after_completed_waypoints'] == 1)
    for key in ('/traj_server/traj_server/target_dist', '/px4_max_distance'):
        assert key in second['parameters'] and key not in first['parameters'], f'seed {seed}: {key}'

    # launch expansion: same interface and high-fast contracts as the 31-35 batch
    cfg = roslaunch.ROSLaunchConfig()
    args = ['strategy:=true', f'field_seed:={seed}', f'target_model_path:={MODEL}'] + [
        f'{k}:={case[k]}' for k in ('world', 'field_config', 'runtime_config', 'gate_geometry_config')]
    roslaunch.xmlloader.XmlLoader().load(
        str(R / 'vision_ws/src/uav_high_view/launch/fast_comparison.launch'), cfg, argv=args, verbose=False)
    value = lambda p: cfg.params[p].value
    assert value('/external_planner_start_max_distance') == 1.2
    assert value('/fast_planner_node/fsm/goal_adjustment_radius') == .3
    assert value('/fast_planner_node/sdf_map/horizontal_avoidance/enabled') is True
    assert '/fast_planner_node/sdf_map/horizontal_avoidance/column_top_z' not in cfg.params
    assert value('/navigation/mission_manager/following_speed_profile/cruise_lead_m') == 1.
    assert value('/navigation/mission_manager/following_speed_profile/corridor_after_waypoints') == 1
    rows.append(dict(seed=seed, door_pattern=pattern, trees=len(trees),
                     launch_expanded=True, fast_runtime=True))

(D / 'scene_validation.json').write_text(json.dumps(
    dict(python=sys.executable, scope='Static scene + launch expansion only', cases=rows), indent=2))
print('PASS five new scenes: geometry, trees, runtime adjustment, launch expansion')