"""Export five NEW fully-random scenes (36-40) and their high-fast runtimes.

Reuses the same recipe as docs/verification/fast_full_random_20260914/prepare.py:
scene_layout -> export_scene -> fast_runtime. Door patterns are explicit per the
user's request, with LL (seed31-like class) weighted twice. No simulator here.
"""
from pathlib import Path
import importlib.util, json, sys
import yaml

ROOT = Path(__file__).resolve().parents[3]
D = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'simulation_tools'))
from r2026_scene import scene_layout  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'scene_export', str(ROOT / 'simulation_tools/tools/export_r2026_scene.py'))
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)

# Explicit door patterns: 36=LL, 37=LR, 38=RL, 39=RR, 40=LL (LL weighted twice).
PATTERNS = {36: 'LL', 37: 'LR', 38: 'RL', 39: 'RR', 40: 'LL'}
SEEDS = sorted(PATTERNS)


def fast_runtime(source, destination):
    """High-fast batch adjustment: move target_dist/px4_max_distance stage 0 -> 1."""
    cfg = yaml.safe_load(Path(source).read_text())
    stages = cfg['mission']['post_delivery_parameter_stages']
    before = next(s for s in stages if s['after_completed_waypoints'] == 0)
    arrival = next(s for s in stages if s['after_completed_waypoints'] == 1)
    for key in ('/traj_server/traj_server/target_dist', '/px4_max_distance'):
        arrival['parameters'][key] = before['parameters'].pop(key)
    assert not any('target_dist' in k or 'px4_max_distance' in k
                   for k in before['parameters'])
    Path(destination).write_text(yaml.safe_dump(cfg, sort_keys=False))


if __name__ == '__main__':
    mission = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission/config'
    world = ROOT / 'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world'
    cases = []
    index = {}
    for seed in SEEDS:
        folder = D / f'seed_{seed}'
        folder.mkdir(parents=True, exist_ok=True)
        layout = scene_layout(seed, door_seed=1000 + seed,
                              obstacle_seed=2000 + seed, pattern=PATTERNS[seed])
        export.export_scene(world, mission / 'coverage_r2026_horizontal.yaml',
                            mission / 'vcl06_full_low_corridor_runtime.yaml',
                            folder, layout)
        fast_runtime(folder / 'experimental_runtime.yaml', folder / 'runtime.yaml')
        entry = dict(seed=seed, world=str(folder / 'field.world'),
                     field_config=str(folder / 'field_config.yaml'),
                     runtime_config=str(folder / 'runtime.yaml'),
                     gate_geometry_config=str(folder / 'gate_geometry.yaml'))
        cases.append(entry)
        index[str(seed)] = dict(entry, door_pattern=layout['door_pattern'],
                                door_seed=1000 + seed, obstacle_seed=2000 + seed)
        print(f'seed {seed}: pattern={layout["door_pattern"]} '
              f'trees={[(round(t["x"],2), round(t["y"],2)) for t in layout["trees"]]}')
    (D / 'cases.json').write_text(json.dumps(cases, indent=2))
    (D / 'scenarios.json').write_text(json.dumps(index, indent=2))
    print('PREPARE_OK')