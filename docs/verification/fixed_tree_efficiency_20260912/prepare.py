"""Freeze R64 tree/box groups with independent random targets and doors.

Uses existing production scene and target-layout helpers; never launches ROS.
"""
from pathlib import Path
import copy, json, math, random, subprocess, sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'simulation_tools'))
sys.path.insert(0, str(ROOT/'simulation_tools/tools'))
sys.path.insert(0, str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'))
from r2026_scene import scene_layout, PATTERNS, polygons_overlap, rectangle_vertices
from export_r2026_scene import export_scene
from uav_mission.random_field_policy import (
    Footprint, plan_footprint_layout, profile_standard_classes,
    STANDARD_FOOTPRINT_RADIUS, RED_CROSS_FOOTPRINT_RADIUS)

WORLD = 'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world'
FIELD = 'patrol_uav_ws-patrol_planner/src/uav_mission/config/coverage_r2026_horizontal.yaml'
RUNTIME = 'patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_full_low_corridor_runtime.yaml'
R64_SOURCE = '1983b2f7571cf3d96916d45af5cc8f3a2f8a9ad4'

def trees(xml):
    return [m for m in ET.fromstring(xml).iter('model') if 'tree' in m.get('name','').lower()]

def prepare():
    template = (ROOT/WORLD).read_text()
    historical = subprocess.check_output(['git','show',R64_SOURCE+':'+WORLD],cwd=ROOT,text=True)
    current_trees, r64_trees = trees(template), trees(historical)
    assert len(current_trees)==len(r64_trees)==4
    assert all(ET.tostring(a)==ET.tostring(b) for a,b in zip(current_trees,r64_trees))
    checks=[]
    for seed in [32,34]:
        door_seed=1000+seed
        pattern=random.Random(door_seed).choice(PATTERNS)
        layout=scene_layout(0,door_seed=door_seed,pattern=pattern)
        layout.update(tree_mode='FIXED_R64',tree_source=R64_SOURCE,
            field_seed=seed,door_sampling='random.Random(1000+field_seed).choice(LL,LR,RL,RR)',
            qualification='Frozen input for a new SITL comparison; not a flight result')
        destination=OUT/f'seed_{seed}'/'scenario_inputs'
        export_scene(ROOT/WORLD,ROOT/FIELD,ROOT/RUNTIME,destination,layout)
        # Preserve all original tree/box geometry; exporter only reformats pose text.
        generated_trees=trees((destination/'field.world').read_text())
        for original,generated in zip(current_trees,generated_trees):
            assert [float(x) for x in original.findtext('pose').split()]==[float(x) for x in generated.findtext('pose').split()]
            checked=copy.deepcopy(generated);checked.find('pose').text=original.findtext('pose')
            assert ET.tostring(checked)==ET.tostring(original)
        runtime=yaml.safe_load((destination/'experimental_runtime.yaml').read_text())
        assert runtime['search']['lane_spacing']==.7 and 'post_delivery_gate' not in runtime
        (destination/'runtime.yaml').write_text(yaml.safe_dump(runtime,sort_keys=False))
        (destination/'experimental_runtime.yaml').unlink()
        cfg=yaml.safe_load((destination/'field_config.yaml').read_text())
        occupied=[Footprint('H0',0,0,.5),Footprint('H1',4.2,8.5,.5)]
        occupied += [Footprint(x['name'],x['world_x'],x['world_y'],x['radius']) for x in cfg['static_exclusions']]
        specs=[(c,STANDARD_FOOTPRINT_RADIUS) for c in profile_standard_classes(cfg['class_profile'])]+[('red_cross',RED_CROSS_FOOTPRINT_RADIUS)]
        bounds=lambda key: tuple(cfg[key][k] for k in ['min_x','max_x','min_y','max_y'])
        rng=random.Random(seed)
        planned=plan_footprint_layout(rng,specs,occupied,bounds('search_region'),bounds('field'),
            cfg['spawn']['boundary_margin'],cfg['spawn']['pair_gap'],occupied_boxes=cfg['static_exclusion_boxes'])
        assert planned is not None
        targets=[]
        walls=[[(a,c),(b,c),(b,d),(a,d)] for a,b,c,d in cfg['static_exclusion_boxes']]
        for name,x,y in planned:
            yaw=rng.uniform(-math.pi,math.pi);size=.35 if name=='red_cross' else 1.
            polygon=rectangle_vertices(x,y,size,size,yaw)
            assert not any(polygons_overlap(polygon,w) for w in walls)
            targets.append(dict(target_class=name,x=x,y=y,yaw=yaw))
        preview=dict(scope='Offline production-policy preview with configured H/tree exclusions; actual Gazebo readback remains authoritative',targets=targets)
        (destination/'target_preview.json').write_text(json.dumps(preview,indent=2)+'\n')
        checks.append(dict(seed=seed,door_seed=door_seed,door_pattern=pattern,tree_subtrees_match_r64=True,
            trees=layout['trees'],door_widths=[d['clear_width'] for d in layout['doors']],
            target_preview_no_wall_overlap=True,lane_spacing=runtime['search']['lane_spacing']))
    a,b=[OUT/f'seed_{s}'/'scenario_inputs/runtime.yaml' for s in [32,34]]
    assert a.read_bytes()==b.read_bytes()
    result=dict(status='PASS',r64_source=R64_SOURCE,scene_checks=checks,runtime_identical_across_door_patterns=True,
        flight_algorithms_changed=False,door_geometry_only_for_private_gate=True)
    (OUT/'preflight.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':prepare()
