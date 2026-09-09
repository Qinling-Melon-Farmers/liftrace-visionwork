#!/usr/bin/env python3
"""Export an opt-in Gazebo fixture and separate evaluation geometry."""
from pathlib import Path
import argparse,copy,json,sys,xml.etree.ElementTree as ET
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r2026_scene import scene_layout


def export_scene(template_world, field_config, runtime_config, output, layout):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(template_world);world=tree.getroot().find('world')
    field=next(m for m in world.findall('model') if m.get('name')=='toudi2')
    for door in layout['doors']:
        link=next(l for l in field.findall('link') if l.get('name','').startswith(door['name']))
        pose=[float(x) for x in link.findtext('pose').split()];pose[1]=door['wall_y'];link.find('pose').text=' '.join(map(str,pose))
        for node in link.findall('./collision/geometry/box/size')+link.findall('./visual/geometry/box/size'):
            size=[float(x) for x in node.text.split()];size[1]=door['wall_length'];node.text=' '.join(map(str,size))
    models=[m for m in field.findall('model') if 'Tree' in m.get('name','')]
    if len(models)!=4:raise ValueError('Template must contain four tree/box groups')
    for model,obstacle in zip(models,layout['trees']):
        pose=[float(x) for x in model.findtext('pose').split()];pose[0]=obstacle['x'];pose[1]=obstacle['y'];pose[5]=obstacle['yaw'];model.find('pose').text=' '.join(map(str,pose))
    tree.write(output/'field.world',encoding='utf-8',xml_declaration=True)
    cfg=yaml.safe_load(Path(field_config).read_text());cfg['static_exclusions']=[dict(name='combined_tree_%d'%i,world_x=t['x'],world_y=t['y'],radius=.43) for i,t in enumerate(layout['trees'])]
    boxes=[]
    for link in field.findall('link'):
        if not link.get('name','').startswith('Wall'):continue
        p=[float(x) for x in link.findtext('pose').split()];size=[float(x) for x in link.findtext('collision/geometry/box/size').split()]
        boxes.append([p[0]-size[0]/2,p[0]+size[0]/2,p[1]-size[1]/2,p[1]+size[1]/2])
    cfg['static_exclusion_boxes']=boxes
    (output/'field_config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    runtime=yaml.safe_load(Path(runtime_config).read_text());gate=runtime.pop('post_delivery_gate')
    for actual in layout['doors']:
        item=next(x for x in gate['doors'] if x['name']==actual['name']);item['lateral_min']=actual['gap_min_y'];item['lateral_max']=actual['gap_max_y']
    # Evaluation-only file, loaded into the assertion node, never the manager.
    (output/'gate_geometry.yaml').write_text(yaml.safe_dump({'post_delivery_gate':gate},sort_keys=False))
    # Fixed wall-before/after centerline goals do not encode L/R ground truth.
    route=runtime['mission']['post_delivery_route']
    for i in range(3,8):route[i][1]=8.35
    runtime['mission']['post_delivery_route_revision']='r2026-experimental-neutral-wall-goals-v1'
    (output/'experimental_runtime.yaml').write_text(yaml.safe_dump(runtime,sort_keys=False))
    (output/'scene.json').write_text(json.dumps(layout,indent=2))
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--template-world',type=Path,required=True);p.add_argument('--field-config',type=Path,required=True);p.add_argument('--runtime-config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--scene-seed',type=int,required=True);p.add_argument('--door-seed',type=int);p.add_argument('--obstacle-seed',type=int);p.add_argument('--door-pattern',choices=['LL','LR','RL','RR']);a=p.parse_args()
    print(export_scene(a.template_world,a.field_config,a.runtime_config,a.output,scene_layout(a.scene_seed,a.door_seed,a.obstacle_seed,a.door_pattern)))
