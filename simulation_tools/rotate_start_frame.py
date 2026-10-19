#!/usr/bin/env python3
"""Bake a fixture into fixed start FLU: new X=old Y, new Y=-old X.

No live TF or body/sensor extrinsics are changed. Supported fixture containers
have identity poses, axis-aligned box links and posed nested tree models.
Reject unfamiliar geometry rather than silently producing inconsistent AABBs.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def xy(point):
    return [point[1], -point[0], *point[2:]]


def bounds(values):
    a,b,c,d=values
    return [c,d,-b,-a]


def bound_dict(data):
    data.update(zip(('min_x','max_x','min_y','max_y'),
                    bounds([data[k] for k in ('min_x','max_x','min_y','max_y')])))


def rotate_pose(element):
    pose=element.find('pose')
    if pose is None:pose=ET.SubElement(element,'pose');pose.text='0 0 0 0 0 0'
    if pose.attrib:raise ValueError('relative pose unsupported')
    p=[float(v) for v in pose.text.split()]
    p=xy(p);p[5]-=math.pi/2
    pose.text=' '.join(map(str,p))


def rotate_world(source,destination):
    tree=ET.parse(source);world=tree.getroot().find('world')
    for model in world.findall('model'):
        if model.get('name') not in ('toudi2','indoor_context'):
            rotate_pose(model);continue
        if any(abs(float(v))>1e-9 for v in model.findtext('pose','0 0 0 0 0 0').split()):
            raise ValueError('container must have identity pose')
        for nested in model.findall('model')+model.findall('include'):rotate_pose(nested)
        for link in model.findall('link'):
            p=[float(v) for v in link.findtext('pose','0 0 0 0 0 0').split()]
            if any(abs(v)>1e-9 for v in p[3:]):raise ValueError('link must be axis aligned')
            # All boxes centered in each link; preserve zero yaw for AABB consumers.
            for shape in link.findall('collision')+link.findall('visual'):
                if any(abs(float(v))>1e-9 for v in shape.findtext('pose','0 0 0 0 0 0').split()):
                    raise ValueError('shape pose unsupported')
                size=shape.find('geometry/box/size')
                if size is None:raise ValueError('non-box link geometry')
                x,y,z=map(float,size.text.split());size.text=f'{y} {x} {z}'
            link.find('pose').text=' '.join(map(str,xy(p)))
    for include in world.findall('include'):
        if include.find('pose') is not None:rotate_pose(include)
    tree.write(destination,encoding='utf-8',xml_declaration=True)


def rotate_runtime(data):
    data=copy.deepcopy(data);m=data['mission']
    for key in ('home_xy','landing_xy'):m[key]=xy(m[key])
    m['post_delivery_route']=[xy(p) for p in m['post_delivery_route']]
    m['post_delivery_yaw']-=math.pi/2
    m['post_delivery_route_revision']+='-start-flu'
    bound_dict(data['search'])
    return data


def rotate_gate(data):
    data=copy.deepcopy(data);gate=data['post_delivery_gate'];bound_dict(gate['low_height_region'])
    for door in gate['doors']:
        if door['axis']=='x':
            door['axis']='y';door['coordinate']=-door['coordinate']
            door['direction']={'positive':'negative','negative':'positive'}[door['direction']]
        elif door['axis']=='y':
            door['axis']='x'
            door['lateral_min'],door['lateral_max']=-door['lateral_max'],-door['lateral_min']
        else:raise ValueError('unsupported gate axis')
    return data


def export(source,destination,truth):
    source=Path(source);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    rotate_world(source/'field.world',destination/'field.world')
    dump=lambda name,data:(destination/name).write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    load=lambda name:yaml.safe_load((source/name).read_text())
    cfg=load('field_config.yaml')
    for key in ('field','search_region'):bound_dict(cfg[key])
    cfg['static_exclusion_boxes']=[bounds(v) for v in cfg['static_exclusion_boxes']]
    for tree in cfg['static_exclusions']:tree['world_x'],tree['world_y']=tree['world_y'],-tree['world_x']
    cfg['ignore_model_names']+=',presentation_overview_camera,presentation_follow_camera'
    targets=yaml.safe_load(Path(truth).read_text())['targets']
    cfg['spawn']['frozen_layout']=[{'class':t['class'],'x':t['y'],'y':-t['x'],'yaw':t['yaw']-math.pi/2} for t in targets]
    dump('field_config.yaml',cfg)
    for name in ('experimental_runtime.yaml','corridor_090_candidate.yaml'):dump(name,rotate_runtime(load(name)))
    for name in ('gate_geometry.yaml','gate_120_candidate.yaml'):dump(name,rotate_gate(load(name)))
    scene=json.loads((source/'scene.json').read_text())
    for tree in scene['trees']:tree['x'],tree['y']=tree['y'],-tree['x'];tree['yaw']-=math.pi/2
    scene['doors']=[dict(name=d['name'],axis='y',coordinate=-d['x'],lateral_min=d['gap_min_y'],lateral_max=d['gap_max_y'],clear_width=.8) for d in scene['doors']]
    scene.update(schema_version=3,frame='fixed start FLU; X inward, Y left, Z up',coordinate_transform='x_new=y_old; y_new=-x_old',qualification='Fixture conversion, SITL validation separate')
    (destination/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    return destination


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True);p.add_argument('--truth',required=True)
    a=p.parse_args();print(export(a.source,a.output,a.truth))
