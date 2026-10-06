#!/usr/bin/env python3
"""Build explicit 4x4 test fixtures; never edit the measured board settings."""
import argparse,copy,json,sys,xml.etree.ElementTree as E
from pathlib import Path
import yaml
from trial_config import generate,TRIAL_FOLDERS

def main():
    ap=argparse.ArgumentParser();ap.add_argument('trial',choices=[name for name in TRIAL_FOLDERS if name!='high_speed_capture']);ap.add_argument('directory');args=ap.parse_args()
    package=Path(__file__).resolve().parents[1];root=package.parents[3];out=Path(args.directory);out.mkdir(parents=True,exist_ok=True)
    settings=yaml.safe_load((root/'deployment/board_trials_4x4'/TRIAL_FOLDERS[args.trial]/'settings.yaml').read_text())
    settings['camera_info_topic']='/downward_camera/camera_info'
    # Unattended SITL has no pilot for the hardware POSCTL terminal handoff.
    if args.trial in ('landing','corridor_landing','full_mission'):
        settings['landing_handoff_mode']='AUTO.LAND'
    if args.trial in ('corridor_landing','full_mission'):
        settings['corridor_waypoints']=[dict(x=x,y=1.1,agl=1.0) for x in (.65,1.20,1.8,2.20,2.8)]
        settings['landing_xy']=[3.1,1.1]
    rig=yaml.safe_load((package/'config/known_rig.yaml').read_text());ref=generate(root,out,settings,(0.,0.,0.),rig)
    (out/'settings.yaml').write_text(yaml.safe_dump(settings,sort_keys=False))
    sdf=E.Element('sdf',version='1.6');world=E.SubElement(sdf,'world',name='default')
    source=E.parse(root/'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world').getroot().find('world')
    for el in source:
        if el.tag not in ('model','include','state','gui'):world.append(copy.deepcopy(el))
    def include(uri,name,x,y,z=0,yaw=0):
        el=E.SubElement(world,'include');E.SubElement(el,'uri').text='model://'+uri;E.SubElement(el,'name').text=name
        E.SubElement(el,'pose').text=f'{x} {y} {z} 0 0 {yaw}'
    include('sun','sun',0,0);include('ground_plane','ground_plane',0,0)
    include('landing_h','landing_h',0,0)
    obstacles=[]
    def box(name,x,y,z,l,w,h):
        model=E.SubElement(world,'model',name=name);E.SubElement(model,'static').text='true'
        link=E.SubElement(model,'link',name='link');E.SubElement(link,'pose').text=f'{x} {y} {z} 0 0 0'
        for kind in ('collision','visual'):
            el=E.SubElement(link,kind,name=kind);geom=E.SubElement(el,'geometry');shape=E.SubElement(geom,'box');E.SubElement(shape,'size').text=f'{l} {w} {h}'
            if kind=='visual':
                m=E.SubElement(el,'material');E.SubElement(m,'ambient').text='.65 .69 .73 1';E.SubElement(m,'diffuse').text='.65 .69 .73 1'
        obstacles.append(dict(name=name,xy=[x,y],size=[l,w,h]))
    # 4x4 work area plus a 0.7 m launch apron behind the edge midpoint.
    box('wall_left',2,2.10,2,4.2,.2,4);box('wall_right',2,-2.10,2,4.2,.2,4)
    box('wall_far',4.10,0,2,.2,4.2,4);box('wall_rear',-.70,0,2,.2,4.2,4)
    cases={
      'visual_interrupt':[('red_cross',1.9,0)],
      'high_view':[('bridge',1.,-1.),('panzer',3.,1.),('red_cross',3.,-1.)],
      'landing':[], 'corridor_landing':[],
      'low_multi':[('red_cross',.95,0),('panzer',2.7,0)],
      'high_priority':[('bridge',1.,-1.),('panzer',3.,1.),('red_cross',3.,-1.)],
      'memory_only':[('panzer',3.,1.),('red_cross',3.,-1.)],
      'full_mission':[('bridge',1.,-.9),('panzer',2.8,-.9),('red_cross',2.,.3)],
    }
    models={'bridge':'qiaoliang','panzer':'zhuangjiache','red_cross':'red_cross'}
    targets=[]
    for name,x,y in cases[args.trial]:
        include(models[name],'target_'+name,x,y,yaw=0)
        targets.append(dict(class_name=name,x=x,y=y))
    if args.trial in ('high_view','high_priority','memory_only'):
        # Existing competition tree mesh and physical pedestal, no fake occupancy.
        tree=copy.deepcopy(source.find("model[@name='toudi2']/model"))
        if tree is None:raise RuntimeError('Competition tree model missing')
        tree.set('name','trial_tree');tree.find('pose').text='2 0 0 0 0 0';world.append(tree)
        obstacles.append(dict(name='trial_tree',xy=[2,0],source='competition tree mesh'))
    if args.trial in ('corridor_landing','full_mission'):
        for i,x in enumerate((1.5,2.5)):
            box(f'gate_{i}_lower',x,.35,.75,.12,.7,1.5)
            box(f'gate_{i}_upper',x,1.80,.75,.12,.6,1.5)
        include('landing_h','landing_h_clone',3.1,1.1)
    elif args.trial=='landing':include('landing_h','landing_h_clone',2.,0)
    E.indent(sdf);E.ElementTree(sdf).write(out/'scene.world',encoding='utf-8',xml_declaration=True)
    (out/'scene.json').write_text(json.dumps(dict(trial=args.trial,targets=targets,obstacles=obstacles,work_area=[0,4,-2,2],launch_apron_min_x=-.6,gate_width=.8 if args.trial in ('corridor_landing','full_mission') else None),indent=2))
    print(json.dumps(dict(trial=args.trial,mode=settings['mode'],low_z=ref['low_z'],directory=str(out))))
if __name__=='__main__':main()
