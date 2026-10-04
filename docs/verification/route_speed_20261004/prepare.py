"""Generate fixed-four-tree, clear-10m scenes and all paired route variants offline."""
from pathlib import Path
import copy,json,math,random,sys,xml.etree.ElementTree as ET
import yaml
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'patrol_uav_ws-patrol_planner/src/uav_mission/src'))
from uav_mission.survey_routes import survey_route
from uav_mission.motion_optimization import MotionOptimization,optimize_post_route
from uav_mission.random_field_policy import Footprint,plan_footprint_layout,frozen_footprint_layout,STANDARD_FOOTPRINT_RADIUS,RED_CROSS_FOOTPRINT_RADIUS
TREES=[(2.,1.8),(2.7,-1.8),(5.2,1.5),(5.7,-1.7)]
BOX=[-.5,9.5,-5.,5.];TARGET=[-.5,7.95,-5.,5.]
def bounds(v):return dict(zip(('min_x','max_x','min_y','max_y'),v))
def dump(p,v):p.write_text(yaml.safe_dump(v,sort_keys=False))
def shape(link,p,size):
 link.find('pose').text=' '.join(map(str,[*p,0,0,0]))
 for n in link.findall('./collision/geometry/box/size')+link.findall('./visual/geometry/box/size'):n.text=' '.join(map(str,size))
def main():
 manifest=[]
 for seed in (31,38):
  base=D/f'templates/seed_{seed}';world=ET.parse(base/'field.world');w=world.getroot().find('world');m=next(m for m in w.findall('model') if m.get('name')=='toudi2')
  # Expand clear interior; thin walls live outside the advertised clear 10m.
  geometry={'Wall_1':([-.525,0,2],[.05,10.1,4]),'Wall_9':([9.525,0,2],[.05,10.1,4]),'Wall_11':([4.5,-5.025,2],[10,.05,4]),'Wall_12':([4.5,5.025,2],[10,.05,4]),'Wall_13':([7.975,-.75,.75],[.05,8.5,1.5])}
  gaps={}
  for l in m.findall('link'):
   name=l.get('name')
   if name in geometry:shape(l,*geometry[name])
   elif name.startswith(('Wall_20','Wall_22')):
    pos=list(map(float,l.findtext('pose').split()));left=pos[0]<8.3
    center=8.35 if left else 9.15
    shape(l,[center,pos[1],.75],[.7,.05,1.5]);gaps['Wall_20' if name.startswith('Wall_20') else 'Wall_22']=[8.7,9.5] if left else [8.,8.8]
  models=[m for m in m.findall('model') if 'Tree' in m.get('name','')]
  assert len(models)==4
  for model,(x,y) in zip(models,TREES):
   pos=list(map(float,model.findtext('pose').split()));pos[0:2]=[x,y];pos[5]=-math.pi/2;model.find('pose').text=' '.join(map(str,pos))
  for inc in w.findall('include'):
   if inc.findtext('name')=='landing_h_clone':inc.find('pose').text='8.75 -4.2 0 0 0 0'
  boxes=[]
  for l in m.findall('link'):
   if not l.get('name','').startswith('Wall'):continue
   x,y,*_=map(float,l.findtext('pose').split());sx,sy,_=map(float,l.findtext('collision/geometry/box/size').split());boxes.append([x-sx/2,x+sx/2,y-sy/2,y+sy/2])
  cfg=yaml.safe_load((base/'field_config.yaml').read_text());cfg['field']=bounds(BOX);cfg['search_region']=bounds(TARGET);cfg['static_exclusion_boxes']=boxes
  cfg['static_exclusions']=[dict(name=f'combined_tree_{i}',world_x=x,world_y=y,radius=.43) for i,(x,y) in enumerate(TREES)]
  occupied=[Footprint(f'tree{i}',x,y,.43) for i,(x,y) in enumerate(TREES)]+[Footprint('H',0,0,.4),Footprint('Hend',8.75,-4.2,.4)]
  specs=[(n,STANDARD_FOOTPRINT_RADIUS) for n in ('tent','pillbox','bridge','panzer')]+[('red_cross',RED_CROSS_FOOTPRINT_RADIUS)]
  preserved=True
  try:frozen_footprint_layout(cfg['spawn']['frozen_layout'],specs,occupied,TARGET,BOX,.02,.05,occupied_boxes=boxes)
  except ValueError:
   preserved=False;rng=random.Random(seed);layout=plan_footprint_layout(rng,specs,occupied,TARGET,BOX,.02,.05,occupied_boxes=boxes)
   assert layout is not None
   cfg['spawn']['frozen_layout']=[dict(**{'class':name},x=x,y=y,yaw=rng.uniform(-math.pi,math.pi)) for name,x,y in layout]
  frozen_footprint_layout(cfg['spawn']['frozen_layout'],specs,occupied,TARGET,BOX,.02,.05,occupied_boxes=boxes)
  for variant in ('rectangle_baseline','rectangle','snake2','snake3'):
   dest=D/f'{variant}_seed{seed}';dest.mkdir(exist_ok=True)
   world.write(dest/'field.world',encoding='utf-8',xml_declaration=True);dump(dest/'field_config.yaml',cfg)
   for f in ('presentation.yaml',): (dest/f).write_bytes((base/f).read_bytes())
   runtime=yaml.safe_load((base/'fast_runtime.yaml').read_text());mission=runtime['mission']
   original=[[6.9,4.25,1.18],[6.9,4.25,.68],[8.75,4.25,.68],[8.75,2.3,.68],[8.75,.9,.68],[8.75,0,.68],[8.75,-.9,.68],[8.75,-2.3,.68],[8.75,-4.2,.68]]
   opt=variant!='rectangle_baseline';options=MotionOptimization(enabled=opt)
   route,meta=optimize_post_route(original,options,8,1,[-1.6,1.6])
   mission.update(post_delivery_route=route,post_delivery_route_revision='route-speed-'+variant,landing_xy=[8.75,-4.2])
   if opt:
    runtime['motion_optimization']=dict(vars(options));runtime['motion_optimization_metadata']=meta
    runtime['corridor_speed_schedule']['entry_waypoints']=1
    mission['post_delivery_parameter_stages']=[dict(after_completed_waypoints=0,parameters={'/external_planner_max_command_z':2.78,'/navigation/planner_bridge/execution/arrival_position_tolerance':.12,'/navigation/planner_bridge/execution/arrival_dwell':.8}),dict(after_completed_waypoints=1,parameters={'/external_planner_max_command_z':.78,'/fast_planner_node/fsm/goal_adjustment_radius':.1})]
   runtime['search'].update(bounds([.4,7.2,-4.25,4.25]));dump(dest/'fast_runtime.yaml',runtime)
   pattern='rectangle' if variant=='rectangle_baseline' else variant
   lanes=[1.8,3.7,5.6] if pattern=='snake3' else [1.8,5.6]
   survey=survey_route(pattern,lanes,-3.7,3.9)
   frame=yaml.safe_load((base/'frame_overrides.yaml').read_text());frame['/navigation_vcl06_assertion/field']=bounds(BOX)
   for k,v in bounds(TARGET).items():frame['/fast_planner_node/sdf_map/horizontal_avoidance/'+k]=v
   repair=yaml.safe_load((base/'repair_overrides.yaml').read_text())
   repair.update({'/navigation/mission_manager/high_view_full/grid/bounds':TARGET,'/navigation/mission_manager/high_view_full/boundary_policy/bounds':TARGET,'/navigation/mission_manager/high_view_probe/config/survey_xy':survey,'/navigation_vcl06_assertion/search_envelope_region':bounds(TARGET),'/experiment/search_inner_bounds':TARGET})
   for k,v in bounds([-.1,7.55,-4.55,4.55]).items():repair['/fast_planner_node/sdf_map/search_region/'+k]=v
   dump(dest/'frame_overrides.yaml',frame);dump(dest/'repair_overrides.yaml',repair)
   ov={'/fast_planner_node/search/line_deviation_weight':0.,'/navigation/planner_bridge/motion_optimization':dict(vars(options)),'/navigation/mission_manager/motion_optimization':dict(vars(options))}
   if opt:ov.update({'/navigation/planner_bridge/target/recovery_height':.6800000071525574,'/uav_vision/recovery_height':.6800000071525574})
   dump(dest/'motion_overrides.yaml',ov)
   gate=yaml.safe_load((base/'fast_gate.yaml').read_text());g=gate['post_delivery_gate'];g['low_height_region'].update(bounds([8,9.5,-5,5]));g['landing_observation_region'].update(bounds([8,9.5,-5,-2.1]))
   g['allow_shared_route_indices']=opt
   for door in g['doors']:
    if door['name']=='corridor_entry':door.update(coordinate=7.975,lateral_min=3.5,lateral_max=5.,route_indices=[2 if opt else 3])
    else:
     door.update(lateral_min=gaps[door['name']][0],lateral_max=gaps[door['name']][1],route_indices=[3 if opt else (5 if door['name']=='Wall_20' else 8)])
   dump(dest/'fast_gate.yaml',gate)
   manifest.append(dict(seed=seed,variant=variant,scene=str(dest.relative_to(R)),camera_agl=2.6,fc_agl=2.76,trees=TREES,historical_target_positions_preserved=preserved,targets=cfg['spawn']['frozen_layout'],survey=survey,post_route=route))
 (D/'scenes.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print('Eight fixed scenes generated; historical targets retained:',[(s['seed'],s['historical_target_positions_preserved']) for s in manifest if s['variant']=='rectangle'])
if __name__=='__main__':main()
