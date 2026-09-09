#!/usr/bin/env python3
"""Offline reports only: run after all ten matrix runs finish."""
from pathlib import Path
import argparse,json,sys,xml.etree.ElementTree as ET
import numpy as np,yaml
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.transforms import Affine2D
I=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--matrix-status',type=Path,required=True);p.add_argument('--pilot-run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
D=args.output;D.mkdir(parents=True,exist_ok=True)
state=json.loads(args.matrix_status.read_text());assert state['status']=='COMPLETE' and len(state['results'])==len(state['seeds'])
sys.path.insert(0,str(I/'top_level_scripts'))
from collect_run_summary import collect
pilot=args.pilot_run
runs=sorted([(int(yaml.safe_load((pilot/'random_field_truth.yaml').read_text())['seed']),pilot)]+[(x['seed'],Path(x['run_dir'])) for x in state['results']])
colors={'tent':'#a76330','pillbox':'#2774b0','bridge':'#9764bb','panzer':'#4c893e','red_cross':'#d52b27'}
def values(e,tag,default='0 0 0 0 0 0'):return [float(x) for x in e.findtext(tag,default).split()]
def scene(ax,run,labels=True):
 world=ET.parse(run/'scenario_inputs/field.world').getroot().find('world');field=next(x for x in world.findall('model') if x.get('name')=='toudi2')
 for link in field.findall('link'):
  if not link.get('name','').startswith('Wall'):continue
  pose=values(link,'pose');size=values(link,'collision/geometry/box/size','0 0 0');ax.add_patch(Rectangle((pose[0]-size[0]/2,pose[1]-size[1]/2),size[0],size[1],facecolor='#7c858d',edgecolor='#4b535a'))
 for model in field.findall('model'):
  if 'Tree' not in model.get('name',''):continue
  p=values(model,'pose');ax.add_patch(Rectangle((p[0]-.3,p[1]-.3),.6,.6,color='#a9845b'));ax.add_patch(Circle(p[:2],.43,color='#3d7c45',alpha=.55))
 for x,y,name in [(0,0,'Start H'),(4.2,8.5,'Landing H')]:
  ax.add_patch(Circle((x,y),.5,fill=False,color='black',lw=1.5));ax.text(x,y,'H',ha='center',va='center',fontsize=9)
 truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
 for t in truth['targets']:
  x,y=t['world_x'],t['world_y'];size=.35 if t['class']=='red_cross' else 1.;patch=Rectangle((x-size/2,y-size/2),size,size,edgecolor=colors[t['class']],facecolor=colors[t['class']],alpha=.20);patch.set_transform(Affine2D().rotate_around(x,y,t['yaw'])+ax.transData);ax.add_patch(patch)
  ax.scatter([x],[y],s=12,c=colors[t['class']])
  if labels:ax.annotate(t['class'],(x,y),xytext=(3,4),textcoords='offset points',fontsize=7,color=colors[t['class']])
 ax.set(xlim=(-5.15,5.15),ylim=(-.85,9.45),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)
 return truth
invalid={x['seed'] for x in json.loads((D/'scene_legality.json').read_text()) if x['target_wall_overlap']} if (D/'scene_legality.json').exists() else set()
rows=(len(runs)+4)//4
summary=[];atlas,atlasaxes=plt.subplots(rows,4,figsize=(15,4.34*rows),squeeze=False)
for (seed,run),atlasax in zip(runs,atlasaxes.flat):
 s=collect(run);g=json.loads((run/'gate_status.json').read_text());a=np.loadtxt(run/'truth_pose.csv',delimiter=',',skiprows=1,ndmin=2);c=np.loadtxt(run/'mavros_setpoint.csv',delimiter=',',skiprows=1,ndmin=2)
 fig,axes=plt.subplots(1,3,figsize=(16,5.7));scene(axes[0],run);axes[0].set_title('Actual layout');scene(axes[1],run,False);axes[1].plot(a[:,1],a[:,2],lw=.9,c='#145d96');axes[1].scatter(a[-1,1],a[-1,2],s=20,c='#e44831');axes[1].set_title('Actual aircraft path')
 axes[2].plot(a[:,0],a[:,3]+.22,label='Actual FC AGL',lw=1);axes[2].plot(c[:,0],c[:,3]+.22,label='Command in ground reference',lw=.6,alpha=.7)
 for e in [json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines()]:
  if e['kind']=='release':axes[2].axvline(e['ros_sec'],color=colors.get(e['data'].get('target_class'),'gray'),ls=':',alpha=.8)
 axes[2].set(xlabel='ROS simulation time (s)',ylabel='Height (m)',title='Height and release times');axes[2].grid(alpha=.2);axes[2].legend(fontsize=8)
 fig.suptitle('Seed %d | %s | releases %d/3 | recovery %d/3'%(seed,s['status'],g['metrics']['release_commit_count'],g['metrics']['recovery_success_count']));fig.tight_layout();name='seed_%02d.png'%seed;fig.savefig(D/name,dpi=150);plt.close(fig)
 scene(atlasax,run,False);atlasax.set_title('Seed %d: %s%s'%(seed,s['status'],' *' if seed in invalid else ''),fontsize=10)
 summary.append({'seed':seed,'status':s['status'],'run_dir':str(run),'source':s['source'],'releases':g['metrics']['release_commit_count'],'recoveries':g['metrics']['recovery_success_count'],'route_points':(g['metrics']['post_delivery_return_success_count'] if g['metrics']['post_delivery_decision_indices'] else 0),'gate_reason':g['reason'],'mission_ros_sec':g['metrics']['mission_ros_sec'],'collisions':json.loads((run/'gazebo_contact_status.json').read_text())['actual_collision_count'],'image':name,'targets':s['layout']['targets'],'bag_files':s['bag_files']})
atlasaxes.flat[-1].axis('off');atlasaxes.flat[-1].text(.05,.8,'Brown: tent\nBlue: pillbox\nPurple: bridge\nGreen square: panzer\nRed: red cross\nGreen circle: tree/box\nGray: physical wall\nH: landing/start pad\n* Recorded target intersects wall',fontsize=12,va='top');atlas.tight_layout();atlas.savefig(D/'all_seed_layouts.png',dpi=150);plt.close(atlas)
(D/'summary.json').write_text(json.dumps(summary,indent=2));(D/'matrix_status.json').write_text(json.dumps(state,indent=2))
cards=''.join('<article><h2>Seed %d — %s</h2><a href="%s"><img src="%s" loading="lazy"></a></article>'%(x['seed'],x['status'],x['image'],x['image']) for x in summary)
(D/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>R64: all eleven flights</title><style>body{font:16px sans-serif;max-width:1500px;margin:24px auto;background:#f1f3f5;color:#26313b}article{background:white;padding:18px;margin:24px 0}img{width:100%}h1,h2{font-weight:600}</style><h1>R64 seed1–11: actual layouts, paths and heights</h1><p>Truth is used for evaluation only. Click an image for full resolution. Dotted height lines are release events.</p>'+cards)
print(json.dumps([{k:v for k,v in x.items() if k not in ['targets','run_dir','source']} for x in summary],indent=2))
