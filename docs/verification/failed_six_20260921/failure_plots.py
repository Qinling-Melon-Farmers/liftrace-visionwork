from pathlib import Path
import csv,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
R=Path(__file__).resolve().parents[3];D=R/'docs/verification/failed_six_20260921'
state=json.loads((R/'logs/failed_six_20260921_batch/matrix.json').read_text())
def csv4(p):
 with p.open() as f:return np.array([[float(v[k]) for k in ['t','x','y','z']] for v in csv.DictReader(f)])
summ=[];fig,axes=plt.subplots(2,2,figsize=(14,10))
for axrow,seed in zip(axes,[38,40]):
 run=Path(next(x['run'] for x in state['results'] if x['seed']==seed));c=json.loads((run/'gazebo_contact_status.json').read_text())['events'][0];t=c['ros_stamp']
 for name,label in [('truth_pose','truth'),('lio_pose','LIO'),('mavros_pose','MAVROS'),('mavros_setpoint','setpoint')]:
  a=csv4(run/(name+'.csv'));keep=(a[:,0]>=t-15)&(a[:,0]<=t+1);a=a[keep]
  axrow[0].plot(a[:,0]-t,a[:,2],label=label);axrow[1].plot(a[:,0]-t,a[:,3],label=label)
 for ax in axrow:ax.axvline(0,color='red',ls=':');ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('Seconds relative to guard contact')
 axrow[0].axhline(-4.481424619063237,ls='--',color='purple',label='planner hard Y');axrow[0].axhline(-4.451424619063237,ls='--',color='gray',label='visual admission Y');axrow[0].legend(fontsize=8)
 axrow[0].set(title=f'Seed {seed}: Y near wall',ylabel='Y (m)');axrow[1].set(title=f'Seed {seed}: Z near contact',ylabel='Z in recorded coordinate (m)')
 hi=json.loads((run/'high_view_full_events.jsonl').read_text().splitlines()[-1])['status'];summ.append(dict(seed=seed,contact_s=t,boundary_rejections=hi.get('boundary_rejections'),contact=c))
fig.tight_layout();fig.savefig(D/'wall11_contact_timeline.png',dpi=150);plt.close(fig)
run=Path(next(x['run'] for x in state['results'] if x['seed']==34));pose=csv4(run/'lio_pose.csv')
fig,axes=plt.subplots(1,3,figsize=(16,6));stats=[]
for ax,p in zip(axes,sorted(run.glob('local_map_failure_*.json'))):
 snap=json.loads(p.read_text());t=snap['ros_sec'];pos=pose[np.argmin(abs(pose[:,0]-t)),1:]
 raw=np.array(snap['clouds']['static_map']['points']);inf=np.array(snap['clouds']['inflated_map']['points'])
 raw=raw[(raw[:,2]>.4)&(raw[:,2]<3.13)];cut=inf[np.abs(inf[:,2]-pos[2])<.05]
 ax.scatter(cut[:,0],cut[:,1],s=4,color='orange',alpha=.35,label='inflated slice at current Z')
 ax.scatter(raw[:,0],raw[:,1],s=3,color='green',alpha=.5,label='static obstacles Z>0.4')
 path=pose[(pose[:,0]>=11)&(pose[:,0]<=t)];ax.plot(path[:,1],path[:,2],'b-',lw=1,label='LIO path');ax.plot(*pos[:2],'bo');ax.plot(*snap['goal'][:2],'rx',ms=9,label='requested goal')
 ax.set(xlim=(-.1,2),ylim=(1.6,4.15),title=f'Seed34 snapshot t={t:.3f}s',xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=6)
 outside=abs(pos[1]-snap['goal'][1])>2
 if outside:ax.text(.04,.97,'Goal-centred crop excludes current area.\nBlank here is NOT evidence of free space.',transform=ax.transAxes,va='top',fontsize=8,bbox=dict(facecolor='white',alpha=.9))
 nearest=float(np.linalg.norm(inf-pos,axis=1).min());stats.append(dict(snapshot=p.name,t=t,pose=pos.tolist(),nearest_inflated_point_m=None if outside else nearest,current_area_outside_goal_crop=bool(outside),scope='point-centre distance, not exact SDF; inflated cloud header stamp is zero'))
fig.tight_layout();fig.savefig(D/'seed34_map_snapshots.png',dpi=150);plt.close(fig)
(D/'failure_geometry.json').write_text(json.dumps(dict(contacts=summ,seed34_snapshots=stats),indent=2)+'\n');print(json.dumps(dict(contacts=[{'seed':s['seed'],'boundary_rejections':s['boundary_rejections']} for s in summ],snapshots=stats)))
