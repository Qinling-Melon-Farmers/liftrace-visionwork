from pathlib import Path
import json,csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
h=Path(__file__).resolve().parents[4];d=h/'docs/verification/latest_six_20261005';batch=json.loads((h/'logs/latest_six_20261005_batch/matrix.json').read_text());run=Path(next(x['run'] for x in batch['results'] if x['seed']==38 and x['variant']=='snake2'));ct=136.178
rows=list(csv.DictReader((run/'truth_pose.csv').open()));a=np.array([[float(r[k]) for k in ['t','x','y','z']] for r in rows]);near=min(rows,key=lambda r:abs(float(r['t'])-ct));xyz=np.array([float(near[k]) for k in ['x','y','z']]);rot=Rotation.from_quat([float(near[k]) for k in ['qx','qy','qz','qw']]);box=np.array([[x,y,z-.02] for x in [-.275,.275] for y in [-.275,.275] for z in [-.2,.2]]);box=rot.apply(box)+xyz;hull=ConvexHull(box[:,:2]);poly=box[hull.vertices,:2]
es=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()];raw=[e['data'] for e in es if e['kind']=='bspline' and e['ros_sec']<ct][-1];k=raw['order'];knots=np.array(raw['knots']);pts=np.array([[p[x] for x in ['x','y','z']] for p in raw['pos_pts']]);spline=BSpline(knots,pts,k);curve=spline(np.linspace(knots[k],knots[-k-1],10000));j=int(np.argmin(np.linalg.norm(curve-xyz,axis=1)));gap=float(np.linalg.norm(curve[j]-xyz));tail=a[(a[:,0]>ct-5)&(a[:,0]<=ct)]
fig,ax=plt.subplots(1,2,figsize=(12,5));left=ax[0]
foot=json.loads((d/'38_snake2/tree_projection.json').read_text())['trees'][2]['footprint'];left.add_patch(plt.Polygon(foot,color='gray',alpha=.22,label='Tree full XY footprint (all heights)'))
left.plot(curve[:,0],curve[:,1],color='#168448',label='Published B-spline, spatial path');left.plot(tail[:,1],tail[:,2],color='#206fcd',lw=2,label='Actual body center before contact');left.add_patch(plt.Polygon(poly,fill=False,color='red',lw=2,label='Rotated 0.55 x 0.55 x 0.40 m guard'))
left.plot([xyz[0],curve[j,0]],[xyz[1],curve[j,1]],'k--',label=f'Distance to spline: {gap*100:.1f} cm');con=json.loads((run/'gazebo_contact_status.json').read_text())['events'][0]['details'][0]['positions'][0];left.scatter([con[0]],[con[1]],marker='*',s=150,color='red',label='Recorded contact');left.set(xlim=(3.4,5.9),ylim=(.6,2.8),xlabel='World X (m)',ylabel='World Y (m)',title='seed38 snake2: low coverage turn');left.set_aspect('equal');left.grid(alpha=.25);left.legend(fontsize=7,loc='upper right')
for name,color in [('mavros_pose','#e09e2b'),('lio_pose','#ad55b6')]:
 vals=np.array([[float(r[k]) for k in ['t','x','y','z']] for r in csv.DictReader((run/(name+'.csv')).open())]);ts=tail[(tail[:,0]<ct-.1),0];tr=np.array([np.interp(ts,a[:,0],a[:,i]) for i in range(1,4)]).T;est=np.array([np.interp(ts,vals[:,0],vals[:,i]) for i in range(1,4)]).T;diff=est-tr
 ax[1].plot(ts-ct,np.linalg.norm(diff[:,:2],axis=1)*100,color=color,label=name+' XY');ax[1].plot(ts-ct,np.abs(diff[:,2])*100,'--',color=color,label=name+' abs Z')
ax[1].set(xlabel='Seconds before contact',ylabel='Estimation error (cm)',title='Position estimate versus Gazebo truth');ax[1].legend();ax[1].grid(alpha=.25);fig.suptitle('Airborne guard/tree contact at ROS 136.178s; not a landing timeout');fig.tight_layout();fig.savefig(d/'collision_snake2_38.png',dpi=160);plt.close(fig)
(d/'collision_snake2_38_spline.json').write_text(json.dumps(dict(time=ct,truth_sample_time=float(near['t']),distance_to_spatial_bspline_m=gap,nearest_spatial_bspline_xyz=curve[j].tolist(),note='Spatial distance, not a time-synchronised tracking error. No recorded occupancy cloud was available to fully certify planner-map clearance.'),indent=2));print('Collision figure, spline distance',gap)