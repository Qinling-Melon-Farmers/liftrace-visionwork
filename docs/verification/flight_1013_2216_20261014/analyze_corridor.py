from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ap=argparse.ArgumentParser();ap.add_argument('--run-dir',type=Path,required=True);ap.add_argument('--out-dir',type=Path,required=True);args=ap.parse_args()
n=args.run_dir/'numeric';o=args.out_dir;o.mkdir(parents=True,exist_ok=True)
def read(k): return json.loads((n/(k+'.json')).read_text())
def cut(k,a=173.68,b=201): return [r for r in read(k) if a<=r['t']<=b]
def near(k,t): return min(read(k),key=lambda r:abs(r['t']-t))
def xyz(p): return [p[k] for k in ('x','y','z')]
def series(k,kind='pose'):
 rows=cut(k)
 return np.array([r['t'] for r in rows]),np.array([xyz(r['m'][kind]['position' if kind=='pose' else 'linear']) for r in rows])
t,p=series('navigation__local_pose');ts,s=series('navigation__setpoint_mission');tv,v=series('mavros__local_position__velocity_local','twist')
events=[(179.672,'P3'),(182.508,'P4: planning fails'),(194.527,'Timeout / ABORT'),(197.653,'ABORT trajectory'),(200.961,'POSCTL')]
fig,ax=plt.subplots(4,1,figsize=(13,10),sharex=True)
for i,label in [(0,'Y (m): corridor direction'),(1,'X (m)'),(2,'FC AGL (m)')]:
 j=[1,0,2][i];offset=.21096973147243261 if j==2 else 0
 ax[i].plot(t,p[:,j]+offset,label='Recorded FC pose');ax[i].plot(ts,s[:,j]+offset,label='Final position command',alpha=.85);ax[i].set_ylabel(label)
ax[0].legend(loc='lower left');ax[3].plot(tv,np.linalg.norm(v[:,:2],axis=1),label='Horizontal speed');ax[3].set_ylabel('Speed (m/s)');ax[3].set_xlabel('Seconds since bag start (22:16:33.035 CST)')
for a in ax:
 a.axvspan(182.538,194.527,color='orange',alpha=.12);a.grid(alpha=.25)
 for x,label in events:a.axvline(x,color='gray',ls=':',lw=.8)
for x,label in events:ax[0].text(x,2.0,label,rotation=90,va='top',fontsize=8)
ax[0].set_ylim(.65,2.15);fig.suptitle('Corridor: no P4 trajectory, fixed hold for 12 s; ABORT goal also adjusted')
fig.tight_layout();fig.savefig(o/'corridor_motion.png',dpi=150);plt.close(fig)

clouds=np.load(n/'corridor_clouds.npz');cm=json.loads((n/'corridor_clouds.json').read_text())
fig,axs=plt.subplots(2,2,figsize=(12,10));a=axs[0,0];route=np.array([[6.516047,1.902763],[7.716047,1.902763],[7.716047,1.002763],[7.716047,-3.447237]])
a.plot(route[:,0],route[:,1],'o--',label='Four input points');a.plot(p[:,0],p[:,1],label='Actual FC path');a.scatter([7.716047],[1.102763],marker='x',s=70,color='red',label='Adjusted P3')
for i,q in enumerate(route):a.annotate('P'+str(i+1),q,xytext=(8,0),textcoords='offset points')
for y in [.002763,-1.397237]:a.axhline(y,ls=':',color='gray')
a.text(5.6,-2.4,'Wall Y from configuration;\nnot surveyed door openings.',fontsize=8);a.set_title('P3 to P4 skips paired door guide points');a.legend(fontsize=8);a.set_xlim(5.5,9.5);a.set_ylim(-3.8,2.3)
for a,m in zip([axs[0,1],axs[1,0],axs[1,1]],cm[:3]):
 c=clouds[m['key']];q=np.array(xyz(near('navigation__local_pose',m['t'])['m']['pose']['position']));sel=np.abs(c[:,2]-q[2])<=.05;c=c[sel]
 for x,y,z in c:
  if 7.15<x<8.45 and .65<y<2.15:a.add_patch(Rectangle((x-.05,y-.05),.1,.1,color='gray',alpha=.4,lw=0))
 hist=(t<=m['t'])&(t>=179.67);a.plot(p[hist,0],p[hist,1],color='tab:blue',lw=1);a.scatter(*q[:2],s=60,color='tab:blue',label='FC at snapshot');a.scatter(7.716047,1.102763,color='red',marker='x',s=80,label='Adjusted P3');a.scatter(7.75982,1.080442,edgecolor='orange',facecolor='none',s=170,label='P4 hold point')
 a.set_title(f"Inflated map t={m['t']:.2f}s, slice near Z={q[2]:.2f}m");a.set_xlim(7.15,8.45);a.set_ylim(.65,2.15);a.legend(fontsize=8)
for a in axs.flat:a.set_aspect('equal');a.grid(alpha=.2);a.set_xlabel('Mission X (m)');a.set_ylabel('Mission Y (m)')
fig.suptitle('Published inflated occupancy; gray cells do not distinguish real obstacles from virtual columns')
fig.tight_layout();fig.savefig(o/'corridor_map.png',dpi=150);plt.close(fig)

stats={'time_origin':'2026-10-13 22:16:33.035 CST','start_epoch':1791900993.0352786,'flight_frame':'camera_init','field_origin_fc_xy':[.016047250479459763,.002762551885098219],'events':events,'input_corridor_points_field_agl':[[6.5,1.9,.9],[7.7,1.9,.9],[7.7,1.0,.9],[7.7,-3.45,.9]],'seq16_attempts':[]}
for r in read('planning__goal_status'):
 m=r['m']
 if m['goal_seq']==16 and m['reason']=='new_trajectory_attempt_failed':stats['seq16_attempts'].append({'t':r['t'],'attempt':m['planning_attempt']})
for a,b,name in [(182.54,194.52,'p4_wait'),(194.575,200.961,'abort_to_takeover')]:
 ix=(t>=a)&(t<=b);is_=(ts>=a)&(ts<=b);z=p[ix];st=s[is_];stats[name]={'pose_min':z.min(0).tolist(),'pose_max':z.max(0).tolist(),'final_command_min':st.min(0).tolist(),'final_command_max':st.max(0).tolist(),'unique_commands':len(np.unique(st,axis=0)),'min_y_t':float(t[ix][np.argmin(z[:,1])]),'max_y_t':float(t[ix][np.argmax(z[:,1])])}
for name in ['navigation__setpoint_mission','mavros__setpoint_position__local','laserMapping__realtime']:
 dt=np.diff([r['t'] for r in cut(name,173.68,200.96)]);stats[name+'_receive_intervals_sec']={k:float(v) for k,v in zip(['median','p95','max'],np.percentile(dt,[50,95,100]))}
lio=[]
for r in cut('laserMapping__realtime',173.68,200.96):
 d={x['key']:float(x['value']) for x in r['m']['status'][0]['values']};lio.append([d['output_age_sec'],d['iteration_wall_sec'],d['lidar_queue']])
stats['lio_columns']=['output_age_sec','iteration_wall_sec','lidar_queue'];stats['lio_median_p95_max']=np.percentile(lio,[50,95,100],axis=0).tolist()
(o/'metrics.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(stats,ensure_ascii=False,indent=2))