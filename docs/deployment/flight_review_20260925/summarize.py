import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
d=json.loads((ROOT/'logs/flight_review_20260925/extracted.json').read_text())
m=d['motion']; t=np.array([r['t'] for r in m]); z=np.array([r['z'] for r in m])
velocity=Rotation.from_quat([[r['orientation'][k] for k in ('x','y','z','w')] for r in m]).apply([[r[k] for k in ('vx','vy','vz')] for r in m])
pose_velocity=np.column_stack([(np.interp(t+.25,t,[r[k] for r in m])-np.interp(t-.25,t,[r[k] for r in m]))/.5 for k in ('x','y','z')])
v=np.linalg.norm(pose_velocity[:,:2],axis=1);vz=pose_velocity[:,2]
for i,r in enumerate(m): r['vxy']=float(v[i]);r['vz']=float(vz[i])
z0=float(np.median(z[t<7]))
phases=[('Ground',0,8.797),('Takeoff / first point',8.797,19.624),('Forward search',19.624,22.725),('Approach',22.725,26.618),('Align / release',26.618,41.758),('Return',41.758,49.149),('Landing command',49.149,100.799),('Pilot POSCTL',100.799,105.692)]
summary={'bag_start_unix':d['start'],'bag_seconds':float(t[-1]),'initial_fc_z':z0,'height_note':'local FC z; estimated AGL = z - initial FC z + 0.22m (known rig, not independent range measurement)','speed_note':'0.5s central position difference in odom frame, based on bag receipt times; not body-frame twist','twist_frame':m[0]['child'],'rotated_twist_vs_pose_median_abs_error':np.median(abs(velocity-pose_velocity),axis=0).tolist(),'phases':[],'events':[]}
for name,a,b in phases:
    mask=(t>=a)&(t<b)
    row=dict(phase=name,start=a,end=b,z_min=float(z[mask].min()),z_max=float(z[mask].max()),z_median=float(np.median(z[mask])),vxy_median=float(np.median(v[mask])),vxy_p95=float(np.percentile(v[mask],95)),vxy_max=float(v[mask].max()),vz_min=float(vz[mask].min()),vz_max=float(vz[mask].max()))
    summary['phases'].append(row)
for label,at in [('cross first mapped',22.378),('cross confirmed',22.669),('interrupt',22.725),('alignment accepted',26.618),('strict context',39.199),('actuator unavailable',41.736),('return',41.758),('panzer mapped rejection',48.561),('land',49.149),('pilot POSCTL',100.799),('landed',105.692)]:
    r=m[int(np.argmin(abs(t-at)))]; summary['events'].append(dict(label=label,t=at,**{k:r[k] for k in ['x','y','z','vxy','vz']}))
summary['panzer_raw']=[{k:r[k] for k in ['t','source','class_confidence','center_px','header']} for r in d['detections'] if r['topic']=='/uav_vision/detections' and r['class_name']=='panzer'][:3]
summary['panzer_rejected']=[r for r in d['detections'] if r['topic']=='/uav_vision/detections_mapped' and r['class_name']=='panzer']
summary['align_modes']=d['modes']
summary['detection_summary']=[]
for topic,cl in sorted(set((r['topic'],r['class_name']) for r in d['detections'])):
    rows=[r for r in d['detections'] if r['topic']==topic and r['class_name']==cl]
    summary['detection_summary'].append(dict(topic=topic,class_name=cl,count=len(rows),first_receipt=rows[0]['t'],first_image=min(r['header']['stamp']['secs']+r['header']['stamp']['nsecs']*1e-9-d['start'] for r in rows),last_receipt=rows[-1]['t'],map_valid=sum(r['map_valid'] for r in rows)))
summary['candidate_summary']=[]
for cl,identity in sorted(set((r['class_name'],r['id']) for r in d['candidates'])):
    rows=[r for r in d['candidates'] if r['class_name']==cl and r['id']==identity]
    confirmed=next((r['t'] for r in rows if r['state']==2),None)
    summary['candidate_summary'].append(dict(class_name=cl,id=identity,count=len(rows),first_receipt=rows[0]['t'],confirmed_receipt=confirmed,last_receipt=rows[-1]['t']))
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ['panzer_raw','panzer_rejected']},indent=2))
fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
axes[0].plot(t,z,label='FC local Z'); axes[0].axhline(1,color='gray',ls='--',label='Search/return goal Z=1.0m'); axes[0].set_ylabel('Height (m)'); axes[0].legend(loc='upper right')
axes[1].plot(t,v,label='Horizontal speed'); axes[1].plot(t,vz,alpha=.65,label='Vertical speed'); axes[1].set_ylabel('Speed (m/s)'); axes[1].legend(loc='upper right')
for label,cl,topic,y in [('Cross raw','red_cross','/uav_vision/detections',3),('Cross mapped','red_cross','/uav_vision/detections_mapped',2),('Panzer raw','panzer','/uav_vision/detections',1),('Panzer map rejected','panzer','/uav_vision/detections_mapped',0)]:
    ts=[r['t'] for r in d['detections'] if r['class_name']==cl and r['topic']==topic]
    axes[2].scatter(ts,[y]*len(ts),s=5,label=label)
axes[2].set_yticks([0,1,2,3],['Panzer rejected','Panzer raw','Cross mapped','Cross raw']); axes[2].set_xlabel('Seconds since bag start (18:20:20.19)')
for ax in axes:
    for at,color in [(22.725,'green'),(41.758,'red'),(49.149,'purple'),(100.799,'gray')]:ax.axvline(at,color=color,ls='--',alpha=.65)
    ax.grid(alpha=.2)
axes[0].set_title('Sep 25 flight: green=interrupt, red=return, purple=LAND, gray=POSCTL')
fig.savefig(OUT/'height_speed_vision.png',dpi=160);plt.close(fig)
