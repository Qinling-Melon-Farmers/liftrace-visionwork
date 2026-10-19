import sys,json,math
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
from scipy.spatial import cKDTree
root=Path('/home/xhj/liftrace-deliverables/flight_review_20261012');out=root/'corridor_geometry_data'
bag=json.loads((out/'bag_extract.json').read_text());data=json.loads((root/'full/analysis.json').read_text())
meta=json.loads((root/'full/timing.json').read_text())['metadata'];ground=meta['ground_reference']['ground_z'];fc0=np.array(meta['ground_reference']['fc_xyz']);walls=np.array(meta['settings']['corridor_geometry']['wall_coordinates'])+fc0[1]
clouds=np.load(out/'inflated_clouds.npz')
poses=np.array([[r[0],r[2],*r[4:11]] for r in data['poses'] if r[1]=='/navigation/local_pose' and 246<=r[0]<=272.3])
sets=np.array([[r[0],r[2],*r[4:11]] for r in data['poses'] if r[1]=='/navigation/setpoint_mission' and 246<=r[0]<=272.3])
goals=[r for r in bag['records'] if r['topic']=='/fastplanner/goal']
focus=json.loads((root/'full/focus_telemetry.json').read_text())
plans=np.array([[r['t'],r['data']['header']['stamp']-data['start'],r['data']['position']['x'],r['data']['position']['y'],r['data']['position']['z']] for r in focus['records'] if r['topic']=='/planning/pos_cmd' and r['t']>=260])
cache={}
def get_cloud(t):
    i=max(0,int(np.searchsorted([c['t'] for c in bag['clouds']],t,side='right')-1));info=bag['clouds'][i]
    if i not in cache:
        a=clouds[info['key']];cache[i]=(a,cKDTree(a))
    return info,*cache[i]
def yaw(q):
    x,y,z,w=q;return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
def cube_distance(point,a,tree):
    d,ii=tree.query(point,k=min(32,len(a)));ii=np.atleast_1d(ii)
    v=np.maximum(np.abs(a[ii]-point)-.05001,0)
    distances=np.linalg.norm(v,axis=1);b=int(np.argmin(distances));return float(distances[b]),a[ii[b]],bool(np.any(np.max(np.abs(a[ii]-point),axis=1)<=.05001))
results=[]
for r in poses:
    if r[2]<6:continue
    info,a,tree=get_cloud(r[0]);dist,near,occupied=cube_distance(r[2:5],a,tree)
    j=int(np.argmin(abs(sets[:,1]-r[1])));sp=sets[j]
    dd,ns,so=cube_distance(sp[2:5],a,tree)
    results.append(dict(t=float(r[0]),pose_stamp=float(r[1]),xyz=r[2:5].tolist(),yaw=float(yaw(r[5:9])),agl=float(r[4]-ground),map_t=info['t'],map_receive_age=float(r[0]-info['t']),inflated_cube_distance=dist,inside_inflated=occupied,nearest_voxel=near.tolist(),setpoint=sp[2:5].tolist(),setpoint_stamp=float(sp[1]),setpoint_skew=float(abs(sp[1]-r[1])),setpoint_inflated_distance=dd,setpoint_inside_inflated=so))
# Selected slices expose only occupied voxel centers, not raw door surfaces.
snapshots=[]
for t in (252,258,264,265,266,267,270,271,272):
    info,a,tree=get_cloud(t);slice_z=float(np.unique(a[:,2])[np.argmin(abs(np.unique(a[:,2])-np.median(poses[poses[:,0]>=264,4])))]);sl=a[abs(a[:,2]-slice_z)<1e-4];xy=np.unique(np.round(sl[:,:2],4),axis=0)
    snapshots.append(dict(t=t,map_t=info['t'],slice_z=slice_z,xy_count=len(xy)))
fig,axs=plt.subplots(1,3,figsize=(16,7),constrained_layout=True)
for ax,t in zip(axs,(264,267,271)):
    info,a,tree=get_cloud(t);slice_z=float(np.unique(a[:,2])[np.argmin(abs(np.unique(a[:,2])-np.median(poses[poses[:,0]>=264,4])))]);sl=a[abs(a[:,2]-slice_z)<1e-4];xy=np.unique(np.round(sl[:,:2],4),axis=0)
    ax.scatter(xy[:,0],xy[:,1],s=18,marker='s',color='#d8b7a3',alpha=.65,label='inflated voxel centers, Z=%.3fm'%slice_z)
    pp=poses[poses[:,0]<=t];ss=sets[sets[:,0]<=t]
    ax.plot(pp[:,2],pp[:,3],color='#1465b4',lw=2,label='measured FC path')
    ax.plot(ss[:,2],ss[:,3],color='#dd7321',lw=1.6,alpha=.75,label='final mission setpoint')
    pc=plans[plans[:,0]<=t];ax.plot(pc[:,2],pc[:,3],color='#209a55',lw=1.1,ls='--',label='planning pos_cmd (from 260s)')
    gp=np.array([[g['data']['pose']['position'][v] for v in ('x','y')] for g in goals]);ax.plot(gp[:,0],gp[:,1],'k--o',ms=5,label='issued waypoint route')
    for i,p in enumerate(gp):ax.text(p[0]+.035,p[1]+.06,'W'+str(i+1),fontsize=9)
    for w in walls:ax.axhline(w,color='purple',ls=':',lw=1,label='configured wall plane' if w==walls[0] else None)
    p=poses[np.argmin(abs(poses[:,0]-t))];q=yaw(p[5:9]);c=np.array([[1,1],[-1,1],[-1,-1],[1,-1]])*.275;rot=np.array([[np.cos(q),-np.sin(q)],[np.sin(q),np.cos(q)]]);poly=c@rot.T+p[2:4]
    ax.scatter(p[2],p[3],s=45,color='#1465b4',zorder=8)
    ax.set_title('t=%.0fs; map received %.2fs'%(t,info['t']));ax.set_xlim(6.0,8.8);ax.set_ylim(-1.1,2.6);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('camera_init X [m]');ax.set_ylabel('camera_init Y [m]')
axs[0].legend(loc='lower left',fontsize=7)
fig.savefig(root/'corridor_geometry_topdown.png',dpi=180);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4));rr=np.array([[r['t'],r['inflated_cube_distance'],r['setpoint_inflated_distance']] for r in results]);ax.plot(rr[:,0],rr[:,1],label='FC distance to inflated voxel cube');ax.plot(rr[:,0],rr[:,2],label='setpoint distance to inflated voxel cube');ax.axhline(0,color='k',lw=.7);ax.set_ylim(-.01,.5);ax.set_xlabel('bag-relative receive time [s]');ax.set_ylabel('shown inflated-map distance [m]; NOT physical clearance');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(root/'corridor_geometry_map_distance.png',dpi=180);plt.close(fig)
summary=dict(cloud_header_stamp='all zero; source age unavailable; receive times only',source=bag['source'],start=bag['start'],ground_z=ground,fc_reference=fc0.tolist(),walls=walls.tolist(),goals=[dict(t=g['t'],xyz=[g['data']['pose']['position'][v] for v in ('x','y','z')]) for g in goals],snapshots=snapshots,results=results)
(out/'geometry_metrics.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
print(json.dumps(dict(frames=len(bag['clouds']),poses=len(poses),sets=len(sets),checked=len(results),first_occupied=next((r for r in results if r['inside_inflated']),None),last_result=results[-1]),indent=2))
