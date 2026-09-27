
from pathlib import Path
import json,subprocess,numpy as np
from scipy.ndimage import maximum_filter1d
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
O=R/'logs/column_rviz_20260927';O.mkdir(exist_ok=True)
# Illustrative connected U walls + disconnected conical tree, not recorded scene truth.
walls=[]
for x in np.arange(-.45,7.46,.1):
 for z in np.arange(.25,3.76,.1):
  walls.extend([[x,-4.5,z],[x,4.5,z]])
for y in np.arange(-4.5,4.51,.1):
 for z in np.arange(.25,3.76,.1):walls.append([-.45,y,z])
tree=[]
for z in np.arange(.25,1.51,.05):
 radius=.48*(1.6-z)/1.35
 for a in np.arange(0,2*np.pi,.12):tree.append([3.5+radius*np.cos(a),radius*np.sin(a),z])
cases={'mechanism':np.array(walls+tree),
       'recorded_local':np.array(json.loads((R/'logs/latesttwo_seed31_20260927_115041/local_map_failure_1.json').read_text())['clouds']['static_map']['points'])}
summary={}
subprocess.run(['g++','-O2','-std=c++14','-I',str(R/'patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/include'),str(D/'rebuild_footprint.cpp'),'-o',str(O/'rebuild')],check=True)
for name,raw in cases.items():
 np.savetxt(O/(name+'_raw.xyz'),raw,fmt='%.6f')
 subprocess.run([str(O/'rebuild'),str(O/(name+'_raw.xyz')),str(O/(name+'_footprint.xyz'))],check=True)
 foot=np.loadtxt(O/(name+'_footprint.xyz')).reshape(-1,3)
 shape=(240,240,84);origin=np.array([-1,-6,-.2]);res=.05
 grid=np.zeros(shape,np.uint8);ix=np.floor((raw-origin)/res).astype(int)
 ix=ix[(ix>=0).all(axis=1)&(ix<shape).all(axis=1)]
 grid[tuple(ix.T)]=1
 # Box inflation at the current 5cm voxel resolution: +/-25cm, down10/up20.
 grid=maximum_filter1d(maximum_filter1d(grid,11,axis=0,mode='constant'),11,axis=1,mode='constant')
 inf=np.zeros_like(grid)
 for dz in range(-2,5):
  src=slice(max(0,-dz),min(shape[2],shape[2]-dz))
  dst=slice(max(0,dz),min(shape[2],shape[2]+dz))
  inf[:,:,dst]|=grid[:,:,src]
 bases=np.full(shape[:2],shape[2],dtype=int)
 for x,y,z in foot:
  x=int(x);y=int(y);lo=max(0,int(np.floor((z-origin[2])/res))-2)
  bases[max(0,x-5):min(shape[0],x+6),max(0,y-5):min(shape[1],y+6)]=np.minimum(bases[max(0,x-5):min(shape[0],x+6),max(0,y-5):min(shape[1],y+6)],lo)
 zidx=np.arange(shape[2])[None,None,:]
 cols=(zidx>=bases[:,:,None])&(zidx<=int(np.ceil((3.13-origin[2])/res)))
 # Thin visualization uniformly to 10cm; calculation stays at 5cm.
 def export(label,a):
  inds=np.argwhere(a);inds=inds[(inds%2==0).all(axis=1)]
  pts=origin+(inds+.5)*res
  np.savetxt(O/(name+'_'+label+'.xyz'),pts,fmt='%.4f')
  return len(pts)
 ninf=export('inflated',inf);ncol=export('columns',cols&~inf.astype(bool))
 goal=np.floor((np.array([.6,.05,2.38])-origin)/res).astype(int)
 summary[name]=dict(source='synthetic mechanism' if name=='mechanism' else 'recorded cropped cloud; not full map',raw=len(raw),display_inflated=ninf,display_columns=ncol,goal_in_3d=bool(inf[tuple(goal)]),goal_in_columns=bool(cols[tuple(goal)]))
(D/'summary.json').write_text(json.dumps(summary,indent=2));print(summary)
