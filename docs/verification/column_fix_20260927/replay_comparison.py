from pathlib import Path
import json,subprocess,time,os,yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
O=R/'logs/column_fix_20260927';O.mkdir(exist_ok=True)
header='patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/include/plan_env/vertical_obstacle_support.h'
old=O/'old/plan_env';old.mkdir(parents=True,exist_ok=True)
(old/'vertical_obstacle_support.h').write_bytes(subprocess.check_output(['git','show','0706a9a:'+header],cwd=R))
param_path=R/'logs/column_rviz_seed31_20260927_131600/rosparams.yaml'
p=yaml.safe_load(param_path.read_text())['fast_planner_node']['sdf_map'];h=p['horizontal_avoidance']
res=p['resolution'];size=np.array([p['map_size_x'],p['map_size_y'],p['map_size_z']])
origin=np.array([-size[0]/2,-size[1]/2,p['ground_height']])
shape=np.ceil(size/res).astype(int)
cfg=O/'geometry.txt'
values=[*shape[:2],res,*origin,*size,h['support_radius'],h['support_min_points'],h['support_min_vertical_span'],
        h['min_x'],h['max_x'],h['min_y'],h['max_y'],h['obstacle_min_z'],
        h['column_band_low_ratio'],h['column_band_high_ratio'],1.6,.35]
cfg.write_text(' '.join(map(str,values)))
rows={'geometry':{'map_origin':origin.tolist(),'map_size':size.tolist(),'resolution':res,
                 'source':str(param_path.relative_to(R)),'source_bounds_filter':'SDFMap::isInMap including Z'}}

for name,inc,flags in [('old',O/'old',['-DOLD_SELECTOR']),('new',R/Path(header).parents[1],[])]:
 exe=O/name
 if name=='old':exe=O/'old_probe'
 subprocess.run(['g++','-O2','-std=c++14','-I',str(inc),str(D/'replay_footprint.cpp'),'-o',str(exe)]+flags,check=True)
 for scene in ['live','mechanism']:
  raw=R/'logs/column_rviz_20260927'/(scene+'_raw.xyz')
  out=O/(scene+'_'+name+'.xyz')
  times=[]
  for _ in range(3):
   start=time.perf_counter();subprocess.run([str(exe),str(raw),str(out),str(cfg)],check=True)
   times.append((time.perf_counter()-start)*1000)
  a=np.loadtxt(out).reshape(-1,3)
  # Query whether the synthetic columns with the unchanged 25cm XY / 10cm
  # down inflation occupy the same target. Normal 3D is separately retained.
  xy=origin[:2]+(a[:,:2]+.5)*res
  goal=np.array([.6,.05,2.38])
  gi=np.floor((goal[:2]-origin[:2])/res).astype(int)
  blocked=bool(np.any((np.abs(a[:,:2]-gi)<=int(np.ceil(p['obstacles_inflation']/res))).all(axis=1)&(a[:,2]-p['obstacles_inflation_down']<=goal[2])))
  rows[scene+'_'+name]={'footprint_cells':len(a),'column_at_goal':blocked,'process_ms':times}
  np.save(O/(scene+'_'+name+'.npy'),a)
raw_full=np.loadtxt(R/'logs/column_rviz_20260927/live_raw.xyz')
valid=((raw_full>=origin+1e-4)&(raw_full<=origin+size-1e-4)).all(axis=1)
np.savetxt(O/'clipped.xyz',raw_full[valid],fmt='%.5f')
subprocess.run([str(O/'new'),str(O/'clipped.xyz'),str(O/'clipped_footprint.xyz'),str(cfg)],check=True)
assert np.array_equal(np.loadtxt(O/'live_new.xyz'),np.loadtxt(O/'clipped_footprint.xyz'))
rows['outside_map_filter_regression']='PASS'
fig,axes=plt.subplots(1,2,figsize=(12,6),sharex=True,sharey=True)
raw=raw_full[valid & (raw_full[:,2]>=h['obstacle_min_z'])]
for ax,name in zip(axes,['old','new']):
 a=np.load(O/('live_'+name+'.npy'));xy=origin[:2]+(a[:,:2]+.5)*res
 ax.scatter(raw[::10,0],raw[::10,1],s=.3,c='gray',label='Map-height-eligible measured cloud XY')
 ax.scatter(xy[:,0],xy[:,1],s=1,c='#e87832',label='Column base (before 25cm inflation)')
 ax.scatter([.6],[.05],marker='*',s=140,c='#bd1646',label='Ascent goal XY')
 ax.set(xlim=(-.8,8.2),ylim=(-5.2,5.2),xlabel='X (m)',ylabel='Y (m)',title=name.upper()+': '+str(len(a))+' footprint cells')
 ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=7,loc='upper right')
fig.suptitle('Same saved Gazebo cloud / current C++ selector replay / not a new flight')
fig.tight_layout();fig.savefig(D/'footprint_comparison.png',dpi=150)
(D/'replay_results.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
