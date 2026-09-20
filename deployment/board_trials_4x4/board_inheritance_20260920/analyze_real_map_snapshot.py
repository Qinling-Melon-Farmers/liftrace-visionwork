from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
D=Path(__file__).resolve().parent;S=D.parents[2]/'logs/board_stall_review_20260920/board_recovered_review'
goal=np.array([4.,-.37,.3]);offset=np.array([[x*.05,y*.05,0] for x in range(-3,4) for y in range(-3,4) if np.hypot(x*.05,y*.05)<=.15+1e-9]);rows=[]
for i in range(3):
 cloud=np.load(S/f'{i}_static_pointcloud.npz')['xyz'];occ=np.load(S/f'{i}_occupancy_inflate.npz')['xyz'];tree=cKDTree(occ)
 dist,idx=tree.query(goal+offset);blocked=np.max(np.abs(occ[idx]-(goal+offset)),axis=1)<=.02501
 near=cKDTree(cloud).query(goal);row=dict(snapshot=i,goal=goal.tolist(),raw_nearest_distance=float(near[0]),raw_nearest_point=cloud[near[1]].tolist(),goal_nearest_occupied_center=occ[tree.query(goal)[1]].tolist(),candidate_count=len(offset),candidate_same_voxel_occupied=int(blocked.sum()),min_nearest_center_distance=float(dist.min()),max_nearest_center_distance=float(dist.max()))
 slice=occ[np.abs(occ[:,2]-.3)<=.02501];fig,ax=plt.subplots(figsize=(9,6));ax.scatter(slice[:,0],slice[:,1],s=2,color='orange',alpha=.5,label='Inflated occupied voxels at goal Z')
 raw=cloud[np.abs(cloud[:,2]-.3)<=.15];ax.scatter(raw[:,0],raw[:,1],s=5,color='black',alpha=.7,label='Observed points within +/-0.15m Z')
 ax.plot([0,2,4,5],[0,.37,-.37,0],'b--',label='Nominal test waypoints')
 ax.add_patch(Circle((4,-.37),.15,fill=False,color='red',lw=2));ax.scatter(*(goal+offset)[:,:2].T,s=18,color='red',label='Goal correction samples (0.15 m radius)')
 ax.set(xlim=(-.3,5.5),ylim=(-2.,2.),xlabel='camera_init X (m)',ylabel='camera_init Y (m)',title=f'Recorded map snapshot {i}: goal (4, -0.37, 0.30)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='lower left',fontsize=8);fig.tight_layout();fig.savefig(D/f'real_map_snapshot_{i}.png',dpi=150,bbox_inches='tight');plt.close(fig)
 rows.append(row)
(D/'real_map_goal_checks.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
