"""Offline whole-guard projection closeups, using recorded truth only."""
from pathlib import Path
import csv,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
D=Path(__file__).resolve().parent;R=D.parents[2]
sys.path.insert(0,str(D.parent/'fast_full_random_20260914/pilot'))
from check_tree_overflight import footprints
items=json.loads((D/'runs.json').read_text());fig,axes=plt.subplots(1,3,figsize=(16,5))
results=[]
for item,ax in zip(items,axes):
    folder=D/('2672_'+item['label']);body=json.loads((folder/'body_projection.json').read_text())
    worst=min(body['trees'],key=lambda v:v['min_separating_axis_gap_m']);t=worst['time_s']
    with (Path(item['run'])/'truth_pose.csv').open() as f:rows=np.array([[float(v[k]) for k in ('t','x','y','z','qx','qy','qz','qw')] for v in csv.DictReader(f)])
    i=min(len(rows)-2,max(0,int(np.searchsorted(rows[:,0],t)-1)));a,b=rows[i],rows[i+1];u=(t-a[0])/(b[0]-a[0])
    xyz=a[1:4]+u*(b[1:4]-a[1:4])+[0,0,.22];q0,q1=a[4:].copy(),b[4:].copy()
    if np.dot(q0,q1)<0:q1=-q1
    q=q0*(1-u)+q1*u;q/=np.linalg.norm(q)
    corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*body['guard_box_size']+np.array(body['guard_box_pose'][:3])
    world=corners@Rotation.from_quat(q).as_matrix().T+xyz;poly=world[ConvexHull(world[:,:2]).vertices,:2]
    for name,tree,top in footprints(item['world']):
        ax.add_patch(Polygon(tree,facecolor='forestgreen',alpha=.3,label='Tree + pedestal hull' if name==worst['tree'] else None))
    ax.add_patch(Polygon(poly,facecolor='#e97035',edgecolor='darkred',alpha=.55,label='55 cm inflated guard projection'))
    local=rows[abs(rows[:,0]-t)<1.5];ax.plot(local[:,1],local[:,2],color='#17649b',label='FC path +/-1.5 s');ax.scatter(*xyz[:2],color='black',s=18)
    ax.set(xlim=(xyz[0]-.9,xyz[0]+.9),ylim=(xyz[1]-.9,xyz[1]+.9),xlabel='Start-frame X (m)',ylabel='Start-frame Y (m)',title=f"{item['label']} | FC AGL {xyz[2]:.2f} m\nworst separating gap {worst['min_separating_axis_gap_m']*100:.1f} cm")
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='lower right')
    results.append(dict(label=item['label'],worst=worst,guard_bottom_agl_m=float(world[:,2].min())))
fig.suptitle('Conservative whole-body over-tree projection | not physical contact at altitude; not an added margin',fontsize=12)
fig.tight_layout();fig.savefig(D/'body_projection_closeups.png',dpi=170);plt.close(fig)
(D/'body_projection_summary.json').write_text(json.dumps(results,indent=2))
