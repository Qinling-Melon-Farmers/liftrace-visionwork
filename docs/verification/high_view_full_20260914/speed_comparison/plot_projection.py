"""Show conservative body/obstacle projection checks separately from flight Gate."""
import json,sys,csv
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;R=D.parents[3]
sys.path.insert(0,str(R/'docs/verification/fast_full_random_20260914/pilot'))
from check_tree_overflight import footprints

def main():
    items=json.loads((D/'runs.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,11));summary=[]
    for row,seed in enumerate((32,34)):
        for col,method in enumerate(('slow_high','fast_high')):
            item=next(i for i in items if i['seed']==seed and i['method']==method)
            path=D/(f'{seed}_{method}_body_projection.json' if (seed,method)!=(34,'fast_high') else '34_fast_high/body_projection.json')
            data=json.loads(path.read_text());worst=min(data['trees'],key=lambda v:v['min_separating_axis_gap_m']);t=worst['time_s']
            with (Path(item['run'])/'truth_pose.csv').open() as f:reader=csv.reader(f);next(reader);a=np.array([[float(v) for v in r] for r in reader])
            j=np.clip(np.searchsorted(a[:,0],t),1,len(a)-1);p,q=a[j-1],a[j];u=(t-p[0])/(q[0]-p[0])
            qa,qb=p[4:].copy(),q[4:].copy()
            if qa@qb<0:qb=-qb
            quat=qa*(1-u)+qb*u;quat/=np.linalg.norm(quat)
            params=yaml.safe_load((Path(item['run'])/'rosparams.yaml').read_text());offset=np.array(params['competition_key_recorder']['truth_world_offset'])
            xyz=p[1:4]+u*(q[1:4]-p[1:4])+offset
            corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*data['guard_box_size']+np.array(data['guard_box_pose'][:3])
            verts=corners@Rotation.from_quat(quat).as_matrix().T+xyz;body=verts[ConvexHull(verts[:,:2]).vertices,:2]
            obstacle=next(poly for name,poly,_ in footprints(item['world']) if name==worst['tree']);ax=axes[row,col]
            ax.fill(obstacle[:,0],obstacle[:,1],color='#80bd88',alpha=.5,label='Tree + pedestal convex footprint')
            ax.fill(body[:,0],body[:,1],color='#ef8d75',alpha=.5,label='Rotated conservative guard box')
            nearby=a[abs(a[:,0]-t)<2]
            ax.plot(nearby[:,1]+offset[0],nearby[:,2]+offset[1],color='#1f67aa',lw=1,label='Center trajectory')
            ax.scatter(xyz[0],xyz[1],marker='x',color='black',s=50)
            center=obstacle.mean(axis=0);ax.set(xlim=(center[0]-1,center[0]+1),ylim=(center[1]-1,center[1]+1),xlabel='World X (m)',ylabel='World Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
            mm=worst['min_separating_axis_gap_m']*1000
            ax.set_title(f'{seed} {method}: gap indicator {mm:+.1f} mm')
            if row==0 and col==0:ax.legend(fontsize=7,loc='upper right')
            summary.append(dict(seed=seed,method=method,minimum_sat_gap_mm=mm,overlap_samples=sum(x['overlap_samples'] for x in data['trees']),source=str(path.relative_to(D))))
    fig.suptitle('Conservative projection review | negative gap is NOT a measured collision depth')
    fig.tight_layout();fig.savefig(D/'projection_review.png',dpi=170);plt.close(fig)
    (D/'projection_review.json').write_text(json.dumps(summary,indent=2))
if __name__=='__main__':main()
