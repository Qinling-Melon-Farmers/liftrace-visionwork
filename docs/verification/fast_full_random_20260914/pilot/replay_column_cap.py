"""Local counterfactual from an archived sensed-map ROI; no control input."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D=Path(__file__).resolve().parent;ROOT=D.parents[3]
def real_inflated(points,queries,res=.05):
    origin=np.array([-6.,-11.,-.2])
    cells=np.floor((points-origin)/res).astype(int)
    q=np.floor((queries-origin)/res).astype(int)
    result=[]
    for v in q:
        delta=v-cells
        result.append(bool(np.any((abs(delta[:,0])<=6)&(abs(delta[:,1])<=6)&(delta[:,2]>=-6)&(delta[:,2]<=2))))
    return np.array(result)

def main():
    p=ROOT/'logs/fast_speed_pilot32_net_20260914_095034/local_map_failure_1.json'
    data=json.loads(p.read_text());g=np.asarray(data['goal'])
    raw=np.asarray(data['clouds']['static_map']['points']);old=np.asarray(data['clouds']['inflated_map']['points'])
    nearest=old[np.argmin(np.linalg.norm(old-g,axis=1))]
    query=nearest.reshape(1,3)
    old_goal=bool(np.min(np.linalg.norm(old-g,axis=1))<.05)
    actual_goal=bool(real_inflated(raw,query)[0])
    assert old_goal and not actual_goal
    # A measured obstacle in the high layer must remain occupied after a cap.
    assert real_inflated(np.vstack([raw,query]),query)[0]
    cap=1.78
    low=old[(abs(old[:,2]-1.175)<.01)&(np.linalg.norm(old[:,:2]-g[:2],axis=1)<.5)]
    assert len(low)>0 and np.all(low[:,2]<=cap)
    high=old[(abs(old[:,2]-nearest[2])<.01)&(np.linalg.norm(old[:,:2]-g[:2],axis=1)<.8)]
    retained=high[real_inflated(raw,high)]
    roi=raw[np.linalg.norm(raw[:,:2]-g[:2],axis=1)<.5]
    summary=dict(snapshot=str(p),source_stamp=data['clouds']['static_map']['stamp'],inflated_stamp=data['clouds']['inflated_map']['stamp'],
                 requested_goal=g.tolist(),column_cap_z=cap,roi_raw_points=len(roi),raw_z=roi[:,2].tolist(),
                 original_high_goal_occupied=old_goal,real_3d_inflation_goal_occupied=actual_goal,
                 injected_real_high_obstacle_retained=True,low_layer_occupied_cells_retained=len(low),
                 high_slice_cells_before=len(high),high_slice_real_cells_after=len(retained),
                 scope='Cropped snapshot counterfactual; not full live ESDF replay or flight acceptance')
    (D/'column_cap_replay.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
    fig,axes=plt.subplots(1,3,figsize=(14,5))
    local=raw[np.linalg.norm(raw[:,:2]-g[:2],axis=1)<1.]
    if len(local):
        sc=axes[0].scatter(local[:,0],local[:,1],c=local[:,2],s=12,vmin=0,vmax=3);fig.colorbar(sc,ax=axes[0],label='Observed Z (m)')
    axes[0].set_title('Sensed points near requested goal')
    axes[1].scatter(high[:,0],high[:,1],s=18,marker='s',color='firebrick');axes[1].set_title('Original occupied high slice')
    if len(retained):axes[2].scatter(retained[:,0],retained[:,1],s=18,marker='s',color='firebrick')
    axes[2].set_title('Capped columns + real 3D inflation')
    for ax in axes:
        ax.scatter(*g[:2],marker='x',s=100,color='black');ax.set(xlim=(g[0]-.85,g[0]+.85),ylim=(g[1]-.85,g[1]+.85),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(D/'column_cap_replay.png',dpi=170);plt.close(fig)
if __name__=='__main__':main()
