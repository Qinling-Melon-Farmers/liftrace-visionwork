from pathlib import Path
import json,yaml,numpy as np,cv2,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from uav_mission.search_policy import SearchPolicy
R=Path(__file__).resolve().parents[3];D=R/'docs/verification/rotation_coverage_20260921';D.mkdir(exist_ok=True)
old=yaml.safe_load((R/'docs/verification/high_fast_five_20260915/seed_34/runtime.yaml').read_text())
new=yaml.safe_load((R/'docs/verification/history_31_40_20260920/seed_34/fast_runtime.yaml').read_text())
def route(v):
 s=v['search'];return np.array([p.as_tuple()[:2] for p in SearchPolicy(**{k:s[k] for k in ['min_x','max_x','min_y','max_y','lane_spacing','altitude']}).waypoints])
o=route(old);rot=np.column_stack([o[:,1],-o[:,0]]);n=route(new)
k=json.loads((R/'logs/review_seed32_models_fixed_20260921_140919/actual_camera_info.json').read_text());fx,fy,cx,cy=k['K'][0],k['K'][4],k['K'][2],k['K'][5]
h=1.4-.16;uv=np.array([[0,0],[1280,0],[1280,720],[0,720]],float);offset=np.column_stack((-(uv[:,0]-cx)*h/fx,(uv[:,1]-cy)*h/fy))
b=(-.5,7.4,-4.8,4.8);step=.01;shape=(960,790)
fig,axes=plt.subplots(1,2,figsize=(12,7));stats=[]
for ax,(label,a) in zip(axes,[('Current regenerated route',n),('Pointwise rotated old route',rot)]):
 mask=np.zeros(shape,np.uint8)
 for p,q in zip(a[:-1],a[1:]):
  pts=(np.vstack((p+offset,q+offset))-np.array([b[0],b[2]]))/step;cv2.fillConvexPoly(mask,cv2.convexHull(np.round(pts).astype(np.int32)),1)
 v=dict(label=label,points=len(a),length_m=float(np.linalg.norm(np.diff(a,axis=0),axis=1).sum()),ideal_fraction=float(mask.mean()))
 stats.append(v);ax.imshow(mask,extent=b,origin='lower',cmap='Blues',vmin=0,vmax=1,alpha=.65);ax.plot(a[:,0],a[:,1],'r.-',lw=.65,ms=2)
 ax.set(xlim=b[:2],ylim=b[2:],xlabel='X (m)',ylabel='Y (m)',title=f'{label}\n{len(a)} points; {mask.mean()*100:.2f}% ideal coverage');ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(D/'coverage.png',dpi=150)
v=dict(old_search=old['search'],current_search=new['search'],same_pointwise_route=n.shape==rot.shape and bool(np.allclose(n,rot)),footprint_m=[1280*h/fx,720*h/fy],field_bounds=b,stats=stats,current_route=n.tolist(),rotated_old_route=rot.tolist(),old_post_route=old['mission']['post_delivery_route'],new_post_route=new['mission']['post_delivery_route'])
(D/'result.json').write_text(json.dumps(v,indent=2)+'\n');print(json.dumps({k:v[k] for k in ['old_search','current_search','same_pointwise_route','footprint_m','stats','old_post_route','new_post_route']}))
