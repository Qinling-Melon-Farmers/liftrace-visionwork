from pathlib import Path
import json, numpy as np, cv2, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[3]
D=R/'docs/verification/shared_boundary_20260921';D.mkdir(exist_ok=True)
K=json.loads((R/'logs/review_seed32_models_fixed_20260921_140919/actual_camera_info.json').read_text())
fx,fy,cx,cy=K['K'][0],K['K'][4],K['K'][2],K['K'][5];w,h=K['width'],K['height']
route=np.array([[.6,.05],[1,3.5],[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5]])
b=(-.5,7.4,-4.8,4.8);step=.01;size=(round((b[3]-b[2])/step),round((b[1]-b[0])/step))
fig,axes=plt.subplots(1,3,figsize=(15,6));results=[]
for ax,(agl,rotate) in zip(axes,[(2.6,False),(2.6,True),(3.,False)]):
 height=agl-.16
 uv=np.array([[0,0],[w,0],[w,h],[0,h]],dtype=float)
 # q_xyzw=(0,1,0,0): optical +u=-body X; optical +v=body Y.
 offsets=np.column_stack((-(uv[:,0]-cx)*height/fx,(uv[:,1]-cy)*height/fy))
 if rotate:offsets=offsets@np.array([[0,-1],[1,0]])
 mask=np.zeros(size,np.uint8)
 for p,q in zip(route[:-1],route[1:]):
  poly=np.vstack((offsets+p,offsets+q));poly=(poly-np.array([b[0],b[2]]))/step
  cv2.fillConvexPoly(mask,cv2.convexHull(np.round(poly).astype(np.int32)),1)
 ratio=float(mask.mean());results.append(dict(fc_agl=agl,camera_agl=height,rotated_90=rotate,ideal_fraction=ratio,footprint_m=[w*height/fx,h*height/fy]))
 ax.imshow(mask,origin='lower',extent=b,interpolation='nearest',cmap='Blues',vmin=0,vmax=1,alpha=.65)
 ax.plot(route[:,0],route[:,1],'r.-',lw=1);ax.plot(0,0,'k>')
 ax.annotate('',xy=(.65,0),xytext=(0,0),arrowprops={'arrowstyle':'->','color':'black'})
 ax.text(0,.35,'nose +X',fontsize=8)
 ax.set(xlabel='X / initial forward (m)',ylabel='Y / initial left (m)',title=f"FC {agl}m; {'rotated 90deg' if rotate else 'current mounting'}\nideal coverage {100*ratio:.1f}%")
 ax.set_xlim(b[:2]);ax.set_ylim(b[2:]);ax.grid(alpha=.2)
fig.suptitle('Complete planned high route; pinhole ground coverage; no occlusion/detection guarantees')
fig.tight_layout();fig.savefig(D/'ideal_coverage.png',dpi=150)
result=dict(scope='ideal pinhole raster estimate, 1cm grid; not measured detection recall',field_bounds=b,route=route.tolist(),results=results)
(D/'coverage.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
