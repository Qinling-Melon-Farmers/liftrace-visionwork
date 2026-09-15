"""Ideal nominal-route footprint on ground; never consumed by navigation."""
import json,runpy,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import cv2,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolygonPath
from scipy.spatial import ConvexHull

D=Path(__file__).resolve().parent;R=D.parents[2]
old=R/'docs/verification/high_fast_five_20260915'
rows=json.loads((old/'matrix.json').read_text())['results']
run=Path(rows[0]['run']);camera=json.loads((run/'actual_camera_info.json').read_text())
launch=ET.parse(R/'vision_ws/src/uav_high_view/launch/full_strategy.launch')
route=np.array(yaml.safe_load(next(n.text for n in launch.findall('rosparam') if n.get('param','').endswith('/survey_xy'))))
route=np.vstack(([0.,0.],route))
w,h=camera['width'],camera['height'];K=np.array(camera['K']).reshape(3,3)
# Dense pixel perimeter accounts for distortion; static yaw=0 roll=pitch=0.
u=np.linspace(0,w-1,100);v=np.linspace(0,h-1,100)
pixels=np.vstack((np.c_[u,np.zeros_like(u)],np.c_[np.full_like(v,w-1),v],np.c_[u[::-1],np.full_like(u,h-1)],np.c_[np.zeros_like(v),v[::-1]]))
rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,np.array(camera['D'])).reshape(-1,2)
foot=-2.44*rays[:,::-1] # FC AGL2.6 minus camera16cm, world XY optical mount.
foot=foot[ConvexHull(foot).vertices]
step=.025;x=np.arange(-4.8+step/2,4.8,step);y=np.arange(-.5+step/2,7.6,step)
xx,yy=np.meshgrid(x,y);points=np.c_[xx.ravel(),yy.ravel()]
covered=np.zeros(len(points),bool);polygons=[]
for a,b in zip(route,route[1:]):
    p=np.vstack((foot+a,foot+b));p=p[ConvexHull(p).vertices];polygons.append(p)
    covered|=PolygonPath(p).contains_points(points)
mask=covered.reshape(xx.shape);ratio=float(covered.mean());area=float(covered.sum()*step**2)
base=runpy.run_path(str(R/'docs/verification/fast_full_random_20260914/analyze_fast.py'))['base']
fig,axes=plt.subplots(1,3,figsize=(18,7))
for ax,seed in zip(axes,(31,32,34)):
    row=next(r for r in rows if r['seed']==seed);rr=Path(row['run']);truth=yaml.safe_load((rr/'random_field_truth.yaml').read_text())
    ax.imshow(mask,origin='lower',extent=(-4.8,4.8,-.5,7.6),cmap=matplotlib.colors.ListedColormap(['#f9b9ab','#c9e6f0']),alpha=.8,zorder=0)
    base.scene(ax,old/f'seed_{seed}/field.world',truth)
    ax.plot(route[:,0],route[:,1],'o--',color='#174d91',lw=1.5,label='Nominal high route (yaw 0)')
    for i,p in enumerate(route):ax.annotate(str(i),p,xytext=(4,5),textcoords='offset points',fontsize=8)
    ax.plot(*(np.vstack((foot+route[1],foot[0]+route[1])).T),color='#008b66',lw=1.4,label='One camera footprint')
    ax.set_title(f'Seed {seed}: same ideal route coverage\nBlue = in swept FOV; salmon = never in FOV')
    ax.legend(fontsize=8,loc='upper right');ax.set_ylim(-.6,10.)
fig.suptitle(f'Ideal full high-route ground FOV: {area:.2f} m² / 77.76 m² = {100*ratio:.1f}%\nFC AGL 2.60m, camera AGL 2.44m; no tree occlusion, attitude dynamics or early interruption',fontsize=13)
fig.tight_layout();fig.savefig(D/'ideal_full_route_coverage.png',dpi=180);plt.close(fig)
data=dict(scope='Ideal planar ground geometric union, convex envelope of undistorted boundary; not detection or obstacle-safe route',fc_agl=2.6,camera_agl=2.44,yaw_rad=0,route=route.tolist(),footprint_xy_m=foot.tolist(),footprint_extent_xy_m=np.ptp(foot,axis=0).tolist(),grid_step_m=step,covered_area_m2=area,region_area_m2=77.76,coverage_fraction=ratio,uncovered_area_m2=77.76-area,camera_info_source=str(run/'actual_camera_info.json'))
(D/'ideal_coverage.json').write_text(json.dumps(data,indent=2))
print(json.dumps({k:v for k,v in data.items() if k!='footprint_xy_m'},indent=2))
