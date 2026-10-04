"""Compare old and optimized rectangles offline; never starts ROS."""
from pathlib import Path
import json
import yaml
import numpy as np
from shapely.geometry import box, MultiPoint
from shapely.ops import unary_union
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
P=Path(__file__).resolve().parent;R=P.parents[2]
spec=yaml.safe_load((R/'deployment/competition/candidates/rectangle_motion.yaml').read_text())
cal=yaml.safe_load((R/'vision_ws/src/camera_sdk/param/calibration_1280x720.yaml').read_text())
K=cal['camera_matrix']['data'];w=cal['image_width'];h=cal['image_height']
area=box(-.5,-5,7.95,5)
routes=[('old_fc2p6',2.44,[[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]]),
        ('old_camera2p6',2.6,[[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]]),
        ('optimized_camera2p6',spec['survey_camera_agl'],spec['survey_xy'])]
results=[];plots=[]
for name,lens,points in routes:
 route=np.array(points);length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum())
 for model in ('measured','K'):
  L,W=(3.6*lens/1.84,1.8*lens/1.84) if model=='measured' else (w*lens/K[0],h*lens/K[4])
  for margin in (0.,.15):
   a,b=-(1-K[2]/w)*L+margin,K[2]/w*L-margin
   c,d=-(K[5]/h)*W+margin,(1-K[5]/h)*W-margin
   fp=np.array([[a,c],[b,c],[b,d],[a,d]])
   shape=unary_union([MultiPoint(np.vstack((fp+p,fp+q))).convex_hull for p,q in zip(route[:-1],route[1:])])
   core=area.buffer(-.25,join_style=2)
   row=dict(name=name,camera_agl=lens,fc_agl=lens+.16,model=model,erosion=margin,length=length,
        point_percent=100*shape.intersection(area).area/area.area,interior_percent=100*shape.intersection(core).area/core.area)
   results.append(row)
   if model=='K' and margin==.15:plots.append((row,route,shape))
(P/'rectangle_coverage.json').write_text(json.dumps(results,indent=2)+'\n')
fig,axs=plt.subplots(1,3,figsize=(14,6),constrained_layout=True)
for ax,(r,route,shape) in zip(axs,plots):
 ax.add_patch(Rectangle((-.5,-5),8.45,10,facecolor='#f6d0c7',edgecolor='black'))
 clipped=shape.intersection(area)
 for p in ([clipped] if clipped.geom_type=='Polygon' else clipped.geoms):
  ax.add_patch(Polygon(np.array(p.exterior.coords),facecolor='#badcc7'))
  for ring in p.interiors:ax.add_patch(Polygon(np.array(ring.coords),facecolor='#f6d0c7'))
 ax.plot(route[:,0],route[:,1],'o-',color='#235b90')
 ax.set(title=f"{r['name']}\n{r['length']:.2f} m; {r['point_percent']:.2f}%",
        xlabel='X forward',ylabel='Y left',aspect='equal',xlim=(-1,8.5),ylim=(-5.3,5.3))
 ax.grid(alpha=.2)
fig.suptitle('Clear 10 x 10 m field; 5 cm partition; 84.5 m2 target region\nK-based footprint eroded 15 cm, horizontal straight segments only')
fig.savefig(P/'rectangle_coverage.png',dpi=145);plt.close(fig)
for r in results:print(r)
