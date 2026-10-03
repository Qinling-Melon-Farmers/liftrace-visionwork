"""Synthetic point-cloud diagram evaluated by the production C++ column helper.
No ROS master, simulator, or flight nodes. This is NOT an acquired lidar scan."""
import argparse,json,subprocess,tempfile,time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
p=argparse.ArgumentParser()
p.add_argument('--out',type=Path,required=True)
p.add_argument('--tree-height',type=float,default=1.8)
p.add_argument('--canopy-base',type=float,default=.55)
p.add_argument('--radius',type=float,default=.45)
p.add_argument('--band',nargs=2,type=float,default=[.4,.6])
p.add_argument('--inflation',nargs=3,type=float,default=[.25,.20,.10])
a=p.parse_args()
if not(.4<a.tree_height<3.0 and 0<a.canopy_base<a.tree_height and 0<a.radius<1.2 and 0<=a.band[0]<a.band[1]<=1 and all(0<=v<=.5 for v in a.inflation)):p.error('invalid geometry')
here=Path(__file__).resolve().parent;root=here.parents[1]
include=root/'patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/include'
a.out.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/'sample'
    subprocess.run(['g++','-std=c++11','-O2','-I',str(include),str(here/'sample.cpp'),'-o',str(binary)],check=True)
    result=subprocess.run([str(binary),*map(str,[a.tree_height,a.canopy_base,a.radius,*a.band,*a.inflation])],text=True,capture_output=True,check=True)
    print(result.stderr.strip())
    from io import StringIO
    points=np.genfromtxt(StringIO(result.stdout),delimiter=',',skip_header=1)
colors=['#18845c','#eb9a28','#8453bb']
labels=['Measured tree (synthetic example)','Physical 3-D inflation','Additional middle-footprint column']
fig=plt.figure(figsize=(15,6.2),layout='constrained')
axes=[fig.add_subplot(131,projection='3d'),fig.add_subplot(132),fig.add_subplot(133)]
rng=np.random.default_rng(26)
for kind in (2,1,0):
    cloud=points[points[:,3]==kind,:3]
    selected=cloud if len(cloud)<4500 else cloud[rng.choice(len(cloud),4500,replace=False)]
    axes[0].scatter(selected[:,0],selected[:,1],selected[:,2],s=1.3,c=colors[kind],alpha=.20 if kind else .8)
    side=cloud[abs(cloud[:,1])<.051]
    axes[1].scatter(side[:,0],side[:,2],s=7,c=colors[kind],alpha=.45 if kind else .9,marker='s')
    top=cloud if kind==0 else cloud[abs(cloud[:,2]-2.6)<.051]
    axes[2].scatter(top[:,0],top[:,1],s=9,c=colors[kind],alpha=.25 if kind else .65,marker='s')
# Reference outline of the old widest-footprint extrusion (XY bounds).
from scipy.spatial import ConvexHull
physical_xy=np.unique(points[points[:,3]==1,:2],axis=0)
hull=ConvexHull(physical_xy); outline=physical_xy[hull.vertices];outline=np.vstack([outline,outline[0]])
axes[2].plot(outline[:,0],outline[:,1],color='#555',ls='--',lw=1.2)
for x in (physical_xy[:,0].min(),physical_xy[:,0].max()):
    axes[1].plot([x,x],[a.canopy_base,3.2],color='#555',ls='--',lw=1)
axes[0].view_init(23,-52);axes[0].set(xlabel='X (m)',ylabel='Y (m)',zlabel='',title='Point-cloud layers',zlim=(0,3.2))
axes[1].set(xlabel='X (m)',ylabel='Z AGL (m)',title='Side slice |Y| < 5.1 cm',ylim=(0,3.2),xlim=(-1,1));axes[1].set_aspect('equal')
axes[2].set(xlabel='X (m)',ylabel='Y (m)',title='High layer Z=2.6 m + tree projection',xlim=(-1,1),ylim=(-1,1));axes[2].set_aspect('equal')
band=[.4+(a.tree_height-.4)*v for v in a.band]
for z in band:axes[1].axhline(z,color='#555',ls='--',lw=1)
for ax in axes[1:]:ax.grid(alpha=.18)
fig.suptitle(f'Middle-canopy policy | tree H={a.tree_height:.2f} m, radius={a.radius:.2f} m | band {a.band[0]:.0%}-{a.band[1]:.0%}\n'
             f'XY inflation {a.inflation[0]:.2f} m, up {a.inflation[1]:.2f} m, down {a.inflation[2]:.2f} m | sampled 5 cm grid',fontsize=14)
fig.legend(handles=[Line2D([0],[0],marker='s',ls='',color=c,label=l) for c,l in zip(colors,labels)]+[Line2D([0],[0],ls='--',color='#555',label='Old widest XY outline')],loc='outside lower center',ncol=2)
fig.savefig(a.out/'column_layers.png',dpi=150)
plt.close(fig)
metadata=dict(example='synthetic cone + trunk, not field lidar',tree_height=a.tree_height,canopy_base=a.canopy_base,radius=a.radius,
              band_ratios=a.band,band_nominal_z=band,inflation_xyz=a.inflation,grid=.05,
              classification_timing=result.stderr.strip(),physical_voxels=int(sum(points[:,3]==1)),additional_column_voxels=int(sum(points[:,3]==2)))
(a.out/'column_example.json').write_text(json.dumps(metadata,indent=2))
print(metadata)
