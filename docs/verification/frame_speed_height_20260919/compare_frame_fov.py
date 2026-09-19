from pathlib import Path
import json
import cv2,numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolyPath
r=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');out=r/'logs/frame_geometry_20260919';out.mkdir(exist_ok=True)
cam=json.loads((r/'logs/frame2672_A_recheck_20260919_202737/actual_camera_info.json').read_text());K=np.array(cam['K']).reshape(3,3)
u=np.linspace(0,cam['width']-1,100);v=np.linspace(0,cam['height']-1,100)
pixels=np.vstack((np.c_[u,0*u],np.c_[0*v+cam['width']-1,v],np.c_[u[::-1],0*u+cam['height']-1],np.c_[0*v,v[::-1]]))
rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,np.array(cam['D'])).reshape(-1,2)
old=np.array([[0,0],[-3.5,1],[3.5,1],[3.5,5.5],[-3.5,5.5],[-3.5,1]])
new=np.c_[old[:,1],-old[:,0]];fig,axes=plt.subplots(2,2,figsize=(12,11));results=[]
for ax,(name,route,bounds,h) in zip(axes.flat,[('Old frame',old,(-4.8,4.8,-.5,7.6),2.6),('Old frame',old,(-4.8,4.8,-.5,7.6),3.),('New start FLU',new,(-.5,7.6,-4.8,4.8),2.6),('New start FLU',new,(-.5,7.6,-4.8,4.8),3.)]):
    x0,x1,y0,y1=bounds;step=.025;x=np.arange(x0+step/2,x1,step);y=np.arange(y0+step/2,y1,step);xx,yy=np.meshgrid(x,y);points=np.c_[xx.ravel(),yy.ravel()]
    foot=-(h-.16)*rays[:,::-1];foot=foot[ConvexHull(foot).vertices];covered=np.zeros(len(points),bool)
    for a,b in zip(route,route[1:]):
        poly=np.vstack((foot+a,foot+b));covered|=PolyPath(poly[ConvexHull(poly).vertices]).contains_points(points)
    pct=float(covered.mean()*100);results.append(dict(frame=name,fc_agl=h,ideal_coverage_percent=pct,footprint_extent_xy=np.ptp(foot,axis=0).tolist()))
    ax.imshow(covered.reshape(xx.shape),origin='lower',extent=bounds,cmap=matplotlib.colors.ListedColormap(['#ffd6cf','#d5eaf2']))
    ax.plot(route[:,0],route[:,1],'o--',lw=1,color='#225c92');f=foot+route[1];ax.plot(*np.vstack((f,f[0])).T,color='darkgreen',lw=1)
    ax.set(title=f'{name} | FC {h:.1f} m | ideal {pct:.2f}%',xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal')
fig.suptitle('Heading remains world yaw=0: camera long/short footprint axes do not rotate with the scene\nZero tilt, complete nominal route, no occlusion/detection model; not observed recall',fontsize=12)
fig.tight_layout();fig.savefig(out/'frame_fov.png',dpi=140);plt.close(fig)
(out/'frame_fov.json').write_text(json.dumps(dict(scope='IDEAL_GEOMETRY_ONLY',cases=results),indent=2));print(json.dumps(results,indent=2))
