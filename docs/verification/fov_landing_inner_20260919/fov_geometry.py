"""Ideal full-route camera footprint; not detection recall or obstacle visibility."""
from pathlib import Path
import json,cv2,numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolygonPath
D=Path(__file__).resolve().parent
items=json.loads((D/'runs.json').read_text())
cam=json.loads((Path(items[0]['run'])/'actual_camera_info.json').read_text())
K=np.array(cam['K']).reshape(3,3)
u=np.linspace(0,cam['width']-1,100);v=np.linspace(0,cam['height']-1,100)
pixels=np.vstack((np.c_[u,0*u],np.c_[0*v+cam['width']-1,v],np.c_[u[::-1],0*u+cam['height']-1],np.c_[0*v,v[::-1]]))
rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,np.array(cam['D'])).reshape(-1,2)
route=np.array([[0,0],[1,3.5],[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5]])
bounds=(-.5,7.4,-4.8,4.8);step=.025
x=np.arange(bounds[0]+step/2,bounds[1],step);y=np.arange(bounds[2]+step/2,bounds[3],step);xx,yy=np.meshgrid(x,y);points=np.c_[xx.ravel(),yy.ravel()]
fig,axes=plt.subplots(2,2,figsize=(12,11));results=[]
for ax,(repaired,h) in zip(axes.flat,[(False,2.6),(True,2.6),(False,3.),(True,3.)]):
    footprint=(rays*np.array([-1,1]) if repaired else -rays[:,::-1])*(h-.16)
    footprint=footprint[ConvexHull(footprint).vertices];covered=np.zeros(len(points),bool)
    for a,b in zip(route,route[1:]):
        poly=np.vstack((footprint+a,footprint+b));covered|=PolygonPath(poly[ConvexHull(poly).vertices]).contains_points(points)
    percent=float(covered.mean()*100);name='Restored mechanical camera yaw' if repaired else 'Before FOV repair'
    ax.imshow(covered.reshape(xx.shape),origin='lower',extent=bounds,cmap=matplotlib.colors.ListedColormap(['#ffd6cf','#d5eaf2']))
    ax.plot(route[:,0],route[:,1],'o--',lw=1,color='#225c92');foot=footprint+route[1];ax.plot(*np.vstack((foot,foot[0])).T,color='darkgreen',lw=1)
    ax.annotate('Initial heading +X',xy=(1,0),xytext=(-.3,.4),arrowprops={'arrowstyle':'->','color':'red'},fontsize=8)
    ax.set(title=f'{name}\nFC {h:.1f} m | ideal coverage {percent:.2f}%',xlabel='Fixed-start +X inward (m)',ylabel='Fixed-start +Y left (m)');ax.set_aspect('equal')
    results.append(dict(repaired=repaired,fc_agl_m=h,ideal_coverage_percent=percent,footprint_extent_xy_m=np.ptp(footprint,axis=0).tolist()))
fig.suptitle('Full nominal route; level body; ground-plane projection\nNo occlusion, no recognition model; early interrupted flights cover less',fontsize=12)
fig.tight_layout();fig.savefig(D/'fov_geometry.png',dpi=150);plt.close(fig)
(D/'fov_geometry.json').write_text(json.dumps(dict(bounds=bounds,scope='IDEAL_GEOMETRY_ONLY',cases=results),indent=2))
