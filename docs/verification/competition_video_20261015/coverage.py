"""Reproduce camera-model FOV and an 8 x 9.7m rectangular route; no flight edits."""
from pathlib import Path
import argparse
import json
import math
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

p = argparse.ArgumentParser()
p.add_argument('--calibration', type=Path, default=Path(__file__).resolve().parent.parent / 'flight_1014_0425_20261014/bridge_metrics.json')
p.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
cal = json.loads(a.calibration.read_text(encoding='utf-8'))
K = np.array(cal['K']).reshape(3, 3)
D = np.array(cal['D'])
w, h = 1280, 720
uv = np.array([[0,K[1,2]],[w,K[1,2]],[K[0,2],0],[K[0,2],h]], float).reshape(-1,1,2)
fovs = {}
for name, coeff in [('pinhole', np.zeros_like(D)), ('with_D', D)]:
    rays = cv2.undistortPoints(uv, K, coeff).reshape(-1,2)
    fovs[name] = [math.degrees(math.atan(rays[1,0])-math.atan(rays[0,0])), math.degrees(math.atan(rays[3,1])-math.atan(rays[2,1]))]
# Uniformly shrink the pinhole rectangle until all sampled boundary points
# project inside the real image. This is a numerical conservative rectangle,
# not an exact maximal inscribed rectangle or a measured flight footprint.
x0, x1 = -K[0,2]/K[0,0], (w-K[0,2])/K[0,0]
y0, y1 = -K[1,2]/K[1,1], (h-K[1,2])/K[1,1]
x, y = np.linspace(x0,x1,1001), np.linspace(y0,y1,1001)
border = np.concatenate([np.column_stack((x,np.full_like(x,y0))), np.column_stack((x,np.full_like(x,y1))), np.column_stack((np.full_like(y,x0),y)), np.column_stack((np.full_like(y,x1),y))])
def fits(scale):
    pts = np.column_stack((border*scale,np.ones(len(border))))
    image = cv2.projectPoints(pts,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
    return bool(image[:,0].min()>=0 and image[:,0].max()<=w and image[:,1].min()>=0 and image[:,1].max()<=h)
lo, hi = 0., 1.2
for _ in range(60):
    mid = (lo+hi)/2
    if fits(mid): lo=mid
    else: hi=mid
base = np.array([-x1,-x0,y0,y1])
rows = []
for fc in [2.16,2.60,2.76,3.00]:
    lens=fc-.16
    bb=base*lens*lo
    rows.append({'fc_agl':fc,'lens_agl':lens,'pinhole_xy':[(x1-x0)*lens,(y1-y0)*lens], 'D_inscribed_xy':[bb[1]-bb[0],bb[3]-bb[2]], 'D_inscribed_bounds':bb.tolist()})
route=np.array([[2.4,.8],[5.7,.8],[5.7,9.1],[2.4,9.1],[2.4,.8]])
bb=np.array(rows[-1]['D_inscribed_bounds']); width=bb[1]-bb[0]; side=math.sqrt(2)
margins=[-bb[0]-2.4,5.7+bb[1]-8,(width-3.3-side)/2]
res={'calibration_source':str(a.calibration),'K':K.flatten().tolist(),'D':D.tolist(),'image_size':[w,h],'camera_z_from_fc':-.16,'fov_task_xy_deg':fovs,'inscribed_scale':lo,'heights':rows,'rectangle_relative_to_search_min':route.tolist(),'route_length':23.2,'rotated_1m_board_extra_margins_x':dict(left=margins[0],right=margins[1],middle_each_side=margins[2]),'conditions':['level downward camera; fixed nominal yaw; flat ground','whole target board inside search bounds','FOV model estimate, not measured effective detection FOV','no obstacle clearance, attitude or tracking guarantee; no flight config modified']}
(a.output/'fov_metrics.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
plt.rcParams['font.family']='DejaVu Sans'
fig,axes=plt.subplots(1,2,figsize=(13,7),constrained_layout=True)
for ax in axes:
    ax.add_patch(Rectangle((0,0),8,9.7,fill=False,lw=2,color='black'))
    ax.plot(route[:,0],route[:,1],'-o',lw=2,color='#c45809')
    for i,(xx,yy) in enumerate(route[:-1]): ax.annotate(str(i+1),(xx,yy),xytext=(6,7),textcoords='offset points',color='#a34700',weight='bold')
    ax.set(xlim=(-.5,8.5),ylim=(-.5,10.2),xlabel='X - search xmin (m)',ylabel='Y - search ymin (m)',aspect='equal')
    ax.grid(alpha=.2)
axes[0].set_title('Ground-point coverage along two long edges\nFC 3.00m / lens 2.84m; calibrated model')
for xx in [2.4,5.7]:
    xl=max(0,xx+bb[0]); xr=min(8,xx+bb[1])
    axes[0].add_patch(Rectangle((xl,0),xr-xl,9.7,color='#3d92cc',alpha=.18))
# Show simultaneous per-frame footprint at a mid-flight position.
axes[0].add_patch(Rectangle((2.4+bb[0],4.85+bb[2]),width,bb[3]-bb[2],fill=False,lw=1.8,ls='--',color='#185d93'))
axes[0].text(.15,9.9,'Orange: route; dashed: one image footprint',fontsize=8)
axes[1].set_title('Whole 1m board: conservative 45-degree envelope\n0.10m extra margin to every image edge')
half=side/2; margin=.1
axes[1].add_patch(Rectangle((half,half),8-side,9.7-side,fill=False,ls=':',color='#444',lw=2))
for xx in [2.4,5.7]:
    xl=max(half,xx+bb[0]+half+margin); xr=min(8-half,xx+bb[1]-half-margin)
    yl=max(half,.8+bb[2]+half+margin); yr=min(9.7-half,9.1+bb[3]-half-margin)
    axes[1].add_patch(Rectangle((xl,yl),xr-xl,yr-yl,color='#2aa064',alpha=.27))
axes[1].text(.15,9.9,'Dotted: allowed centres for sqrt(2)m envelope',fontsize=8)
fig.suptitle('Rectangular candidate for an 8.0 x 9.7m search area\nRelative to SEARCH BOUNDS, not takeoff origin; fixed heading; no obstacle model',fontsize=13)
fig.savefig(a.output/'rectangle_fov.png',dpi=150)
print(json.dumps(res,ensure_ascii=False,indent=2))
