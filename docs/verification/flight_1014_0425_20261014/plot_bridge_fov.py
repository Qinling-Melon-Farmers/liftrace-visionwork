"""Offline camera-footprint illustration for the 2026-10-14 04:25 flight.
Uses recorded intrinsics, dynamic/static TF, metadata and detections. No ROS nodes.
"""
from pathlib import Path
import argparse, json, collections
import numpy as np
import cv2
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from PIL import Image, ImageDraw, ImageFont

ap=argparse.ArgumentParser()
ap.add_argument('--run-dir',type=Path)
ap.add_argument('--out-dir',type=Path,default=Path(__file__).resolve().parent)
a=ap.parse_args()
root=Path(__file__).resolve().parents[3]
run=a.run_dir or next(root.glob('*/analysis_1014_0425'))
n=run/'numeric'; out=a.out_dir;out.mkdir(exist_ok=True,parents=True)
def load(name):return json.loads((n/(name+'.json')).read_text())
ci=load('camera__camera_info_first');base=ci['stamp']-ci['t'];ci=ci['m']
K=np.array(ci['K']).reshape(3,3);D=np.array(ci['D']);w,h=ci['width'],ci['height']
meta=load('board_trials__run_metadata')[0]['m'];settings=meta['settings']
route=np.array(settings['flight_area']['survey_xy']); bounds=settings['flight_area']['search_bounds']
# Recorded ground reference / relative takeoff origin, reconstructed from dispatched waypoints.
ground=-0.20592633428052062
origin=np.array([-0.0045494735240936,-0.006116796983405948])
st={}
for r in load('tf_static'):
 for t in r['m']['transforms']:st[(t['header']['frame_id'],t['child_frame_id'])]=t['transform']
def tr(m):
 return np.array([m['translation'][k] for k in 'xyz']),Rotation.from_quat([m['rotation'][k] for k in 'xyzw'])
p1,r1=tr(st[('vision_body','mapping_imu')]);p2,r2=tr(st[('mapping_imu','downward_camera_optical_frame')]);mount=p1+r1.apply(p2);rm=r1*r2
camera_height=settings['high_agl']+mount[2]
# Exact pinhole extents with principal point, zero roll/pitch/yaw; intentionally ignores distortion here.
corners=np.array([[0,0],[w,0],[w,h],[0,h]],dtype=float)
ray=np.column_stack(((corners[:,0]-K[0,2])/K[0,0],(corners[:,1]-K[1,2])/K[1,1],np.ones(4)))
rays=rm.apply(ray);xy=(rays*(-camera_height/rays[:,2,None]))[:,:2]+mount[:2]
lo=xy.min(axis=0);hi=xy.max(axis=0);span=hi-lo
hole=[route[:,0].min()+hi[0],route[:,0].max()+lo[0],route[:,1].min()+hi[1],route[:,1].max()+lo[1]]
poses={}
for row in load('tf'):
 for t in row['m']['transforms']:
  if (t['header']['frame_id'],t['child_frame_id'])==('map','vision_body'):
   pos,rot=tr(t['transform']);poses[t['header']['stamp']-base]=(pos,rot.as_quat())
ts=np.array(sorted(poses));ps=np.array([poses[t][0] for t in ts]);qs=np.array([poses[t][1] for t in ts]);slerp=Slerp(ts,Rotation.from_quat(qs))
# Sample all four distorted image boundaries, not merely the four corners.
u=np.linspace(0,w-1,41);v=np.linspace(0,h-1,25)
pix=np.concatenate([np.column_stack([u,np.zeros_like(u)]),np.column_stack([np.full_like(v,w-1),v]),np.column_stack([u[::-1],np.full_like(u,h-1)]),np.column_stack([np.zeros_like(v),v[::-1]])])
un=cv2.undistortPoints(pix.reshape(-1,1,2),K,D).reshape(-1,2);rays0=np.column_stack([un,np.ones(len(un))])
def footprint(t):
 rb=slerp(t);pb=np.array([np.interp(t,ts,ps[:,i]) for i in range(3)]);pc=pb+rb.apply(mount);rays=(rb*rm).apply(rays0)
 if np.any(rays[:,2]>=0):raise ValueError('Camera ray does not intersect ground')
 return (pc+(ground-pc[2])/rays[:,2,None]*rays)[:,:2]-origin,pb[:2]-origin
fig,axs=plt.subplots(1,2,figsize=(13,6.5),layout='constrained')
for ax in axs:
 ax.add_patch(Rectangle((bounds[0],bounds[2]),bounds[1]-bounds[0],bounds[3]-bounds[2],fill=False,ec='black',lw=1.5,ls='--',label='Declared search boundary'))
 ax.set(xlabel='Mission X relative to takeoff (m)',ylabel='Mission Y relative to takeoff (m)',xlim=(-.7,7),ylim=(-3.3,3.4));ax.set_aspect('equal');ax.grid(alpha=.2)
ax=axs[0]
for start,end in zip(route[:-1],route[1:]):
 mn=np.minimum(start,end)+lo;mx=np.maximum(start,end)+hi
 ax.add_patch(Rectangle(mn,*(mx-mn),fc='#57a9cf',ec='none',alpha=.22))
ax.plot(route[:,0],route[:,1],'-o',c='#135b85',label='Planned survey rectangle')
ax.add_patch(Rectangle((hole[0],hole[2]),hole[1]-hole[0],hole[3]-hole[2],facecolor='#f18d73',edgecolor='#9c3827',hatch='//'))
ax.text((hole[0]+hole[1])/2,(hole[2]+hole[3])/2,f'Ideal gap\n{hole[1]-hole[0]:.2f} x {hole[3]-hole[2]:.2f} m',ha='center',va='center',fontsize=10)
bridge=np.array([2.478949033,.925896159])-origin
for ax in axs:ax.plot(*bridge,'*',ms=14,c='#bc2388',label='Low-view bridge estimate (not ground truth)')
axs[0].set_title(f'Ideal level pinhole: FC {settings["high_agl"]:.2f} m / lens {camera_height:.2f} m\nFootprint X x Y = {span[0]:.2f} x {span[1]:.2f} m')
sel=(ts>=162.96)&(ts<=190.26)
axs[1].plot(ps[sel,0]-origin[0],ps[sel,1]-origin[1],color='#222',lw=1.5,label='Recorded FC trajectory')
selected=json.loads((run/'bridge_review/selected_frames.json').read_text())
for frame,color in zip(selected,['#c34a22','#3263bc','#309b52']):
 t=frame['source']
 fp,pb=footprint(t);axs[1].add_patch(Polygon(fp,fc=color,ec=color,alpha=.14));axs[1].plot(*pb,'o',color=color);axs[1].text(*pb,f' {t:.2f}s',color=color,fontsize=9)
axs[1].set_title('Selected recorded footprints (including distortion + attitude)\nOrange/blue: high-view edge crops; green: low revisit')
for ax in axs:ax.legend(loc='lower left',fontsize=8)
fig.savefig(out/'bridge_fov.png',dpi=160);plt.close(fig)
# Contact sheet uses actual recorded camera frames; resizing and captions only.
fr=run/'bridge_review';captions=['high view, bridge cropped left/bottom','high view, bridge cropped at top','low revisit, full bridge visible']
cells=[(frame['path'],f"{frame['source']:.2f}s: {caption}") for frame,caption in zip(selected,captions)]
canvas=Image.new('RGB',(1440,316),'white');draw=ImageDraw.Draw(canvas)
fontpath=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf');font=ImageFont.truetype(str(fontpath),15)
for i,(name,label) in enumerate(cells):
 im=Image.open(fr/name).convert('RGB').resize((480,270));canvas.paste(im,(i*480,36));draw.text((i*480+8,9),label,fill='black',font=font)
canvas.save(out/'bridge_observations.jpg',quality=92)
ref=[(r,d) for r in load('uav_vision__detections_refined') for d in r['m']['detections'] if d['class_name']=='bridge' and 160<r['t']<190.3]
mapped=[(r,d) for r in load('uav_vision__detections_mapped') for d in r['m']['detections'] if d['class_name']=='bridge' and d['map_valid']]
hints=[(r,d) for r in load('uav_vision__navigation_hints') for d in r['m']['detections'] if d['class_name']=='bridge']
def item(row):
 r,d=row;return {'receipt_sec':r['t'],'image_sec':d['header']['stamp']-base,'confidence':d['class_confidence'],'map_point':d['map_point'],'center_source':d['center_source'],'geometry_verified':d['geometry_verified']}
metrics={'bag_start_epoch':base,'frame':'camera_init/map (identity between them)','plot_relative_origin':origin.tolist(),'ground_z':ground,'K':ci['K'],'D':ci['D'],'fc_agl':settings['high_agl'],'lens_agl':camera_height,'route':route.tolist(),'search_bounds':bounds,'ideal_footprint_xy_m':span.tolist(),'ideal_central_gap_xy_bounds':hole,'ideal_gap_dimensions_m':[hole[1]-hole[0],hole[3]-hole[2]],'ideal_model_note':'Flat ground, level, yaw=0, pinhole, continuous commanded rectangle. Gap is not an asserted actual missed patch. Right plot uses recorded TF + distortion at selected instants.','high_bridge_refined_rows':len(ref),'high_bridge_reject_reasons':dict(collections.Counter(d['reject_reason'] for r,d in ref)),'high_bridge_rois_touch_border':sum(d['roi']['x_offset']<=2 or d['roi']['y_offset']<=2 or d['roi']['x_offset']+d['roi']['width']>=1277 or d['roi']['y_offset']+d['roi']['height']>=717 for r,d in ref),'first_bridge_hint':item(hints[0]),'first_bridge_map_valid':item(mapped[0])}
(out/'bridge_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(metrics,ensure_ascii=False,indent=2))