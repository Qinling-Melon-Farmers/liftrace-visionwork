"""Offline five-frame metric check. Corners refer to ORIGINAL 1280x720 images."""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

R=Path(__file__).resolve().parents[3]
LOCAL=R/'logs/flight_fov_20260926';OUT=Path(__file__).resolve().parent
corners={
    25:[[605.,130.],[1154.,154.],[1108.,695.],[575.,662.]],
    54:[[304.,52.],[761.,79.],[727.,542.],[272.,506.]],
    55:[[259.,60.],[747.,92.],[713.,585.],[225.,549.]],
    53:[[231.,69.],[699.,87.],[681.,555.],[212.,534.]],
    56:[[166.,72.],[702.,94.],[675.,633.],[137.,604.]],
}
data=json.loads((LOCAL/'motion.json').read_text())
meta=json.loads((LOCAL/'frames.json').read_text())
cal=json.loads((LOCAL/'camera_info.json').read_text())[0]
K=np.array(cal['K']).reshape(3,3);D=np.array(cal['D'])
motion=sorted(data['motion'],key=lambda p:p['stamp'])
times=np.array([p['stamp'] for p in motion])
pos=np.array([[p['x'],p['y'],p['z']] for p in motion])
rots=Rotation.from_quat([[p['orientation'][k] for k in ['x','y','z','w']] for p in motion])
slerp=Slerp(times,rots)
ground_fc=float(np.median(pos[times<data['start']+5,2]))
ground=ground_fc-.22
image_pixels=np.array([[0,0],[1279,0],[1279,719],[0,719]],float)
unit=np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]])
def rays(pixels):
    return cv2.undistortPoints(np.array(pixels).reshape(-1,1,2),K,D).reshape(-1,2)
def plane(pixels,body,cam_z):
    ray=np.column_stack([rays(pixels),np.ones(len(pixels))])
    directions=(body@np.diag([-1.,1.,-1.])@ray.T).T
    return directions[:,:2]*(-cam_z/directions[:,2,None])
def area(p):
    return abs(np.dot(p[:,0],np.roll(p[:,1],-1))-np.dot(p[:,1],np.roll(p[:,0],-1)))/2
rows=[]
for sec,pixels in corners.items():
    f=next(f for f in meta if f['requested']==sec)
    t=data['start']+f['stamp'];xyz=np.array([np.interp(t,times,pos[:,j]) for j in range(3)])
    rot=slerp([t]).as_matrix()[0]
    h=float(xyz[2]+(rot@np.array([0.,0.,-.16]))[2]-ground)
    metric=plane(pixels,rot,h)
    sides=np.linalg.norm(np.roll(metric,-1,axis=0)-metric,axis=1)
    H=cv2.getPerspectiveTransform(rays(pixels).astype(np.float32),unit.astype(np.float32))
    footprint=cv2.perspectiveTransform(rays(image_pixels).astype(np.float32)[None,:,:],H)[0]
    footprint_K=plane(image_pixels,rot,h)
    view_sides=np.linalg.norm(np.roll(footprint,-1,axis=0)-footprint,axis=1)
    w=float(np.mean(view_sides[[0,2]]));height=float(np.mean(view_sides[[1,3]]))
    euler=slerp([t]).as_euler('xyz',degrees=True)[0]
    row=dict(t=f['stamp'],image_stamp=t,nearest_pose_dt=float(np.min(abs(times-t))),local_z=float(xyz[2]),
             fc_agl=float(xyz[2]-ground),camera_agl=h,roll=float(euler[0]),pitch=float(euler[1]),
             corners_px=pixels,target_sides_from_K=sides.tolist(),target_mean_from_K=float(sides.mean()),
             footprint_from_square=dict(mean_width=w,mean_height=height,area=float(area(footprint)),corners=footprint.tolist()),
             footprint_from_K=dict(area=float(area(footprint_K)),level_width=float(1280/K[0,0]*h),level_height=float(720/K[1,1]*h)),
             scaled_at_fc_2_6_same_tilt=dict(width=w/h*2.44,height=height/h*2.44,area=float(area(footprint)/h**2*2.44**2)))
    object_pts=np.column_stack([unit,np.zeros(4)])
    ok,rvec,tvec=cv2.solvePnP(object_pts,np.array(pixels),K,D,flags=cv2.SOLVEPNP_IPPE)
    if ok:
        normal=cv2.Rodrigues(rvec)[0][:,2]
        row['square_implied_camera_height_with_K']=abs(float(normal@tvec.reshape(3)))
    # Repeat independent corner noise to show extrapolation sensitivity.
    rng=np.random.default_rng(26);areas=[]
    for _ in range(400):
        noisy=np.array(pixels)+rng.uniform(-3,3,(4,2))
        hmat=cv2.getPerspectiveTransform(rays(noisy).astype(np.float32),unit.astype(np.float32))
        p=cv2.perspectiveTransform(rays(image_pixels).astype(np.float32)[None,:,:],hmat)[0]
        areas.append(area(p))
    row['area_corner_sensitivity_pm3px_p05_p95']=np.percentile(areas,[5,95]).tolist()
    im=cv2.imread(str(LOCAL/f['file']))
    cv2.polylines(im,[np.array(pixels,np.int32)],True,(0,240,255),2)
    for i,(x,y) in enumerate(pixels):
        cv2.circle(im,(round(x),round(y)),5,(0,240,255),-1)
        cv2.putText(im,f'P{i+1}',(round(x)+7,round(y)-7),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,240,255),2)
    panel=np.full((122,1280,3),25,np.uint8)
    lines=[f'Raw bag frame t={f["stamp"]:.3f}s | calibrated 1280x720 | manually marked 1.00 x 1.00 m OUTER board',
           f'FC local Z={xyz[2]:.3f} m | FC AGL~{xyz[2]-ground:.3f} m | camera AGL~{h:.3f} m | tilt={euler[0]:.1f}/{euler[1]:.1f} deg',
           f'Square-derived footprint: {w:.2f} x {height:.2f} m (opposite-side means), area {area(footprint):.2f} m2',
           f'CameraInfo projects marked board to {sides.mean():.3f} m mean edge | +/-3 px corner sensitivity recorded; height is estimated']
    for i,line in enumerate(lines):cv2.putText(panel,line,(12,25+i*27),cv2.FONT_HERSHEY_SIMPLEX,.53,(230,230,230),1,cv2.LINE_AA)
    cv2.imwrite(str(OUT/f'measure_{sec}.jpg'),np.vstack([im,panel]))
    rows.append(row)
result=dict(calibration=cal,ground_fc_z=ground_fc,estimated_ground_z=ground,
            rig_camera_offset_fc=-.16,known_fc_ground_clearance=.22,board_side_assumed=1.0,
            source='recorded camera + stamped odom; no detector rerun; manual outer corners',
            nominal_previous_2_6=[4.31,2.42,4.31*2.42],rows=rows)
(OUT/'result.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ordered=sorted(rows,key=lambda r:r['t']);x=np.arange(len(ordered));labels=[f"{r['t']:.2f}s" for r in ordered]
fig,axes=plt.subplots(1,2,figsize=(12,4.3),layout='constrained')
axes[0].bar(x,[r['target_mean_from_K'] for r in ordered],color='#277ca7')
axes[0].axhline(1,color='#c45731',ls='--',label='Known board = 1.00 m')
axes[0].set(xticks=x,xticklabels=labels,ylabel='Reconstructed mean board edge (m)',ylim=(.8,1.15),title='CameraInfo + logged height / attitude')
axes[0].legend()
axes[1].plot(x,[r['camera_agl'] for r in ordered],'o-',label='Odom + inherited rig')
axes[1].plot(x,[r['square_implied_camera_height_with_K'] for r in ordered],'s-',label='1 m board + CameraInfo (PnP)')
axes[1].set(xticks=x,xticklabels=labels,ylabel='Camera-to-floor height (m)',title='Two estimates; neither is independent range truth')
axes[1].legend()
for ax in axes:ax.grid(axis='y',alpha=.2)
fig.savefig(OUT/'fov_comparison.png',dpi=160);plt.close(fig)
