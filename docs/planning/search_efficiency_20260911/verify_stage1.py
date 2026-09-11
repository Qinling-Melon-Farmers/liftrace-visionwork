"""Offline fixtures + existing image replay. No new flight or simulator launch."""
from pathlib import Path
from dataclasses import replace
import json
import time
import sys
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'vision_ws/src/uav_coverage_memory/src'))
from uav_coverage_memory.memory import Config, Camera, Memory, State

OUT=ROOT/'docs/verification/coverage_memory_stage1_20260911'
OUT.mkdir(parents=True,exist_ok=True)
COLORS=['#e6e9ed','#84b9de','#efbe59','#ad719f','#aed79e','#317b53']
CMAP=ListedColormap(COLORS)
NORM=BoundaryNorm([-1,10,30,50,70,90,101],6)


def show(ax,memory,title):
    c=memory.config
    ax.imshow(memory.state.reshape(memory.ny,memory.nx),origin='lower',
              extent=(c.min_x,c.max_x,c.min_y,c.max_y),cmap=CMAP,norm=NORM,interpolation='nearest')
    ax.set(title=title,xlabel='Map X (m)',ylabel='Map Y (m)')


def fixtures():
    config=Config(-2,2,-2,2,.1,0,'map')
    camera=Camera(320,240,(200.,0,160.,0,200.,120.,0,0,1),(),'optical')
    image=np.random.RandomState(1).randint(30,225,(240,320),dtype=np.uint8)
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[2,3]=2
    fig,axes=plt.subplots(1,4,figsize=(14,4))
    rows=[]
    for ax,(name,kind) in zip(axes,[('Map missing','missing'),('Fresh map, one image','pending'),
                                 ('Repeated + known blocker','seen'),('New epoch','reset')]):
        memory=Memory(config)
        points=np.array([[0.,0.,1.],[10.,10.,1.]]) if kind=='seen' else np.array([[10.,10.,1.]])
        for stamp in ([1.,1.3,1.6] if kind in ['seen','reset'] else [1.]):
            result=memory.observe(image,camera,matrix,stamp,stamp+.01,stamp,stamp,'optical',
                None if kind=='missing' else points,None if kind=='missing' else stamp,'map')
        if kind=='reset':memory.reset('new_scene','round2');result=memory.summary(2.)
        rows.append(dict(name=name,**result));show(ax,memory,name)
    fig.suptitle('Offline fixture only: gray=unseen, blue=unverified, purple=occluded, light green=pending, dark green=estimate')
    fig.tight_layout();fig.savefig(OUT/'state_semantics.png',dpi=140);plt.close(fig)
    assert rows[0]['estimated_seen_area_m2']==rows[1]['estimated_seen_area_m2']==rows[3]['estimated_seen_area_m2']==0
    assert rows[2]['estimated_seen_area_m2']>0 and rows[2]['state_area_m2']['OCCLUDED']>0
    return rows


def replay():
    run=Path(json.loads((ROOT/'docs/verification/r64_matrix/flight_metrics.json').read_text())[0]['run_dir'])
    raw=json.loads((run/'actual_camera_info.json').read_text())
    camera=Camera(raw['width'],raw['height'],tuple(raw['K']),tuple(raw['D']),raw['header']['frame_id'],raw['distortion_model'])
    config=Config(**yaml.safe_load((ROOT/'vision_ws/src/uav_coverage_memory/config/shadow.yaml').read_text())['memory'])
    # Historical metadata contains one static calibration snapshot, not every
    # CameraInfo packet. This override applies ONLY to this documented replay.
    memory=Memory(replace(config,max_camera_age=600.))
    calibration_stamp=raw['header']['stamp']['stamp_ns']/1e9
    frames=np.loadtxt(run/'downward_camera.csv',delimiter=',',skiprows=1)
    poses=np.loadtxt(run/'mavros_pose.csv',delimiter=',',skiprows=1)
    _,unique=np.unique(poses[:,0],return_index=True);poses=poses[np.sort(unique)]
    rotations=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:8]))
    measures=yaml.safe_load((run/'scenario_inputs/aircraft_measurements.yaml').read_text())
    offset=np.array(measures['derived']['fc_to_camera_xyz'])
    launch=ET.parse(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/launch/toudi3_visual_delivery_guarded.launch').getroot()
    quaternion=[float(x) for x in launch.find("arg[@name='camera_extrinsic_quat_xyzw']").get('default').split()]
    extrinsic=Rotation.from_quat(quaternion).as_matrix()
    capture=cv2.VideoCapture(str(run/'downward_camera.mp4'))
    chosen=[];last=-1
    for frame,stamp,receipt in frames:
        if 25 <= stamp <= 55 and stamp-last >= 1/3:
            chosen.append((int(frame),stamp,receipt));last=stamp
    rows=[];cost=[];snapshot=None;last_frame=None
    for frame,stamp,receipt in chosen:
        capture.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,image=capture.read()
        if not ok:raise RuntimeError('cannot decode recorded frame '+str(frame))
        i=np.searchsorted(poses[:,0],stamp)
        if i==0 or i==len(poses) or poses[i,0]-poses[i-1,0]>.25:continue
        matrix=np.eye(4);rotation=rotations(stamp).as_matrix()
        matrix[:3,:3]=rotation@extrinsic
        matrix[:3,3]=np.array([np.interp(stamp,poses[:,0],poses[:,k]) for k in [1,2,3]])+rotation@offset
        begin=time.perf_counter()
        row=memory.observe(image,camera,matrix,stamp,receipt,stamp,calibration_stamp,camera.frame_id)
        cost.append((time.perf_counter()-begin)*1000)
        rows.append(dict(frame=frame,stamp=stamp,receipt=receipt,**row))
        last_frame=image
        if row.get('footprint_area_m2',0)>0:snapshot=memory.state.copy()
    capture.release()
    assert rows and all(x['estimated_seen_area_m2']==0 for x in rows)
    fig,axes=plt.subplots(1,2,figsize=(11,5))
    axes[0].imshow(cv2.cvtColor(last_frame,cv2.COLOR_BGR2RGB));axes[0].set_title('Existing R64 seed11 image, ~55 ROS s');axes[0].axis('off')
    if snapshot is not None:memory.state=snapshot
    show(axes[1],memory,'Image footprint / quality only\nNo synchronous obstacle stream: zero seen credit')
    fig.tight_layout();fig.savefig(OUT/'recorded_image_replay.png',dpi=140);plt.close(fig)
    return dict(run_dir=str(run),interval_ros_s=[25,55],frames=len(rows),accepted=sum(x['accepted'] for x in rows),
        estimated_seen_area_m2=0,scope='Existing images and interpolated estimated pose; fixed calibration snapshot (max_camera_age=600 only in replay). Missing contemporaneous obstacle stream is NOT filled with later map snapshots or world truth. Not online TF/calibration/recall acceptance.',
        core_process_ms={'p50':float(np.median(cost)),'p95':float(np.percentile(cost,95)),'max':float(max(cost))},rows=rows)


def benchmark():
    config=Config(**yaml.safe_load((ROOT/'vision_ws/src/uav_coverage_memory/config/shadow.yaml').read_text())['memory'])
    memory=Memory(config)
    camera=Camera(1280,720,(725.35,0.,631.67,0.,725.35,397.56,0.,0.,1.),(),'optical')
    image=np.random.RandomState(3).randint(30,225,(720,1280),dtype=np.uint8)
    points=np.random.RandomState(2).uniform([-4.8,-.5,0],[4.8,7.4,2],(50000,3))
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[:3,3]=[0,3,1.02]
    costs=[]
    for i in range(15):
        t=1+i/3;begin=time.perf_counter()
        memory.observe(image,camera,matrix,t,t+.01,t,t,'optical',points,t,'camera_init')
        costs.append((time.perf_counter()-begin)*1000)
    return dict(scope='Local x86 offline core only, synthetic inputs; excludes ROS decoding/TF/transport, not OrangePi performance',
        cells=len(memory.centers),points=len(points),image_size=[1280,720],iterations=len(costs),
        p50_ms=float(np.median(costs)),p95_ms=float(np.percentile(costs,95)),max_ms=float(max(costs)))


if __name__=='__main__':
    result=dict(fixtures=fixtures(),recorded_replay=replay(),benchmark=benchmark())
    (OUT/'offline_results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'replay_frames':result['recorded_replay']['frames'],
                      'replay_accepted':result['recorded_replay']['accepted'],
                      'benchmark':result['benchmark']},indent=2))
