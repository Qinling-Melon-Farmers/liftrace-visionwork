#!/usr/bin/env python3
"""Offline PT inference on captured Gazebo images. Truth is evaluation-only.

Geometric eligibility does not label tree occlusion. Map error uses the exact
rig pose and raw detector box center, NOT LIO or refined release coordinates.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml
from ultralytics import YOLO


def project(points, row):
    rotation=np.array(row['optical_to_world'],dtype=float).T
    position=np.array(row['camera_world'])
    rvec=cv2.Rodrigues(rotation)[0]
    uv=cv2.projectPoints(np.array(points,dtype=float),rvec,-rotation@position,
        np.array(row['k']).reshape(3,3),np.array(row['d']))[0].reshape(-1,2)
    # Do not let high-order distortion fold far-outside rays into the image.
    pc=(rotation@(np.array(points,dtype=float)-position).T).T
    K=np.array(row['k']).reshape(3,3)
    pinhole=(K@pc.T).T
    pinhole=pinhole[:,:2]/pinhole[:,2,None]
    valid=(pc[:,2]>.1)&(pinhole[:,0]>-.1*row['width'])&(pinhole[:,0]<1.1*row['width'])&(pinhole[:,1]>-.1*row['height'])&(pinhole[:,1]<1.1*row['height'])
    uv[~valid]=np.nan
    return uv


def unproject(uv, row, ground_z):
    point=cv2.undistortPoints(np.array([[uv]],dtype=float),
        np.array(row['k']).reshape(3,3),np.array(row['d'])).reshape(2)
    ray=np.array(row['optical_to_world'])@np.array([point[0],point[1],1.])
    center=np.array(row['camera_world'])
    return center+ray*((ground_z-center[2])/ray[2])


def target_geometry(target, model_root):
    sdf=ET.parse(model_root/target['source']/'model.sdf')
    size=[float(v) for v in sdf.findtext('.//collision/geometry/box/size').split()]
    link_pose=[float(v) for v in sdf.findtext('.//link/pose').split()]
    z=link_pose[2]+size[2]/2
    xy=np.array([target['world_x'],target['world_y']])
    yaw=target['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    corners=[np.r_[xy+rot@np.array([sx*size[0]/2,sy*size[1]/2]),z]
             for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    return dict(class_name=target['class'],model=target['model'],center=[*xy,z],corners=corners,size=size,yaw=yaw)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='append',required=True)
    parser.add_argument('--model',required=True)
    parser.add_argument('--model-root',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    model=YOLO(args.model)
    by_condition={};segments=defaultdict(list);rows_out=[];timings=[]
    for run_name in args.run:
        run=Path(run_name)
        truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
        geometries=[target_geometry(t,Path(args.model_root)) for t in truth['targets']]
        rows=[json.loads(line) for line in (run/'captures.jsonl').read_text().splitlines()]
        for row in rows:
            image=cv2.imread(str(run/row['image']))
            if image is None:raise RuntimeError('missing image')
            begin=time.perf_counter()
            prediction=model.predict(image,imgsz=640,conf=.5,device=0,verbose=False)[0]
            timings.append((time.perf_counter()-begin)*1000)
            boxes=prediction.boxes
            detections=[]
            for box in boxes:
                bbox=box.xyxy[0].cpu().numpy().tolist()
                detections.append(dict(class_name=model.names[int(box.cls[0])],
                    confidence=float(box.conf[0]),bbox=bbox,
                    center=[(bbox[0]+bbox[2])/2,(bbox[1]+bbox[3])/2]))
            targets=[];pairs=[]
            for index,g in enumerate(geometries):
                corners=project(g['corners'],row)
                center=project([g['center']],row)[0]
                eligible=bool(np.all((corners[:,0]>=2)&(corners[:,0]<row['width']-2)&
                                     (corners[:,1]>=2)&(corners[:,1]<row['height']-2)))
                t=dict(class_name=g['class_name'],model=g['model'],center_uv=center.tolist() if np.isfinite(center).all() else None,
                       polygon=corners.tolist() if np.isfinite(corners).all() else None,fully_in_frame=eligible,
                       occlusion='NOT_ANNOTATED',matched=None)
                targets.append(t)
                # Match in the target plane, so a valid partial detection is
                # not discarded because one board corner lies outside the lens domain.
                for di,p in enumerate(detections):
                    hit=unproject(p['center'],row,g['center'][2])
                    delta=hit[:2]-np.array(g['center'][:2])
                    yaw=g['yaw'];rotation=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
                    local=rotation@delta
                    if abs(local[0])<=g['size'][0]/2 and abs(local[1])<=g['size'][1]/2:
                        pairs.append((float(np.linalg.norm(delta)),index,di))
            used_t,used_d=set(),set()
            for dist,ti,di in sorted(pairs):
                if ti in used_t or di in used_d:continue
                used_t.add(ti);used_d.add(di)
                detection=detections[di];g=geometries[ti]
                xyz=unproject(detection['center'],row,g['center'][2])
                targets[ti]['matched']=dict(predicted=detection['class_name'],confidence=detection['confidence'],
                    correct=detection['class_name']==g['class_name'],pixel_error=float(np.linalg.norm(np.array(detection['center'])-np.array(targets[ti]['center_uv']))) if targets[ti]['center_uv'] is not None else None,
                    exact_pose_box_center_error_m=float(np.linalg.norm(xyz[:2]-g['center'][:2])))
            record=dict(seed=truth['seed'],fc_agl=row['fc_agl'],view=row['view'],frame=row['frame'],
                stamp=row['stamp'],image=str(run/row['image']),targets=targets,detections=detections,
                unmatched_predictions=[detections[i] for i in range(len(detections)) if i not in used_d])
            rows_out.append(record);segments[(truth['seed'],row['fc_agl'],row['view'])].append(record)
            if len(rows_out)%60==0:print('Processed',len(rows_out),flush=True)
    summaries=[]
    for (seed,height,view),frames in segments.items():
        for i in range(5):
            ts=[f['targets'][i] for f in frames]
            eligible=all(t['fully_in_frame'] for t in ts)
            matched=[t['matched'] for t in ts if t['matched'] is not None]
            correct=[m for m in matched if m['correct']]
            summaries.append(dict(seed=seed,fc_agl=height,view=view,class_name=ts[0]['class_name'],
                eligible=eligible,observed_any=bool(correct),confirmed_three=len(correct)==3,
                wrong_confirmations=sum(not m['correct'] for m in matched),
                errors=[m['exact_pose_box_center_error_m'] for m in correct]))
    grouped=[]
    for height in sorted({s['fc_agl'] for s in summaries}):
        for cls in sorted({s['class_name'] for s in summaries}):
            all_rows=[s for s in summaries if s['fc_agl']==height and s['class_name']==cls]
            eligible=[s for s in all_rows if s['eligible']]
            errors=[e for s in eligible for e in s['errors']]
            grouped.append(dict(fc_agl=height,class_name=cls,geometric_eligible_views=len(eligible),
                three_frame_confirmed_views=sum(s['confirmed_three'] for s in eligible),
                conditional_confirmation_ratio=sum(s['confirmed_three'] for s in eligible)/len(eligible) if eligible else None,
                discovered_layouts=sum(any(s['seed']==seed and s['observed_any'] for s in all_rows) for seed in sorted({s['seed'] for s in all_rows})),
                wrong_in_eligible=sum(s['wrong_confirmations'] for s in eligible),
                exact_pose_box_center_error_p95=float(np.percentile(errors,95)) if errors else None))
    report=dict(scope='GAZEBO_STATIONARY_PT_SCREENING_NOT_FLIGHT_OR_RKNN_ACCEPTANCE',
        model=args.model,model_names=model.names,imgsz=640,confidence=.5,
        frames=len(rows_out),independent_layouts=len(args.run),segments=len(segments),
        groups=grouped,inference_ms_median=float(np.median(timings[1:])),
        inference_ms_p95=float(np.percentile(timings[1:],95)),
        unmatched_predictions=sum(len(r['unmatched_predictions']) for r in rows_out),
        p0_status='PARTIAL_SCREENING_OCCLUSION_DYNAMIC_POSE_AND_SAMPLE_SIZE_PENDING')
    (out/'predictions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows_out))
    (out/'segments.json').write_text(json.dumps(summaries,indent=2))
    (out/'metrics.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
