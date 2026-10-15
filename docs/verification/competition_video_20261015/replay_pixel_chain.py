"""Run production fusion/refiner methods on offline detections, with real messages.
No ROS initialization, subscribers, publishers, timers, TF, or flight services.
"""
from pathlib import Path
import argparse,sys,json,math,importlib.util,collections,copy
from types import SimpleNamespace
import yaml,cv2
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[3]);ap.add_argument('--run-dir',type=Path);a=ap.parse_args();root=a.root;run=a.run_dir or next(root.glob('*/competition_video_20261015'));cfg=root/'vision_ws/src/uav_vision/config'
sys.path[:0]=[str(root/'vision_ws/devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rospy
from uav_vision.msg import TargetDetection,TargetDetectionArray
from uav_vision.detection_fusion import DetectionFusion
spec=importlib.util.spec_from_file_location('offline_production_refiner',root/'vision_ws/src/uav_vision/scripts/target_refiner.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
fusion=DetectionFusion.__new__(DetectionFusion);fp=yaml.safe_load((cfg/'detection_fusion.yaml').read_text())
for k in ['require_red_cross_dual_confirmation','deduplicate_same_class','dedup_iou_threshold','dedup_center_ratio','aux_geometry_confidence','suppress_bridge_on_red_cross','suppress_bridge_on_landing_pad']:setattr(fusion,'_'+k,fp[k])
refiner=mod.TargetRefiner.__new__(mod.TargetRefiner);rp=yaml.safe_load((cfg/'target_refiner.yaml').read_text())
for k in ['roi_margin_px','max_center_distance_ratio','min_ring_quality']:setattr(refiner,'_'+k,rp[k])
refiner._selectable_standard_classes={'bridge','panzer','pillbox','tent'}
resolved=[];refined=[];fusion._publisher=SimpleNamespace(publish=lambda m:resolved.append(m));refiner._pub=SimpleNamespace(publish=lambda m:refined.append(m))
pred=json.loads((run/'v8_geometry_sample_predictions.json').read_text());byframe={r['frame']:r for r in pred['frames']};geometry=[json.loads(l) for l in (run/'output/geometry/candidates.jsonl').read_text().splitlines()];output=[]
def det(cls,confidence,xy,roi,stamp,radius=0,geometric=False,source='bbox'):
 d=TargetDetection();d.header.stamp=stamp;d.header.frame_id='phone_video';d.class_name=cls;d.class_confidence=confidence;d.geometry_confidence=confidence;d.geometry_verified=geometric;d.center_refined=geometric;d.center_source=source;d.association_valid=geometric;d.center_px.x=xy[0];d.center_px.y=xy[1];d.center_px.z=radius;d.roi.x_offset=max(0,int(roi[0]));d.roi.y_offset=max(0,int(roi[1]));d.roi.width=int(roi[2]);d.roi.height=int(roi[3]);d.transform_age_sec=-1.;d.reject_reason='' if geometric else 'geometry_not_refined';return d
def brief(d):return dict(class_name=d.class_name,class_confidence=d.class_confidence,geometry_confidence=d.geometry_confidence,geometry_verified=d.geometry_verified,center_refined=d.center_refined,association_valid=d.association_valid,center_source=d.center_source,reject_reason=d.reject_reason,xy=[d.center_px.x,d.center_px.y],radius=d.center_px.z,map_valid=d.map_valid)
for g in geometry:
 stamp=rospy.Time.from_sec(g['t']+1);p=byframe[g['frame']];arr=TargetDetectionArray();arr.header.stamp=stamp;arr.header.frame_id='phone_video'
 for b in p['boxes']:
  x1,y1,x2,y2=map(int,b['xyxy']);arr.detections.append(det(b['class_name'],b['confidence'],[(x1+x2)/2,(y1+y2)/2],[x1,y1,x2-x1,y2-y1],stamp))
 for c in g['circles']:arr.detections.append(det('circle',c['quality'],[c['center_x'],c['center_y']],c['roi'],stamp,c['radius'],True,'circle_geometry'))
 c=g['cross']
 if c['found']:arr.detections.append(det('red_cross',c['quality'],[c['x'],c['y']],c['roi'],stamp,0,True,'red_cross_geometry'))
 h=g['landing']
 if h['found']:arr.detections.append(det('landing_pad',h['quality'],[h['x'],h['y']],h['roi'],stamp,h['radius'],True,'landing_geometry'))
 resolved.clear();refined.clear();fusion._publish_bucket({'header':arr.header,'sources':{'target_detector','circle_detector','cross_detector','landing_detector'},'detections':arr.detections},align_mode='disabled');refiner._on_detections(resolved[-1]);normal=refined[-1]
 resolved.clear();fusion._publish_bucket({'header':arr.header,'sources':{'landing_detector'},'detections':arr.detections},align_mode='landing')
 row=dict(frame=g['frame'],t=g['t'],raw_yolo=p['boxes'],search_mode=[brief(d) for d in normal.detections],landing_mode=[brief(d) for d in resolved[-1].detections]);output.append(row)
 im=cv2.imread(g['file'])
 for b in p['boxes']:
  xy=list(map(int,b['xyxy']));cv2.rectangle(im,tuple(xy[:2]),tuple(xy[2:]),(0,255,0),2);cv2.putText(im,f"{b['class_name']} {b['confidence']:.2f}",(xy[0],max(18,xy[1]-5)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
 for d in normal.detections:
  if d.class_name!='circle' and d.geometry_verified:cv2.drawMarker(im,(int(d.center_px.x),int(d.center_px.y)),(255,0,255),cv2.MARKER_CROSS,25,3)
 dest=run/'chain_overlays';dest.mkdir(exist_ok=True);cv2.imwrite(str(dest/f"{g['frame']:05d}.jpg"),im)
summary=dict(sample_frames=len(output),search_geometric_counts=dict(collections.Counter(d['class_name'] for r in output for d in r['search_mode'] if d['geometry_verified'])),search_semantic_geometric_counts=dict(collections.Counter(d['class_name'] for r in output for d in r['search_mode'] if d['geometry_verified'] and d['class_name']!='circle')),landing_outputs=sum(len(r['landing_mode']) for r in output),map_valid_count=sum(d['map_valid'] for r in output for d in r['search_mode']),scope='Same-frame production fusion and refiner methods; pure in-memory publishers capture outputs. No ROS scheduling or TF/map/target memory/control/release. H sampled with default geometry params, mode gating tested separately.')
(run/'chain_results.json').write_text(json.dumps({'summary':summary,'frames':output},ensure_ascii=False,indent=2));print(json.dumps(summary,ensure_ascii=False,indent=2));print('focus',json.dumps([r for r in output if r['frame']==360],ensure_ascii=False,indent=2))
