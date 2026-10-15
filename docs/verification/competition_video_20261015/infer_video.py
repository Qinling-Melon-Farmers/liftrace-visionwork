"""Offline fixed-contract five-class video inference. No ROS or flight interfaces."""
from pathlib import Path
import argparse,json,time,collections
import cv2,numpy as np,torch
from ultralytics import YOLO
ap=argparse.ArgumentParser();ap.add_argument('--video',type=Path,required=True);ap.add_argument('--model',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--tag',default='v8');args=ap.parse_args()
args.output.mkdir(exist_ok=True,parents=True);torch.set_num_threads(4);model=YOLO(str(args.model));cap=cv2.VideoCapture(str(args.video));fps=cap.get(cv2.CAP_PROP_FPS);rows=[];idx=0;t0=time.monotonic()
params=dict(imgsz=640,conf=.5,iou=.45,agnostic_nms=False,rect=False,half=False,augment=False,max_det=300,device=0,verbose=False,save=False)
writer=cv2.VideoWriter(str(args.output/(args.tag+'_annotated.avi')),cv2.VideoWriter_fourcc(*'MJPG'),fps,(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
while True:
 ok,im=cap.read()
 if not ok:break
 r=model.predict(im,**params)[0];boxes=[dict(class_name=model.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy.cpu().numpy()[0].tolist()) for b in r.boxes]
 rows.append(dict(frame=idx,t=idx/fps,boxes=boxes))
 draw=r.plot();cv2.putText(draw,f'{args.tag} | {idx/fps:.2f}s | PT CUDA | conf >=0.50',(10,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2);writer.write(draw)
 if idx%100==0:print(args.tag,idx,'elapsed',round(time.monotonic()-t0,1),flush=True)
 idx+=1
writer.release();cap.release()
summary=dict(model=str(args.model),video=str(args.video),names=model.names,params=params,frames=idx,fps=fps,wall_seconds=time.monotonic()-t0,box_counts={str(th):dict(collections.Counter(b['class_name'] for r in rows for b in r['boxes'] if b['confidence']>=th)) for th in [.5,.7,.75,.8]},scope='PT CUDA square letterbox640, classwise NMS0.45. Same training model as exported RKNN where matching asset exists; not NPU timing or flight validation. No exhaustive ground truth.')
(args.output/(args.tag+'_predictions.json')).write_text(json.dumps(dict(summary=summary,frames=rows),ensure_ascii=False,indent=2));print(json.dumps(summary,ensure_ascii=False),flush=True)
