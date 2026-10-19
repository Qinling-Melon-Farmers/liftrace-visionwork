"""Extract reproducible samples and infer on the same JPEG pixels as geometry."""
from pathlib import Path
import argparse
import json
import cv2
import torch
from ultralytics import YOLO

p=argparse.ArgumentParser()
p.add_argument('--video',type=Path,required=True)
p.add_argument('--predictions',type=Path,required=True,help='Full-video JSON from infer_video.py')
p.add_argument('--output',type=Path,required=True)
p.add_argument('--stride',type=int,default=15)
a=p.parse_args()
if a.stride<1: p.error('stride must be positive')
meta=json.loads(a.predictions.read_text(encoding='utf-8'))['summary']
a.output.mkdir(parents=True,exist_ok=True)
frames=a.output/'frames'; frames.mkdir(exist_ok=True)
cap=cv2.VideoCapture(str(a.video))
if not cap.isOpened(): raise RuntimeError(f'Cannot open {a.video}')
fps=cap.get(cv2.CAP_PROP_FPS); rows=[]; idx=0
width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
while True:
    ok,im=cap.read()
    if not ok: break
    if idx%a.stride==0:
        path=frames/f'{idx:05d}.jpg'
        if not cv2.imwrite(str(path),im,[cv2.IMWRITE_JPEG_QUALITY,95]): raise RuntimeError(str(path))
        rows.append(dict(frame=idx,t=idx/fps,file=str(path)))
    idx+=1
cap.release()
(a.output/'frames.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(a.output/'video.json').write_text(json.dumps(dict(input=str(a.video),fps=fps,decoded_frames=idx,duration_sec=idx/fps,width=width,height=height),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
torch.set_num_threads(4); model=YOLO(meta['model']); predictions=[]
for row in rows:
    r=model.predict(row['file'],**meta['params'])[0]
    boxes=[dict(class_name=model.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy.cpu().numpy()[0].tolist()) for b in r.boxes]
    predictions.append(dict(frame=row['frame'],t=row['t'],boxes=boxes))
result={'summary':dict(scope='Same decoded JPEG pixels as production geometry sample',model=meta['model'],params=meta['params']),'frames':predictions}
(a.output/'v8_geometry_sample_predictions.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(f'{idx} decoded frames; {len(rows)} same-pixel inference samples')
