"""Reuse bag_replay toolkit extraction for paired detector video, preserving source time."""
import argparse,bisect,collections,json,subprocess,time
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
NAMES=['bridge','panzer','pillbox','tent','red_cross']
def main():
 p=argparse.ArgumentParser();p.add_argument('--replay',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--fps',type=float,default=5);p.add_argument('--split-note',default='HELD-OUT FLIGHT');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);torch.set_num_threads(2)
 data=json.loads((a.replay/'data.json').read_text());frames=sorted(data['frames'],key=lambda f:f['stamp']);stamps=[f['stamp'] for f in frames];timeline=np.arange(max(0,stamps[0]),stamps[-1]+1e-5,1/a.fps);chosen=[]
 for t in timeline:
  j=bisect.bisect_left(stamps,t);j=min(j,len(frames)-1)
  if j and abs(stamps[j-1]-t)<abs(stamps[j]-t):j-=1
  chosen.append(frames[j])
 missing=[str(a.replay/f['file']) for f in chosen if not (a.replay/f['file']).is_file()]
 if missing:raise SystemExit('Missing extracted bag images; restore from source bag before replay: '+missing[0])
 weights=[a.baseline,a.candidate];pred=[];counts=[]
 for name,weight in zip(['baseline','candidate'],weights):
  model=YOLO(str(weight));rows=[];totals={str(thr):collections.Counter() for thr in [.5,.6,.7]}
  for i in range(0,len(chosen),8):
   results=model.predict([str(a.replay/f['file']) for f in chosen[i:i+8]],conf=.25,iou=.45,imgsz=640,device=0,verbose=False)
   for f,r in zip(chosen[i:i+8],results):
    dets=[dict(class_name=r.names[int(c)],score=float(s),xyxy=b.tolist()) for b,c,s in zip(r.boxes.xyxy.cpu().numpy(),r.boxes.cls.cpu().numpy(),r.boxes.conf.cpu().numpy())]
    rows.append(dict(stamp=f['stamp'],file=f['file'],detections=dets))
    for thr,total in totals.items():total.update(set(d['class_name'] for d in dets if d['score']>=float(thr)))
   time.sleep(.04)
  pred.append(rows);counts.append(dict(model=name,positive_frames={k:dict(v) for k,v in totals.items()}));del model;torch.cuda.empty_cache();print(name,'done',len(rows),flush=True)
 raw=a.output/'comparison_raw.mp4';w=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),a.fps,(1280,470));assert w.isOpened()
 colors={'panzer':(50,220,60),'bridge':(30,180,255),'red_cross':(30,50,255),'pillbox':(255,180,50),'tent':(220,100,180),'tank':(180,180,180)}
 for i,f in enumerate(chosen):
  im=cv2.imread(str(a.replay/f['file']));panel=np.zeros((470,1280,3),np.uint8)
  for m,label in enumerate(['OLD six-class','NEW five-class']):
   v=im.copy()
   for d in pred[m][i]['detections']:
    if d['score']<.5:continue
    x1,y1,x2,y2=map(int,d['xyxy']);color=colors.get(d['class_name'],(255,255,255));cv2.rectangle(v,(x1,y1),(x2,y2),color,3);cv2.putText(v,d['class_name']+' %.2f'%d['score'],(x1,max(22,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2)
   panel[42:402,m*640:(m+1)*640]=cv2.resize(v,(640,360));cv2.putText(panel,label,(m*640+12,28),0,.65,(255,255,255),1)
  cv2.putText(panel,'Source image t=%.2fs | 1x | %s'%(f['stamp'],a.split_note),(10,428),0,.5,(230,230,230),1)
  cv2.putText(panel,'Same bag frames / conf>=0.50 / offline detector only; navigation & release are NOT rerun.',(10,452),0,.5,(230,230,230),1);w.write(panel)
 w.release();subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(raw),'-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output/'comparison.mp4')],check=True);raw.unlink()
 (a.output/'predictions.json').write_text(json.dumps(dict(timeline=timeline.tolist(),baseline=pred[0],candidate=pred[1])))
 (a.output/'summary.json').write_text(json.dumps(dict(source=str(a.replay),split_note=a.split_note,frames=len(chosen),fps=a.fps,duration=len(chosen)/a.fps,counts=counts,caveat='Positive-frame counts are not recall; source includes unlabeled/truncated targets.'),indent=2));print('VIDEO_READY',a.output/'comparison.mp4',flush=True)
if __name__=='__main__':main()
