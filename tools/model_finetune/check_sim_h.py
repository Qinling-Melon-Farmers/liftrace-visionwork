"""Offline H/background check from recorded SITL camera frames, independent of mission mode."""
import argparse,csv,json,bisect,collections
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO

def main():
 p=argparse.ArgumentParser();p.add_argument('--logs',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output=a.output.resolve();a.logs=a.logs.resolve();a.output.mkdir(parents=True,exist_ok=True);torch.set_num_threads(2);samples=[]
 for run in sorted(a.logs.glob('model5_*')):
  data=run/'generated'
  if not (run/'gate_status.json').exists():continue
  truth=list(csv.DictReader((data/'truth_pose.csv').open()));t=np.array([float(r['t']) for r in truth]);frames=list(csv.DictReader((data/'camera_frames.csv').open()));cap=cv2.VideoCapture(str(data/'camera_raw.mp4'));last=-999.
  for f in frames:
   stamp=float(f['image_ros_sec']);j=int(np.argmin(np.abs(t-stamp)));pos=truth[j];x,y,z=[float(pos[k]) for k in ['x','y','z']]
   is_h_landing='model5_landing_' in run.name
   select=(abs(t[j]-stamp)<.4 and .60<z<1.8 and ((x*x+y*y<.15**2) or (is_h_landing and (x-2)**2+y*y<.3**2)))
   if not select or stamp-last<.5:continue
   last=stamp;cap.set(cv2.CAP_PROP_POS_FRAMES,int(f['frame']));ok,im=cap.read()
   if not ok:continue
   file=a.output/(run.name+'_'+f['frame']+'.jpg');cv2.imwrite(str(file),im);samples.append(dict(run=run.name,file=str(file),stamp=stamp,truth_xyz=[x,y,z],selection='only H scene near landing' if is_h_landing else 'stationary above takeoff H'))
  cap.release()
 if not samples:raise SystemExit('No recorded H views selected; not a passing check')
 result=dict(samples=samples,models={},scope='H-containing camera views; inspect central H separately from edge delivery targets; no truth fed into navigation')
 for label,weight in [('baseline',a.baseline),('candidate',a.candidate)]:
  model=YOLO(str(weight));rows=[]
  for i in range(0,len(samples),8):
   for sample,pred in zip(samples[i:i+8],model.predict([r['file'] for r in samples[i:i+8]],conf=.5,imgsz=640,device=0,verbose=False)):
    rows.append(dict(file=sample['file'],detections=[dict(class_name=pred.names[int(c)],score=float(s),box=b.tolist()) for b,c,s in zip(pred.boxes.xyxy.cpu().numpy(),pred.boxes.cls.cpu().numpy(),pred.boxes.conf.cpu().numpy())]))
  result['models'][label]=dict(rows=rows,frames_with_delivery_detection=sum(bool(r['detections']) for r in rows),panzer_frames=sum(any(d['class_name']=='panzer' for d in r['detections']) for r in rows));del model;torch.cuda.empty_cache()
 # The contact-sheet-reviewed H lies centrally; two low_multi frames also
 # contain a real red_cross at the left edge. Do not count it as an H error.
 for value in result['models'].values():
  h_errors=[]
  for row in value['rows']:
   im=cv2.imread(row['file']);width=im.shape[1]
   for d in row['detections']:
    cx=(d['box'][0]+d['box'][2])/2
    if .2*width<=cx<=.9*width:h_errors.append(dict(file=row['file'],detection=d))
  value['central_H_errors']=h_errors
  value['central_H_error_frames']=len(set(r['file'] for r in h_errors))
 result['H_region_method']='Contact-sheet-reviewed central H; box center u in [0.2W,0.9W]; edge real red_cross excluded. Not a general-purpose GT annotator.'
 (a.output/'results.json').write_text(json.dumps(result,indent=2));print('H_CHECK',len(samples),{k:(v['frames_with_delivery_detection'],v['panzer_frames']) for k,v in result['models'].items()})
 # Visual contact sheet for checking that selected frames actually contain H only.
 cells=[]
 for s in samples[::max(1,len(samples)//24)]:
  im=cv2.resize(cv2.imread(s['file']),(320,180));cv2.putText(im,s['run'].split('_2026')[0].replace('model5_','')+' t=%.1f'%s['stamp'],(5,18),0,.4,(0,255,255),1);cells.append(im)
 if cells:
  while len(cells)%4:cells.append(np.zeros((180,320,3),np.uint8))
  cv2.imwrite(str(a.output/'selected_views.jpg'),np.vstack([np.hstack(cells[i:i+4]) for i in range(0,len(cells),4)]))
if __name__=='__main__':main()
