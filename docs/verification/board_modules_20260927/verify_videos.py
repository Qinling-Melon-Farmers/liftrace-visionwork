"""Decode samples from all recorded module videos; generate a review sheet."""
from pathlib import Path
import json, csv
import cv2,numpy as np
D=Path(__file__).resolve().parent;R=D.parents[2];rows=json.loads((D/'summary.json').read_text());results=[];tiles=[]
for row in rows:
    if 'run' not in row:continue
    data=R/row['run']/'generated';item=dict(trial=row['trial'],videos={})
    stamps=list(csv.DictReader((data/'camera_frames.csv').open()))
    item['camera_record_ros_span_s']=float(stamps[-1]['record_ros_sec'])-float(stamps[0]['record_ros_sec']) if stamps else None
    panels=[]
    for name in ('report_replay','overview','camera_raw','camera_annotated'):
        p=data/(name+'_h264.mp4')
        if not p.exists():p=data/(name+'.mp4')
        cap=cv2.VideoCapture(str(p));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS);samples=[];mid=None
        for k in (0,n//2,max(0,n-1)):
            cap.set(cv2.CAP_PROP_POS_FRAMES,k);ok,frame=cap.read();samples.append(bool(ok and frame is not None and frame.size))
            if k==n//2 and ok:mid=frame
        cap.release();item['videos'][name]=dict(path=str(p.relative_to(R)),frames=n,fps=fps,duration_s=n/fps if fps else None,sample_decode=samples)
        assert n>0 and all(samples),(p,samples)
        tile=np.full((265,360,3),245,np.uint8)
        if mid is not None:
            scale=min(360/mid.shape[1],220/mid.shape[0]);im=cv2.resize(mid,(int(mid.shape[1]*scale),int(mid.shape[0]*scale)))
            y=35+(220-im.shape[0])//2;x=(360-im.shape[1])//2;tile[y:y+im.shape[0],x:x+im.shape[1]]=im
        cv2.putText(tile,row['trial']+' / '+name,(6,18),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
        panels.append(tile)
    tiles.append(np.hstack(panels));results.append(item)
for i in range(0,len(tiles),4):cv2.imwrite(str(D/f'video_samples_{i//4+1}.jpg'),np.vstack(tiles[i:i+4]))
(D/'video_validation.json').write_text(json.dumps(results,indent=2))
print(json.dumps([dict(trial=r['trial'],span=r['camera_record_ros_span_s'],camera_video=r['videos']['camera_raw']['duration_s'],decode=True) for r in results],indent=2))
