#!/usr/bin/env python3
"""Localization proposals only. No proposal is a training annotation."""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO


def iou(a,b):
    lo=np.maximum(a[:2],b[:2]); hi=np.minimum(a[2:],b[2:])
    inter=np.maximum(0,hi-lo).prod()
    return float(inter/max(1,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--models',nargs='+',required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'overlays').mkdir(); (a.output/'crops').mkdir()
    m=json.loads(a.manifest.read_text()); rows=m['samples']
    predictions=[[] for _ in rows]
    for path in a.models:
        model=YOLO(path)
        results=model.predict([s['image'] for s in rows],imgsz=640,conf=.15,iou=.45,
                              rect=False,device='0',batch=8,verbose=False)
        for j,r in enumerate(results):
            for b in r.boxes:
                predictions[j].append({'xyxy':b.xyxy.cpu().numpy()[0].tolist(),
                    'suggested_class':model.names[int(b.cls.item())],
                    'confidence':float(b.conf.item()),'model':path})
    tiles=[]; crop_tiles=[]; reviewed=[]
    colors=[(0,255,0),(0,200,255),(255,150,0),(255,0,255)]
    for j,(s,boxes) in enumerate(zip(rows,predictions)):
        im=cv2.imread(s['image']); h,w=im.shape[:2]; overlay=im.copy(); merged=[]
        for box in sorted(boxes,key=lambda b:-b['confidence']):
            overlap=next((b for b in merged if iou(b['xyxy'],box['xyxy'])>.55),None)
            if overlap: overlap['votes'].append(box)
            else: merged.append({'box_id':f'{j+1:03d}_{len(merged)+1}',
                                  'xyxy':box['xyxy'],'votes':[box],
                                  'class_name':None,'review_status':'pending'})
        for k,b in enumerate(merged):
            x1,y1,x2,y2=[round(v) for v in b['xyxy']]; c=colors[k%len(colors)]
            cv2.rectangle(overlay,(x1,y1),(x2,y2),c,3)
            cv2.putText(overlay,b['box_id'],(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.8,c,2)
            pad=round(max(x2-x1,y2-y1)*.10)
            crop=im[max(0,y1-pad):min(h,y2+pad),max(0,x1-pad):min(w,x2+pad)]
            tile=np.full((280,360,3),25,np.uint8)
            ch,cw=crop.shape[:2]; scale=min(350/cw,225/ch)
            resized=cv2.resize(crop,(max(1,round(cw*scale)),max(1,round(ch*scale))))
            th,tw=resized.shape[:2]; tile[0:th,0:tw]=resized
            cv2.putText(tile,f'{b["box_id"]} t={s["time_s"]:.2f}',(5,242),cv2.FONT_HERSHEY_SIMPLEX,.55,c,1)
            votes=','.join(sorted(set(v['suggested_class'] for v in b['votes'])))
            cv2.putText(tile,'MODEL: '+votes,(5,263),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
            cv2.imwrite(str(a.output/'crops'/f'{b["box_id"]}.jpg'),crop)
            crop_tiles.append(tile)
        cv2.imwrite(str(a.output/'overlays'/Path(s['image']).name),overlay)
        tile=np.full((525,320,3),25,np.uint8); scale=min(320/w,480/h)
        thumb=cv2.resize(overlay,(round(w*scale),round(h*scale))); th,tw=thumb.shape[:2]
        tile[:th,(320-tw)//2:(320-tw)//2+tw]=thumb
        cv2.putText(tile,f'{j+1:03d} {s["time_s"]:.2f}s boxes={len(merged)}',(5,504),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
        tiles.append(tile); reviewed.append({**s,'objects':merged,'review_status':'pending_full_frame_bbox_review'})
    pages=[]
    for collection,stem,cols,size in [(tiles,'overlay_contact',4,(525,320)),(crop_tiles,'crop_contact',4,(280,360))]:
        for start in range(0,len(collection),12):
            subset=collection[start:start+12]; sh,sw=size
            canvas=np.full((math.ceil(len(subset)/cols)*sh,cols*sw,3),25,np.uint8)
            for k,t in enumerate(subset):canvas[(k//cols)*sh:(k//cols+1)*sh,(k%cols)*sw:(k%cols+1)*sw]=t
            name=f'{stem}_{start//12+1:02d}.jpg'; cv2.imwrite(str(a.output/name),canvas); pages.append(name)
    m['samples']=reviewed; m['proposal_models']=a.models; m['status']='pending_bbox_and_class_review'
    (a.output/'bbox_review.json').write_text(json.dumps(m,indent=2)+'\n')
    content='<meta charset="utf-8"><h1>Localization proposals, pending review</h1><p>Green/yellow/blue boxes have IDs. Model names are suggestions only. Check all identifiable edge targets, correct missing boxes.</p>'
    content+=''.join(f'<p><a href="{n}">{n}<br><img width="1280" src="{n}"></a></p>' for n in pages)
    (a.output/'index.html').write_text(content)
    print(json.dumps({'frames':len(rows),'boxes':len(crop_tiles),'output':str(a.output)}))


if __name__=='__main__':main()
