"""Offline appearance probe on the recorded YOLO sampling times; never publishes ROS.

The PT model is a diagnostic reference, not a rerun of the deployed RKNN binary.
Temporal variants use only current/past images and do not invent extra observations.
"""
import argparse,json,time,bisect
from pathlib import Path
import cv2,numpy as np

def variants(image,previous):
    lut=np.rint((np.arange(256)/255.)**.65*255).astype(np.uint8)
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
    lab[:,:,0]=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(lab[:,:,0])
    out={'raw':image,'gamma065':cv2.LUT(image,lut),'clahe':cv2.cvtColor(lab,cv2.COLOR_LAB2BGR),'rot180':cv2.rotate(image,cv2.ROTATE_180)}
    out['mean3_unregistered']=np.mean([image,*previous],axis=0).astype(np.uint8)
    start=time.perf_counter();accepted=[image.astype(np.float32)];ccs=[]
    target=cv2.resize(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(320,180)).astype(np.float32)/255.
    for old in previous:
        source=cv2.resize(cv2.cvtColor(old,cv2.COLOR_BGR2GRAY),(320,180)).astype(np.float32)/255.
        warp=np.eye(2,3,dtype=np.float32)
        try:
            cc,warp=cv2.findTransformECC(target,source,warp,cv2.MOTION_TRANSLATION,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,1e-4),None,3)
            ccs.append(float(cc))
            if cc<.85 or np.linalg.norm(warp[:,2])>20:continue
            warp[0,2]*=image.shape[1]/320.;warp[1,2]*=image.shape[0]/180.
            shifted=cv2.warpAffine(old,warp,(image.shape[1],image.shape[0]),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP,borderMode=cv2.BORDER_REPLICATE)
            accepted.append(shifted.astype(np.float32))
        except cv2.error:ccs.append(None)
    out['mean3_registered']=np.mean(accepted,axis=0).astype(np.uint8)
    return out,dict(accepted=len(accepted),ecc=ccs,ms=(time.perf_counter()-start)*1000)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--model',required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    from ultralytics import YOLO
    cv2.setNumThreads(2);d=json.loads((a.run/'replay/data.json').read_text());frames=sorted(d['frames'],key=lambda x:x['stamp']);ts=[x['stamp'] for x in frames]
    samples=[r for r in d['rows']['raw'] if r['m']['source']=='target_detector' and 118<=r['stamp']<=129]
    model=YOLO(a.model);output=[];registration=[]
    for i,sample in enumerate(samples):
        t=sample['stamp'];j=min(range(max(0,bisect.bisect_left(ts,t)-1),min(len(ts),bisect.bisect_left(ts,t)+1)),key=lambda j:abs(ts[j]-t))
        assert abs(ts[j]-t)<.001,(t,ts[j]);im=cv2.imread(str(a.run/'replay'/frames[j]['file']))
        previous=[cv2.imread(str(a.run/'replay'/frames[k]['file'])) for k in (j-1,j-2) if k>=0 and 0<t-ts[k]<=.12]
        transformed,reg=variants(im,previous);registration.append(dict(t=t,**reg))
        for mode,img in transformed.items():
            result=model.predict(img,conf=.05,iou=.45,imgsz=640,device=0,verbose=False)[0]
            dets=[]
            for b in result.boxes:
                box=b.xyxy[0].cpu().numpy().tolist()
                if mode=='rot180':box=[im.shape[1]-box[2],im.shape[0]-box[3],im.shape[1]-box[0],im.shape[0]-box[1]]
                dets.append(dict(cls=result.names[int(b.cls.item())],conf=float(b.conf.item()),box=box))
            output.append(dict(t=t,mode=mode,image=frames[j]['file'],detections=dets))
        if i%20==0:print('Appearance probe',i,len(samples),flush=True)
    summary={}
    for mode in transformed:
        subset=[r for r in output if r['mode']==mode]
        best=[max((q['conf'] for q in r['detections'] if q['cls']=='panzer'),default=0.) for r in subset]
        summary[mode]=dict(samples=len(best),hits={str(v):sum(x>=v for x in best) for v in (.3,.5,.6,.7)},peak=max(best))
    packed=dict(kind='OFFLINE_PT_DIAGNOSIS_NOT_ONBOARD',model=a.model,window=[118,129],imgsz=640,conf=.05,iou=.45,source='exact recorded target_detector source image stamps',summary=summary,registration=registration,rows=output)
    (a.run/'appearance_probe.json').write_text(json.dumps(packed,indent=2))
    a.report.mkdir(parents=True,exist_ok=True)
    (a.report/'appearance_metrics.json').write_text(json.dumps(dict(summary=summary,registration_ms_p50=float(np.median([r['ms'] for r in registration])),registration_ms_p95=float(np.percentile([r['ms'] for r in registration],95)),samples=len(samples),source=packed['kind']),indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)
if __name__=='__main__':main()