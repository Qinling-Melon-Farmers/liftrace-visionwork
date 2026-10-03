"""Paired five-class evaluation by semantic name (baseline retains six output IDs)."""
import argparse,json,collections,time
from pathlib import Path
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils.metrics import ap_per_class
from prepare import NAMES

def iou(a,b):
    a=np.asarray(a);b=np.asarray(b)
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);inter=np.prod(np.maximum(hi-lo,0));union=np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-inter
    return float(inter/max(union,1e-9))

def evaluate(rows,preds):
    targets=[];scores=[];classes=[];true=[];confusion=collections.Counter();thresholds={str(t):collections.defaultdict(lambda:dict(tp=0,fp=0,fn=0)) for t in [.5,.6,.7]};background_false=collections.Counter()
    for r,pr in zip(rows,preds):
        gt=np.asarray(r['labels'],float).reshape(-1,5);xy=[]
        for _,x,y,w,h in gt:xy.append(np.array([x-w/2,y-h/2,x+w/2,y+h/2]))
        targets.extend(gt[:,0].astype(int));tp=np.zeros((len(pr),10),bool)
        for j,thr in enumerate(np.linspace(.5,.95,10)):
            pairs=sorted([(iou(p['box'],b),k,g) for k,p in enumerate(pr) for g,b in enumerate(xy) if p['class_id']==gt[g,0]],reverse=True)
            pp=set();gg=set()
            for overlap,k,g in pairs:
                if overlap>=thr and k not in pp and g not in gg:tp[k,j]=True;pp.add(k);gg.add(g)
        scores.extend([p['score'] for p in pr]);classes.extend([p['class_id'] for p in pr]);true.extend(tp)
        for threshold,table in thresholds.items():
            live=[k for k,p in enumerate(pr) if p['score']>=float(threshold)]
            # Match only boxes surviving this threshold: a low-confidence box
            # must not steal a ground-truth match from an accepted prediction.
            pairs=sorted([(iou(pr[k]['box'],b),k,g) for k in live for g,b in enumerate(xy) if pr[k]['class_id']==gt[g,0]],reverse=True)
            pp=set();gg=set()
            for overlap,k,g in pairs:
                if overlap>=.5 and k not in pp and g not in gg:pp.add(k);gg.add(g)
            for c,name in enumerate(NAMES):
                ids=[k for k in live if pr[k]['class_id']==c];hits=sum(k in pp for k in ids);table[name]['tp']+=hits;table[name]['fp']+=len(ids)-hits;table[name]['fn']+=int((gt[:,0]==c).sum())-hits
        for g,b in enumerate(xy):
            candidates=[p for p in pr if p['score']>=.5 and iou(p['box'],b)>=.5]
            if candidates:
                best=max(candidates,key=lambda p:p['score']);confusion[NAMES[int(gt[g,0])]+'->'+NAMES[best['class_id']]]+=1
            else:confusion[NAMES[int(gt[g,0])]+'->miss']+=1
        if not len(gt):
            for c in set(p['class_id'] for p in pr if p['score']>=.5):background_false[NAMES[c]]+=1
    ap=ap_per_class(np.asarray(true,bool).reshape(-1,10),np.array(scores),np.array(classes),np.array(targets),names=dict(enumerate(NAMES))) if targets else None
    per={}
    if ap is not None:
        for j,c in enumerate(ap[6]):per[NAMES[c]]=dict(ap50=float(ap[5][j,0]),ap50_95=float(ap[5][j].mean()),targets=int(sum(np.array(targets)==c)))
    for table in thresholds.values():
        for c,stat in table.items():stat.update(precision=stat['tp']/max(1,stat['tp']+stat['fp']),recall=stat['tp']/max(1,stat['tp']+stat['fn']))
    return dict(images=len(rows),ap=per,thresholds=thresholds,confusion_at_05=dict(confusion),background_frames=sum(not r['labels'] for r in rows),background_false_frames_at_05=dict(background_false))

def infer(model,rows):
    preds=[]
    for i in range(0,len(rows),16):
        results=model.predict([r['image'] for r in rows[i:i+16]],conf=.001,iou=.45,imgsz=640,device=0,verbose=False,half=False)
        for res in results:
            boxes=res.boxes.xywhn.cpu().numpy();classes=res.boxes.cls.cpu().numpy();scores=res.boxes.conf.cpu().numpy();out=[]
            for (x,y,w,h),c,s in zip(boxes,classes,scores):
                name=res.names[int(c)]
                if name not in NAMES:continue
                out.append(dict(class_id=NAMES.index(name),score=float(s),box=[float(x-w/2),float(y-h/2),float(x+w/2),float(y+h/2)]))
            preds.append(out)
    return preds

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);torch.set_num_threads(4)
    allrows=json.loads((a.dataset/'manifest.json').read_text());rows=[r for r in allrows if r['split'] in ['val','test']];report={};saved={}
    for label,weight in [('baseline',a.baseline),('candidate',a.candidate)]:
        model=YOLO(str(weight));preds=infer(model,rows);saved[label]=preds;groups={}
        for group,rule in [('legacy_val',lambda r:r['split']=='val'),('flight_holdout',lambda r:r['kind']=='real_flight'),('H_background_views',lambda r:r['kind']=='sim_H_negative')]:
            ids=[i for i,r in enumerate(rows) if rule(r)];groups[group]=evaluate([rows[i] for i in ids],[preds[i] for i in ids])
        report[label]=dict(weights=str(weight),names=model.names,groups=groups);print(label,json.dumps(groups),flush=True)
        del model;torch.cuda.empty_cache()
    (a.output/'metrics.json').write_text(json.dumps(report,indent=2));(a.output/'predictions.json').write_text(json.dumps(dict(rows=rows,predictions=saved)))
if __name__=='__main__':main()
