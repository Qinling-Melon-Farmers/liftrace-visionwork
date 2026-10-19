#!/usr/bin/env python3
"""Fixed PT comparison on historical regression and reviewed bag ROIs.

805 bag frames have been reviewed during model selection. They remain held out
from training but are not blind tests. ROI reviews are not exhaustive labels.
"""
import argparse
import collections
import json
from pathlib import Path
import numpy as np
import torch
import yaml
from ultralytics import YOLO
from review_target_boxes import iou

PARAMS=dict(imgsz=640,conf=.5,iou=.45,agnostic_nms=False,rect=False,half=False,
            augment=False,max_det=300,device=0,verbose=False,save=False,batch=1)


def save(path,obj):
    path.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--bag-reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);torch.set_num_threads(4)
    model=YOLO(str(a.model))
    summary={'model':str(a.model),'names':model.names,'splits':{},'fixed_prediction_params':PARAMS,
             'evaluation_scope':'PT CUDA FP32, no RKNN/board conclusions; historical val/test source overlap and pre-exposure; bag held-out regression, not blind test.'}
    baseline_root=a.bag_reference.parent
    old_weights=json.loads((a.bag_reference/'bag_1/old_0928_predictions.json').read_text())['weight']
    v7_weights=json.loads((a.bag_reference/'bag_1/candidate_1011_predictions.json').read_text())['weight']
    summary['baseline_splits']={}
    for tag,weight in [('old_0928',old_weights),('candidate_1011',v7_weights),('new_v8',str(a.model))]:
      evaluated=model if tag=='new_v8' else YOLO(weight)
      target=summary['splits'] if tag=='new_v8' else summary['baseline_splits'].setdefault(tag,{})
      for split in ('val','test'):
        metrics=evaluated.val(data=str(a.dataset/'data.yaml'),split=split,device=0,imgsz=640,
                          batch=1,rect=False,iou=.45,workers=2,plots=True,project=str(a.output),name=tag+'_'+split,verbose=False)
        target[split]={'metrics':{k:float(v) for k,v in metrics.results_dict.items()},
          'measured_classes':[{'class_id':int(cid),'class_name':model.names[int(cid)],
                              'precision':float(metrics.box.p[k]),'recall':float(metrics.box.r[k]),
                              'ap50':float(metrics.box.ap50[k]),'ap50_95':float(metrics.box.ap[k])}
                             for k,cid in enumerate(metrics.box.ap_class_index)],
          'confusion_matrix':metrics.confusion_matrix.matrix.tolist(),
          'confusion_axes':'rows=predicted classes, columns=truth; final row/column background',
          'precision_recall_scope':'AP uses full confidence curve (validator conf .001), P/R validator-selected operating point. Bag diagnostics fixed conf .5. Batch1 square640 and NMS IoU .45 for every model.'}
    save(a.output/'historical_metrics.json',summary)
    sampling=json.loads((a.bag_reference/'sampling_manifest.json').read_text())
    visual=json.loads((a.bag_reference/'visual_review.json').read_text())
    roi_results=[];bag_counts={};cache={}
    for group,info in sampling['groups'].items():
        rows=[]
        for k,s in enumerate(info['samples']):
            r=model.predict(s['source'],**PARAMS)[0]
            boxes=[{'class_id':int(b.cls.item()),'semantic_name':model.names[int(b.cls.item())],
                    'confidence':float(b.conf.item()),'xyxy':b.xyxy.cpu().numpy()[0].tolist()} for b in r.boxes]
            boxes.sort(key=lambda b:-b['confidence'])
            row={**s,'boxes':boxes,'top1':boxes[0] if boxes else None};rows.append(row)
            cache[(group,s['frame'])]=row
            if (k+1)%100==0:print('BAG_PROGRESS',group,k+1,len(info['samples']),flush=True)
        bag_counts[group]={'frames':len(rows),'box_class_counts':dict(collections.Counter(b['semantic_name'] for row in rows for b in row['boxes'])),
          'top1_class_counts':dict(collections.Counter(row['top1']['semantic_name'] if row['top1'] else 'no_detection' for row in rows)),
          'confidence_threshold_box_counts':{str(threshold):dict(collections.Counter(b['semantic_name'] for row in rows for b in row['boxes'] if b['confidence']>=threshold)) for threshold in (.5,.7,.75)}}
        save(a.output/(group+'_predictions.json'),{'frames':rows,'params':PARAMS,'weight':str(a.model),'summary':bag_counts[group]})
    for r in visual['reviews']:
        group='bag_'+str(r['bag']);ref=cache[(group,r['frame'])]
        roi=r.get('review_roi_xyxy',r.get('roi_xyxy'))
        if roi is None:raise ValueError('ROI coordinates missing '+str(r))
        truth=r['visual_class'];compared={}
        for tag in ('old_0928','candidate_1011','new_v8'):
            if tag=='new_v8':frame=ref
            else:
                path=a.bag_reference/group/(tag+'_predictions.json')
                index={row['frame']:row for row in json.loads(path.read_text())['frames']};frame=index[r['frame']]
            matched=[{**b,'roi_iou':iou(b['xyxy'],roi)} for b in frame['boxes'] if iou(b['xyxy'],roi)>=.20]
            matched.sort(key=lambda b:-b['confidence']);best=matched[0] if matched else None
            compared[tag]={'matched_boxes':matched,'top1':best,
              'top1_correct':bool(best and best['semantic_name']==truth),
              'high_confidence_wrong_boxes':[b for b in matched if b['semantic_name']!=truth and b['confidence']>=.7]}
        roi_results.append({'bag':group,'frame':r['frame'],'image_time_s':r['image_time_s'],
                            'truth':truth,'roi':roi,'scope':'partial reviewed ROI, not exhaustive whole-frame ground truth','models':compared})
    roi_counts={tag:{'reviewed_roi_observations':len(roi_results),
                     'correct_top1':sum(row['models'][tag]['top1_correct'] for row in roi_results),
                     'high_conf_wrong_boxes':sum(len(row['models'][tag]['high_confidence_wrong_boxes']) for row in roi_results),
                     'wrong_boxes_at_conf_075':sum(sum(b['semantic_name']!=row['truth'] and b['confidence']>=.75 for b in row['models'][tag]['matched_boxes']) for row in roi_results)}
                 for tag in ('old_0928','candidate_1011','new_v8')}
    save(a.output/'bag_roi_comparison.json',{'association_iou':.20,'wrong_conf_threshold':.7,
       'counts':roi_counts,'reviews':roi_results,'limits':'Repeated views of the same instances; partial ROI labels only. No exhaustive bag P/R/AP or accuracy claim.'})
    manifest=json.loads((a.dataset/'manifest.json').read_text())
    diagnostic_paths=[s['image'] for s in manifest['added_samples'] if not s['augmentation']]
    diagnostic=[]
    for path in diagnostic_paths:
        r=model.predict(path,**PARAMS)[0]
        diagnostic.append({'image':path,'boxes':[{'semantic_name':model.names[int(b.cls.item())],
          'confidence':float(b.conf.item()),'xyxy':b.xyxy.cpu().numpy()[0].tolist()} for b in r.boxes]})
    save(a.output/'new_video_training_fit.json',{'scope':'Same-video training fit diagnostic, not generalization','frames':diagnostic})
    # Explicit H and background checks at the same fixed working point, no class-H output.
    v6=Path(json.loads((a.dataset/'manifest.json').read_text())['base']).parent/'yolo_dataset_v6_5cls_flight_h_20260928'
    h_rows=[r for r in json.loads((v6/'manifest.json').read_text()) if r['split']=='test' and not r['labels'] and r['kind']=='sim_H_negative']
    if len(h_rows)!=6:raise ValueError('Expected six inherited H test negatives')
    negative_rows=[]
    for tag,weight in [('old_0928',old_weights),('candidate_1011',v7_weights),('new_v8',str(a.model))]:
        evaluated=model if tag=='new_v8' else YOLO(weight)
        for row in h_rows:
            r=evaluated.predict(row['image'],**PARAMS)[0]
            negative_rows.append({'model':tag,'image':row['image'],'kind':'sim_H_negative','split':'test',
              'boxes':[{'semantic_name':evaluated.names[int(b.cls.item())],'confidence':float(b.conf.item()),'xyxy':b.xyxy.cpu().numpy()[0].tolist()} for b in r.boxes]})
    save(a.output/'H_negative_comparison.json',{'scope':'6 inherited simulated H negatives, no real H generalization claim','rows':negative_rows})
    summary['H_negative_false_positive_frames']={tag:sum(bool(row['boxes']) for row in negative_rows if row['model']==tag) for tag in ('old_0928','candidate_1011','new_v8')}
    summary['bag_counts']=bag_counts;summary['reviewed_roi_counts']=roi_counts
    save(a.output/'summary.json',summary)
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
