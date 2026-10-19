#!/usr/bin/env python3
"""One-pass three-model comparison on a new held-out bag, with partial ROI truth."""
import argparse
import collections
import csv
import json
import zipfile
from pathlib import Path
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from evaluate_new_video_candidate import PARAMS, save
from review_target_boxes import iou


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames-manifest',type=Path,required=True);p.add_argument('--rois',type=Path,required=True)
    p.add_argument('--old',required=True);p.add_argument('--v7',required=True);p.add_argument('--v8',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--final',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);torch.set_num_threads(4)
    m=json.loads(a.frames_manifest.read_text());rv=json.loads(a.rois.read_text());rows=m['frames']
    if len(rows)!=403 or len({r['path'] for r in rows})!=403:raise ValueError('Expected 403 unique frames')
    summary={'source_bag':m['source_bag'],'frames':403,'models':{},'params':PARAMS,
      'scope':'New acquisition after training selection, train contribution zero, evaluated once per model. Repeated correlated frames, only five approximate primary-reviewed ROIs. No exhaustive P/R/AP or accuracy claim.'}
    lookup={};model_paths={'old_0928':a.old,'candidate_1011':a.v7,'new_v8':a.v8}
    for tag,path in model_paths.items():
        model=YOLO(path);predictions=[]
        for k,row in enumerate(rows):
            r=model.predict(row['path'],**PARAMS)[0]
            boxes=[{'semantic_name':model.names[int(b.cls.item())],'confidence':float(b.conf.item()),'xyxy':b.xyxy.cpu().numpy()[0].tolist()} for b in r.boxes]
            boxes.sort(key=lambda b:-b['confidence'])
            prediction={**row,'boxes':boxes};predictions.append(prediction);lookup[(tag,row['frame'])]=prediction
            if (k+1)%100==0:print('NEW_BAG_PROGRESS',tag,k+1,403,flush=True)
        stats={'weight':path,'frames':len(predictions),
          'box_counts_by_confidence':{str(th):dict(collections.Counter(b['semantic_name'] for row in predictions for b in row['boxes'] if b['confidence']>=th)) for th in (.5,.7,.75)}}
        summary['models'][tag]=stats;save(a.output/(tag+'_predictions.json'),{'params':PARAMS,'stats':stats,'frames':predictions})
        del model;torch.cuda.empty_cache()
    comparisons=[];tiles=[]
    for review in rv['reviews']:
        item={**review,'models':{}};cols=[]
        for tag in model_paths:
            row=lookup[(tag,review['frame'])];item['image_header_relative_sec']=row['image_header_relative_sec'];item['bag_relative_sec']=row['bag_relative_sec']
            associated=[{**box,'roi_iou':iou(box['xyxy'],review['roi_xyxy'])} for box in row['boxes'] if iou(box['xyxy'],review['roi_xyxy'])>=.2]
            associated.sort(key=lambda b:-b['confidence']);top=associated[0] if associated else None
            item['models'][tag]={'matched_boxes':associated,'top1':top,
              'high_confidence_wrong_boxes':[box for box in associated if box['semantic_name']!=review['truth'] and box['confidence']>=.7],
              'wrong_boxes_at_075':[box for box in associated if box['semantic_name']!=review['truth'] and box['confidence']>=.75]}
            im=cv2.imread(row['path']);x1,y1,x2,y2=review['roi_xyxy']
            cv2.rectangle(im,(x1,y1),(x2,y2),(255,0,255),2)
            for box in associated:
                bx1,by1,bx2,by2=[round(v) for v in box['xyxy']]
                color=(0,255,0) if box['semantic_name']==review['truth'] else (0,0,255)
                cv2.rectangle(im,(bx1,by1),(bx2,by2),color,2)
            crop=im[max(0,y1-60):min(720,y2+60),max(0,x1-60):min(1280,x2+60)]
            h,w=crop.shape[:2];scale=min(460/w,300/h);thumb=cv2.resize(crop,(round(w*scale),round(h*scale)))
            tile=np.full((380,480,3),25,np.uint8);th,tw=thumb.shape[:2];tile[:th,:tw]=thumb
            cv2.putText(tile,tag+' truth='+review['truth'],(5,322),cv2.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),1)
            text=(f'{top["semantic_name"]} {top["confidence"]:.3f}' if top else 'no associated box')
            cv2.putText(tile,text,(5,345),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
            cv2.putText(tile,review['frame'],(5,369),cv2.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),1)
            cols.append(tile)
        comparisons.append(item);tiles.append(np.concatenate(cols,axis=1))
    cv2.imwrite(str(a.output/'five_reviewed_roi_comparison.jpg'),np.concatenate(tiles,axis=0))
    summary['reviewed_roi_comparison']=comparisons;summary['review_authority']=rv['authority']
    save(a.output/'summary.json',summary)
    report=['新场次flight_debug_0-1011-1444：403帧留出对照',
      '训练结束后新增，训练选型未使用；每个模型只跑一次，403帧均不训练。固定batch1、square640、FP32、conf0.5、IoU0.45、按类NMS；0.7/0.75从相同输出统计。',
      '五处近似ROI由主代理审原图确认类别，仅可匹配/查错，不是精确bbox AP真值或全片穷尽标注。不声称5张图代表全测试准确率。',
      '红十字贴在桥底图上，同位置桥纹不是另一个独立靶。',
      'frame image_header_s truth old0928 v7 v8']
    for item in comparisons:
        parts=[]
        for tag,comp in item['models'].items():
            top=comp['top1'];parts.append(f'{tag}='+ (f'{top["semantic_name"]}@{top["confidence"]:.3f}' if top else '无匹配框'))
        report.append(f'{item["frame"]} {item["image_header_relative_sec"]:.3f} {item["truth"]} '+'; '.join(parts))
        for tag,comp in item['models'].items():
            for box in comp['high_confidence_wrong_boxes']:
                report.append(f'高置信错类：{tag} {item["frame"]} 真{item["truth"]} -> {box["semantic_name"]}@{box["confidence"]:.3f}')
    report+=['旧805帧仍留有v8裁切tent->pillbox@0.801，不能因新增代表ROI通过就称零误报。',
             '旧报告的val默认rect与较大batch、NMS配置不同；本轮三个模型统一batch1/rect=False方形640重算，不能直接将这些AP与旧报告数值作提升差分。',
             '候选未自动推广、未部署；这里是笔记本PT测试，不是RKNN/板端验收。']
    (a.output/'NEW_BAG_REPORT.txt').write_text('\n'.join(report)+'\n',encoding='utf-8')
    # Append actual completed results to the final training handoff without changing PT/selection.
    final_summary=json.loads((a.final/'test_summary.json').read_text());final_summary['new_bag_1444']=summary;save(a.final/'test_summary.json',final_summary)
    sources=json.loads((a.final/'source_statistics.json').read_text());sources['new_bag_1444']={'frames':403,'training_images':0,'scope':summary['scope'],'manifest':str(a.frames_manifest)};save(a.final/'source_statistics.json',sources)
    for name in ('summary.json','NEW_BAG_REPORT.txt','five_reviewed_roi_comparison.jpg'):
        import shutil
        shutil.copy2(a.output/name,a.final/('new_bag_1444_'+name))
    with (a.final/'TRAINING_REPORT.txt').open('a',encoding='utf-8') as f:f.write('\n\n'+'\n'.join(report)+'\n')
    handoff=json.loads((a.final/'handoff.json').read_text());handoff['new_bag_summary']=str(a.final/'new_bag_1444_summary.json');handoff['held_out_inference_frames_total']=1208;save(a.final/'handoff.json',handoff)
    archive=a.final.parent/'new_video_v8_training_handoff_with_newbag_20261011.zip'
    if archive.exists():raise ValueError('Output archive exists')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(a.final.iterdir()):
            if path.is_file():z.write(path,path.name)
    print(json.dumps({'comparisons':comparisons,'archive':str(archive)},indent=2),flush=True)


if __name__=='__main__':main()
