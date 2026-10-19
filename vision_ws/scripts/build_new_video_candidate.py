#!/usr/bin/env python3
"""Build a new reviewed train-only integration, leaving historical data untouched."""
import argparse
import collections
import json
import math
from pathlib import Path
import cv2
import numpy as np
import yaml

NAMES={0:'bridge',1:'panzer',2:'pillbox',3:'tent',4:'red_cross'}


def labels(objects,w,h):
    rows=[]
    for obj in objects:
        x1,y1,x2,y2=obj['xyxy']
        if not (0<=x1<x2<=w and 0<=y1<y2<=h):raise ValueError('Invalid box '+str(obj))
        rows.append([list(NAMES.values()).index(obj['class_name']),(x1+x2)/2/w,(y1+y2)/2/h,(x2-x1)/w,(y2-y1)/h])
    return rows


def write_label(path,rows):
    path.write_text(''.join(str(r[0])+' '+' '.join(f'{v:.8f}' for v in r[1:])+'\n' for r in rows))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--review',type=Path,required=True)
    p.add_argument('--decisions',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
    m=json.loads(a.review.read_text()); d=json.loads(a.decisions.read_text())
    class_by_id={bid:cls for cls,ids in d['classes'].items() for bid in ids.split()}
    cfg=yaml.safe_load((a.base/'data_with_scale.yaml').read_text())
    if cfg['names']!=NAMES:raise ValueError('Required five-class mapping differs')
    for split in ('train','val','test'):
        for kind in ('images','labels'):(a.output/kind/split).mkdir(parents=True)
    added=[]; excluded=[]; tiles=[]
    (a.output/'review').mkdir()
    for ordinal,s in enumerate(m['samples'],1):
        if ordinal in d['excluded_frame_ordinals']:
            excluded.append({**s,'exclusion_reason':d['excluded_reason']});continue
        objects=[]
        for b in s['objects']:
            if b['box_id'] in d['removed_boxes']:continue
            if b['box_id'] not in class_by_id:raise ValueError('Unreviewed box '+b['box_id'])
            objects.append({**b,'class_name':class_by_id[b['box_id']],'review_status':'approved_by_visual_review'})
        objects+=d['added_boxes'].get(f'{ordinal:03d}',[])
        im=cv2.imread(s['image']); h,w=im.shape[:2]
        rows=labels(objects,w,h); name=Path(s['image']).name
        (a.output/'images/train'/name).symlink_to(Path(s['image']).resolve())
        write_label(a.output/'labels/train'/Path(name).with_suffix('.txt').name,rows)
        added.append({**s,'image':str(a.output/'images/train'/name),'objects':objects,
                      'review_status':'approved','augmentation':None,'labels':rows})
        # One spatial and one mild photometric variant per source, applied equally to all classes.
        # Shrink retains the original portrait aspect and all visible target annotations.
        nh,nw=round(h*.64),round(w*.64); top,left=(h-nh)//2,(w-nw)//2
        fill=np.rint(im.mean(axis=(0,1))).astype(np.uint8); canvas=np.empty_like(im);canvas[:]=fill
        canvas[top:top+nh,left:left+nw]=cv2.resize(im,(nw,nh),interpolation=cv2.INTER_AREA)
        scaled=[{**b,'xyxy':[b['xyxy'][0]*nw/w+left,b['xyxy'][1]*nh/h+top,
                           b['xyxy'][2]*nw/w+left,b['xyxy'][3]*nh/h+top]} for b in objects]
        scale_name=Path(name).stem+'_shrink064.jpg'
        cv2.imwrite(str(a.output/'images/train'/scale_name),canvas,[cv2.IMWRITE_JPEG_QUALITY,95])
        scale_rows=labels(scaled,w,h);write_label(a.output/'labels/train'/Path(scale_name).with_suffix('.txt').name,scale_rows)
        added.append({**s,'image':str(a.output/'images/train'/scale_name),'objects':scaled,'labels':scale_rows,
                      'review_status':'approved_transform','augmentation':{'type':'shrink_mean_pad','scale_xy':[nw/w,nh/h]},'parent_image':s['image']})
        contrast=.88 if ordinal%2 else 1.10
        brightness=-8 if ordinal%2 else 6
        wb=np.array([1.04,1.0,.96] if ordinal%2 else [.97,1.0,1.03],np.float32)
        photo=np.clip((im.astype(np.float32)-127.5)*contrast+127.5+brightness,0,255)
        photo=np.clip(photo*wb,0,255).astype(np.uint8)
        photo_name=Path(name).stem+'_photo.jpg'
        cv2.imwrite(str(a.output/'images/train'/photo_name),photo,[cv2.IMWRITE_JPEG_QUALITY,95])
        write_label(a.output/'labels/train'/Path(photo_name).with_suffix('.txt').name,rows)
        added.append({**s,'image':str(a.output/'images/train'/photo_name),'objects':objects,'labels':rows,
                      'review_status':'approved_transform','augmentation':{'type':'mild_contrast_exposure_white_balance','contrast':contrast,'brightness':brightness,'wb_bgr':wb.tolist()},'parent_image':s['image']})
        overlay=im.copy()
        for b in objects:
            x1,y1,x2,y2=[round(v) for v in b['xyxy']]
            cv2.rectangle(overlay,(x1,y1),(x2,y2),(0,255,0),3)
            cv2.putText(overlay,b['class_name'],(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),1)
        cv2.imwrite(str(a.output/'review'/name),overlay)
        tile=np.full((525,320,3),25,np.uint8); z=min(320/w,480/h)
        thumb=cv2.resize(overlay,(round(w*z),round(h*z)));th,tw=thumb.shape[:2]
        tile[:th,(320-tw)//2:(320-tw)//2+tw]=thumb
        cv2.putText(tile,f'{ordinal:03d} {s["time_s"]:.2f}s labels={len(objects)}',(5,504),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
        tiles.append(tile)
    # Inherit v7 exactly once, including its existing 58 scale variants.
    inherited=[]
    for split in ('train','val','test'):
        directories=[a.base/'images'/split]
        if split=='train':directories.append(a.base/'scale_variant/images/train')
        for folder in directories:
            for f in sorted(folder.glob('*')):
                if not f.is_file():continue
                src_label=folder.parent.parent/'labels'/split/(f.stem+'.txt')
                if not src_label.is_file():raise ValueError('Missing label '+str(src_label))
                dest=a.output/'images'/split/f.name
                if dest.exists():raise ValueError('Duplicate image name '+str(dest))
                dest.symlink_to(f.resolve())
                (a.output/'labels'/split/src_label.name).symlink_to(src_label.resolve())
                inherited.append({'image':str(dest),'source':str(f),'label':str(src_label),'split':split})
    counts={}
    for split in ('train','val','test'):
        c=collections.Counter()
        for f in (a.output/'labels'/split).glob('*.txt'):
            c['images']+=1; lines=[line.split() for line in f.read_text().splitlines() if line.strip()]
            if not lines:c['negative_images']+=1
            for line in lines:
                if len(line)!=5 or int(line[0]) not in NAMES:raise ValueError('Invalid inherited label '+str(f))
                for val in line[1:]:
                    if not 0<=float(val)<=1:raise ValueError('Invalid normalized box')
                c[NAMES[int(line[0])]]+=1
        counts[split]=dict(c)
    data={'path':str(a.output),'train':'images/train','val':'images/val','test':'images/test','nc':5,'names':NAMES}
    (a.output/'data.yaml').write_text(yaml.safe_dump(data,sort_keys=False))
    manifest={'base':str(a.base),'names':NAMES,'label_scope':d['scope'],'source_video':m['source_video'],
      'new_unique_source_frames':len(tiles),'new_offline_augmented_images':len(added)-len(tiles),
      'new_labels_by_class':dict(collections.Counter(b['class_name'] for s in added if not s['augmentation'] for b in s['objects'])),
      'excluded_frames':excluded,'added_samples':added,'inherited':inherited,'counts':counts,
      'policy':'New video and all variants train only. Legacy val/test are historical regression with known train-val source overlap; 805 bag images remain held-out regression/hard cases, repeatedly reviewed, not blind test.',
      'review_decisions':d,'H_negatives':'Keep all inherited H examples: train 5 sources +75 light variants; test 6. No real H and none in new video.'}
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for start in range(0,len(tiles),12):
        subset=tiles[start:start+12]; canvas=np.full((math.ceil(len(subset)/4)*525,1280,3),25,np.uint8)
        for k,t in enumerate(subset):canvas[(k//4)*525:(k//4+1)*525,(k%4)*320:(k%4+1)*320]=t
        cv2.imwrite(str(a.output/'review'/f'approved_contact_{start//12+1:02d}.jpg'),canvas)
    print(json.dumps({k:v for k,v in manifest.items() if k in ('counts','new_unique_source_frames','new_offline_augmented_images','new_labels_by_class','label_scope')},indent=2))


if __name__=='__main__':main()
