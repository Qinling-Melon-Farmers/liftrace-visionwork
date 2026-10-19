#!/usr/bin/env python3
"""Import reviewed single-pillbox video proposals; never runs inference or training.

Uses existing 2-fps frame/prediction manifests. Human class is always pillbox(2).
Output must be new; use --overrides to repeat import with reviewed xyxy corrections.
"""
import argparse
import html
import json
import math
import shutil
from pathlib import Path
import cv2
import numpy as np
import yaml

NAMES = {0: 'bridge', 1: 'panzer', 2: 'pillbox', 3: 'tent', 4: 'red_cross'}


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def draw(im, box):
    result = im.copy()
    x1, y1, x2, y2 = [round(v) for v in box]
    cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 3)
    return result


def tile(im, text, crop=False, box=None):
    if crop:
        h, w = im.shape[:2]
        x1, y1, x2, y2 = box
        pad = int(max(x2-x1, y2-y1)*.18)
        im = im[max(0, int(y1)-pad):min(h, int(y2)+pad), max(0, int(x1)-pad):min(w, int(x2)+pad)]
    h, w = im.shape[:2]
    scale = min(480/w, 270/h)
    resized = cv2.resize(im, (max(1, round(w*scale)), max(1, round(h*scale))))
    result = np.full((310, 480, 3), 25, dtype=np.uint8)
    th, tw = resized.shape[:2]
    result[(270-th)//2:(270-th)//2+th, (480-tw)//2:(480-tw)//2+tw] = resized
    cv2.putText(result, text[:65], (5, 289), cv2.FONT_HERSHEY_SIMPLEX, .47, (255,255,255), 1)
    cv2.putText(result, 'LABEL pillbox(2) | center black circle', (5, 306), cv2.FONT_HERSHEY_SIMPLEX, .41, (0,255,0), 1)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', required=True, type=Path)
    p.add_argument('--predictions', required=True, type=Path)
    p.add_argument('--frames-manifest', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--with-scale-variant', action='store_true', help='Also generate optional 0.64 whole-frame shrink data YAML')
    p.add_argument('--overrides', type=Path, help='JSON mapping source frame basename -> reviewed xyxy')
    a = p.parse_args()
    a.base = a.base.resolve(); a.output = a.output.absolute()
    if a.output.exists():
        p.error('Output already exists; use a new directory. Existing data is never overwritten.')
    if a.output == a.base or a.base in a.output.parents:
        p.error('Output must be outside the immutable base dataset.')
    config = yaml.safe_load((a.base/'data.yaml').read_text())
    names = config['names']
    if isinstance(names, list): names = dict(enumerate(names))
    if names != NAMES: p.error('Base class mapping differs from required five-class mapping.')
    inputs = json.loads(a.frames_manifest.read_text())
    predictions = json.loads(a.predictions.read_text())
    rows = predictions['frames']
    lookup = {Path(s['path']).name: s for s in inputs['samples']}
    if len(rows) != len(lookup) or len({r['frame'] for r in rows}) != len(rows):
        p.error('Frame manifest / prediction count mismatch or duplicate frame names.')
    overrides = json.loads(a.overrides.read_text()) if a.overrides else {}
    if set(overrides)-set(lookup): p.error('Unknown override frame names.')
    for r in rows:
        source = Path(lookup[r['frame']]['path'])
        if not source.is_file() or Path(r['source']).resolve() != source.resolve() or not r['boxes']:
            p.error('Missing source, mismatched prediction source or missing box: '+r['frame'])
    a.output.mkdir(parents=True)
    review = a.output/'review'; review.mkdir()
    (review/'frames').mkdir()
    inherited = []
    counts = {}
    for split in ('train', 'val', 'test'):
        for kind, pattern in [('images', '*'), ('labels', '*.txt')]:
            dst = a.output/kind/split; dst.mkdir(parents=True)
            src = a.base/kind/split
            files = sorted(f for f in src.glob(pattern) if f.is_file())
            for f in files:
                if kind == 'labels':
                    for line in f.read_text().splitlines():
                        if line.strip() and int(line.split()[0]) not in NAMES:
                            raise ValueError('Unexpected base class: '+str(f))
                (dst/f.name).symlink_to(f.absolute())
                inherited.append({'split':split, 'kind':kind, 'path':str(dst/f.name), 'source':str(f), 'reuse':'symlink'})
            counts[split+'_'+kind] = len(files)
        if counts[split+'_images'] != counts[split+'_labels']:
            raise ValueError('Base image/label counts differ for '+split)
    (a.output/'base_manifest.json').symlink_to(a.base/'manifest.json')
    shutil.copyfile(a.predictions, review/'baseline_predictions.json')
    shutil.copyfile(a.frames_manifest, review/'input_frames_manifest.json')
    if a.overrides: shutil.copyfile(a.overrides, review/'reviewed_overrides.json')
    samples=[]; pages=[]; crop_pages=[]; full_tiles=[]; crop_tiles=[]; table=[]
    for i, r in enumerate(rows):
        source = Path(lookup[r['frame']]['path']); im = cv2.imread(str(source))
        h,w = im.shape[:2]
        if [h,w] != r['image_shape_hw']: raise ValueError('Image dimensions changed: '+str(source))
        proposal = max(r['boxes'], key=lambda b:b['confidence'])
        box = [float(v) for v in overrides.get(r['frame'], proposal['xyxy'])]
        x1,y1,x2,y2 = box
        if not (0<=x1<x2<=w and 0<=y1<y2<=h): raise ValueError('Invalid box '+r['frame'])
        name = 'pillbox_20261011_'+source.name
        image = a.output/'images/train'/name
        shutil.copyfile(source, image)
        normalized = [(x1+x2)/(2*w),(y1+y2)/(2*h),(x2-x1)/w,(y2-y1)/h]
        label = a.output/'labels/train'/Path(name).with_suffix('.txt').name
        label.write_text('2 '+' '.join(format(v,'.8f') for v in normalized)+'\n')
        annotated = draw(im, box)
        cv2.imwrite(str(review/'frames'/name), annotated)
        t=lookup[r['frame']]['requested_time_s']
        top=proposal.get('semantic_name',proposal.get('raw_name','unknown'))
        title=f'{i+1:02d} t={t:05.2f}s baseline={top} {proposal["confidence"]:.2f}'
        full_tiles.append(tile(annotated,title)); crop_tiles.append(tile(annotated,title,True,box))
        border = min(x1,y1,w-x2,h-y2)<=1
        samples.append({**lookup[r['frame']], 'source_video':inputs['source_video'], 'group':'pillbox_video_20261011', 'split':'train', 'image':str(image),'label':str(label), 'class_id':2, 'class_name':'pillbox', 'class_authority':'user confirmed entire video single real pillbox', 'box_scope':'center black circle and internal graphic; exclude outer blue/white rings and board', 'xyxy':box, 'normalized_xywh':normalized, 'box_source':'reviewed_override' if r['frame'] in overrides else 'cached_0928_top_confidence_localization_only', 'baseline_top1':top,'baseline_confidence':proposal['confidence'], 'all_baseline_boxes':r['boxes'],'touches_image_border':border, 'review_status':'pending_full_frame_review'})
        table.append(f'<tr><td>{html.escape(title)}<br>pillbox id 2<br>{"BORDER / clipped" if border else ""}</td><td><a href="frames/{name}"><img width="640" src="frames/{name}"></a></td></tr>')
    for start in range(0,len(rows),15):
        for tiles,stem,collection in [(full_tiles,'contact',pages),(crop_tiles,'center_contact',crop_pages)]:
            subset=tiles[start:start+15]
            canvas=np.full((math.ceil(len(subset)/3)*310,1440,3),25,np.uint8)
            for j,im in enumerate(subset): canvas[(j//3)*310:(j//3+1)*310,(j%3)*480:(j%3+1)*480]=im
            name=f'{stem}_{start//15+1:02d}.jpg'; cv2.imwrite(str(review/name),canvas); collection.append(name)
    data={'path':str(a.output),'train':'images/train','val':'images/val','test':'images/test','nc':5,'names':NAMES}
    (a.output/'data.yaml').write_text('# Candidate annotations: full-frame human review pending. Import does not train.\n'+yaml.safe_dump(data,sort_keys=False))
    manifest={'status':'prepared_pending_full_annotation_review', 'base_dataset':str(a.base), 'base_manifest':str(a.base/'manifest.json'), 'predictions':str(a.predictions),'source_video':inputs['source_video'],'video_fps':inputs['fps'],'video_frames':inputs['declared_frames'],'sample_hz':inputs['sample_hz'],'class_mapping':NAMES, 'policy':'Entire new video and every derived augmentation are train only; val/test inherit base unchanged. Same-video before/after is diagnostic, not generalization.', 'inherited_counts':counts, 'inherited_files':inherited,'added_samples':samples, 'added_count':len(samples), 'offline_augmentation_count':0,'contact_pages':pages,'center_contact_pages':crop_pages,'training_started':False}
    dump(a.output/'manifest.json',manifest)
    dump(review/'review_status.json',{'status':'pending','review_scope':'All 58 imported source frames; earlier 6 preview approvals apply to earlier preview images only','class_id':2,'convention':'center black circle','training_started':False})
    (review/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Pillbox v7 annotation review</title><style>body{font-family:sans-serif;background:#eee}td{padding:8px}img{max-width:100%}</style><h1>Pillbox v7: all '+str(len(rows))+' frames, label 2</h1><p>Center black circle only. All frames train only. Full annotation review pending; no training started.</p><p>'+ ' '.join(f'<a href="{n}">{n}</a>' for n in pages+crop_pages)+'</p><table>'+''.join(table)+'</table>',encoding='utf-8')
    if a.with_scale_variant: add_scale_variant(a.output)
    print(json.dumps({'output':str(a.output),'added':len(samples),'inherited':counts,'contact':str(review/'index.html')},indent=2),flush=True)


def add_scale_variant(root, scale=.64):
    """Optional train-only whole-frame shrink; default data.yaml remains untouched."""
    manifest = json.loads((root/'manifest.json').read_text())
    dst = root/'scale_variant'
    if dst.exists(): raise ValueError('Scale variant already exists; never overwrite it.')
    (dst/'images/train').mkdir(parents=True)
    (dst/'labels/train').mkdir(parents=True)
    variants=[]; tiles=[]
    for i, sample in enumerate(manifest['added_samples']):
        im=cv2.imread(sample['image']); h,w=im.shape[:2]
        nw,nh=round(w*scale),round(h*scale); sx,sy=nw/w,nh/h
        left,top=(w-nw)//2,(h-nh)//2
        fill=np.rint(im.mean(axis=(0,1))).astype(np.uint8)
        canvas=np.empty_like(im); canvas[:]=fill
        canvas[top:top+nh,left:left+nw]=cv2.resize(im,(nw,nh),interpolation=cv2.INTER_AREA)
        x1,y1,x2,y2=sample['xyxy']; box=[x1*sx+left,y1*sy+top,x2*sx+left,y2*sy+top]
        xx1,yy1,xx2,yy2=box
        normalized=[(xx1+xx2)/(2*w),(yy1+yy2)/(2*h),(xx2-xx1)/w,(yy2-yy1)/h]
        name=Path(sample['image']).stem+'_shrink064.jpg'
        image=dst/'images/train'/name; label=dst/'labels/train'/Path(name).with_suffix('.txt').name
        cv2.imwrite(str(image),canvas,[cv2.IMWRITE_JPEG_QUALITY,95])
        label.write_text('2 '+' '.join(format(v,'.8f') for v in normalized)+'\n')
        annotated=draw(canvas,box)
        cv2.imwrite(str(root/'review/frames'/name),annotated)
        tiles.append(tile(annotated,f'{i+1:02d} t={sample["requested_time_s"]:.2f}s shrink064 mean fill'))
        variants.append({'image':str(image),'label':str(label),'parent_image':sample['image'],'source_video':manifest['source_video'],'group':sample['group'],'split':'train','class_id':2,'xyxy':box,'normalized_xywh':normalized,'augmentation':{'type':'whole_frame_shrink_mean_pad','requested_scale':scale,'actual_scale_xy':[sx,sy],'resized_wh':[nw,nh],'translation_xy':[left,top],'fill_bgr':fill.tolist(),'crop':False}})
    pages=[]
    for start in range(0,len(tiles),15):
        subset=tiles[start:start+15]; sheet=np.full((math.ceil(len(subset)/3)*310,1440,3),25,np.uint8)
        for j,tile_im in enumerate(subset): sheet[(j//3)*310:(j//3+1)*310,(j%3)*480:(j%3+1)*480]=tile_im
        name=f'scale_contact_{start//15+1:02d}.jpg'; cv2.imwrite(str(root/'review'/name),sheet); pages.append(name)
    config=yaml.safe_load((root/'data.yaml').read_text()); config['train']=['images/train','scale_variant/images/train']
    (root/'data_with_scale.yaml').write_text('# Optional train-only 0.64 whole-frame shrink; review inherited originals first.\n'+yaml.safe_dump(config,sort_keys=False))
    dump(root/'scale_variant_manifest.json',{'added_count':len(variants),'variants':variants,'contact_pages':pages,'default_data_yaml_unchanged':True})
    manifest['optional_scale_variant']={'manifest':str(root/'scale_variant_manifest.json'),'data_yaml':str(root/'data_with_scale.yaml'),'count':len(variants),'scale':scale,'split':'train'}
    dump(root/'manifest.json',manifest)
    review=root/'review/index.html'; content=review.read_text()
    content=content.replace('<table>','<p>Optional whole-frame shrink / mean pad: '+' '.join(f'<a href="{n}">{n}</a>' for n in pages)+'</p><table>',1); review.write_text(content)


if __name__=='__main__': main()
