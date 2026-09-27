"""Build an isolated five-class dataset; never edit the July source dataset."""
import argparse, collections, json, os
from pathlib import Path
import cv2
import numpy as np
import yaml
NAMES = ['bridge', 'panzer', 'pillbox', 'tent', 'red_cross']
REMAP = {0: 0, 1: 1, 2: 2, 3: 3, 5: 4}

def lighting(image, rng, variant):
    # Geometry-preserving camera exposure/contrast, gamma, spatial shadow.
    a = float(rng.uniform(.6, 1.4)); b = float(rng.uniform(-22, 22))
    gamma = float(rng.uniform(.65, 1.6))
    x = np.clip((image.astype(np.float32)-127.5)*a+127.5+b, 0, 255)/255.
    x = np.power(x, gamma)
    shadow = 1.
    if variant % 2:
        xx = np.linspace(0, 1, image.shape[1])[None, :, None]
        if rng.random() < .5: xx = 1-xx
        shadow = float(rng.uniform(.3, .7)) + xx * .4
        x *= shadow
    return np.clip(x*255, 0, 255).astype(np.uint8), dict(contrast=a, brightness=b, gamma=gamma, spatial_shadow=bool(variant%2))

def main():
    p=argparse.ArgumentParser();p.add_argument('--legacy',type=Path,required=True);p.add_argument('--annotations',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=928);a=p.parse_args()
    if a.output.exists(): raise SystemExit('Output exists: choose a fresh directory; no destructive rebuild.')
    old=yaml.safe_load((a.legacy/'data.yaml').read_text());assert list(old['names'].values())==NAMES[:4]+['tank','red_cross']
    cv2.setNumThreads(2);rng=np.random.default_rng(a.seed);a.output.mkdir(parents=True)
    manifest=[];counts=collections.Counter();excluded=[]
    def put(source, name, split, labels, group, kind, variants=0):
        source=Path(source);image=cv2.imread(str(source));assert image is not None,source
        for sub in ['images','labels']:(a.output/sub/split).mkdir(parents=True,exist_ok=True)
        target=a.output/'images'/split/(name+source.suffix.lower());os.link(source,target)
        label=''.join(str(c)+' '+' '.join(f'{v:.7f}' for v in box)+'\n' for c,*box in labels)
        (a.output/'labels'/split/(name+'.txt')).write_text(label)
        manifest.append(dict(source=str(source),image=str(target),group=group,split=split,kind=kind,labels=labels,augmentation=None))
        counts[split+':'+kind]+=1
        for i in range(variants):
            aug,params=lighting(image,rng,i);ip=a.output/'images'/split/f'{name}_light{i}.jpg';cv2.imwrite(str(ip),aug,[cv2.IMWRITE_JPEG_QUALITY,95]);(a.output/'labels'/split/f'{name}_light{i}.txt').write_text(label)
            manifest.append(dict(source=str(source),image=str(ip),group=group,split=split,kind=kind,labels=labels,augmentation=params));counts[split+':augmentation']+=1
    for split in ['train','val']:
        for lab in sorted((a.legacy/'labels'/split).glob('*.txt')):
            ls=[list(map(float,l.split())) for l in lab.read_text().splitlines() if l.strip()]
            # Drop whole tank-containing images, rather than leave an unlabeled active object.
            if any(int(l[0])==4 for l in ls):excluded.append(str(lab));continue
            labels=[[REMAP[int(l[0])],*l[1:]] for l in ls]
            im=next((a.legacy/'images'/split).glob(lab.stem+'.*'))
            put(im,'legacy_'+lab.stem,split,labels,'legacy_'+split,'legacy',int(split=='train' and rng.random()<.25))
    reviewed=json.loads(a.annotations.read_text());groups={}
    for r in reviewed['rows']:
        group=r['group'];split=r['split'];assert groups.setdefault(group,split)==split
        labels=[]
        for q in r['objects']:
            x1,y1,x2,y2=q['xyxy'];w,h=r['width'],r['height'];assert 0<=x1<x2<=w and 0<=y1<y2<=h
            assert NAMES[q['cid']]==q['cls']
            labels.append([q['cid'],(x1+x2)/2/w,(y1+y2)/2/h,(x2-x1)/w,(y2-y1)/h])
        variants=(15 if r['kind']=='sim_H_negative' else 3) if split=='train' else 0
        put(r['source'],r['name'],split,labels,group,r['kind'],variants)
    # Verify no source image or session crosses train/val/test in newly sampled flights.
    sources={}
    for r in manifest:
        assert sources.setdefault(r['source'],r['split'])==r['split']
        for lab in r['labels']:assert len(lab)==5 and 0<=lab[0]<5 and all(0<=v<=1 for v in lab[1:])
    cfg=dict(path=str(a.output.resolve()),train='images/train',val='images/val',test='images/test',nc=5,names=dict(enumerate(NAMES)))
    (a.output/'data.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (a.output/'summary.json').write_text(json.dumps(dict(counts=counts,excluded_tank_images=len(excluded),excluded_tank_labels=excluded,seed=a.seed,groups=groups),indent=2))
    print(json.dumps(dict(counts=counts,excluded_tank_images=len(excluded),groups=groups),indent=2))
if __name__=='__main__': main()
