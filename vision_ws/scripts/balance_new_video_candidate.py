#!/usr/bin/env python3
"""Apply recorded mild photometric train-only balancing of bridge/red_cross."""
import argparse
import collections
import json
import math
from pathlib import Path
import cv2
import numpy as np
from build_new_video_candidate import NAMES, write_label


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--legacy-manifest',type=Path,required=True)
    p.add_argument('--decisions',type=Path,required=True)
    a=p.parse_args(); root=a.dataset; m=json.loads((root/'manifest.json').read_text())
    if (root/'balance_manifest.json').exists():raise ValueError('Balancing already applied')
    m['review_decisions']=json.loads(a.decisions.read_text())
    source_rows=[r for r in json.loads(a.legacy_manifest.read_text())
                 if r['split']=='train' and not r.get('augmentation') and '_rot' not in Path(r['image']).stem]
    source_rows+=[r for r in m['added_samples'] if not r['augmentation']]
    pools={c:[r for r in source_rows if any(int(l[0])==cid for l in r['labels'])]
           for cid,c in NAMES.items() if c in ('bridge','red_cross')}
    rng=np.random.default_rng(1011)
    for pool in pools.values():rng.shuffle(pool)
    counts=m['counts']['train']; positions=collections.Counter(); variants=[]
    while True:
        target=math.ceil(max(counts[c] for c in NAMES.values())/2)
        deficits={c:target-counts[c] for c in pools}
        cls=max(deficits,key=deficits.get)
        if deficits[cls]<=0:break
        if len(variants)>400:raise ValueError('Balance limit exceeded')
        source=pools[cls][positions[cls]%len(pools[cls])];positions[cls]+=1
        im=cv2.imread(source['image'])
        contrast=float(rng.uniform(.86,1.14)); brightness=float(rng.uniform(-8,8))
        wb=rng.uniform(.97,1.03,3).astype(np.float32)
        out=np.clip(((im.astype(np.float32)-127.5)*contrast+127.5+brightness)*wb,0,255).astype(np.uint8)
        name=f'balanced_{cls}_{positions[cls]:04d}.jpg'; dst=root/'images/train'/name
        if dst.exists():raise ValueError('Existing destination '+str(dst))
        cv2.imwrite(str(dst),out,[cv2.IMWRITE_JPEG_QUALITY,95])
        write_label(root/'labels/train'/Path(name).with_suffix('.txt').name,source['labels'])
        for lab in source['labels']:counts[NAMES[int(lab[0])]]+=1
        counts['images']+=1
        variants.append({'image':str(dst),'source':source['image'],'source_group':source.get('group',source.get('source_group')),
                         'split':'train','labels':source['labels'],'balance_class':cls,
                         'augmentation':{'type':'mild_contrast_exposure_white_balance','contrast':contrast,'brightness':brightness,'wb_bgr':wb.tolist()}})
    balance={'seed':1011,'policy':'Augment bridge/red_cross source images only until maximum/minimum instance count <=2; unchanged historical regression splits. Augmentation is artificial balancing, not new independent data.',
             'counts_by_balance_class':dict(positions),'variants':variants,'counts_after':counts}
    (root/'balance_manifest.json').write_text(json.dumps(balance,indent=2)+'\n')
    m['balancing']={'manifest':str(root/'balance_manifest.json'),'count':len(variants),'counts_by_balance_class':dict(positions)}
    (root/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    print(json.dumps({'balancing_count':len(variants),'counts_by_balance_class':dict(positions),'final_counts':m['counts']},indent=2))


if __name__=='__main__':main()
