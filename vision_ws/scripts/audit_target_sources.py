#!/usr/bin/env python3
"""Count sources, inspect inherited split leakage and perceptual near duplicates."""
import argparse
import collections
import json
import re
from pathlib import Path
import cv2
import numpy as np


def family(row):
    source=Path(row['source']); name=source.stem
    # Capture/date/video sequences are a source group, not the generic legacy split.
    name=re.sub(r'_(?:light|aug)\d+$','',name)
    if re.search(r'_f\d+',name):return re.split(r'_f\d+',name)[0]
    if re.search(r'_rot\d+',name):return re.split(r'_rot\d+',name)[0]+'_rotation_source_unknown'
    if name.startswith('IMG_'):return '_'.join(name.split('_')[:2])
    if row['kind']!='legacy':return row['group']
    # Unknown image origins cannot be proven disjoint by a filename.
    return 'unresolved:'+name


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rows=json.loads((a.base/'manifest.json').read_text()); groups=collections.defaultdict(collections.Counter)
    counts=collections.defaultdict(collections.Counter); sketches=[]; sources=collections.defaultdict(list)
    for row in rows:
        groups[family(row)][row['split']]+=1; counts[row['split']]['images']+=1
        counts[row['split']]['offline_aug' if row.get('augmentation') else 'source_frames']+=1
        if not row['labels']: counts[row['split']]['negative_images']+=1
        for lab in row['labels']: counts[row['split']]['class_'+str(int(lab[0]))]+=1
        sources[row['source']].append(row)
        gray=cv2.imread(row['image'],cv2.IMREAD_GRAYSCALE)
        dct=cv2.dct(cv2.resize(gray,(32,32)).astype(np.float32))[:8,:8].ravel()
        sketch=dct[1:]>np.median(dct[1:]); sketches.append(sketch)
    sketches=np.stack(sketches); near=[]
    for i,row in enumerate(rows):
        mask=np.array([r['split']!=row['split'] for r in rows[i+1:]])
        if not mask.any():continue
        distances=np.count_nonzero(sketches[i+1:]!=sketches[i],axis=1)
        for rel in np.flatnonzero(mask & (distances<=4)):
            other=rows[i+1+int(rel)]
            near.append({'distance':int(distances[rel]),'a':row['image'],'b':other['image'],
                         'group_a':family(row),'group_b':family(other),
                         'status':'perceptual_flag_not_exact_duplicate_proof'})
    summary={'base':str(a.base),'counts':{k:dict(v) for k,v in counts.items()},
       'unique_source_paths':len(sources),'groups':{k:dict(v) for k,v in groups.items()},
       'cross_split_source_groups':{k:dict(v) for k,v in groups.items() if len(v)>1},
       'exact_source_paths_crossing_splits':[k for k,v in sources.items() if len({r['split'] for r in v})>1],
       'perceptual_cross_split_flag_count':len(near),
       'limits':'pHash flags require inspection; same acquisition/date grouping conservative; unknown legacy origins unresolved. Parent weights already saw historical data.'}
    (a.output/'audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    (a.output/'near_duplicate_flags.json').write_text(json.dumps(near,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='groups'},indent=2))


if __name__=='__main__':main()
