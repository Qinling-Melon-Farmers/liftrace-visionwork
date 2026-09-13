"""Diagnostic blue-ring ellipse versus recorded CameraInfo projection; no calibration changes."""
import json
from pathlib import Path
import cv2
import numpy as np

root=Path(__file__).resolve().parent
rows=[json.loads(line) for line in (root/'results/predictions.jsonl').read_text().splitlines()]
checks=[]
for row in rows:
    if row['frame']!=0:continue
    image=cv2.imread(row['image'])
    mask=cv2.inRange(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),np.array([85,80,50]),np.array([130,255,255]))
    for t in row['targets']:
        if not t['fully_in_frame'] or t['class_name']=='red_cross':continue
        poly=np.array(t['polygon']);center=np.array(t['center_uv'])
        low=np.maximum(np.floor(poly.min(axis=0)-55),[0,0]).astype(int)
        high=np.minimum(np.ceil(poly.max(axis=0)+55),[image.shape[1]-1,image.shape[0]-1]).astype(int)
        yy,xx=np.nonzero(mask[low[1]:high[1]+1,low[0]:high[0]+1])
        if len(xx)<500:continue
        points=np.ascontiguousarray(np.c_[xx+low[0],yy+low[1]][::5],dtype=np.float32)
        ellipse=cv2.fitEllipse(points)
        if min(ellipse[1])/max(ellipse[1])<.85:continue
        checks.append(dict(seed=row['seed'],fc_agl=row['fc_agl'],view=row['view'],class_name=t['class_name'],
            projected=center.tolist(),blue_ellipse_center=list(ellipse[0]),
            delta_px=(np.array(ellipse[0])-center).tolist()))
summary=[]
for h in sorted({c['fc_agl'] for c in checks}):
    values=np.array([c['delta_px'] for c in checks if c['fc_agl']==h])
    summary.append(dict(fc_agl=h,views=len(values),median_delta_px=np.median(values,axis=0).tolist()))
report=dict(scope='DIAGNOSTIC_NOT_CALIBRATION_ACCEPTANCE',
    method='blue pixels in 55px-padded projected board ROI, ellipse fit, axis ratio >= .85; no correction applied',
    summary=summary,views=checks)
(root/'results/projection_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(summary,indent=2))
