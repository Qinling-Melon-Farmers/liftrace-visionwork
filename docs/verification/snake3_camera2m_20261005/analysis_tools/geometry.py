from pathlib import Path
import json, math
import numpy as np
R=Path(__file__).resolve().parents[4];D=R/'docs/verification/snake3_camera2m_20261005'
xx,yy=np.meshgrid(np.arange(-.49,7.95,.02),np.arange(-4.99,5,.02));legal=(xx>=0)&(xx<=7.45)&(abs(yy)<=4.5)
old=json.loads((D.parent/'latest_six_20261005/scenes.json').read_text());new=json.loads((D/'scenes.json').read_text())
results=[]
for label,x in [('original_2.6m',next(x for x in old if x['variant']=='snake3' and x['seed']==31)),('correct_2m',new[0])]:
 w,h=3.6*x['camera_agl']/1.84,1.8*x['camera_agl']/1.84
 masks=[]
 for margin in [0,.5]:
  mask=np.zeros(xx.shape,bool)
  for a,b in zip(x['survey'],x['survey'][1:]):
   mask|=(xx>=min(a[0],b[0])-w/2+margin)&(xx<=max(a[0],b[0])+w/2-margin)&(yy>=min(a[1],b[1])-h/2+margin)&(yy<=max(a[1],b[1])+h/2-margin)
  masks.append(mask)
 results.append(dict(label=label,camera_agl=x['camera_agl'],fc_agl=x['fc_agl'],footprint=[w,h],path_m=sum(math.dist(a,b) for a,b in zip(x['survey'],x['survey'][1:])),search_area_m2=84.5,search_point_coverage=float(masks[0].mean()),legal_1m_target_center_visibility=float(masks[0][legal].mean()),legal_1m_full_board_visibility=float(masks[1][legal].mean())))
checks=[]
for a in new:
 b=next(x for x in old if x['seed']==a['seed'] and x['variant']=='snake3');src=Path(b['scene']);dst=R/a['scene']
 assert a['targets']==b['targets']
 same={f:(src/f).read_bytes()==(dst/f).read_bytes() for f in ['field.world','field_config.yaml','motion_overrides.yaml','fast_runtime.yaml','fast_gate.yaml','frame_overrides.yaml','presentation.yaml']}
 assert all(same.values());checks.append(dict(seed=a['seed'],unchanged_files=same,targets_equal=True))
(D/'geometry_comparison.json').write_text(json.dumps(dict(assumptions='Measured FOV extrapolation, nadir fixed yaw, complete ideal axis-aligned segments; no occlusion or detection probability. One-metre full board means axis-aligned square.',results=results,scene_checks=checks),indent=2))
print(json.dumps(results,indent=2))