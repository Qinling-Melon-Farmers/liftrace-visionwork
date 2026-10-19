import importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull,distance
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--research-root',type=Path,required=True);args=parser.parse_args();R=args.research_root
p=R/'docs/verification/fast_full_random_20260914/pilot/check_tree_overflight.py'
spec=importlib.util.spec_from_file_location('tree_geometry',p);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
mesh=mod.mesh_points()*np.array([.2,.2,.236322445576211])
def dims(points):
 hull=points[ConvexHull(points[:,:2]).vertices,:2]
 return dict(min=points.min(axis=0).tolist(),max=points.max(axis=0).tolist(),span=np.ptp(points,axis=0).tolist(),max_horizontal_span=float(distance.pdist(hull).max()),horizontal_hull=hull.tolist())
box=np.array([[x,y,z] for x in (-.3,.3) for y in (-.3,.3) for z in (-.3,0.)])
both=np.vstack([mesh,box]);data=dict(mesh=dims(mesh),with_pedestal=dims(both),pedestal_size=[.6,.6,.3],world_tree_base_z=.3)
data['axis_widths_inflated']={str(margin):[float(v+2*margin) for v in np.ptp(both[:,:2],axis=0)] for margin in (.1,.3,.4)}
Path(__file__).resolve().with_name('tree_dimensions.json').write_text(json.dumps(data,indent=2));print(json.dumps({k:v for k,v in data.items() if k not in ('mesh','with_pedestal')},indent=2));print('mesh',data['mesh']['span'],'with pedestal',data['with_pedestal']['span'],'diameter',data['with_pedestal']['max_horizontal_span'])
