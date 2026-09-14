"""Offline projected simulated guard-box check; no geometry enters control."""
import argparse,csv,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from check_tree_overflight import footprints,ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('world',type=Path);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    tree=ET.parse(ROOT/'vision_ws/src/uav_vision_eval/models/r2026_iris_dynamics/model.sdf')
    box=next(v for v in tree.iter('collision') if v.get('name')=='competition_guard_collision')
    size=np.fromstring(box.findtext('geometry/box/size'),sep=' ')
    offset=np.fromstring(box.findtext('pose'),sep=' ');assert np.linalg.norm(offset[3:])<1e-8
    corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size+offset[:3]
    with (args.run/'truth_pose.csv').open() as f:
        reader=csv.reader(f);header=next(reader);a=np.array([[float(v) for v in row] for row in reader])
    assert len(header)==8
    params=yaml.safe_load((args.run/'rosparams.yaml').read_text());world_offset=np.array(params['competition_key_recorder']['truth_world_offset'])
    obstacles=footprints(args.world);results={name:dict(tree=name,samples=0,overlap_samples=0,min_separating_axis_gap_m=None) for name,_,_ in obstacles}
    for first,last in zip(a,a[1:]):
        if not 0<last[0]-first[0]<=.5:continue
        n=max(1,int(np.ceil(np.linalg.norm(last[1:4]-first[1:4])/.025)))
        q0,q1=first[4:].copy(),last[4:].copy()
        if np.dot(q0,q1)<0:q1=-q1
        for u in np.linspace(0,1,n+1):
            xyz=first[1:4]+u*(last[1:4]-first[1:4])+world_offset
            if xyz[2]<1.5:continue
            q=q0*(1-u)+q1*u;q/=np.linalg.norm(q)
            verts=corners@Rotation.from_quat(q).as_matrix().T+xyz
            poly=verts[ConvexHull(verts[:,:2]).vertices,:2]
            for name,obstacle,top in obstacles:
                # Definitely an over-top case: the entire guard box is above the tree.
                if verts[:,2].min()<=top:continue
                row=results[name];row['samples']+=1
                axes=np.vstack([ConvexHull(poly).equations[:,:2],ConvexHull(obstacle).equations[:,:2]])
                pa,pb=poly@axes.T,obstacle@axes.T
                gap=float(np.maximum(pa.min(axis=0)-pb.max(axis=0),pb.min(axis=0)-pa.max(axis=0)).max())
                if gap<=0:row['overlap_samples']+=1
                if row['min_separating_axis_gap_m'] is None or gap<row['min_separating_axis_gap_m']:
                    row.update(min_separating_axis_gap_m=gap,time_s=float(first[0]+u*(last[0]-first[0])),fc_world_xyz=xyz.tolist())
    result=dict(run=str(args.run),guard_box_size=size.tolist(),guard_box_pose=offset.tolist(),scope='Rotated simulated guard box versus conservative tree+pedestal convex horizontal hull, entire box above tree, 0.025m path interpolation; not a real-airframe certification.',trees=list(results.values()))
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
