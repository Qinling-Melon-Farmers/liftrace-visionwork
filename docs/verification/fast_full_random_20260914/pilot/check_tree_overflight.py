"""Offline geometry check only: scene geometry never feeds flight planning."""
import argparse,csv,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[4]
def mesh_points():
    root=ET.parse(ROOT/'simulation_assets/models/juniper_Tree/meshes/pine_tree.dae').getroot()
    ns={'c':root.tag.split('}')[0][1:]};geoms={}
    for g in root.findall('.//c:geometry',ns):
        rows=[]
        for s in g.findall('.//c:source',ns):
            f=s.find('c:float_array',ns)
            if f is not None and 'position' in s.get('id','').lower():rows.append(np.fromstring(f.text,sep=' ').reshape(-1,3))
        geoms[g.get('id')]=np.vstack(rows)
    out=[]
    for node in root.findall('.//c:visual_scene//c:node',ns):
        instance=node.find('c:instance_geometry',ns)
        if instance is None:continue
        transform=node.find('c:matrix',ns)
        matrix=np.fromstring(transform.text,sep=' ').reshape(4,4) if transform is not None else np.eye(4)
        points=geoms[instance.get('url')[1:]]
        out.append((np.column_stack([points,np.ones(len(points))])@matrix.T)[:,:3])
    return np.vstack(out)

def footprints(world):
    points=mesh_points();result=[]
    for model in ET.parse(world).iter('model'):
        if 'Tree' not in model.get('name',''):continue
        pose=np.fromstring(model.findtext('pose','0 0 0 0 0 0'),sep=' ')
        assert abs(pose[3])+abs(pose[4])<1e-8
        parts=[]
        for link in model.findall('link'):
            link_pose=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')
            assert np.linalg.norm(link_pose)<1e-8
            for collision in link.findall('collision'):
                cp=np.fromstring(collision.findtext('pose','0 0 0 0 0 0'),sep=' ')
                assert np.linalg.norm(cp[3:])<1e-8
                mesh=collision.find('geometry/mesh');box=collision.find('geometry/box')
                if mesh is not None:
                    local=points*np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ')
                elif box is not None:
                    size=np.fromstring(box.findtext('size'),sep=' ')
                    local=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
                else:continue
                parts.append(local+cp[:3])
        local=np.vstack(parts);c,s=np.cos(pose[5]),np.sin(pose[5]);rotation=np.array([[c,-s],[s,c]])
        xy=local[:,:2]@rotation.T+pose[:2];hull=ConvexHull(xy);poly=xy[hull.vertices]
        result.append((model.get('name'),poly,float(local[:,2].max()+pose[2])))
    assert len(result)==4
    return result

def signed_distance(xy,poly):
    hull=ConvexHull(poly);inside=np.all(xy@hull.equations[:,:2].T+hull.equations[:,2]<=1e-9,axis=1)
    distance=np.full(len(xy),np.inf)
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        edge=b-a;t=np.clip((xy-a)@edge/(edge@edge),0,1)
        distance=np.minimum(distance,np.linalg.norm(xy-(a+t[:,None]*edge),axis=1))
    return np.where(inside,-distance,distance)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('world',type=Path);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    with (args.run/'truth_pose.csv').open() as f:a=np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])
    dense=[]
    for p,q in zip(a,a[1:]):
        if not 0<q[0]-p[0]<=.5:continue
        n=max(1,int(np.ceil(np.linalg.norm(q[1:]-p[1:])/.025)))
        dense.extend(p+(q-p)*v for v in np.linspace(0,1,n+1))
    dense=np.array(dense);rows=[]
    params=yaml.safe_load((args.run/'rosparams.yaml').read_text())
    fc_offset=float(params['competition_key_recorder']['truth_world_offset'][2])
    dense[:,3]+=fc_offset
    for name,poly,top in footprints(args.world):
        above=dense[dense[:,3]>top]
        distance=signed_distance(above[:,1:3],poly)
        index=int(np.argmin(distance))
        rows.append(dict(tree=name,tree_top_world_z=top,footprint=poly.tolist(),
                         center_above_footprint_samples=int(np.count_nonzero(distance<0)),
                         min_center_horizontal_clearance_m=float(distance[index]),
                         min_clearance_time_s=float(above[index,0]),
                         nominal_0_275m_radius_overlap_samples=int(np.count_nonzero(distance<.275))))
    result=dict(run=str(args.run),fc_world_z_offset_m=fc_offset,scope='Offline collision-mesh convex horizontal hull including pedestal; FC above tree top; 0.025m segment sampling. Nominal radius is not a full swept-body certification.',trees=rows)
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2));print(json.dumps([{k:v for k,v in row.items() if k!='footprint'} for row in rows],indent=2))
if __name__=='__main__':main()
