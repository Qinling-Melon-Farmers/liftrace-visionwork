from pathlib import Path
import json,csv,importlib.util
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
D=Path(__file__).resolve().parent;spec=importlib.util.spec_from_file_location('analysis',D/'analyze_current.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
item=next(v for v in json.loads((D/'runs.json').read_text()) if v['seed']==38);m=next(v for v in json.loads((D/'metrics.json').read_text()) if v['seed']==38);run=Path(item['run'])
with (run/'truth_pose.csv').open() as f:a=np.array([[float(v[k]) for k in ('t','x','y','z','qx','qy','qz','qw')] for v in csv.DictReader(f)])
corners=np.array([[x,y,z] for x in (-.275,.275) for y in (-.275,.275) for z in (-.22,.18)]);v=np.einsum('tij,kj->tki',Rotation.from_quat(a[:,4:]).as_matrix(),corners)+a[:,None,1:4]
clearance=np.minimum.reduce([v[:,:,0].min(axis=1)+.5,7.4-v[:,:,0].max(axis=1),v[:,:,1].min(axis=1)+4.8,4.8-v[:,:,1].max(axis=1)])
end=m['gate_terminal_ros_s'] or m['observed_end_ros_s'];index=int(np.argmin(abs(a[:,0]-end)))
fig,axes=plt.subplots(1,2,figsize=(13,5));ax=axes[0];mod.scene(ax,item['world'],yaml.safe_load((run/'random_field_truth.yaml').read_text()))
for text in ax.texts:text.set_clip_on(True)
near=abs(a[:,0]-end)<4;ax.plot(a[near,1],a[near,2],lw=1.5,label='True FC track near Gate stop')
poly=v[index,ConvexHull(v[index,:,:2]).vertices,:2];ax.add_patch(Polygon(poly,fill=False,edgecolor='red',lw=2,label='55cm guard projection, nearest recorded pose'))
ax.add_patch(Rectangle((-.5,-4.8),7.9,9.6,fill=False,ls='--',edgecolor='orange',lw=1.5,label='Search inner bounds'))
ax.set_xlim(a[index,1]-1,a[index,1]+1);ax.set_ylim(a[index,2]-1,a[index,2]+1);ax.legend(fontsize=7)
axes[1].plot(a[near,0]-m['start_ros_s'],clearance[near]*100);axes[1].axhline(0,color='red',ls='--');axes[1].axvline(end-m['start_ros_s'],color='black',ls=':',label='Gate termination');axes[1].set(xlabel='Mission seconds',ylabel='Whole-body inner clearance (cm)');axes[1].legend();axes[1].grid(alpha=.2)
fig.suptitle('Seed38 search-boundary stop | actual collision count remains separately reported\nCSV 10Hz; nearest frame may differ from the higher-rate Gate event')
fig.tight_layout();fig.savefig(D/'seed38_boundary.png',dpi=160);plt.close(fig)
(D/'boundary38_snapshot.json').write_text(json.dumps(dict(gate_ros_s=end,nearest_pose_ros_s=float(a[index,0]),nearest_fc_xyz=a[index,1:4].tolist(),nearest_clearance_cm=float(clearance[index]*100),minimum_recorded_clearance_cm=float(clearance.min()*100)),indent=2))
