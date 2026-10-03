"""Independent post-run checks using recorded truth, phase and raw support logs."""
from pathlib import Path
import csv,json,re
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
results=[];fig,axes=plt.subplots(3,1,figsize=(12,9))
for item,ax in zip(json.loads((D/'runs.json').read_text()),axes):
    run=Path(item['run']);ev=[json.loads(v) for v in (run/'key_events.jsonl').read_text().splitlines()]
    dec=[e for e in ev if e['kind']=='decision'];start=dec[0]['data']['header']['stamp']['stamp_ns']/1e9
    tail=next((e['ros_sec'] for e in dec if e['data']['command']==4 and e['data']['reason'].startswith('post_delivery_route:')),np.inf)
    land=next((e['ros_sec'] for e in dec if e['data']['command']==5),None)
    with (run/'truth_pose.csv').open() as f:a=np.array([[float(v[k]) for k in ('t','x','y','z','qx','qy','qz','qw')] for v in csv.DictReader(f)])
    rot=Rotation.from_quat(a[:,4:]).as_matrix();corners=np.array([[x,y,z] for x in (-.275,.275) for y in (-.275,.275) for z in (-.22,.18)])
    verts=np.einsum('tij,kj->tki',rot,corners)+a[:,None,1:4]
    clearance=np.minimum.reduce([verts[:,:,0].min(axis=1)+.5,7.4-verts[:,:,0].max(axis=1),verts[:,:,1].min(axis=1)+4.8,4.8-verts[:,:,1].max(axis=1)])
    mask=(a[:,0]>=start)&(a[:,0]<tail)
    contact=json.loads((run/'gazebo_contact_status.json').read_text());gate=json.loads((run/'gate_status.json').read_text())
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    supports=[v for v in contact.get('support_events',[]) if land is not None and v['ros_stamp']>=land]
    videos={}
    import subprocess
    for name in ('follow','overview','presentation'):
        if not (run/(name+'.mp4')).exists():continue
        v=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size','-of','json',str(run/(name+'.mp4'))],text=True))['format']
        videos[name]={'duration_s':float(v['duration']),'size_bytes':int(v['size'])}
    data=dict(label=item['label'],start_ros_s=start,tail_start_ros_s=tail if np.isfinite(tail) else None,land_ros_s=land,
        minimum_pre_tail_whole_body_inner_clearance_m=float(clearance[mask].min()) if mask.any() else None,
        pre_tail_whole_body_outside_samples=int((clearance[mask]<-1e-4).sum()),search_samples=int(mask.sum()),
        max_pre_tail_center_x=float(a[mask,1].max()) if mask.any() else None,
        final_guard_xy_m=contact.get('guard_xy_m'),collision_count=contact.get('actual_collision_count'),landing_support_events=supports,
        configured_land_speed_mps=params.get('simulation',{}).get('px4_parameters',{}).get('MPC_LAND_SPEED'),
        gate_status=gate.get('status'),gate_metrics=gate.get('metrics'),videos=videos)
    ax.plot(a[mask,0]-start,clearance[mask],lw=.8);ax.axhline(0,color='red',ls='--');ax.set(title=item['label'],ylabel='55 cm body clearance (m)',xlabel='Mission seconds before formal post-delivery route');ax.grid(alpha=.2)
    results.append(data)
fig.suptitle('Recorded whole-body projection versus inner search walls | before corridor release')
fig.tight_layout();fig.savefig(D/'search_inner_clearance.png',dpi=160);plt.close(fig)
(D/'repair_checks.json').write_text(json.dumps(results,indent=2))
print(json.dumps([{k:v for k,v in row.items() if k not in ('gate_metrics','landing_support_events')} for row in results],indent=2))
