"""Recompute FC-to-target alignment at successful mock release ACKs from logs."""
from pathlib import Path
import json,csv
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2];V=D.parent
sources=[('R64',V/'r64_matrix/flight_metrics.json'),('full_random_31_35',V/'full_random_five_20260910/flight_metrics.json'),('paired_32_34',V/'high_view_full_20260914/summary.json'),('previous2672',V/'reliability_trial_20260919/metrics.json'),('before_fixABC',V/'frame_speed_height_20260919/summary.json'),('repairedABC',D/'summary.json')]
samples=[];missing=[];seen=set()
for group,source in sources:
    data=json.loads(source.read_text());data=data if isinstance(data,list) else [data]
    for item in data:
        run=Path(item.get('run',item.get('run_dir','')))
        if str(run) in seen:continue
        seen.add(str(run));needed=['key_events.jsonl','truth_pose.csv','random_field_truth.yaml']
        if not all((run/name).exists() for name in needed):
            missing.append(dict(group=group,run=str(run),reason='raw log path missing'));continue
        with (run/'truth_pose.csv').open() as f:pose=np.array([[float(row[k]) for k in ('t','x','y','z')] for row in csv.DictReader(f)])
        pose=pose[np.argsort(pose[:,0],kind='stable')]
        truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
        targets={t['class']:np.array([t.get('world_x',t['x']),t.get('world_y',t['y'])]) for t in truth['targets']}
        params=yaml.safe_load((run/'rosparams.yaml').read_text()) if (run/'rosparams.yaml').exists() else {}
        offset=params.get('competition_key_recorder',{}).get('truth_world_offset',[0,0,.22])
        unique=set()
        for line in (run/'key_events.jsonl').read_text().splitlines():
            e=json.loads(line)
            if e['kind']!='release' or not e['data'].get('success'):continue
            d=e['data'];key=d.get('execution_id',(d['payload_slot'],d['target_class']))
            if key in unique:continue
            unique.add(key);cls=d['target_class']
            if cls not in targets:continue
            stamp=d.get('header',{}).get('stamp',{}).get('stamp_ns',0)/1e9 or e['ros_sec']
            i=int(np.searchsorted(pose[:,0],stamp))
            if i==0 or i==len(pose) or pose[i,0]-pose[i-1,0]>.5:
                missing.append(dict(group=group,run=str(run),reason='release outside dense truth samples',stamp=stamp));continue
            xy=np.array([np.interp(stamp,pose[:,0],pose[:,j]) for j in (1,2)])+offset[:2]
            delta=xy-targets[cls]
            samples.append(dict(group=group,run=str(run),label=item.get('label',str(item.get('seed',''))),class_name=cls,slot=d['payload_slot'],ros_stamp=stamp,actual_fc_xy=xy.tolist(),target_xy=targets[cls].tolist(),dx_m=float(delta[0]),dy_m=float(delta[1]),error_m=float(np.linalg.norm(delta))))
def stats(rows):
    a=np.array([r['error_m'] for r in rows])
    return dict(n=len(a),p50_m=float(np.median(a)),p95_m=float(np.percentile(a,95)),max_m=float(a.max()),within_5cm=int((a<=.05).sum()),within_10cm=int((a<=.10).sum()),within_15cm=int((a<=.15).sum())) if len(a) else dict(n=0)
groups={g:stats([r for r in samples if r['group']==g]) for g,_ in sources}
classes={c:stats([r for r in samples if r['class_name']==c]) for c in sorted({r['class_name'] for r in samples})}
result=dict(scope='Actual FC XY versus scenario target model centre at successful mock ACK; not parcel impact accuracy',samples=samples,groups=groups,classes=classes,all=stats(samples),missing=missing)
(D/'alignment_history.json').write_text(json.dumps(result,indent=2))
fig,axes=plt.subplots(1,2,figsize=(13,5))
valid=[g for g,_ in sources if groups[g]['n']]
axes[0].boxplot([[100*r['error_m'] for r in samples if r['group']==g] for g in valid],labels=valid,showmeans=True)
axes[0].set(ylabel='True FC distance to target centre (cm)',title='Successful mock release ACKs; later failed flights retained');axes[0].tick_params(axis='x',rotation=20)
for c in classes:
    rows=[r for r in samples if r['class_name']==c and r['group']=='repairedABC']
    axes[1].scatter([100*r['dx_m'] for r in rows],[100*r['dy_m'] for r in rows],s=22,alpha=.6,label=c)
for radius in (5,10,15):axes[1].add_patch(plt.Circle((0,0),radius,fill=False,ls=':',color='.6'))
axes[1].set(xlabel='FC - target X (cm)',ylabel='FC - target Y (cm)',title='Repaired A/B/C only; fixed-start XY frame');axes[1].set_aspect('equal');axes[1].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(D/'alignment_history.png',dpi=160);plt.close(fig)
print(json.dumps({k:result[k] for k in ('groups','classes','all','missing')},indent=2))
