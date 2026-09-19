from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
current=json.loads((D/'metrics.json').read_text());old=json.loads((D.parent/'high_fast_five_20260915/metrics.json').read_text())+json.loads((D.parent/'high_fast_new_seeds_20260915/metrics.json').read_text())
historical={m['seed']:m for m in old if m['label'].startswith('fast_high')};rows=[]
for m in current:
    prev=historical[m['seed']]
    if m['status']!='PASS' or prev['status']!='PASS':continue
    for label,case in [('old',prev),('current',m)]:
        events=[json.loads(l) for l in (Path(case['run'])/'key_events.jsonl').read_text().splitlines()]
        land=next(e['data']['header']['stamp']['stamp_ns']/1e9-case['start_ros_s'] for e in events if e['kind']=='decision' and e['data']['command']==5)
        rows.append(dict(seed=case['seed'],group=label,start_to_third=case['third_commit_s'],third_to_land=land-case['third_commit_s'],land_to_complete=case['completed_mission_s']-land))
(D/'matched_stages.json').write_text(json.dumps(rows,indent=2));fig,ax=plt.subplots(figsize=(12,6));left=np.zeros(len(rows))
for key in ('start_to_third','third_to_land','land_to_complete'):
    values=[r[key] for r in rows];ax.barh(range(len(rows)),values,left=left,label=key);left+=values
ax.set_yticks(range(len(rows)),[str(r['seed'])+' '+r['group'] for r in rows]);ax.set(xlabel='ROS mission seconds',title='Same-seed, both-successful phase comparison');ax.legend(fontsize=8);ax.grid(axis='x',alpha=.2);fig.tight_layout();fig.savefig(D/'matched_stages.png',dpi=160);plt.close(fig)
print(json.dumps(rows,indent=2))
