"""Separate visual release waiting from navigation stalls using archived audit events."""
from pathlib import Path
import json,re,collections
import numpy as np
D=Path(__file__).resolve().parent;output=[]
for item in json.loads((D/'runs.json').read_text()):
    run=Path(item['run']);log=(run/'run.log').read_text(errors='replace')
    audit=json.loads((run/'visual_delivery_audit.json').read_text())['events']
    aligns=[];locks=[]
    for line in log.splitlines():
        m=re.search(r'\[[0-9.]+,\s*([0-9.]+)\].*External ALIGN target=\d+ class=(\w+) at',line)
        if m:aligns.append((float(m[1]),m[2]))
        m=re.search(r'\[[0-9.]+,\s*([0-9.]+)\].*commitment locked slot=\d+ target=(\w+)/',line)
        if m:locks.append((float(m[1]),m[2]))
    row=dict(label=item['label'],audit_event_types=dict(collections.Counter(e['type'] for e in audit)),transactions=[])
    for start,cls in aligns:
        stop=next((t for t,c in locks if c==cls and t>=start),None)
        if stop is None:continue
        permissions=[e for e in audit if e['type']=='permission' and start<=e['time']<=stop]
        # Audit records transitions. Accumulate their duration; add the state at the start.
        prior=[e for e in audit if e['type']=='permission' and e['time']<start]
        states=([dict(time=start,data=prior[-1]['data'])] if prior else [])+permissions
        durations=collections.defaultdict(float)
        for i,e in enumerate(states):
            end=states[i+1]['time'] if i+1<len(states) else stop
            durations[e['data'].get('reason','unknown')]+=max(0,end-e['time'])
        details=[e for e in audit if start<=e['time']<=stop and e['type'] not in ('selected_target','permission')]
        row['transactions'].append(dict(class_name=cls,align_start_ros=start,commitment_lock_ros=stop,alignment_to_lock_s=stop-start,permission_reason_duration_s=dict(durations),other_event_types=dict(collections.Counter(e['type'] for e in details)),last_other_events=details[-4:]))
    output.append(row)
(D/'alignment_waits.json').write_text(json.dumps(output,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
metrics=json.loads((D/'summary.json').read_text());fig,ax=plt.subplots(figsize=(10,5))
parts=[]
for row,m in zip(output,metrics):
    p=next(v for v in row['transactions'] if v['class_name']=='panzer')
    action=next(v for v in m['actions'] if v['target']=='panzer' and v['command']==1)
    ack=next(v['data']['header']['stamp']['stamp_ns']/1e9 for v in m['releases'] if v['data']['target_class']=='panzer')
    parts.append([p['align_start_ros']-m['start_ros_s']-action['start'],p['alignment_to_lock_s'],ack-p['commitment_lock_ros'],m['start_ros_s']+action['end']-ack])
left=np.zeros(3)
for i,name in enumerate(('Approach to ALIGN','ALIGN to commitment lock','Lock to mock ACK','Post-ACK recovery')):
    values=np.array(parts)[:,i];ax.barh(range(3),values,left=left,label=name);left+=values
ax.set_yticks(range(3),[r['label'] for r in output]);ax.set(xlabel='Simulation seconds',title='Panzer transaction: visual alignment wait is not a planner stall');ax.legend(fontsize=8);ax.grid(axis='x',alpha=.2)
fig.tight_layout();fig.savefig(D/'panzer_alignment_wait.png',dpi=160);plt.close(fig)
for row in output:
    print(row['label'],row['audit_event_types'])
    for v in row['transactions']:print(json.dumps({k:v[k] for k in ('class_name','alignment_to_lock_s','permission_reason_duration_s','other_event_types')},indent=2))
