"""Reuse the prior report's measurements, writing only this report's output directory."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
OLD=ROOT/'docs/verification/local_entry_sitl_20260912'
OUT=Path(__file__).resolve().parent
OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(OLD))
import base_analysis, analyze, supplement
from tree_passages import search_mask
base_analysis.OUT=analyze.OUT=supplement.OUT=OUT
for name in sys.argv[1:] or [ROOT/r['run'] for r in json.loads((OUT/'runs.json').read_text())['runs']]:
    run=Path(name).resolve()
    assert run.is_relative_to(ROOT/'logs')
    analyze.summarize(run)
    output=OUT/(run.name.split('_20260912_')[0]+'_metrics.json')
    row=json.loads(output.read_text())
    events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines()]
    pose=base_analysis.csv(run/'truth_pose.csv');dt=np.diff(pose[:,0]);ds=np.linalg.norm(np.diff(pose[:,1:3],axis=0),axis=1)
    mask=search_mask(pose,events)&(dt>0)&(dt<=.5)
    assert abs(float(dt[mask].sum())-row['command_motion']['search_resume_s'])<1e-6
    row['search_kinematics']=dict(threshold_mps=.1,search_resume_slow_s=float(dt[mask&(ds/np.maximum(dt,1e-9)<.1)].sum()),
        scope='Actual XY finite-difference speed during SEARCH/RESUME; includes hover and slow tracking, not a pure stop-time attribution')
    decisions={x['data']['decision_seq']:x['data'] for x in events if x['kind']=='decision'}
    for entry in row['local_entries']:
        if entry['resolution']=='target_preempted':continue
        original=decisions[entry['parent_seq']]['deadline']['stamp_ns']
        restored=decisions[entry['next_seq']]['deadline']['stamp_ns']
        entry.update(deadline_delta_ns=restored-original,deadline_within_1ns=abs(restored-original)<=1,
                     deadline_not_extended=restored<=original)
    output.write_text(json.dumps(row,indent=2)+'\n')
    supplement.supplemental(run)
