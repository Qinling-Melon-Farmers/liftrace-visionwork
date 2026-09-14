"""Compare fixed-layout baseline before/after; not a random-layout trial."""
import json
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent
ROOT=D.parents[3]

def timings(run):
    events=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
    dec=[e for e in events if e['kind']=='decision']
    start=min(e['data']['header']['stamp']['stamp_ns']/1e9 for e in dec)
    committed={}
    for e in events:
        if e['kind']=='result' and e['data'].get('payload_committed'):
            committed.setdefault(e['data']['payload_slot'],e['data']['header']['stamp']['stamp_ns']/1e9)
    returns=[];seen=set()
    for e in dec:
        d=e['data']
        if d['command']==4 and d['decision_seq'] not in seen:
            returns.append(e);seen.add(d['decision_seq'])
    third=max(committed.values()) if len(committed)==3 else None
    entry=returns[1]['data']['header']['stamp']['stamp_ns']/1e9 if len(returns)>1 else None
    gate=json.loads((run/'gate_status.json').read_text())
    return dict(run=str(run),status=gate['status'],third_commit_s=third-start if third else None,
                third_to_corridor_entry_s=entry-third if entry and third else None,
                first_transit_s=entry-returns[0]['data']['header']['stamp']['stamp_ns']/1e9 if entry else None,
                full_mission_s=gate['metrics'].get('mission_ros_sec') if gate['status']=='PASS' else None)

def main():
    items=json.loads((D/'runs.json').read_text())
    newrun=Path(next(i['run'] for i in items if i['label']=='baseline'))
    oldrun=ROOT/'logs/high_view_full_frozen_baseline_seed32_20260914_015313'
    truth=lambda r:sorted((t['class'],t['world_x'],t['world_y']) for t in yaml.safe_load((r/'random_field_truth.yaml').read_text())['targets'])
    assert truth(newrun)==truth(oldrun)
    result=dict(old=timings(oldrun),new=timings(newrun),same_targets=True,
                entry_definition='Second post-delivery RETURN decision: first staging waypoint complete')
    (D/'before_after.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
