"""Analyze the frozen local-entry comparison; never starts ROS or simulation."""
from pathlib import Path
import json, sys, collections, re
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
OUT.mkdir(exist_ok=True)

def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x]

def percentiles(values):
    return {k: float(np.percentile(values, v)) for k, v in [('p50', 50), ('p95', 95), ('max', 100)]} if values else {}

def alignment_review(run, events):
    decisions=[e for e in events if e['kind']=='decision' and e['data'].get('has_target')]
    reports=[]
    pattern=re.compile(r'\[\d+\.\d+,\s*(\d+\.\d+)\].*drop_ready=false reason=(\S+)')
    for line in (run/'run.log').read_text(errors='replace').splitlines():
        match=pattern.search(line)
        if match: reports.append((float(match.group(1)),match.group(2).split('\x1b')[0]))
    out=[]
    for e in decisions:
        d=e['data'];seq=d['decision_seq']
        if d.get('command')!=1:continue
        aligned=next((x for x in events if x['kind']=='result' and x['data'].get('decision_seq')==seq and x['data'].get('reason')=='patrol_control_alignment_accepted'),None)
        released=next((x for x in events if x['kind']=='release' and x['ros_sec']>=e['ros_sec'] and x['data'].get('success') and x['data'].get('target_class')==d.get('target_class')),None)
        end=released['ros_sec'] if released else events[-1]['ros_sec']
        start=aligned['ros_sec'] if aligned else e['ros_sec']
        out.append(dict(seq=seq,target_class=d.get('target_class'),approach_issued_ros=e['ros_sec'],
            alignment_accepted_ros=aligned['ros_sec'] if aligned else None,
            release_ros=released['ros_sec'] if released else None,
            alignment_to_release_s=end-start if released and aligned else None,
            throttled_drop_rejection_log_counts=dict(collections.Counter(reason for t,reason in reports if start<=t<=end)),
            log_count_scope='Throttled diagnostic lines during this alignment, not frame rejection rate'))
    return out

def local_review(run, events, advice):
    decisions = {e['data']['decision_seq']: e for e in events if e['kind'] == 'decision'}
    ordered = sorted(decisions.values(), key=lambda e: e['ros_sec'])
    outcomes = {}
    for e in events:
        if e['kind'] == 'mission':
            value = e['data'].get('local_motion', {}).get('last_outcome')
            if value: outcomes[value['decision_seq']] = value
    requests = {a.get('request_id'): a for a in advice if a.get('accepted')}
    result = []
    for i, e in enumerate(ordered):
        d = e['data']
        if not d.get('reason', '').startswith('research_local_entry:'): continue
        req = int(d['reason'].split(':')[-1])
        a = requests[req]
        origin = np.array(a['source_pose'][:2]); nominal = np.array(a['nominal_goal'][:2])
        entry = np.array(a['selected']['entry'][:2]); v = nominal-origin
        delta = entry-origin
        parent = decisions[a['decision_seq']]['data']
        following = ordered[i+1]['data'] if i+1 < len(ordered) else {}
        outcome = outcomes.get(d['decision_seq'], {})
        preempted = following.get('reason') == 'high_weight_search_interrupt' and following.get('has_target')
        resumed = next((x['data'] for x in ordered[i+1:] if x['data'].get('command') in [0,3]), None) if preempted else None
        result.append(dict(seq=d['decision_seq'], request_id=req, issued_ros=e['ros_sec'],
            parent_seq=a['decision_seq'], parent_reason=parent['reason'], entry=a['selected']['entry'],
            source=a['source_pose'], nominal=a['nominal_goal'],
            lateral_m=float(abs(v[0]*delta[1]-v[1]*delta[0])/np.linalg.norm(v)),
            gain_proxy_m2=a['selected']['entry_gain_m2'], outcome=outcome,
            next_seq=following.get('decision_seq'), next_reason=following.get('reason'),
            resolution='target_preempted' if preempted else outcome.get('reason','unresolved'),
            restored_deadline=None if preempted else (following.get('deadline') == parent.get('deadline')),
            resume_after_target_seq=resumed.get('decision_seq') if resumed else None,
            resume_after_target_goal_matches=(resumed['goal']['pose']==parent['goal']['pose'] and
                resumed['goal']['header']['frame_id']==parent['goal']['header']['frame_id']) if resumed else None,
            estimate_only=True))
    return result

def summarize(run_path):
    run = Path(run_path).resolve()
    from base_analysis import analyze
    row = analyze(run)
    events = rows(run/'key_events.jsonl'); advice = rows(run/'local_search_proposals.jsonl')
    gate = json.loads((run/'gate_status.json').read_text()) if (run/'gate_status.json').exists() else {}
    contacts = json.loads((run/'gazebo_contact_status.json').read_text())
    row['gate_errors'] = gate.get('errors', [])
    row['gate_checks'] = gate.get('checks', {})
    row['gate_metrics'] = gate.get('metrics', {})
    row['collision_events'] = contacts.get('events', [])
    row['alignment_intervals'] = alignment_review(run, events)
    row['local_entries'] = local_review(run, events, advice)
    valid = [a for a in advice if a.get('accepted')]
    row['adviser'] = dict(rows=len(advice), accepted_rows=len(valid),
        unique_accepted_requests=len({a['request_id'] for a in valid}),
        reasons=dict(collections.Counter(a.get('reason') for a in advice)),
        probe_work_ms=percentiles([a['probe_work_ms'] for a in valid if 'probe_work_ms' in a]),
        reply_age_ros_ms=percentiles([(a['receipt_ros']-a['created'])*1000 for a in valid]),
        checked_voxels=percentiles([a['checked_voxels'] for a in valid if 'checked_voxels' in a]))
    row['observer_processing_scope'] = 'heavy-processing ticks only; cooldown rows excluded; not whole-node CPU'
    row['release_times'] = [dict(ros_sec=e['ros_sec'], since_first_decision_s=e['ros_sec']-row['mission_start_ros'],
        target_class=e['data'].get('target_class')) for e in events if e['kind']=='release' and e['data'].get('success')]
    row['resource_samples'] = rows(run/'resource_samples.jsonl') if (run/'resource_samples.jsonl').exists() else []
    row['px4_logs'] = [dict(path=str(p.relative_to(run)),bytes=p.stat().st_size) for p in sorted((run/'px4').rglob('*.ulg*'))]
    row['run_dir'] = str(run.relative_to(ROOT))
    name = run.name.split('_20260912_')[0]
    (OUT/(name+'_metrics.json')).write_text(json.dumps(row, indent=2)+'\n')
    print(json.dumps({k:row[k] for k in ['run_dir','status','drops','third_release_s','command_motion','collisions']}, indent=2))

if __name__ == '__main__':
    paths=sys.argv[1:] or [ROOT/r['run'] for r in json.loads((OUT/'runs.json').read_text())['runs']]
    for path in paths:summarize(path)
