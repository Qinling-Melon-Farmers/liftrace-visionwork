from pathlib import Path
import json,collections,csv,math,statistics
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/latest_six_20261005'
batch=json.loads((H/'logs/latest_six_20261005_batch/matrix.json').read_text());rows=[]
for r in batch['results']:
 run=Path(r['run']);gate=json.loads((run/'gate_status.json').read_text());gm=gate['metrics'];events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines()]
 start=min(e['data']['header']['stamp']['stamp_ns']/1e9 for e in events if e['kind']=='decision');end=start+gm['mission_ros_sec'] if gm.get('mission_ros_sec') is not None else None
 decisions={}
 for e in events:
  if e['kind']=='decision':decisions.setdefault(e['data']['decision_seq'],e)
 returns=[e for e in decisions.values() if e['data']['command']==4];lands=[e for e in decisions.values() if e['data']['command']==5]
 hs=[json.loads(l) for l in (run/'high_view_full_events.jsonl').read_text().splitlines()];last=hs[-1]['status'];he=last.get('events',[])
 descent=next((e['time'] for e in he if e.get('stage')=='DESCEND'),None)
 ret=min((e['data']['header']['stamp']['stamp_ns']/1e9 for e in returns),default=None);land=min((e['data']['header']['stamp']['stamp_ns']/1e9 for e in lands),default=None)
 results=[e for e in events if e['kind']=='result'];release=[e for e in events if e['kind']=='release' and e['data'].get('success')];rec=[]
 for e in release:
  dd=e['data'];terminal=next((x for x in results if x['data'].get('terminal') and x['data'].get('status')==3 and x['data'].get('stage')==5 and x['data'].get('payload_slot')==dd['payload_slot'] and x['ros_sec']>=e['ros_sec']),None)
  if terminal:rec.append(dict(class_name=dd['target_class'],ack_ros=e['ros_sec'],handoff_ros=terminal['ros_sec'],seconds=terminal['ros_sec']-e['ros_sec']))
 duration=lambda a,b:None if a is None or b is None else b-a
 row=dict(seed=r['seed'],variant=r['variant'],status=r['status'],reason=r['reason'],mission_s=gm['mission_ros_sec'],classes=[e['data']['target_class'] for e in release],all_top3=set(e['data']['target_class'] for e in release)=={'panzer','bridge','red_cross'},phases=dict(search_to_descent=duration(start,descent),low_revisit_delivery=duration(descent,ret),post_delivery_route=duration(ret,land),H_landing=duration(land,end)),recovery=rec,recovery_sum_s=sum(x['seconds'] for x in rec),survey_interrupt=any(e.get('stage')=='SURVEY_INTERRUPTED_TOP3' for e in he),low_coverage=any(e.get('stage')=='LOW_COVERAGE' for e in he),contacts=gate['checks'].get('zero_collisions'),run=str(run))
 metrics=D/f"{r['seed']}_{r['variant']}/metrics.json"
 if metrics.exists():
  m=json.loads(metrics.read_text());row.update({k:m.get(k) for k in ('xy_distance_m','xyz_distance_m','speed_xy_p95_mps','touchdown_truth_center_error_m')})
 rows.append(row)
pairs=[]
for seed in (31,38):
 d={r['variant']:r for r in rows if r['seed']==seed}
 if all(k in d for k in ('rectangle','rectangle_baseline')):
  b,n=d['rectangle_baseline'],d['rectangle'];pairs.append(dict(seed=seed,both_pass=b['status']==n['status']=='PASS',same_top3=b['all_top3'] and n['all_top3'],total_saved_s=b['mission_s']-n['mission_s'] if b['mission_s'] is not None and n['mission_s'] is not None else None,phase_saved_s={k:b['phases'][k]-n['phases'][k] if b['phases'][k] is not None and n['phases'][k] is not None else None for k in b['phases']},recovery_saved_s=b['recovery_sum_s']-n['recovery_sum_s']))
(D/'comparison.json').write_text(json.dumps(dict(batch_status=batch['status'],source=batch['source'],rows=rows,rectangle_pairs=pairs),indent=2))
print(json.dumps(dict(rows=[{k:r[k] for k in ('seed','variant','status','mission_s','classes','phases','recovery_sum_s')} for r in rows],pairs=pairs),indent=2))