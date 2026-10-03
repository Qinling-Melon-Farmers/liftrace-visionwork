from pathlib import Path
import json,collections,re,csv
R=Path(__file__).resolve().parents[3];D=R/'docs/verification/failed_six_20260921'
s=json.loads((R/'logs/failed_six_20260921_batch/matrix.json').read_text());out=[]
for row in s['results']:
 r=Path(row['run']);g=json.loads((r/'gate_status.json').read_text());ev=[json.loads(v) for v in (r/'key_events.jsonl').read_text().splitlines() if v];h=[json.loads(v) for v in (r/'high_view_full_events.jsonl').read_text().splitlines() if v];text=(r/'run.log').read_text(errors='replace')
 counters=collections.Counter()
 for line in text.splitlines():
  if 'rejection summary' in line:
   m=re.search('start_occ=(-?\\d+) goal_occ=(-?\\d+)',line)
   if m:counters[m.group(0)]+=1
 result=dict(seed=row['seed'],gate=g['status'],reason=g['reason'],metrics=g['metrics'],errors=g.get('errors'),contacts=json.loads((r/'gazebo_contact_status.json').read_text()),last_high=h[-1],rejection_counts=dict(counters),actions=[dict(t=e['ros_sec'],**e['data']) for e in ev if e['kind']=='decision'],terminals=[dict(t=e['ros_sec'],**e['data']) for e in ev if e['kind']=='result' and e['data'].get('terminal')],warnings=[line for line in text.splitlines() if any(k in line for k in ['pose continuity','tracking_hold_replan','budget_exhausted','rejection summary','actual collision','unsafe optimization'])][-30:])
 out.append(result)
 print('SEED',row['seed'],g['status'],'metrics', {k:g['metrics'].get(k) for k in ['mission_ros_sec','release_commit_count','recovery_success_count','max_observed_height','boundary_violations','search_envelope_violations']})
 if row['status']!='PASS':
  print('CONTACTS',json.dumps(result['contacts'])[:5000]);print('HIGH',json.dumps(h[-1]['status'].get('events',[]))[-4500:]);print('TERMINALS',json.dumps([(v['t'],v.get('decision_seq'),v.get('target_class'),v.get('reason')) for v in result['terminals']])[-3000:]);print('REJECT',dict(counters));print('LAST_WARN',result['warnings'][-3:])
(D/'failure_evidence.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
