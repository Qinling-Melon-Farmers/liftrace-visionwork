from pathlib import Path
import json,math
R=Path(__file__).resolve().parents[4];D=R/'docs/verification/snake3_camera2m_20261005'
rows=[]
for x in json.loads((D/'metrics.json').read_text()):
 events=[json.loads(l) for l in (Path(x['run'])/'high_view_full_events.jsonl').read_text().splitlines()];final=events[-1]['status'];sc=next(s for s in json.loads((D/'scenes.json').read_text()) if s['seed']==x['seed']);targets=sc['targets'];first={}
 for cls,h in final.get('first_hint_ready',{}).items():
  target=next(t for t in targets if t['class']==cls);nearest=min(targets,key=lambda t:math.dist(h['xy'],[t['x'],t['y']]))
  first[cls]=dict(h,correct_class_center_error_m=math.dist(h['xy'],[target['x'],target['y']]),nearest_class=nearest['class'],nearest_error_m=math.dist(h['xy'],[nearest['x'],nearest['y']]))
 selected=[e for e in final.get('events',[]) if e.get('stage') in ['SURVEY_INTERRUPTED_TOP3','LOW_VIEW_LABEL_RESOLVED','TARGET_DEFERRED','LOW_COVERAGE','REACQUIRE','REVISIT_VIEWPOINT']]
 rows.append(dict(seed=x['seed'],first_hints=first,transitions=selected,final_failure=final.get('failure'),fallback_started=final.get('fallback_started'),degraded_from=final.get('degraded_from'),reacquisitions=final.get('reacquisitions')))
(D/'behavior.json').write_text(json.dumps(rows,indent=2))
print(json.dumps([dict(seed=x['seed'],first_hints=x['first_hints'],transitions=[{k:e[k] for k in ('stage','time','class_name','previous_class','target','reason') if k in e} for e in x['transitions']]) for x in rows],indent=2))