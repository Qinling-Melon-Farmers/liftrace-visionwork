"""Offline first-failure summary; invoke only once all three are finished."""
from pathlib import Path
import json,collections,re
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')
D=R/'docs/verification/failed_three_20260923'
state=json.loads((D/'selected_matrix.json').read_text())
assert state['status']=='COMPLETE'
out=[]
for row in state['results']:
    run=Path(row['run'])
    high=json.loads((run/'high_view_full_events.jsonl').read_text().splitlines()[-1])['status']
    events=[json.loads(s) for s in (run/'key_events.jsonl').read_text().splitlines()]
    results=[e for e in events if e['kind']=='result' and e['data'].get('terminal')]
    log=(run/'run.log').read_text(errors='replace')
    rejection=collections.Counter(re.findall(r'start_occ=([01]).*?goal_occ=([01])',log))
    out.append(dict(seed=row['seed'],status=row['status'],reason=row['reason'],drops=row['drops'],high=high,
        terminals=results,occupancy_rejections={str(k):v for k,v in rejection.items()},
        contacts=json.loads((run/'gazebo_contact_status.json').read_text())))
(D/'diagnostic.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
print(json.dumps([{k:r[k] for k in ('seed','status','reason','drops','occupancy_rejections')} for r in out],indent=2))