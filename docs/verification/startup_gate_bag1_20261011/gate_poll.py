import csv,json
from pathlib import Path
p=Path('/home/xhj/liftrace-deliverables/takeoff_failure_20261011')
r=list(csv.DictReader(open(p/'startup_gate_20ms.csv')))
results=[]
for phase in range(10):
 since=None;first=None
 for v in r[phase::10]:
  t=float(v['t'])
  if v['reasons']!='ready':since=None
  elif since is None:since=t
  if since is not None and t-since>=1-1e-8:first=t;break
 results.append(dict(poll_offset_sec=phase*.02,first_eligible_t=first))
print(json.dumps(results,indent=2))
json.dump(results,open(p/'startup_poll_phase_sensitivity.json','w'),indent=2)