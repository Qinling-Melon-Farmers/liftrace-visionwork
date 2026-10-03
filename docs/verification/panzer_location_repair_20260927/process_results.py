"""Offline analysis of the one authorized seed38 replay; never starts ROS."""
from pathlib import Path
import importlib.util,json,subprocess,sys
D=Path(__file__).resolve().parent;R=D.parents[2]
batch=json.loads((R/'logs/panzer_location_repair_20260927_batch/matrix.json').read_text())
assert batch['status']=='COMPLETE' and len(batch['results'])==1
row=batch['results'][0]
item=dict(seed=38,label='panzer_location_repair',run=row['run'],world=str(R/'docs/verification/history_31_40_20260920/seed_38/field.world'),source=batch['source'])
(D/'runs.json').write_text(json.dumps([item],indent=2))
(D/'matrix.json').write_text(json.dumps(batch,indent=2))
spec=importlib.util.spec_from_file_location('history',R/'docs/verification/history_31_40_20260920/analyze_current.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h);h.D=D
m=h.one(item)
(D/'metrics.json').write_text(json.dumps([m],indent=2))
subprocess.run([sys.executable,str(R/'docs/verification/failed_six_20260921/compose_review.py'),item['run'],'--output',item['run']+'/presentation_review.mp4','--flight-limit','4','--corridor-limit','1.2','--wall-height','4','--corridor-bounds','7.6','9.1','-4.8','4.8','--case-label','Seed38 panzer interrupt repair | columns ON | XY 0.275m'],check=True)
print('Flight plots, metrics and 1x review video complete')