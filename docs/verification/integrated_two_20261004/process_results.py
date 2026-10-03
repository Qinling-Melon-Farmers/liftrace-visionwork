from pathlib import Path
import json,importlib.util,subprocess,sys,yaml
R=Path(__file__).resolve().parents[3]
D=R/'docs/verification/integrated_two_20261004';D.mkdir(exist_ok=True)
batch=json.loads((R/'logs/integrated_two_20261004_batch/matrix.json').read_text())
items=[dict(seed=r['seed'],label='integrated_two',run=r['run'],world=str(R/f"docs/verification/integrated_two_20261004/seed_{r['seed']}/field.world"),source=batch['source']) for r in batch['results']]
(D/'runs.json').write_text(json.dumps(items,indent=2))
spec=importlib.util.spec_from_file_location('hist',R/'docs/verification/history_31_40_20260920/analyze_current.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.D=D
metrics=[m.one(i) for i in items]
(D/'metrics.json').write_text(json.dumps(metrics,indent=2))
for i in items:
 subprocess.run([sys.executable,str(R/'docs/verification/failed_six_20260921/compose_review.py'),i['run'],'--output',i['run']+'/presentation_review.mp4','--flight-limit','4','--corridor-limit','1.2','--wall-height','4','--corridor-bounds','7.6','9.1','-4.8','4.8','--case-label',f"Seed{i['seed']} corrected columns ON | XY 0.25m"],check=True)
print('ANALYSIS AND VIDEOS COMPLETE')