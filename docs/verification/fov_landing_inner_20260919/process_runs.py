"""Offline artifacts only. Does not start ROS, Gazebo, PX4 or replay simulation."""
from pathlib import Path
import json,subprocess,sys
D=Path(__file__).resolve().parent;R=D.parents[2]
items=[]
for letter,label in [('A','A_baseline'),('B','B_corridor'),('C','C_high3')]:
    runs=sorted((R/'logs').glob(f'fovfix2672_{letter}_*'))
    if len(runs)!=1:raise RuntimeError(f'Expected one authorized {letter} run; found {runs}')
    run=runs[0]
    if not (run/'gate_status.json').exists():raise RuntimeError(f'Run has no terminal Gate: {run}')
    items.append(dict(seed=2672,label=label,run=str(run),world=str(D/'seed_2672/field.world'),flight_source='708529d',navigation_source='30ff44d'))
(D/'runs.json').write_text(json.dumps(items,indent=2)+'\n')
for name,args in [('analyze.py',[str(D/'runs.json')]),('landing_sequence.py',[]),('contact_figures.py',[]),('body_closeups.py',[]),('fov_geometry.py',[]),('alignment_history.py',[])]:
    print('Processing',name,flush=True)
    with (D/(name.removesuffix('.py')+'_output.txt')).open('w') as f:subprocess.run([sys.executable,str(D/name),*args],stdout=f,check=True)
for item in items:
    run=Path(item['run']);output=run/'presentation.mp4'
    if output.exists():
        if not output.with_suffix('.json').exists():raise RuntimeError(f'Incomplete presentation exists: {output}')
        continue
    print('Composing',item['label'],flush=True)
    command=[sys.executable,str(R/'vision_ws/src/uav_high_view/scripts/presentation_compose.py'),str(run),'--output',str(output),'--flight-limit','4','--corridor-limit','.7' if item['label']=='A_baseline' else '1.2','--wall-height','4','--corridor-bounds','7.6','9.1','-4.8','4.8','--case-label',{'A_baseline':'A | 2.6m高位 / 原走廊 / 新视场与降落修复','B_corridor':'B | 2.6m高位 / 快走廊 / 新视场与降落修复','C_high3':'C | 3.0m高位 / 快走廊 / 新视场与降落修复'}[item['label']]]
    with (D/('compose_'+item['label']+'.txt')).open('w') as f:subprocess.run(command,stdout=f,check=True)
with (D/'repair_checks_output.txt').open('w') as f:subprocess.run([sys.executable,str(D/'repair_checks.py')],stdout=f,check=True)
print('Offline metrics and videos complete',flush=True)
