"""Offline ten-run metrics and corrected videos. Never launches a simulator."""
from pathlib import Path
import json,sys,subprocess,importlib.util,concurrent.futures,os
import numpy as np,yaml
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');D=R/'docs/verification/history_31_40_20260920'

def analyze(item):
    spec=importlib.util.spec_from_file_location('current_analysis',D/'analyze_current.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod.one(item)

def compose(item):
    run=Path(item['run']);output=run/'presentation.mp4'
    if output.exists():
        if output.with_suffix('.json').exists():return str(output)
        raise RuntimeError('Incomplete video exists: '+str(output))
    cmd=[sys.executable,str(R/'vision_ws/src/uav_high_view/scripts/presentation_compose.py'),str(run),'--output',str(output),'--flight-limit','4','--corridor-limit','1.2','--wall-height','4','--corridor-bounds','7.6','9.1','-4.8','4.8','--case-label',f'Seed {item["seed"]} | 2.6m高位 / 历史旋转布局 / 当前修复版']
    with (D/f'compose_seed{item["seed"]}.txt').open('w') as f:subprocess.run(cmd,check=True,stdout=f)
    return str(output)

def main():
    state=json.loads((R/'logs/history31_40_20260920_batch/matrix.json').read_text())
    if state['status']!='COMPLETE' or len(state['results'])!=10:raise RuntimeError('Wait for all ten authorized runs to finish')
    items=[dict(seed=v['seed'],label='current',run=v['run'],world=str(D/f'seed_{v["seed"]}/field.world'),source=state['source']) for v in state['results']]
    (D/'runs.json').write_text(json.dumps(items,indent=2));(D/'matrix.json').write_text(json.dumps(state,indent=2))
    histories=json.loads((D/'historical_runs.json').read_text());equivalence=[];sources=[]
    for item in items:
        run=Path(item['run']);manifest=yaml.safe_load((run/'manifest.yaml').read_text())
        sources.append(dict(seed=item['seed'],head=manifest['git_head'],uav_ws=manifest['uav_ws'],vision_ws=manifest['vision_ws'],resolved_uav_mission=manifest['resolved_uav_mission'],resolved_uav_vision=manifest['resolved_uav_vision']))
        assert manifest['git_head']==state['source'] and manifest['resolved_uav_mission']==str(R/'patrol_uav_ws-patrol_planner/src/uav_mission') and manifest['resolved_uav_vision']==str(R/'vision_ws/src/uav_vision')
        old=next(v for v in histories if v['seed']==item['seed'] and v['label'].startswith('fast_high'))
        oldtruth=yaml.safe_load((Path(old['run'])/'random_field_truth.yaml').read_text())['targets'];newtruth=yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets']
        deltas=[]
        for t in oldtruth:
            nt=next(v for v in newtruth if v['class']==t['class']);expected=np.array([t['world_y'],-t['world_x']]);actual=np.array([nt['world_x'],nt['world_y']]);deltas.append(float(np.linalg.norm(actual-expected)))
        assert max(deltas)<1e-4
        equivalence.append(dict(seed=item['seed'],max_actual_target_rotation_error_m=max(deltas),source_run=old['run']))
    (D/'source_consistency.json').write_text(json.dumps(sources,indent=2));(D/'actual_layout_equivalence.json').write_text(json.dumps(equivalence,indent=2))
    print('Verified ten source manifests and actual paired target layouts',flush=True)
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
        metrics=list(pool.map(analyze,items))
    (D/'metrics.json').write_text(json.dumps(metrics,indent=2));print('Per-run metrics and figures complete',flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for path in pool.map(compose,items):print('Video complete:',path,flush=True)
    print('All ten metrics and videos complete',flush=True)
if __name__=='__main__':main()
