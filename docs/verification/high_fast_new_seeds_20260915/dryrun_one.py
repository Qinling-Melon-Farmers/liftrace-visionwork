"""Dry-run the report pipeline on one finished seed to validate the code path early.

Writes the same per-seed folder analyze.py will reuse later. Offline only.
"""
import json, runpy, sys
from pathlib import Path

D = Path(__file__).resolve().parent
R = D.parents[2]
spec = runpy.run_path(str(D / 'analyze.py'))
NS = spec['NS']
batch = json.loads((R / 'logs/high_fast_new_seeds_20260915_batch/matrix.json').read_text())
cases = json.loads((D / 'cases.json').read_text())
done = [r for r in batch['results'] if r['status'] != 'RUNNING']
print('finished so far:', [(r['seed'], r['status']) for r in done])
assert done, 'no finished run yet'
row = done[0]
case = next(c for c in cases if c['seed'] == row['seed'])
run = Path(row['run'])
item = dict(seed=case['seed'], label='fast_high_new', run=str(run),
            world=case['world'], source=batch['source'])
m = NS['analyze'](item, D)
print('analysis OK for seed', row['seed'])
print('keys present:', all(k in m for k in ('commit_times', 'gate_metrics', 'releases', 'high_view_final', 'status')))
print('loop:', json.dumps(spec['loop_metrics'](m), ensure_ascii=False))
print('tail:', json.dumps(spec['tail_metrics'](run, m), ensure_ascii=False))
dest = D / f"{row['seed']}_fast_high_new"
print('charts:', sorted(p.name for p in dest.glob('*.png')))