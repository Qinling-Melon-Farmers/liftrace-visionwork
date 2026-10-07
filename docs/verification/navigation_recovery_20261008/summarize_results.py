#!/usr/bin/env python3
"""Read existing runs only; write compact metrics and one overview figure."""
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RUNS = [
    ('column_initial_fail', 'navigation_recovery_column_20261008_012437'),
    ('column', 'navigation_recovery_column_20261008_013239'),
    ('buffer', 'navigation_recovery_buffer_20261008_013332'),
    ('height', 'navigation_recovery_height_20261008_013439'),
]
out = HERE / 'results'
out.mkdir(exist_ok=True)
metrics = {'scope': 'production_map_fsm_server_controller_with_synthetic_plant',
           'runtime_bridge': False, 'px4_gazebo': False,
           'time_basis': 'CSV first sample; transition times are sampled observations',
           'runs': []}
fig, axes = plt.subplots(4, 3, figsize=(12, 10), constrained_layout=True)
for i, (label, name) in enumerate(RUNS):
    run = ROOT / 'logs' / name
    gate = json.loads((run / 'gate_status.json').read_text())
    rows = [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader((run / 'production_recovery.csv').open())]
    t0 = rows[0]['stamp']
    t = [r['stamp'] - t0 for r in rows]
    start = [rows[0][k] for k in ('x', 'y', 'z')]
    goal = gate['original_goal']
    displacement = [math.dist([r[k] for k in ('x', 'y', 'z')], start) for r in rows]
    goal_distance = [math.dist([r[k] for k in ('x', 'y', 'z')], goal) for r in rows]
    height = [r['z'] for r in rows]
    speed = [math.sqrt(sum(r[k] ** 2 for k in ('vx', 'vy', 'vz'))) for r in rows]
    onset = next((r['stamp'] - t0 for r in rows if r['fsm_commands'] > 0), None)
    resume = next((r['stamp'] - t0 for r in rows if r['resume_splines'] > 0), None)
    cleanup = 'SITL cleanup verification: PASS' in (run / 'run.log').read_text(errors='replace')
    item = dict(case=label, run=str(run.relative_to(ROOT)), status=gate['status'],
                reason=gate['reason'], cleanup_pass=cleanup, csv_samples=len(rows),
                csv_elapsed_seconds=t[-1], recovery_first_observed_seconds=onset,
                ordinary_resume_first_observed_seconds=resume,
                final_goal_distance_m=math.dist(gate['final_position'], goal),
                min_height_m=min(height), max_height_m=max(height),
                max_sampled_speed_mps=max(speed), resumed_splines=gate['resumed_splines'],
                release_commands=gate['release_commands'])
    metrics['runs'].append(item)
    (out / (label + '_gate.json')).write_text(json.dumps(gate, indent=2) + '\n')
    for ax, values, title in zip(axes[i], [displacement, goal_distance, height],
                                  ['Distance from start (m)', 'Original-goal distance (m)', 'Reference height (m)']):
        ax.plot(t, values, linewidth=1.5)
        if onset is not None:
            ax.axvline(onset, color='orange', linestyle=':', linewidth=1)
        if resume is not None:
            ax.axvline(resume, color='green', linestyle='--', linewidth=1)
        ax.grid(alpha=.25)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('Seconds from first CSV sample')
    axes[i, 0].set_ylabel(label + '\n' + gate['status'])
    axes[i, 1].axhline(.1, color='gray', linestyle=':', linewidth=1)
    axes[i, 2].axhline(2.98, color='gray', linestyle=':', linewidth=1)
fig.suptitle('Navigation recovery: retained failure + three fixed cases\n'
             'Synthetic plant; orange: first recovery sample; green: first ordinary resume spline', fontsize=12)
fig.savefig(out / 'recovery_curves.png', dpi=140)
plt.close(fig)
(out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
print(json.dumps(metrics, indent=2))
