#!/usr/bin/env python3
"""Analytic-motion fault injection. Synthetic evidence, NOT flight validation."""
import argparse
import csv
import json
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    'patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts'))
from ev_predictor import Predictor, State, Imu


def truth(t):
    return (np.array([.4*np.sin(t), .2*np.cos(t)-.2, 1.+.1*np.sin(.8*t)]),
            np.array([.4*np.cos(t), -.2*np.sin(t), .08*np.cos(.8*t)]),
            np.array([-.4*np.sin(t), -.2*np.cos(t), -.064*np.sin(.8*t)]))


def experiment(dropout=False):
    p=Predictor()
    snapshots=[]
    for t in np.arange(.02, 3., .1):
        if dropout and 1.0 <= t <= 1.65:
            continue
        pos, vel, _=truth(t)
        # Deliberately introduce a 3.5 cm small LIO correction. Do not smooth
        # physical acceleration. Delays are known because this is synthetic.
        pos=pos+np.array([0., 0., .035 if t >= 1.2 else 0.])
        s=State(10+t, pos, np.eye(3), vel, np.zeros(3), np.zeros(3),
                np.array([0., 0., -9.81]), np.eye(18)*1e-5, np.ones(12)*1e-4)
        delay=.18 if .7 <= t <= 1.0 else .075
        snapshots.append((10+t+delay, s))
    snapshots.sort(key=lambda x: x[0])
    rows=[]; costs=[]; last_stamp=None
    for i in range(601):
        t=10+i*.005
        pos, vel, acc=truth(t-10)
        p.add_imu(Imu(t, acc+np.array([0., 0., 9.81]), np.zeros(3)), t)
        while snapshots and snapshots[0][0] <= t+1e-9:
            _, s=snapshots.pop(0)
            p.correction(s, t)
        if i % 4:
            continue
        start=time.perf_counter()
        pair=p.output(t)
        costs.append(time.perf_counter()-start)
        if pair and (last_stamp is None or pair[0].t > last_stamp):
            raw, smooth=pair; last_stamp=raw.t
            reference=truth(raw.t-10)[0]
            rows.append([t-10, raw.t-10, raw.p[2], smooth.p[2], reference[2],
                         np.linalg.norm(smooth.p-reference), t-p.correction_t])
    a=np.asarray(rows)
    return rows, dict(synthetic=True, dropout=dropout, output_count=len(rows),
        first_output_sec=float(a[0, 0]), last_output_sec=float(a[-1, 0]),
        max_output_gap_sec=float(np.diff(a[:, 0]).max()),
        max_pose_error_m=float(a[:, 5].max()), final_fault=p.fault,
        compute_p95_ms=float(np.percentile(costs,95)*1000), stats=p.stats)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args=parser.parse_args(); args.out.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    results={}
    for ax, dropout in zip(axes, (False, True)):
        name='radar_outage' if dropout else 'delayed_corrections'
        rows, results[name]=experiment(dropout)
        with (args.out/(name+'.csv')).open('w') as f:
            writer=csv.writer(f); writer.writerow(['receipt_s','sample_s','raw_z','smooth_z','truth_z','error_m','correction_age_s']); writer.writerows(rows)
        a=np.asarray(rows)
        ax.plot(a[:,0], a[:,4], label='analytic truth')
        ax.plot(a[:,0], a[:,2], label='IMU prediction + delayed LIO correction')
        ax.plot(a[:,0], a[:,3], label='same-time correction blend')
        if dropout:
            ax.axvspan(1., 1.65, color='red', alpha=.1, label='no LIO corrections')
        ax.set_title(name+' (synthetic, no PX4)'); ax.set_ylabel('height (m)'); ax.legend(fontsize=8)
    axes[-1].set_xlabel('time (s)'); fig.tight_layout()
    fig.savefig(args.out/'synthetic_prediction.png', dpi=130)
    (args.out/'synthetic_results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))
    assert results['delayed_corrections']['final_fault'] is None
    assert results['delayed_corrections']['max_pose_error_m'] < .04
    assert results['radar_outage']['final_fault'] == 'correction_timeout_requires_restart'
    assert results['radar_outage']['last_output_sec'] < 1.3


if __name__ == '__main__':
    main()
