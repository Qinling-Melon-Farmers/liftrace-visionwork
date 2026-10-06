#!/usr/bin/env python3
"""Final offline comparison; write only new compact evaluation artifacts."""
import csv
import json
import math
import shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
DOC = ROOT / 'docs/verification/drop_precision_20261006'
OUT = DOC / 'results_capture'
HEAD = '7d312fa252d02ee937640839b6aac8d74bd11ad7'
OLD = {31: {'red_cross': 3.317, 'bridge': 11.420, 'panzer': 4.815},
       38: {'red_cross': 1.941, 'bridge': 7.725, 'panzer': 13.982}}
SOURCE_OLD = {31: {'red_cross': 3.474, 'bridge': 11.431, 'panzer': 4.871},
              38: {'red_cross': 1.861, 'bridge': 7.927, 'panzer': 13.972}}
RUNS = {31: 'drop_precision_seed31_20261007_020102',
        38: 'drop_precision_seed38_20261007_022323'}


def read(path):
    return json.loads(path.read_text())


def events(run):
    return [json.loads(line) for line in (run / 'key_events.jsonl').read_text().splitlines()]


def csv_rows(path):
    with path.open() as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


rows = []
cases = {}
for seed, name in RUNS.items():
    run = ROOT / 'logs' / name
    precision = read(DOC / f'results_seed{seed}_capture/precision.json')
    es = events(run)
    first_decision = next(e for e in es if e['kind'] == 'decision')
    stamp = first_decision['data']['header']['stamp']
    start = stamp['stamp_ns']/1e9 if 'stamp_ns' in stamp else stamp['secs']+stamp['nsecs']/1e9
    recovery = [dict(ros_s=e['ros_sec'], decision=e['data']['decision_seq']) for e in es
                if e['kind'] == 'result' and e['data'].get('terminal')
                and e['data'].get('reason') == 'release_recovery_motion_handoff']
    for d in precision['drops']:
        cls = d['class_name']
        decisions = [e for e in es if e['kind'] == 'decision'
                     and e['data'].get('target_class') == cls and e['data'].get('command') == 1]
        ack = d['completed']['source_ros_s']
        rows.append(dict(seed=seed, target=cls, slot=d['slot'], ack_source_ros_s=ack,
                         ack_task_s=ack-start, approach_start_ros_s=decisions[0]['ros_sec'] if decisions else None,
                         old_ack_nearest_cm=OLD[seed][cls], new_ack_nearest_cm=d['legacy_ack_receipt_nearest']['error_m']*100,
                         old_ack_source_interpolated_cm=SOURCE_OLD[seed][cls], new_ack_source_interpolated_cm=d['completed']['pose']['same_class_error_m']*100,
                         correct_class=d['completed']['pose']['label_matches'], inside_nominal_board=d['completed']['pose']['inside_nominal_board']))
    own = [r for r in rows if r['seed'] == seed]
    cases[seed] = dict(run=str(run), source=HEAD, final_status='PASS' if seed == 31 else 'INTERRUPTED_DIAGNOSTIC_FAIL',
                       raw_gate_status=precision['raw_gate'], gate_file_present=(run/'gate_status.json').exists(),
                       successful_releases=len(own), recoveries=recovery, collision_count=precision['actual_collision_count'],
                       task_start_ros_s=start, old_matched_mean_cm=float(np.mean([r['old_ack_nearest_cm'] for r in own])),
                       new_matched_mean_cm=float(np.mean([r['new_ack_nearest_cm'] for r in own])),
                       old_matched_max_cm=max(r['old_ack_nearest_cm'] for r in own), new_matched_max_cm=max(r['new_ack_nearest_cm'] for r in own),
                       old_physical_touchdown_task_s={31:204.517,38:363.540}[seed],
                       new_physical_touchdown_task_s=213.831-start if seed == 31 else None,
                       exact_path=read(DOC/f'results_seed{seed}_capture/exact_path_summary.json'),
                       effective_flags=dict(exact_visual=True, exact_control=True, circle_quality_ordered_nms=False, slot_mode='zero', stop_on_collision=False),
                       cleanup='wrapper_PASS_zero_residual' if seed == 31 else 'original_FAIL_then_supplementary_zero_check_PASS')
    if seed == 31:
        gate = read(run/'gate_status.json')
        cases[seed]['gate_checks'] = {k: gate.get(k) for k in ['status', 'reason', 'failed_checks']}
        cases[seed]['gate_metrics'] = {k:gate.get('metrics',{}).get(k) for k in ['release_commit_count','capture_started_count','recovery_success_count','post_delivery_return_success_count','actual_collision_count','latest_armed','latest_landed_state','mission_ros_sec','mission_wall_sec','simulation_realtime_factor','door_crossings']}
    else:
        cases[seed]['third_slot'] = dict(approach_arrival=113.366, alignment_accepted=113.382, strict_valid=114.998,
            arbiter_anchor_approx_xy=[7.0068593025,-4.2417464256], captured_source=114.943, capture_entry=115.022,
            captured_xy=[7.018559384473214,-4.227047670587916], captured_tolerance_m=.059202443808317184,
            last_fixed_goal_log=118.670, exact_rejection=118.686, first_updated_xy_log=118.721,
            jumped_offset_source=118.977, jumped_offset_receipt=119.002, first_clamped_changed_goal_log=119.020,
            first_permission_granted_receipt=121.102, first_drift_denial_source=121.301, first_drift_denial_receipt=121.306,
            stopped_ros_s=read(DOC/'results_seed38_capture/stop_request.json')['last_progress']['ros_sec'],
            no_successful_recapture_observed=True, trigger='Generic rejection; future 3ms context is consistent but not a proven predicate')

summary = dict(source=HEAD, baseline_source='337b583689f46d907546d1d28a8fc52b13f47398', cases=cases, rows=rows,
    conclusion='DO_NOT_PROMOTE_DEFAULT', tested_release_count=len(rows), worse_than_paired_old_count=sum(r['new_ack_nearest_cm']>r['old_ack_nearest_cm'] for r in rows),
    matched_old_mean_cm=float(np.mean([r['old_ack_nearest_cm'] for r in rows])), matched_new_mean_cm=float(np.mean([r['new_ack_nearest_cm'] for r in rows])),
    matched_old_max_cm=max(r['old_ack_nearest_cm'] for r in rows), matched_new_max_cm=max(r['new_ack_nearest_cm'] for r in rows),
    missing_release='seed38 slot3 panzer: no error assigned; never imputed as zero',
    metric_note='Model/body origin at mock ACK, not measured parcel detachment/impact. Historical nearest-receipt and source-interpolated columns are separate.')
(OUT/'metrics.json').write_text(json.dumps(summary,indent=2)+'\n')
with (OUT/'release_comparison.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
for src,dst in [('drop31_ack_decomposition.json','results_seed31_capture/ack_decomposition.json'),
                ('drop31_ack_decomposition_notes.md','results_seed31_capture/ACK_DECOMPOSITION.md'),
                ('drop38_slot3_commitment_notes.md','results_seed38_capture/SLOT3_DIAGNOSIS.md')]:
    path=Path('/tmp')/src
    if path.exists():shutil.copyfile(path,DOC/dst)

fig,axes=plt.subplots(1,2,figsize=(11,4.7),sharey=True)
for ax,seed in zip(axes,[31,38]):
    labels=['red_cross','bridge','panzer']; xs=np.arange(3)
    old=[OLD[seed][c] for c in labels]
    new=[next((r['new_ack_nearest_cm'] for r in rows if r['seed']==seed and r['target']==c),np.nan) for c in labels]
    ax.bar(xs-.18,old,.36,label='Historical paired ACK',color='#657c96')
    ax.bar(xs+.18,new,.36,label='Candidate ACK',color='#e59843')
    for x,v in zip(xs,old):ax.text(x-.18,v+.25,f'{v:.2f}',ha='center',fontsize=9)
    for x,v in zip(xs,new):
        if math.isfinite(v):ax.text(x+.18,v+.25,f'{v:.2f}',ha='center',fontsize=9)
        else:ax.text(x,2,'NO RELEASE\n(not zero)',ha='center',color='#b33b3b',fontsize=9)
    ax.set_xticks(xs,labels);ax.set_title(f'Seed {seed}: '+('full PASS' if seed==31 else 'diagnostic FAIL, 2 releases'))
    ax.grid(axis='y',alpha=.2);ax.set_ylim(0,18)
axes[0].set_ylabel('Body origin error at ACK [cm]');axes[1].legend(loc='upper right',fontsize=8)
fig.suptitle('Same nearest-receipt truth metric: 4 of 5 measured releases worsened')
fig.tight_layout();fig.savefig(OUT/'release_comparison.png',dpi=170);plt.close(fig)

r=ROOT/'logs'/RUNS[38]; pose=csv_rows(r/'mavros_pose.csv'); sp=csv_rows(r/'mavros_setpoint.csv')
anchor=np.array(cases[38]['third_slot']['arbiter_anchor_approx_xy']); goal=np.array(cases[38]['third_slot']['captured_xy'])
fig,axes=plt.subplots(2,1,figsize=(11,6.5),sharex=True)
for data,label,color in [(pose,'FC distance from initial anchor','#2685b5'),(sp,'Emitted setpoint distance from capture goal','#c47e24')]:
    a=np.array([[v['t'],v['x'],v['y'],v['z']] for v in data if 113<=v['t']<=164])
    origin=anchor if data is pose else goal
    axes[0].plot(a[:,0],np.linalg.norm(a[:,1:3]-origin,axis=1),label=label,color=color)
axes[0].axhline(.2,color='#b33b3b',linestyle='--',label='Existing arbiter drift limit 0.20m')
offsets=[json.loads(l) for l in (DOC/'results_seed38_capture/exact_offsets.jsonl').read_text().splitlines()]
a=np.array([[e['m']['header']['stamp']['secs']+e['m']['header']['stamp']['nsecs']/1e9,e['m']['radius_px']] for e in offsets if e.get('kind')=='drop_offset'])
a=a[(a[:,0]>=113)&(a[:,0]<=164)]
axes[1].plot(a[:,0],a[:,1],color='#7350a2',label='Recorded ring radius [px]')
for ax in axes:
    for t,label in [(115.022,'capture'),(118.686,'reject / latch lost'),(121.306,'first drift denial')]:ax.axvline(t,linestyle=':',alpha=.6,label=label)
    ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
axes[0].set_ylabel('XY distance [m]');axes[1].set_ylabel('Ring radius [px]');axes[1].set_xlabel('Absolute ROS seconds (CSV recorder time / offset exposure time)')
fig.suptitle('Seed38 slot3: rejected observation exits frozen descent; no second capture')
fig.tight_layout();fig.savefig(DOC/'results_seed38_capture/panzer_block_timeline.png',dpi=170);plt.close(fig)
print(json.dumps({k:summary[k] for k in ['tested_release_count','worse_than_paired_old_count','matched_old_mean_cm','matched_new_mean_cm','matched_old_max_cm','matched_new_max_cm','conclusion']}))
