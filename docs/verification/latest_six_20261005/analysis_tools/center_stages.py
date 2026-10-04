#!/usr/bin/env python3
"""Add source-time phases and frozen survey snapshots to recorded center results."""
import argparse
import bisect
import collections
import csv
import json
import math
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--run', type=Path, required=True)
p.add_argument('--centers', type=Path, required=True)
p.add_argument('--data', type=Path, required=True)
p.add_argument('--truth-local-ground-z', type=float, required=True)
p.add_argument('--high-min-agl', type=float, default=2.0)
p.add_argument('--pose-max-gap', type=float, default=.5)
a = p.parse_args()
out = a.centers / 'stages'
out.mkdir(exist_ok=False)
base = json.loads((a.centers / 'center_summary.json').read_text())
truth = base['truth']
events = [json.loads(x) for x in (a.run/'high_view_full_events.jsonl').read_text().splitlines() if x.strip()]
events.sort(key=lambda e: e['t'])
gate = json.loads((a.run/'gate_status.json').read_text())
poses = list(csv.DictReader((a.run/'truth_pose.csv').open()))
poses.sort(key=lambda r: float(r['t']))
pt = [float(r['t']) for r in poses]
mode = []
with a.data.open() as f:
    for line in f:
        e = json.loads(line)
        if e.get('kind') == 'mode':
            mode.append((e['t'], e['m'].get('data', 'UNKNOWN')))
mode.sort()
mt = [r[0] for r in mode]

def asof(times, values, t, default):
    i = bisect.bisect_right(times, t)-1
    return values[i][1] if i >= 0 else default

def agl(t):
    i = bisect.bisect_right(pt, t)-1
    if i < 0 or i+1 >= len(pt):
        return None
    span = pt[i+1]-pt[i]
    if span <= 0 or span > a.pose_max_gap:
        return None
    z = float(poses[i]['z'])+(float(poses[i+1]['z'])-float(poses[i]['z']))*(t-pt[i])/span
    return z-a.truth_local_ground_z

# Freeze events on first publication, never take the final mutable top_hints.
frozen_events = []
seen_events = set()
status_changes = []
prev_stage = None
for e in events:
    s = e['status']
    stage = s.get('stage', 'UNKNOWN')
    if stage != prev_stage:
        candidates = [float(v['time']) for v in s.get('events', [])
                      if v.get('stage') == stage and 'time' in v and float(v['time']) <= e['t']]
        t = max(candidates) if candidates else float(e['t'])
        status_changes.append(dict(t=t, stage=stage, first_publication_ros_s=e['t'],
                                   timing_source='embedded event time' if candidates else 'status publication time'))
        prev_stage = stage
    for v in s.get('events', []):
        key = (v.get('stage'), v.get('time'))
        if key not in seen_events:
            frozen_events.append(dict(first_publication_ros_s=e['t'], event=v))
            seen_events.add(key)
status_changes.sort(key=lambda x: x['t'])
stages = [(v['t'], v['stage']) for v in status_changes]
st = [v[0] for v in stages]

def phase(t):
    stage = asof(st, stages, t, 'UNKNOWN')
    align = asof(mt, mode, t, 'UNKNOWN')
    height = agl(t)
    if align == 'landing':
        result = 'LANDING'
    elif stage == 'SURVEY':
        result = 'HIGH_SURVEY' if height is not None and height >= a.high_min_agl else 'SURVEY_BELOW_HIGH_OR_NO_POSE'
    else:
        result = stage
    return dict(source_stage=stage, source_align_mode=align, source_phase=result, source_fc_agl_m=height)

def association(cls, xy):
    all_dist = [(math.hypot(xy[0]-t['eval_xy'][0], xy[1]-t['eval_xy'][1]), t) for t in truth]
    same = [r for r in all_dist if r[1]['class_name'] == cls]
    distance, near = min(all_dist, key=lambda x: x[0])
    sd, own = min(same, key=lambda x: x[0]) if same else (None, {})
    return dict(nearest_any_instance=near['name'], nearest_any_class=near['class_name'],
                nearest_any_distance_m=distance, same_class_instance=own.get('name'), same_class_distance_m=sd,
                category_confusion_suspected=bool(sd is not None and near['class_name'] != cls and
                    distance <= base['confusion_radius_m'] and sd-distance >= base['confusion_margin_m']))

def snapshot(kind, cls, hint, pub, event_time=None):
    t = float(hint.get('source_stamp_ns', hint.get('last_seen_ns', 0)))/1e9
    xy = list(hint['xy'])
    return dict(kind=kind, class_name=cls, source=hint.get('source', 'vision'), xy=xy,
                source_ros_s=t, source_mission_s=t-base['mission_start_ros_s'],
                first_publication_ros_s=pub, event_ros_s=event_time,
                source_age_at_publication_s=pub-t, raw_hint=hint, **phase(t), **association(cls, xy))

first = []
seen_instances = set()
first_class = set()
for e in events:
    s = e['status']
    if s.get('stage') != 'SURVEY':
        continue
    for cls, hints in s.get('navigation_support', {}).items():
        for h in hints:
            row = snapshot('first_high_survey_by_nearest_instance', cls, h, e['t'])
            key = (cls, row['nearest_any_instance'])
            if row['source_phase'] != 'HIGH_SURVEY' or key in seen_instances:
                continue
            row['first_high_observation_of_class'] = cls not in first_class
            first.append(row)
            first_class.add(cls)
            seen_instances.add(key)
interrupt = []
for v in frozen_events:
    ev = v['event']
    if ev.get('stage') == 'SURVEY_INTERRUPTED_TOP3':
        for cls, hints in ev.get('support', {}).items():
            for h in hints:
                interrupt.append(snapshot('survey_interrupt_frozen_support', cls, h,
                                          v['first_publication_ros_s'], ev['time']))
reacq = []
seen_reacq = set()
for e in events:
    for h in e['status'].get('reacquisitions', []):
        key = (h['class_name'], h.get('target_id'), h.get('time'))
        if key in seen_reacq:
            continue
        reacq.append(snapshot('low_reacquisition_event', h['class_name'], h, e['t'], h.get('time')))
        seen_reacq.add(key)

grouped = collections.defaultdict(list)
row_counts = collections.Counter()
with (a.centers/'center_samples.csv').open() as f, (out/'center_samples_by_phase.csv').open('x', newline='') as g:
    reader = csv.DictReader(f)
    writer = csv.DictWriter(g, fieldnames=reader.fieldnames+['source_stage', 'source_align_mode', 'source_phase', 'source_fc_agl_m'])
    writer.writeheader()
    for r in reader:
        r.update(phase(float(r['source_ros_s'])))
        writer.writerow(r)
        row_counts['all_rows'] += 1
        if r['exclusion'] or r['duplicate'] == 'True' or r['fresh'] != 'True' or r['in_mission'] != 'True':
            continue
        row_counts['fresh_unique_valid'] += 1
        for scope in ['all_associations']+(['nearest_class_agrees'] if r['class_name'] == r['nearest_any_class'] else []):
            key = (scope, r['stream'], r['source_phase'], r['source_align_mode'], r['class_name'], r['truth_name'])
            grouped[key].append(r)
stats = []
for key, rows in sorted(grouped.items()):
    errors = np.array([float(r['xy_error_m']) for r in rows])
    near = np.array([float(r['nearest_any_distance_m']) for r in rows])
    stats.append(dict(zip(['scope','stream','phase','align_mode','class_name','same_class_instance'], key),
        n=len(rows), median_same_class_m=float(np.median(errors)), p95_same_class_m=float(np.percentile(errors,95)),
        median_nearest_any_m=float(np.median(near)),
        suspected_confusion_n=sum(r['category_confusion_suspected']=='True' for r in rows),
        first_source_ros_s=min(float(r['source_ros_s']) for r in rows),
        last_source_ros_s=max(float(r['source_ros_s']) for r in rows)))
with (out/'phase_statistics.csv').open('x', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=list(stats[0]) if stats else ['scope'])
    writer.writeheader(); writer.writerows(stats)
mark = gate.get('metrics', {}).get('valid_landing_h_mark')
mark_result = None
if mark:
    mark_result = dict(recorded_mark=mark, **association('landing_pad', [mark['x'],mark['y']]))
result = dict(run=str(a.run), gate_status=gate.get('status'),
    method='First published SURVEY support and embedded interrupt/reacquisition events. Never final top_hints.',
    phase_method='Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.',
    stage_publication_limit='Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.',
    truth_local_ground_z=a.truth_local_ground_z, high_min_agl=a.high_min_agl,
    status_transitions=status_changes, first_high_survey=first, survey_interrupt=interrupt,
    low_reacquisition=reacq, final_gate_H_mark=mark_result, counts=row_counts, phase_statistics=stats,
    frozen_interrupt_events=[v for v in frozen_events if v['event'].get('stage')=='SURVEY_INTERRUPTED_TOP3'])
(out/'phase_summary.json').write_text(json.dumps(result, indent=2)+'\n')
lines=['# Centers by observation phase and frozen high-view snapshots', '',
       'Run: '+str(a.run), '',
       'These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.', '',
       result['phase_method'], result['stage_publication_limit'], '',
       'Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.', '']
for title, rows in [('First high SURVEY observations (per predicted class / nearest instance)',first),
                    ('Frozen SURVEY interrupt support (all hypotheses retained)',interrupt),
                    ('Low reacquisition events',reacq)]:
    lines += ['## '+title,'', '| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |',
              '|---|---:|---:|---:|---|---:|---|---:|---|']
    for r in rows:
        height='NA' if r['source_fc_agl_m'] is None else f"{r['source_fc_agl_m']:.3f}"
        own='NA' if r['same_class_distance_m'] is None else f"{r['same_class_distance_m']:.4f}"
        lines.append(f"| {r['class_name']} | {r['source_ros_s']:.3f} | {r['first_publication_ros_s']:.3f} | {height} | ({r['xy'][0]:.4f}, {r['xy'][1]:.4f}) | {own} | {r['nearest_any_instance']} | {r['nearest_any_distance_m']:.4f} | {r['category_confusion_suspected']} |")
    lines.append('')
lines += ['## Fresh unique valid centers by source phase','',
          'All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.', '',
          '| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |',
          '|---|---|---|---|---:|---:|---:|---:|---:|']
for r in stats:
    if r['scope']=='all_associations':
        lines.append('| {stream} | {phase} | {align_mode} | {class_name} | {n} | {median_same_class_m:.4f} | {p95_same_class_m:.4f} | {median_nearest_any_m:.4f} | {suspected_confusion_n} |'.format(**r))
lines += ['', '## Final Gate H mark', '', json.dumps(mark_result, indent=2) if mark_result else 'No valid H mark recorded.', '',
          'See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.']
(out/'PHASE_REPORT.md').write_text('\n'.join(lines)+'\n')
parent=a.centers/'CENTER_REPORT.md'
text=parent.read_text()
notice='Phase-specific interpretation: [frozen SURVEY, interrupt and low reacquisition comparison](stages/PHASE_REPORT.md). The full-mission statistics below are not initial-high or low-only values.\n\n'
parent.write_text(text.replace('\n\n','\n\n'+notice,1))
print(json.dumps(dict(output=str(out), first_high=first, interrupt=interrupt, low_reacquisition=reacq,
                      final_H=mark_result, phase_statistics_rows=len(stats)), indent=2))
