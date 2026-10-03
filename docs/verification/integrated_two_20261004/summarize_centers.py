#!/usr/bin/env python3
"""Refresh a supplemental center/video summary from completed per-seed artifacts."""
import json
import csv
import collections
import statistics
from pathlib import Path

D = Path(__file__).resolve().parent
results = []
for seed in (31,38):
    folder = D/f'{seed}_centers'
    required = [folder/'center_summary.json', folder/'stages/phase_summary.json',
                D/f'{seed}_mapped_overlay.json', D/f'{seed}_mapped_overlay.validation.json']
    if not all(p.exists() for p in required):
        continue
    center, phases, video, validation = [json.loads(p.read_text()) for p in required]
    if validation['status'] != 'PASS':
        raise ValueError(f'Seed {seed} video validation is not PASS')
    far = collections.defaultdict(list)
    with (folder/'stages/center_samples_by_phase.csv').open() as f:
        for row in csv.DictReader(f):
            if row['exclusion'] or row['duplicate']=='True' or row['fresh']!='True' or row['in_mission']!='True':
                continue
            if float(row['nearest_any_distance_m']) > .5:
                far[(row['stream'],row['source_phase'],row['class_name'])].append(float(row['nearest_any_distance_m']))
    distant = [dict(stream=k[0],phase=k[1],class_name=k[2],n=len(v),
                    median_nearest_any_m=statistics.median(v),max_nearest_any_m=max(v)) for k,v in sorted(far.items())]
    results.append(dict(seed=seed, run=center['run'], gate=center['gate_status'],
        mapped_H_landing=[r for r in center['statistics'] if r['scope']=='fresh_H_landing_mode' and r['stream']=='mapped'],
        mapped_H_source_landing=[r for r in phases['phase_statistics'] if r['scope']=='all_associations' and r['stream']=='mapped' and r['phase']=='LANDING' and r['class_name']=='landing_pad'],
        selected_centers=[r for r in center['statistics'] if r['scope']=='fresh_nearest_class_agrees' and r['stream']=='selected'],
        class_confusions=[r for r in center['class_instance_associations'] if r['suspected_confusion_n']],
        mapped_phase_confusions=[r for r in phases['phase_statistics'] if r['scope']=='all_associations' and r['stream']=='mapped' and r['suspected_confusion_n']],
        far_from_all_truth_threshold_m=.5, far_from_all_truth=distant,
        first_high_survey=phases['first_high_survey'], survey_interrupt=phases['survey_interrupt'],
        low_reacquisition=phases['low_reacquisition'], final_gate_H_mark=phases['final_gate_H_mark'],
        video=video, video_validation=validation,
        links=dict(center_report=f'{seed}_centers/CENTER_REPORT.md',
                   phase_report=f'{seed}_centers/stages/PHASE_REPORT.md',
                   centers_csv=f'{seed}_centers/center_samples.csv',
                   phases_csv=f'{seed}_centers/stages/center_samples_by_phase.csv',
                   phase_json=f'{seed}_centers/stages/phase_summary.json',
                   final_inspection=f'{seed}_centers/inspection/inspection.json',
                   video=f'{seed}_mapped_overlay.mp4', video_validation=f'{seed}_mapped_overlay.validation.json',
                   timestamp_sidecar=f'{seed}_mapped_overlay.frames.csv')))
doc=dict(scope='Corrected formal runs only; excludes 002834 preparation and 003709 diagnostic.',
         source='4cb75a3b1308ef0e17fcf6c7a296c946e0d9e751',
         comparison='Different code/model/inflation/ceiling/H geometry from history; not strict A/B.',
         results=results, pending_seeds=[s for s in (31,38) if not any(r['seed']==s for r in results)])
(D/'center_results.json').write_text(json.dumps(doc,indent=2)+'\n')
lines=['# Corrected formal center and downward-video results','',doc['scope'],doc['comparison'],'',
       'All video entries below passed actual ffprobe and complete FFmpeg decode with one thread. Source video and bag files were not modified. The overlay uses recorded image-stamped detections, never offline inference.', '',
       'Initial high-view values come from first published SURVEY support and frozen SURVEY interrupt events. Final mutable top_hints is excluded. Nearest-any-instance identity is a diagnostic association, not independently verified classification ground truth.','']
for r in results:
    s=r['seed'];link=r['links'];v=r['video']
    lines += [f'## Seed {s}', '', f"Run: `{r['run']}`; Gate **{r['gate']}**.", '',
        f"[Center report]({link['center_report']}) | [Source-phase and frozen snapshots]({link['phase_report']}) | [Overlay MP4]({link['video']}) | [Validation]({link['video_validation']})", '',
        '| Low reacquisition event | XY error / m | Source ROS / s |', '|---|---:|---:|']
    for rec in r['low_reacquisition']:
        lines.append(f"| {rec['class_name']} | {rec['same_class_distance_m']:.5f} | {rec['source_ros_s']:.3f} |")
    lines += ['', 'H landing observations, with align mode evaluated at source image time:']
    for h in r['mapped_H_source_landing']:
        lines.append(f"- {h['same_class_instance']}: mapped N={h['n']}, median {h['median_same_class_m']:.5f} m, P95 {h['p95_same_class_m']:.5f} m.")
    lines.append('- Parent center_summary also retains receipt-time mode statistics; boundary samples can differ (seed38: 97 receipt-mode versus 95 source-mode H samples).')
    if r['final_gate_H_mark']:
        lines.append(f"- Final Gate H mark: {r['final_gate_H_mark']['same_class_distance_m']:.5f} m; distinct from the distribution of mapped observations.")
    lines += ['', 'Recorded mapped class/instance ambiguities:']
    conf=[c for c in r['class_confusions'] if c['stream']=='mapped']
    if not conf:lines.append('- None under the stated nearest-instance suspicion thresholds.')
    for c in conf:
        lines.append(f"- {c['predicted_class']} near {c['nearest_instance']}: N={c['n']}, median nearest distance {c['median_nearest_distance_m']:.5f} m versus same-class {c['median_same_class_distance_m']:.5f} m. Do not describe the latter as localization-only error.")
    for c in r['mapped_phase_confusions']:
        lines.append(f"- Phase {c['phase']}, align mode {c['align_mode']}: {c['class_name']} has {c['suspected_confusion_n']} suspected confusions among {c['n']} fresh unique observations; source ROS interval {c['first_source_ros_s']:.3f} to {c['last_source_ros_s']:.3f} s. Full phase interval includes non-confused observations where present.")
    lines += ['', 'Separate distance outliers (more than 0.5 m from every truth center; cause not inferred as class confusion):']
    if not r['far_from_all_truth']:
        lines.append('- None among fresh unique valid samples.')
    for c in r['far_from_all_truth']:
        lines.append(f"- {c['stream']} / {c['phase']} / {c['class_name']}: N={c['n']}, median nearest-any {c['median_nearest_any_m']:.4f} m, max {c['max_nearest_any_m']:.4f} m.")
    lines += ['',f"Video: {v['output_frames']} frames at {v['fps']:g} fps; {v['output_frames']/v['fps']:.1f} s. Visited source frames: {v['visited_source_frames']}; nonempty matched {v['frames_with_nonempty_match']}, empty matched {v['frames_with_empty_match']}, unmatched {v['frames_without_match']}. Maximum absolute image-stamp match difference {v['max_match_abs_delta_ms']:.3f} ms. Held frames are labelled; complete per-frame mapping is retained.",'']
if doc['pending_seeds']:
    lines += ['Pending formal processing: '+', '.join(map(str,doc['pending_seeds']))+'.','']
lines += ['Machine-readable data and relative links for the main index: [center_results.json](center_results.json).',
          'Final coverage, LAND publications, release association and long-tail diagnosis: [FINAL_CENTER_DIAGNOSTICS.md](FINAL_CENTER_DIAGNOSTICS.md).',
          'Raw CSV includes invalid/stale/duplicate rows; use the report filters. No sample means missing evidence, not zero error.']
(D/'CENTER_RESULTS.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(dict(completed_seeds=[r['seed'] for r in results], pending_seeds=doc['pending_seeds'], output=str(D/'CENTER_RESULTS.md'))))
