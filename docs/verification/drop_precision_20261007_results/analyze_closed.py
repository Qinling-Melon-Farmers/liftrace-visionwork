#!/usr/bin/env python3
"""Offline adapter; read closed artifacts and write a new result directory only."""
import argparse
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OLD = ROOT / 'docs/verification/drop_precision_20261006'
sys.dont_write_bytecode = True
sys.path.insert(0, str(OLD))
import precision_eval as ev


def dump(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed', type=int, choices=(31, 38), required=True)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--closed', action='store_true', required=True,
                   help='Use only after operator confirms this run is closed')
    p.add_argument('--stop-record', type=Path)
    p.add_argument('--output-dir', type=Path, help='New result directory; existing directories are never overwritten')
    a = p.parse_args()
    run = a.run.resolve()
    out = a.output_dir.resolve() if a.output_dir else HERE / f'seed{a.seed}'
    if out.exists():
        p.error('Result directory already exists; no overwrite')
    matrix = ev.read_json(ROOT / 'logs/snake3_camera2m_20261005_batch/matrix.json')
    row = next(r for r in matrix['results'] if r['seed'] == a.seed and r['variant'] == 'snake3')
    report = ROOT / 'docs/verification/snake3_camera2m_20261005'
    if 'scene' in row:
        scene = Path(row['scene'])
    else:
        scenes = ev.read_json(report / 'scenes.json')
        scene = ROOT / next(s['scene'] for s in scenes if s['seed'] == a.seed and s['variant'] == 'snake3')
    baseline_run = Path(row['run'])
    if not baseline_run.is_absolute():
        baseline_run = ROOT / baseline_run
    baseline = ev.evaluate(baseline_run, scene)
    gate_path = run / 'gate_status.json'
    stop = None
    if not gate_path.exists():
        if a.stop_record is None:
            p.error('Missing Gate: supply operator closure record; no inferred verdict')
        stop = ev.read_json(a.stop_record)
        if Path(stop.get('run', '')).resolve() != run:
            p.error('Closure record must name this run')
        original = ev.read_json
        ev.read_json = lambda path: {} if Path(path) == gate_path else original(path)
        try:
            current = ev.evaluate(run, scene)
        finally:
            ev.read_json = original
    else:
        current = ev.evaluate(run, scene)
    current['gate_file_present'] = gate_path.exists()
    current['operator_stop_record'] = stop
    previous = ev.read_json(OLD / f'results_seed{a.seed}_capture/precision.json')
    records = list(ev.events(run / 'key_events.jsonl'))
    recoveries = [r for r in records if r.get('kind') == 'result' and
                  r.get('data', {}).get('terminal') and
                  r['data'].get('reason') == 'release_recovery_motion_handoff']
    gate = ev.read_json(gate_path) if gate_path.exists() else None
    contacts = ev.read_json(run / 'gazebo_contact_status.json')
    provenance = dict(expected_head='66c36a9e', controller_fix='1c700dd3',
                      operator_version_note='Entry fix only; control and camera unchanged from 074f8381 per operator',
                      run=str(run), frozen_scene=str(scene),
                      historical_source=matrix.get('source'),
                      calibration_note='Control and simulated calibration both changed; not pure algorithm attribution',
                      recovery_events=recoveries, original_gate=gate,
                      contacts=contacts,
                      physical_completion='Review contact/landed/disarm sequence separately; not inferred from Gate',
                      artifact_paths={name: str(run / name) if (run / name).exists() else None
                                      for name in ('manifest.yaml', 'actual_camera_info.json', 'rosparams.yaml')})
    comparison = []
    for group, result in [('historical_snake3', baseline), ('previous_capture', previous), ('current', current)]:
        for slot in (1, 2, 3):
            drops = [d for d in result['drops'] if d['slot'] == slot]
            if not drops:
                comparison.append(dict(group=group, seed=a.seed, slot=slot, ack=False))
            for d in drops:
                ack = d['completed']; pose = ack.get('pose', {})
                near = d.get('legacy_ack_receipt_nearest', {})
                comparison.append(dict(group=group, seed=a.seed, slot=slot, ack=True,
                    identity=d['identity'], requested_class=d['class_name'],
                    actual_nearest_class=pose.get('nearest_class'), class_matches=pose.get('label_matches'),
                    source_stamp=ack.get('source_ros_s'), receipt_minus_source_s=ack.get('receipt_minus_source_s'),
                    source_nearest_error_m=pose.get('nearest_error_m'),
                    source_same_class_error_m=pose.get('same_class_error_m'),
                    interpolation_status=pose.get('status'), bracket_gap_s=pose.get('bracket_gap_s'),
                    nearest_age_s=pose.get('nearest_age_s'), legacy_receipt_nearest_m=near.get('error_m')))
    out.mkdir(parents=True)
    for name, data in [('precision.json', current), ('historical.json', baseline),
                       ('comparison.json', comparison), ('completion_evidence.json', provenance)]:
        dump(out / name, data)
    columns = list(dict.fromkeys(key for item in comparison for key in item))
    with (out / 'comparison.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader(); writer.writerows(comparison)
    print(json.dumps(dict(output=str(out), completed=current['completed_transactions'],
                         gate=current['raw_gate'], recoveries=len(recoveries),
                         collisions=current['actual_collision_count']), indent=2))


if __name__ == '__main__':
    main()
