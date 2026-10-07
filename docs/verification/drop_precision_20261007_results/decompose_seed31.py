"""Local offline calculation; no ROS nodes and no truth-based parameter fitting."""
import bisect
import argparse
import csv
import json
import math
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / 'seed31'
ROOT = HERE.parents[2]
RUN = ROOT / 'logs/drop_precision_new_seed31_20261007_135022'


def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def stamp(d):
    return d['secs'] + d['nsecs'] / 1e9


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def interpolate(records, t):
    times = [p[0] for p in records]
    i = bisect.bisect_right(times, t)
    assert 0 < i < len(times)
    lo, hi = records[i-1], records[i]
    gap = hi[0]-lo[0]
    assert 0 < gap <= .25
    k = (t-lo[0])/gap
    return dict(xyz=[x+k*(y-x) for x, y in zip(lo[1], hi[1])],
                bracket=[lo[0], hi[0]], gap_s=gap,
                nearest_age_s=min(t-lo[0], hi[0]-t),
                endpoint_xy_span_m=math.dist(lo[1][:2], hi[1][:2]))


def main():
    global OUT, RUN
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, default=OUT,
                        help='Directory containing precision.json and the three offline bag exports')
    args = parser.parse_args()
    OUT = args.data_dir.resolve()
    precision = json.loads((OUT / 'precision.json').read_text())
    RUN = Path(precision['run'])
    offsets = [r for r in rows(OUT / 'exact_offsets.jsonl') if r['kind'] == 'drop_offset']
    contexts = rows(OUT / 'alignment_context.jsonl')
    fc = rows(OUT / 'fc_pose_source.jsonl')
    fc_interp = sorted({r['source']: r['xyz'] for r in fc}.items())
    with (RUN / 'truth_pose.csv').open() as f:
        gt = [(float(r['t']), [float(r[k]) for k in ('x', 'y', 'z')]) for r in csv.DictReader(f)]
    with (RUN / 'mavros_setpoint.csv').open() as f:
        sp = [(float(r['t']), [float(r[k]) for k in ('x', 'y', 'z')]) for r in csv.DictReader(f)]
    result = []
    for drop in precision['drops']:
        ack = drop['completed']['source_ros_s']
        last = max((s for s in sp if ack-1 <= s[0] <= ack), key=lambda s: s[0])
        emitted_goal = last[1]
        assert abs(emitted_goal[2] - f32(.1)) < 1e-10
        matches = [o for o in offsets if ack-12 < o['t'] < ack and
                   [f32(o['m']['map_point'][k]) for k in ('x', 'y')] == emitted_goal[:2]]
        assert len(matches) == 1, ('Capture source ambiguous', drop['class_name'], len(matches))
        offset = matches[0]
        source = stamp(offset['m']['header']['stamp'])
        goal = [offset['m']['map_point'][k] for k in ('x', 'y')]
        context = max((c for c in contexts if c['t'] <= offset['t']), key=lambda c: c['t'])['m']
        assert context['active'] and context['semantic_target_class'] == drop['class_name']
        assert (context['mission_id'], context['decision_seq'], context['attempt'], context['payload_slot']) == tuple(drop['identity'][:4])
        assert context['semantic_target_id'] == drop['target_id']
        descent = [s for s in sp if offset['t'] <= s[0] <= ack]
        # Production's pre-existing 3D command limiter; fixed goal throughout.
        lead = f32(.4 if drop['class_name'] == 'panzer' else .2)
        residuals = []
        direct = 0
        for time, command in descent:
            candidates = [p for p in fc if time-.12 <= p['t'] <= time+.002]
            assert candidates
            replay = []
            for p in candidates:
                delta = sub(emitted_goal, p['xyz'])
                distance = math.sqrt(sum(v*v for v in delta))
                ratio = min(1., lead/distance) if distance else 1.
                replay.append([v+ratio*d for v, d in zip(p['xyz'], delta)])
            residuals.append(min(math.dist(command, pred) for pred in replay))
            direct += int(math.dist(command, emitted_goal) < 1e-9)
        # Recorder receipt ordering need not equal controller callback ordering.
        # Keep unmatched samples explicit; never widen a timing window to hide them.
        pose = dict(fc_src=interpolate(fc_interp, source), gt_src=interpolate(gt, source),
                    fc_ack=interpolate(fc_interp, ack), gt_ack=interpolate(gt, ack))
        xy = {k: v['xyz'][:2] for k, v in pose.items()}
        target = next(t['csv_xy'] for t in precision['truth'] if t['class_name'] == drop['class_name'])
        bias_src = sub(xy['fc_src'], xy['gt_src'])
        bias_ack = sub(xy['fc_ack'], xy['gt_ack'])
        vectors = dict(tracking=sub(xy['fc_ack'], goal),
                       relative_projection=sub(sub(goal, target), bias_src),
                       localization_change=sub(bias_src, bias_ack),
                       source_bias=bias_src, ack_bias=bias_ack,
                       raw_map_goal_error=sub(goal, target), total=sub(xy['gt_ack'], target))
        closure = [sum(vectors[k][i] for k in ('tracking', 'relative_projection', 'localization_change'))-vectors['total'][i] for i in range(2)]
        norms = {k: math.hypot(*v)*100 for k, v in vectors.items()}
        result.append(dict(class_name=drop['class_name'], slot=drop['slot'], capture_source=source,
                           capture_receipt=offset['t'], ack=ack, goal_xy=goal, truth_xy=target,
                           semantic_id=drop['target_id'], geometry_id=offset['m']['target_id'],
                           tolerance_m=offset['m']['alignment_tolerance_m'], poses=pose,
                           vectors_m=vectors, norms_cm=norms, closure_m=closure,
                           setpoint_match=dict(unique_float32_offset=True, samples=len(descent),
                               direct_samples=direct, max_replay_residual_m=max(residuals),
                               matched_within_1um=sum(v < 1e-6 for v in residuals),
                               unmatched_samples=[dict(receipt=descent[i][0], residual_m=v)
                                                  for i, v in enumerate(residuals) if v >= 1e-6],
                               first_descent_sample=descent[0][0], lead_m=lead,
                               fc_receipt_window_s=[-.12, .002]),
                           note='Capture identified from unique final command equality and descent limiter replay, not nearest observation. Geometry and semantic IDs differ by design.'))
        print(drop['class_name'], 'source', source, 'norms_cm', norms, 'replay', max(residuals))
    with (OUT / 'ack_decomposition.json').open('x') as f:
        json.dump(result, f, indent=2)


if __name__ == '__main__':
    main()
