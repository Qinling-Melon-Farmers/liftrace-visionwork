#!/usr/bin/env python3
"""Offline center comparison. export reads a closed ROS1 bag; analyze needs no ROS."""
import argparse
import bisect
import collections
import csv
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

TOPICS = {
    '/uav_vision/detections_mapped': 'mapped',
    '/uav_vision/targets': 'targets',
    '/uav_vision/selected_target': 'selected',
    '/uav_vision/align_mode': 'mode',
    '/tf': 'tf',
    '/tf_static': 'tf_static',
}

def plain(value):
    if hasattr(value, '__slots__'):
        return {k: plain(getattr(value, k)) for k in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value

def stamp(value):
    if not isinstance(value, dict):
        return 0.0
    return float(value.get('secs', 0)) + float(value.get('nsecs', 0)) * 1e-9

def export_bag(args):
    import rosbag
    if args.bag.suffix != '.bag' or not args.bag.is_file():
        raise ValueError('Use a closed .bag, never a live .bag.active')
    topics = dict(TOPICS)
    if args.topics:
        topics.update(json.loads(args.topics.read_text()))
    counts = collections.Counter()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x', encoding='utf-8') as output:
        with rosbag.Bag(str(args.bag), 'r') as bag:
            available = bag.get_type_and_topic_info().topics
            output.write(json.dumps(dict(kind='metadata', bag=str(args.bag.resolve()),
                available={k: v.msg_type for k, v in available.items()},
                topic_map=topics, missing=[k for k in topics if k not in available])) + '\n')
            for topic, msg, time in bag.read_messages(topics=list(topics)):
                kind = topics[topic]
                output.write(json.dumps(dict(kind=kind, t=time.to_sec(), m=plain(msg))) + '\n')
                counts[kind] += 1
    print(json.dumps(dict(output=str(args.out), counts=counts)))

def analyze(args):
    import numpy as np
    import yaml
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if args.out.exists():
        raise ValueError('Output exists: choose a new directory')
    args.out.mkdir(parents=True)
    records = [json.loads(line) for line in args.data.read_text().splitlines() if line.strip()]
    metadata = next(r for r in records if r['kind'] == 'metadata')
    events = [r for r in records if r['kind'] != 'metadata']
    hint_file = args.run / 'high_view_full_events.jsonl'
    if hint_file.exists() and args.hint_frame:
        with hint_file.open() as hint_input:
            for line in hint_input:
                status = json.loads(line)
                for cls, supports in status['status'].get('navigation_support', {}).items():
                    for hint in supports:
                        ns = int(hint.get('source_stamp_ns') or hint.get('last_seen_ns') or 0)
                        xy = hint.get('xy', [])
                        if len(xy) != 2 or not ns:
                            continue
                        # Support XY is in the mission frame; it is not a camera pixel.
                        msg = dict(class_name=cls, map_valid=True, map_frame=args.hint_frame,
                            map_point=dict(x=xy[0], y=xy[1], z=args.hint_plane_z),
                            header=dict(stamp=dict(secs=ns//1000000000,nsecs=ns%1000000000)),
                            source=hint.get('source', 'unknown'), center_source='navigation_support',
                            geometry_confidence=hint.get('geometry_confidence'),
                            class_confidence=hint.get('class_confidence'))
                        events.append(dict(kind='hint_'+str(hint.get('source','unknown')),
                            t=status['t'], m=msg))
        events.sort(key=lambda r:r['t'])
    tf_rows = {k: [dict(t=r['t'], m=r['m']) for r in events if r['kind'] == k]
               for k in ('tf', 'tf_static')}
    source = args.pipeline_root / 'tools/bag_replay/bag_replay.py'
    spec = importlib.util.spec_from_file_location('recorded_tf', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    tree = module.TransformTree(tf_rows, 0.0)
    modes = sorted((r['t'], r['m'].get('data', '')) for r in events if r['kind'] == 'mode')
    mode_times = [r[0] for r in modes]
    truth_doc = yaml.safe_load((args.run / 'random_field_truth.yaml').read_text())
    truth = [dict(name=t.get('model', t['class']), class_name=t['class'],
                  world_xy=[float(t['world_x']), float(t['world_y'])], source='random_field_truth.yaml')
             for t in truth_doc['targets']]
    world = ET.parse(args.world).getroot().find('world')
    for obj in world.findall('include'):
        if not obj.findtext('uri', '').rstrip('/').split('/')[-1].startswith('landing_h'):
            continue
        pose = obj.find('pose')
        if pose is not None and pose.get('relative_to'):
            raise ValueError('H pose is relative to another frame; resolve explicitly first')
        values = [float(v) for v in obj.findtext('pose', '0 0 0 0 0 0').split()]
        truth.append(dict(name=obj.findtext('name', 'landing_h'), class_name='landing_pad',
                          world_xy=values[:2], source='world include pose', model_uri=obj.findtext('uri'),
                          nominal_h_size_m=args.h_size))
    if not any(t['class_name'] == 'landing_pad' for t in truth):
        raise ValueError('No landing_h includes found in supplied actual world')
    yaw = args.world_to_eval_yaw
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    shift = np.asarray(args.world_to_eval_xyz, dtype=float)
    for t in truth:
        t['eval_xy'] = (rotation @ t['world_xy'] + shift[:2]).tolist()
    grouped_truth = collections.defaultdict(list)
    for t in truth:
        grouped_truth[t['class_name']].append(t)

    start = None
    key_events = args.run / 'key_events.jsonl'
    if key_events.exists():
        for line in key_events.read_text().splitlines():
            event = json.loads(line)
            if event.get('kind') == 'decision':
                raw = event['data'].get('header', {}).get('stamp', {})
                value = raw.get('stamp_ns', 0) / 1e9
                if value:
                    start = value if start is None else min(start, value)
    if start is None:
        start = min((r['t'] for r in events), default=0.0)
    gate = json.loads((args.run / 'gate_status.json').read_text()) if (args.run / 'gate_status.json').exists() else {}
    duration = gate.get('metrics', {}).get('mission_ros_sec')
    end = start + duration if duration is not None else None

    rows = []
    seen = set()
    counts = collections.Counter()
    for event in events:
        kind = event['kind']
        if kind not in ('mapped', 'targets', 'selected') and not kind.startswith('hint_'):
            continue
        msg = event['m']
        items = msg.get('detections', []) if kind == 'mapped' else msg.get('targets', []) if kind == 'targets' else [msg]
        counts[kind + '_messages'] += 1
        mode_index = bisect.bisect_right(mode_times, event['t']) - 1
        event_mode = modes[mode_index][1] if mode_index >= 0 else 'UNKNOWN'
        if kind == 'mapped' and event_mode == 'landing':
            counts['landing_mapped_arrays'] += 1
            if 'landing_detector' in msg.get('completed_sources', []):
                counts['landing_mapped_H_branch_complete_arrays'] += 1
            if any(d.get('class_name') == 'landing_pad' for d in items):
                counts['landing_mapped_with_H_arrays'] += 1
            counts['landing_mapped_valid_H_items'] += sum(d.get('class_name') == 'landing_pad' and bool(d.get('map_valid')) for d in items)
        for item in items:
            counts[kind + '_items'] += 1
            cls = item.get('class_name', '')
            source_stamp = stamp(item.get('header', {}).get('stamp', {}))
            if kind != 'mapped':
                source_stamp = stamp(item.get('last_seen', {})) or source_stamp
            if not source_stamp:
                source_stamp = stamp(msg.get('header', {}).get('stamp', {}))
            mode_index = bisect.bisect_right(mode_times, event['t']) - 1
            mode = modes[mode_index][1] if mode_index >= 0 else 'UNKNOWN'
            p = item.get('map_point', {})
            point = [float(p.get(k, float('nan'))) for k in ('x', 'y', 'z')]
            age = event['t'] - source_stamp if source_stamp else None
            fresh = age is not None and -.03 <= age <= args.max_age
            row = dict(stream=kind, class_name=cls, id=item.get('id', ''),
                source=msg.get('source', ''), receipt_ros_s=event['t'], source_ros_s=source_stamp,
                mission_s=event['t']-start, age_s=age, fresh=fresh, align_mode=mode,
                map_frame=item.get('map_frame', ''), map_valid=bool(item.get('map_valid')),
                center_refined=bool(item.get('center_refined')), center_source=item.get('center_source', ''),
                association_valid=bool(item.get('association_valid')), reject_reason=item.get('reject_reason', ''),
                transform_age_sec=item.get('transform_age_sec'), class_confidence=item.get('class_confidence'),
                geometry_confidence=item.get('geometry_confidence'), state=item.get('state', ''),
                observe_count=item.get('observe_count', ''), consecutive_observe_count=item.get('consecutive_observe_count', ''),
                raw_x=point[0], raw_y=point[1], raw_z=point[2], eval_x=None, eval_y=None,
                truth_name='', truth_x=None, truth_y=None, dx_m=None, dy_m=None, xy_error_m=None,
                nearest_truth_gap_m=None, nearest_any_class='', nearest_any_instance='',
                nearest_any_distance_m=None, same_class_distance_m=None,
                same_vs_any_gap_m=None, category_confusion_suspected=False,
                duplicate=False, exclusion='',
                in_mission=event['t']>=start and (end is None or event['t']<=end))
            reason = ''
            if not item.get('map_valid'):
                reason = 'map_invalid'
            elif not source_stamp:
                reason = 'missing_source_stamp'
            elif not np.isfinite(point).all():
                reason = 'nonfinite_map_point'
            elif cls not in grouped_truth:
                reason = 'no_same_class_truth'
            else:
                transform = tree.matrix(row['map_frame'], args.eval_frame, source_stamp)
                if transform is None:
                    reason = 'missing_or_stale_transform'
                else:
                    evaluated = transform[:3, :3] @ point + transform[:3, 3]
                    any_candidates = sorted(((float(np.linalg.norm(evaluated[:2]-t['eval_xy'])), t)
                        for t in truth), key=lambda v:v[0])
                    any_error, any_target = any_candidates[0]
                    candidates = sorted(((float(np.linalg.norm(evaluated[:2]-t['eval_xy'])), t)
                                         for t in grouped_truth[cls]), key=lambda v:v[0])
                    error, target = candidates[0]
                    row.update(eval_x=float(evaluated[0]), eval_y=float(evaluated[1]),
                        truth_name=target['name'], truth_x=target['eval_xy'][0], truth_y=target['eval_xy'][1],
                        dx_m=float(evaluated[0]-target['eval_xy'][0]), dy_m=float(evaluated[1]-target['eval_xy'][1]),
                        xy_error_m=error, same_class_distance_m=error,
                        nearest_any_class=any_target['class_name'], nearest_any_instance=any_target['name'],
                        nearest_any_distance_m=any_error, same_vs_any_gap_m=error-any_error,
                        category_confusion_suspected=(any_target['class_name']!=cls and
                            any_error<=args.confusion_radius and error-any_error>=args.confusion_margin),
                        nearest_truth_gap_m=candidates[1][0]-error if len(candidates)>1 else None)
            row['exclusion'] = reason
            identity = (kind, cls, str(item.get('id', '')), msg.get('source', ''), source_stamp,
                        row['map_frame'], *[round(v, 6) if math.isfinite(v) else None for v in point],
                        json.dumps(item.get('center_px', {}), sort_keys=True) if kind == 'mapped' else '')
            row['duplicate'] = identity in seen
            seen.add(identity)
            counts[kind + '_' + (reason or 'valid_transformed')] += 1
            if not fresh:
                counts[kind + '_stale_or_missing_time_items'] += 1
            if row['duplicate']:
                counts[kind + '_duplicate_items'] += 1
            rows.append(row)

    fields = list(rows[0]) if rows else ['stream', 'class_name', 'exclusion']
    with (args.out/'center_samples.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    def summarize(values):
        e = np.array([r['xy_error_m'] for r in values])
        return dict(n=len(e), median_m=float(np.median(e)), p95_m=float(np.percentile(e,95)),
            rmse_m=float(np.sqrt(np.mean(e**2))), max_m=float(e.max()),
            mean_dx_m=float(np.mean([r['dx_m'] for r in values])),
            mean_dy_m=float(np.mean([r['dy_m'] for r in values])),
            first_mission_s=min(r['mission_s'] for r in values), last_mission_s=max(r['mission_s'] for r in values))
    stats = []
    eligible = [r for r in rows if not r['exclusion'] and not r['duplicate'] and r['in_mission']]
    scopes = {
        'unique_valid_mission': eligible,
        'fresh_unique_mission': [r for r in eligible if r['fresh']],
        'fresh_nearest_class_agrees': [r for r in eligible if r['fresh'] and r['nearest_any_class']==r['class_name']],
        'fresh_H_landing_mode': [r for r in eligible if r['fresh'] and r['class_name']=='landing_pad' and r['align_mode']=='landing'],
    }
    for scope, sample in scopes.items():
        groups = collections.defaultdict(list)
        for row in sample:
            groups[(row['stream'], row['class_name'], row['truth_name'])].append(row)
        for (stream, cls, name), sample in sorted(groups.items()):
            stats.append(dict(scope=scope, stream=stream, class_name=cls, truth_name=name, **summarize(sample)))
    association_groups = collections.defaultdict(list)
    # Hints remain useful after becoming older than the low-level freshness budget.
    # Keep their full unique mission history, separately from fresh control candidates.
    association_samples = [r for r in eligible if r['fresh'] or r['stream'].startswith('hint_')]
    for row in association_samples:
        association_groups[(row['stream'],row['class_name'],row['nearest_any_class'],row['nearest_any_instance'])].append(row)
    associations=[]
    for (stream,predicted,nearest,instance), sample in sorted(association_groups.items()):
        associations.append(dict(stream=stream,predicted_class=predicted,nearest_class=nearest,
            nearest_instance=instance,n=len(sample),
            median_nearest_distance_m=float(np.median([r['nearest_any_distance_m'] for r in sample])),
            median_same_class_distance_m=float(np.median([r['same_class_distance_m'] for r in sample])),
            suspected_confusion_n=sum(r['category_confusion_suspected'] for r in sample)))
    with (args.out/'class_instance_associations.csv').open('w',newline='') as output:
        fields=list(associations[0]) if associations else ['stream','predicted_class','nearest_class','n']
        writer=csv.DictWriter(output,fieldnames=fields);writer.writeheader();writer.writerows(associations)
    notes = [
        'Offline recorded map centers, not parcel impacts or physical release accuracy.',
        'Association is nearest same-class truth; not an independent instance recall evaluation.',
        'Same-class distance is not localization-only error: a wrong class may be near a different true instance.',
        'Nearest-any-instance association is diagnostic, not independently verified object identity.',
        'Suspected category confusion requires a different nearest class within the stated radius and a same-vs-any distance margin.',
        'Hint streams are reconstructed from high_view_full_events navigation_support, with explicitly supplied mission frame and projection plane.',
        'H start and landing pads are separate truth instances; a class/position error can change nearest association.',
        'World-to-evaluation transform is explicitly supplied, not estimated from the detections.',
        'Reported error is XY only. H dimensions do not change its center; no Z/size accuracy claim.',
        'Valid but old candidate publications are separate from fresh unique observations.',
        'TF lookup reuses bag_replay.TransformTree (latest past sample, max dynamic age 0.5s, no interpolation).',
        'Raw duplicates and invalid samples remain in CSV; errors aggregate only inside the mission interval.',
        'H branch-complete counters only show recorded pipeline completion, not proof of a detected H contour; raw detector output was not in this lightweight bag.',
    ]
    summary = dict(run=str(args.run), bag=metadata['bag'], world=str(args.world), eval_frame=args.eval_frame,
        world_to_eval_xyz=args.world_to_eval_xyz, world_to_eval_yaw=yaw,
        transform_provenance=args.transform_note, h_nominal_size_m=args.h_size, max_fresh_age_s=args.max_age,
        mission_start_ros_s=start, mission_end_ros_s=end, gate_status=gate.get('status', 'MISSING'),
        missing_topics=metadata.get('missing', []), counts=dict(counts), truth=truth, statistics=stats,
        scope_label=args.scope_label, hint_frame=args.hint_frame, hint_plane_z=args.hint_plane_z,
        confusion_radius_m=args.confusion_radius,confusion_margin_m=args.confusion_margin,
        class_instance_associations=associations, notes=notes)
    (args.out/'center_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    colors={'mapped':'#2279b8','targets':'#dc862a','selected':'#32954b'}
    fig, axes = plt.subplots(2, 1, figsize=(12,8), layout='constrained')
    for stream in colors:
        for cls, marker in [('landing_pad','x'), ('other','.')]:
            sample=[r for r in scopes['fresh_unique_mission'] if r['stream']==stream and (r['class_name']=='landing_pad')==(cls=='landing_pad')]
            if sample:
                axes[0].scatter([r['mission_s'] for r in sample],[r['xy_error_m'] for r in sample],
                    s=8 if cls=='other' else 20,alpha=.5,color=colors[stream],marker=marker,label=stream+' '+cls)
    hstats=[s for s in stats if s['scope']=='fresh_H_landing_mode']
    if hstats:
        labels=[s['stream']+' / '+s['truth_name'] for s in hstats]
        axes[1].bar(range(len(hstats)),[s['p95_m'] for s in hstats])
        axes[1].set_xticks(range(len(hstats)),labels,rotation=15,ha='right')
    else:
        axes[1].text(.5,.5,'No fresh H samples in landing mode',ha='center',transform=axes[1].transAxes)
    axes[0].set(xlabel='Mission ROS seconds',ylabel='Distance to same-class truth (m)',title='Includes class confusions; x = H')
    if axes[0].collections: axes[0].legend(fontsize=8,ncol=3)
    axes[1].set(ylabel='H center P95 error (m)',title='H pads kept separate; landing mode only')
    for ax in axes: ax.grid(alpha=.2)
    fig.savefig(args.out/'center_errors.png',dpi=150)
    plt.close(fig)
    report=['# Recorded target-center comparison','',
        'Scope: '+args.scope_label,
        'Gate: '+str(summary['gate_status'])+'. Historical comparison only; changed model/inflation/ceiling/H size.',
        '', 'Run: '+str(args.run), 'World: '+str(args.world),
        'Coordinate transform: '+args.transform_note, '',
        'Distances below are to same-class truth; inspect the class/instance table before interpreting localization.', '',
        '| Scope | Stream | Class | Truth instance | N | Median/m | P95/m | RMSE/m | Max/m |',
        '|---|---|---|---|---:|---:|---:|---:|---:|']
    for s in stats:
        report.append('| {scope} | {stream} | {class_name} | {truth_name} | {n} | {median_m:.4f} | {p95_m:.4f} | {rmse_m:.4f} | {max_m:.4f} |'.format(**s))
    report += ['', '## Nearest any-class instance versus predicted class', '',
        'Fresh unique mapped/targets/selected; all unique mission hints. A large same-class distance can be a class error.', '',
        '| Stream | Predicted | Nearest class | Nearest instance | N | Median nearest/m | Median same-class/m | Suspected confusion N |',
        '|---|---|---|---|---:|---:|---:|---:|']
    for s in associations:
        report.append('| {stream} | {predicted_class} | {nearest_class} | {nearest_instance} | {n} | {median_nearest_distance_m:.4f} | {median_same_class_distance_m:.4f} | {suspected_confusion_n} |'.format(**s))
    report += ['', '![Center errors](center_errors.png)', '', 'Missing bag topics: '+json.dumps(summary['missing_topics']),
        '', 'No rows is missing evidence, not zero error.', '', *['- '+n for n in notes],
        '', 'All samples including invalid, stale and duplicate publications: center_samples.csv.',
        'Machine-readable provenance, truth and counts: center_summary.json.']
    (args.out/'CENTER_REPORT.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(args.out), counts=counts, statistics_rows=len(stats)),indent=2))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='mode', required=True)
    exp=sub.add_parser('export'); exp.add_argument('--bag',type=Path,required=True)
    exp.add_argument('--out',type=Path,required=True); exp.add_argument('--topics',type=Path)
    ana=sub.add_parser('analyze')
    for key in ('data','run','world','out','pipeline-root'):
        ana.add_argument('--'+key,type=Path,required=True)
    ana.add_argument('--eval-frame',default='camera_init')
    ana.add_argument('--world-to-eval-xyz',type=float,nargs=3,required=True)
    ana.add_argument('--world-to-eval-yaw',type=float,required=True)
    ana.add_argument('--transform-note',required=True)
    ana.add_argument('--h-size',type=float,default=.8)
    ana.add_argument('--max-age',type=float,default=.5)
    ana.add_argument('--scope-label',default='Recorded evidence, not a new Gate assessment')
    ana.add_argument('--hint-frame',help='Explicit mission frame of navigation_support XY; omit to skip hints')
    ana.add_argument('--hint-plane-z',type=float,default=-.22)
    ana.add_argument('--confusion-radius',type=float,default=.5)
    ana.add_argument('--confusion-margin',type=float,default=.2)
    args=p.parse_args()
    if args.mode=='export': export_bag(args)
    else: analyze(args)
if __name__=='__main__': main()
