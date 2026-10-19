"""Recount existing outputs only; no inference, ROS or manager replay."""
import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser(__doc__)
p.add_argument('--input-dir', type=Path, required=True,
               help='Existing pillbox_20261011 deliverables directory')
a = p.parse_args()
thresholds = (.60, .70, .75)
result = {'production': {}, 'pt_class_frame_outputs': {}}
for bag in (1, 2):
    source = json.loads((a.input_dir / ('hint_threshold_today_%d.json' % bag)).read_text())
    data = source['bags'][str(bag)]
    result['production'][str(bag)] = {'scope': source['scope'], 'thresholds': [], 'runs': {}}
    for threshold in thresholds:
        rows = [r for r in data['rows'] if r['confidence'] >= threshold]
        counts, regions = {}, {}
        for row in rows:
            cl = row['class_name']
            counts[cl] = counts.get(cl, 0) + 1
            loc = regions.setdefault(row['region'], {})
            loc[cl] = loc.get(cl, 0) + 1
        expected = next(r for r in data['thresholds'] if r['threshold'] == threshold)
        assert counts == expected['counts'] and regions == expected['regions']
        result['production'][str(bag)]['thresholds'].append(dict(threshold=threshold, counts=counts, regions=regions))
        continuity = {}
        for cl in ('bridge', 'panzer', 'red_cross', 'tent'):
            selected = [r for r in rows if r['class_name'] == cl and
                        (bag != 2 or cl not in ('bridge', 'panzer') or
                         r['region'] == ('bridge_location' if cl == 'bridge' else 'true_panzer_location'))]
            selected.sort(key=lambda r: r['t'])
            runs = []
            for row in selected:
                if not runs or row['t'] - runs[-1][-1]['t'] > 1.:
                    runs.append([])
                runs[-1].append(row)
            longest = max(runs, key=lambda r: r[-1]['t'] - r[0]['t'], default=[])
            continuity[cl] = dict(count=len(selected), first=selected[0]['t'] if selected else None,
                longest_gap_bounded_run=dict(start=longest[0]['t'], end=longest[-1]['t'],
                count=len(longest), span=longest[-1]['t']-longest[0]['t']) if longest else None)
        result['production'][str(bag)]['runs'][str(threshold)] = continuity
for model in ('old_0928', 'candidate_1011'):
    frames = []
    for bag in (1, 2):
        data = json.loads((a.input_dir / 'today_bag_compare' / ('bag_%d' % bag) /
                           (model + '_predictions.json')).read_text())
        frames.extend(data['frames'])
    result['pt_class_frame_outputs'][model] = {cl: {str(t): sum(any(
        b['semantic_name'] == cl and b['confidence'] >= t for b in f['boxes']) for f in frames)
        for t in thresholds} for cl in ('bridge', 'panzer', 'red_cross')}
print(json.dumps(result, ensure_ascii=False, indent=2))
