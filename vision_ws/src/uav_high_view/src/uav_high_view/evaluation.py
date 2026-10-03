"""Independent-observation-segment evaluator, not confidence-as-accuracy.

Each segment contains manually identified ground-truth instances and confirmed
predictions assigned to physical instances by an OFFLINE annotation/matching
process. These annotations must never be passed to the online catalog.
"""
import math


def wilson(success, count):
    if not count:
        return None
    z = 1.96
    p = success/count
    divisor = 1+z*z/count
    mid = (p+z*z/(2*count))/divisor
    delta = z*math.sqrt(p*(1-p)/count+z*z/(4*count*count))/divisor
    return [max(0., mid-delta), min(1., mid+delta)]


def evaluate_segments(segments, classes):
    counts = {c: dict(all=0, visible=0, discovered=0, visible_discovered=0) for c in classes}
    seen, layouts, errors = set(), set(), []
    confirmed, wrong_class, unmatched = 0, 0, 0
    top_correct, top_total = 0, 0
    for s in segments:
        identity = (s['layout_id'], s['segment_id'])
        if identity in seen:
            raise ValueError('duplicate independent segment')
        seen.add(identity); layouts.add(s['layout_id'])
        if (not 2.4 <= s['fc_agl'] <= 3.0 or s['height_verified'] is not True
                or not s['calibration_id'] or not s['model_revision']):
            raise ValueError('verified high-view height/calibration/model required')
        truth = {v['instance_id']: v for v in s['truth']}
        if len(truth) != len(s['truth']):
            raise ValueError('duplicate truth instance')
        predictions = {}
        assigned = set()
        for v in s['confirmed_predictions']:
            confirmed += 1
            key = v.get('matched_instance_id')
            if key not in truth:
                unmatched += 1
                continue
            if key in assigned:
                raise ValueError('one final hypothesis per annotated instance/segment required')
            assigned.add(key)
            target = truth[key]
            if v['class_name'] != target['class_name']:
                wrong_class += 1
                continue
            predictions.setdefault(key, []).append(v)
        for key,t in truth.items():
            if t['class_name'] not in counts:
                raise ValueError('unknown truth class')
            c = counts[t['class_name']]
            c['all'] += 1; c['visible'] += int(t['visible'])
            detected = bool(predictions.get(key))
            c['discovered'] += int(detected)
            c['visible_discovered'] += int(detected and t['visible'])
            if detected:
                # Penalize the worst confirmed spatial hypothesis, not a lucky
                # nearest prediction or a best frame among hundreds.
                d = [math.hypot(v['xy'][0]-t['xy'][0], v['xy'][1]-t['xy'][1])
                     for v in predictions[key]]
                if not all(math.isfinite(x) for x in d):
                    raise ValueError('nonfinite position')
                errors.append(max(d))
        if 'expected_top3' in s:
            if len(s['expected_top3']) != 3 or len(set(s['expected_top3'])) != 3:
                raise ValueError('invalid expected top3')
            top_total += 1
            selected = s.get('selected_top3', [])
            top_correct += int(len(selected)==3 and len(set(selected))==3
                               and set(selected)==set(s['expected_top3']))
    for c in counts.values():
        c['visible_recall'] = c['visible_discovered']/c['visible'] if c['visible'] else None
        c['all_recall'] = c['discovered']/c['all'] if c['all'] else None
        c['visible_recall_wilson95'] = wilson(c['visible_discovered'], c['visible'])
    errors.sort()
    p95 = errors[max(0,math.ceil(.95*len(errors))-1)] if errors else None
    wrong_fraction = wrong_class/confirmed if confirmed else None
    enough = len(layouts)>=30 and all(c['visible']>=20 for c in counts.values()) and top_total>=30
    thresholds = (enough and all(c['visible_recall']>=.95 for c in counts.values())
                  and wrong_fraction is not None and wrong_fraction<=.01 and unmatched==0
                  and top_correct/top_total>=.95 and p95 is not None and p95<=.25)
    return dict(scope='OFFLINE_ANNOTATED_P0_ONLY', segments=len(seen), layouts=len(layouts),
                classes=counts, confirmed_predictions=confirmed, wrong_class=wrong_class,
                wrong_class_fraction=wrong_fraction, wrong_class_wilson95=wilson(wrong_class,confirmed),
                unmatched_confirmations=unmatched, position_p95_m=p95,
                top3_correct=top_correct, top3_total=top_total, sufficient_sample_counts=enough,
                point_thresholds_met=bool(thresholds),
                p0_status='NEEDS_INDEPENDENT_REVIEW' if thresholds else 'NOT_DEMONSTRATED',
                caveat='Point thresholds do not establish confidence bounds or authorize flight.')
