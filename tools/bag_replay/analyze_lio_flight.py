#!/usr/bin/env python3
"""Read closed ROS bags offline; compare LIO age, EV forwarding and FC jumps.

Requires the existing ROS Python environment. No ROS node, publisher or service.
Receipt ages use bag recording time, not estimator execution duration.
"""
import argparse
import json
import math
from pathlib import Path

import rosbag


POSE_TOPICS = ('/Odometry', '/mavros/vision_pose/pose',
               '/mavros/local_position/pose', '/navigation/local_pose')


def quantiles(values):
    if not values:
        return None
    ordered = sorted(values)
    def at(q):
        x = (len(ordered) - 1) * q
        a = int(x)
        b = min(a + 1, len(ordered) - 1)
        return ordered[a] + (ordered[b] - ordered[a]) * (x - a)
    return {key: at(q) for key, q in
            (('min', 0), ('p50', .5), ('p95', .95), ('p99', .99), ('max', 1))}


def analyze(path, max_age, jump_speed, jump_margin):
    rows = {topic: [] for topic in POSE_TOPICS}
    states = []
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, receipt in bag.read_messages(
                topics=[*POSE_TOPICS, '/mavros/state']):
            t = receipt.to_sec()
            if topic == '/mavros/state':
                states.append({'t': t, 'armed': msg.armed, 'mode': msg.mode})
                continue
            pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
            rows[topic].append({'t': t, 'stamp_ns': msg.header.stamp.to_nsec(),
                                'stamp': msg.header.stamp.to_sec(),
                                'xyz': [pose.position.x, pose.position.y, pose.position.z]})
    armed_intervals = [(r['t'], states[i + 1]['t']) for i, r in enumerate(states[:-1])
                       if r['armed']]
    def armed(t):
        return any(a <= t < b for a, b in armed_intervals)
    result = {'bag': str(path), 'max_age_sec': max_age,
              'armed_segmentation': 'bag state receipt intervals; 1 Hz state may bound transitions by ~1 s',
              'pose_jump_rule': {'speed_mps': jump_speed, 'margin_m': jump_margin},
              'topics': {}}
    previous_state = None
    transitions = []
    for r in states:
        if (r['armed'], r['mode']) != previous_state:
            transitions.append(r)
            previous_state = r['armed'], r['mode']
    result['state_transitions'] = transitions
    for topic, values in rows.items():
        ages = [r['t'] - r['stamp'] for r in values]
        flight = [r for r in values if armed(r['t'])]
        jumps = []
        for a, b in zip(values, values[1:]):
            dt = b['stamp'] - a['stamp']
            distance = math.sqrt(sum((x-y)**2 for x, y in zip(a['xyz'], b['xyz'])))
            if dt > 0 and distance > jump_speed * dt + jump_margin:
                jumps.append({'t': b['t'], 'stamp': b['stamp'], 'dt': dt,
                              'distance_m': distance, 'delta_xyz': [y-x for x, y in zip(a['xyz'], b['xyz'])],
                              'armed': armed(b['t'])})
        flight_ages = [r['t'] - r['stamp'] for r in flight]
        result['topics'][topic] = {
            'count': len(values), 'receipt_age_sec': quantiles(ages),
            'age_over_limit': sum(age > max_age for age in ages),
            'armed_count': len(flight), 'armed_age_sec': quantiles(flight_ages),
            'armed_age_over_limit': sum(age > max_age for age in flight_ages),
            'receipt_gap_max_sec': max((b['t']-a['t'] for a, b in zip(values, values[1:])), default=0),
            'armed_receipt_gap_max_sec': max((b['t']-a['t'] for a, b in zip(values, values[1:])
                                            if armed(a['t']) and armed(b['t'])), default=0),
            'header_rewinds': sum(b['stamp_ns'] < a['stamp_ns'] for a, b in zip(values, values[1:])),
            'jumps': jumps}
    ev_stamps = {r['stamp_ns'] for r in rows['/mavros/vision_pose/pose']}
    missing = [r for r in rows['/Odometry'] if r['stamp_ns'] not in ev_stamps]
    result['lio_not_forwarded'] = {
        'count': len(missing), 'armed_count': sum(armed(r['t']) for r in missing),
        'receipt_age_sec': quantiles([r['t']-r['stamp'] for r in missing]),
        'note': 'Header membership, not proof of input packet loss; inspect bridge limits and bag boundaries.'}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-age', type=float, default=.3)
    parser.add_argument('--jump-speed', type=float, default=3.)
    parser.add_argument('--jump-margin', type=float, default=.25)
    args = parser.parse_args()
    if any(not math.isfinite(v) or v < 0 for v in
           (args.max_age, args.jump_speed, args.jump_margin)):
        parser.error('thresholds must be finite and nonnegative')
    result = [analyze(b, args.max_age, args.jump_speed, args.jump_margin) for b in args.bags]
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    for r in result:
        lio = r['topics']['/Odometry']
        jumps = sum(j['armed'] for j in r['topics']['/mavros/local_position/pose']['jumps'])
        print('{}: LIO {}, not forwarded {} (armed {}), FC jumps armed {}'.format(
            Path(r['bag']).parent.name, lio['count'], r['lio_not_forwarded']['count'],
            r['lio_not_forwarded']['armed_count'], jumps))


if __name__ == '__main__':
    main()
