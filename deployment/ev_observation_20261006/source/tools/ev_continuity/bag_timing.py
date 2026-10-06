#!/usr/bin/env python3
"""Read-only original EV arrival/source timing; does not emulate PX4 EKF."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag


def distribution(values):
    a = np.asarray(values, dtype=float)
    return None if not len(a) else dict(n=len(a), p50=float(np.percentile(a, 50)),
                                      p95=float(np.percentile(a, 95)),
                                      p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--lio', default='/Odometry')
    parser.add_argument('--ev', default='/mavros/vision_pose/pose')
    parser.add_argument('--imu', default='/livox/imu')
    parser.add_argument('--snapshot', default='/laserMapping/prediction_state')
    parser.add_argument('--state', default='/mavros/state')
    args=parser.parse_args()
    rows={args.lio: [], args.ev: []}
    armed=False
    with rosbag.Bag(str(args.bag)) as bag:
        info=bag.get_type_and_topic_info().topics
        for topic, msg, t in bag.read_messages(topics=list(rows)+[args.state]):
            if topic == args.state:
                armed=bool(msg.armed)
            else:
                rows[topic].append((t.to_sec(), msg.header.stamp.to_sec(), armed))
    result=dict(bag=str(args.bag), topics={}, imu_present=args.imu in info,
                snapshot_present=args.snapshot in info,
                complete_prediction_replay_possible=args.imu in info and args.snapshot in info,
                note='Arrival gaps are bag times, not PX4 internal fusion timeouts. Armed windows use latest recorded state.')
    for topic, data in rows.items():
        result['topics'][topic]={}
        for label, armed_only in [('whole_recording', False), ('armed', True)]:
            current=[x for x in data if x[2] or not armed_only]
            # Only adjacent original messages; do not bridge disarmed periods.
            pairs=[(a,b) for a,b in zip(data, data[1:]) if not armed_only or a[2] and b[2]]
            result['topics'][topic][label]=dict(
                age_sec=distribution([r-s for r,s,_ in current]),
                arrival_gap_sec=distribution([b[0]-a[0] for a,b in pairs]),
                source_gap_sec=distribution([b[1]-a[1] for a,b in pairs]),
                age_over_300ms=sum(r-s > .3 for r,s,_ in current),
                source_nonmonotonic=sum(b[1] <= a[1] for a,b in pairs))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
