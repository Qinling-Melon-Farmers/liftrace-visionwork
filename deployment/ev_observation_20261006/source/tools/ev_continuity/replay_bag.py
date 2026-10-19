#!/usr/bin/env python3
"""Offline shadow prediction from FULL PredictionState + raw MID360 IMU.

Never invent bias/gravity/velocity for old Odometry-only recordings. Does not
start a ROS master or publish anything. Receipt times preserve bag scheduling.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import rosbag

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    'patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts'))
from ev_predictor import Predictor, Limits, Imu
from ev_shadow import snapshot, vector


def main():
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--imu', default='/livox/imu')
    parser.add_argument('--snapshot', default='/laserMapping/prediction_state')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2]/
                        'patrol_uav_ws-patrol_planner/src/FAST_LIO/config/ev_shadow.yaml')
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text())
    p=Predictor(Limits(**config['limits']))
    step=1./float(config['output_hz'])
    with rosbag.Bag(str(args.bag)) as bag:
        present=bag.get_type_and_topic_info().topics
        missing=[t for t in (args.imu,args.snapshot) if t not in present]
        if missing:
            raise SystemExit('Cannot replay full-state prediction: missing '+', '.join(missing))
        args.out.mkdir(parents=True, exist_ok=True)
        rows=[]; status=[]; next_tick=None; last=None
        def tick(now):
            nonlocal last
            pair=p.output(now)
            if pair is not None and (last is None or pair[0].t>last):
                raw,smooth=pair; last=raw.t
                rows.append([now,raw.t,*raw.p,*smooth.p,now-p.correction_t])
            if not status or p.reason != status[-1]['reason']:
                status.append(dict(time=now,reason=p.reason))
        end=None
        for topic,msg,t in bag.read_messages(topics=[args.imu,args.snapshot]):
            now=t.to_sec(); end=now
            if next_tick is None:
                next_tick=now
            while next_tick < now:
                tick(next_tick); next_tick+=step
            if topic==args.imu:
                p.add_imu(Imu(msg.header.stamp.to_sec(),vector(msg.linear_acceleration),
                              vector(msg.angular_velocity),msg.header.frame_id),now)
            elif msg.valid:
                p.correction(snapshot(msg),now)
            elif p.state is not None:
                p.fail('source_state_invalid')
        if end is not None:
            tick(end)
    with (args.out/'shadow_replay.csv').open('w') as f:
        w=csv.writer(f); w.writerow(['receipt','stamp','raw_x','raw_y','raw_z',
                                     'smooth_x','smooth_y','smooth_z','correction_age']); w.writerows(rows)
    summary=dict(source=str(args.bag),mode='offline_imu_reference_not_body_or_PX4',
                 output_count=len(rows),final_fault=p.fault,states=status,stats=p.stats)
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
