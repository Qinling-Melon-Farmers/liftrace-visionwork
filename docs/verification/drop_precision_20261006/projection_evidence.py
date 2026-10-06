#!/usr/bin/env python3
"""Read a CLOSED precision bag; prove/export exact DropOffset fields offline.
Use ROS system Python. No ROS nodes, playback, control or file writes.
"""
import argparse
import collections
import json
import math
from pathlib import Path


def plain(value):
    if hasattr(value,'__slots__'):
        return {key:plain(getattr(value,key)) for key in value.__slots__}
    if isinstance(value,(tuple,list)):
        return [plain(v) for v in value]
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--export',action='store_true',help='JSONL on stdout; redirect to NEW case directory')
    args=parser.parse_args()
    if not args.bag.is_file() or args.bag.suffix!='.bag':
        parser.error('Only a closed .bag is accepted')
    import rosbag
    counts=collections.Counter();by_target=collections.Counter();mode='UNKNOWN'
    with rosbag.Bag(str(args.bag),'r') as bag:
        if args.export:
            print(json.dumps(dict(kind='metadata',bag=str(args.bag.resolve()),
                note='Exact map_point is post-compensation FC goal; simulation slot mode zero makes target XY coincide. No parcel impacts.')))
        for topic,msg,time in bag.read_messages(topics=['/uav_vision/drop_offset','/uav_vision/align_mode']):
            if topic=='/uav_vision/align_mode':
                mode=msg.data
                if args.export:
                    print(json.dumps(dict(kind='mode',t=time.to_sec(),m=plain(msg))))
                continue
            counts['all_offsets']+=1
            if args.export:
                print(json.dumps(dict(kind='drop_offset',t=time.to_sec(),m=plain(msg))))
            if not hasattr(msg,'map_valid'):
                counts['legacy_schema']+=1;continue
            counts['exact_schema']+=1
            if not msg.map_valid or mode not in ('drop_circle','drop_cross'):
                counts['nonvalid_or_other_mode']+=1;continue
            point=msg.map_point
            age=time.to_sec()-msg.header.stamp.to_sec()
            valid=(msg.map_frame=='camera_init' and msg.header.frame_id=='downward_camera_optical_frame'
                and all(math.isfinite(v) for v in (point.x,point.y,point.z,msg.alignment_error_m,msg.alignment_tolerance_m))
                and abs(point.z+.22)<=1e-5 and msg.alignment_tolerance_m>0
                and msg.alignment_error_m>=0 and msg.header.stamp.to_sec()>0 and 0<=age<=.5)
            counts['fresh_exact_valid' if valid else 'invalid_exact']+=1
            if valid:
                by_target[str(msg.target_id)]+=1
    if not args.export:
        print(json.dumps(dict(status='RECORDED_EXACT_PATH' if counts['fresh_exact_valid'] else 'NO_FRESH_EXACT_PATH_EVIDENCE',
            bag=str(args.bag),counts=counts,valid_by_target=by_target,
            note='Parameter enablement plus these offsets demonstrate the visual experimental branch; control acceptance/actual release still need transaction logs.'),indent=2))

if __name__=='__main__':
    main()