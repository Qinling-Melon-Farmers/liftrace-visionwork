#!/usr/bin/env python3
"""Read extra ROS1 bag messages offline using ROS system Python, no ROS master."""
import argparse,json,sys
from pathlib import Path
import rosbag
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/bag_replay'))
from bag_replay import plain
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve();out=run/'analysis';out.mkdir(exist_ok=True)
topics=['/navigation/local_pose','/navigation/local_odom','/navigation/setpoint_mission','/patrol_control/external_landing_handoff','/mavros/extended_state','/mavros/statustext/recv','/uav_vision/drop_ready','/uav_vision/alignment_target_context','/mission/release_authorization','/board_trials/landing_context','/camera/camera_info']
rows={t:[] for t in topics}
with rosbag.Bag(str(run/'flight_debug_0.bag')) as b:
 start=b.get_start_time()
 for t,m,bt in b.read_messages(topics=topics):rows[t].append(dict(t=bt.to_sec()-start,m=plain(m)))
(out/'extra_rows.json').write_text(json.dumps(rows,ensure_ascii=False),encoding='utf-8')