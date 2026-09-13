#!/usr/bin/env python3
"""Small simulation research-status log; no control interfaces."""
import json
import os
from pathlib import Path
import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node('high_view_status_recorder')
    if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):raise RuntimeError('simulation recording only')
    root=Path(os.environ['SIM_RUN_DIR']);last=[None]
    def callback(msg):
        if msg.data==last[0]:return
        data=json.loads(msg.data)
        with (root/'high_view_full_events.jsonl').open('a') as out:out.write(json.dumps(dict(t=rospy.Time.now().to_sec(),status=data))+'\n')
        last[0]=msg.data
    sub=rospy.Subscriber(rospy.get_param('~topic','/uav_high_view/probe_status'),String,callback,queue_size=1)
    rospy.spin()
