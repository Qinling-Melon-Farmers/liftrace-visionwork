#!/usr/bin/env python3
"""Optional bounded, read-only execution diagnostics for simulation review."""
import json
import math
import os
from pathlib import Path
import threading
import rospy
from plan_manage.msg import TrajectoryProgress


def plain(value):
    if hasattr(value, 'to_nsec'):return value.to_nsec()
    if hasattr(value, '__slots__'):return {k:plain(getattr(value,k)) for k in value.__slots__}
    if isinstance(value, float) and not math.isfinite(value):return None
    return value


class Recorder:
    def __init__(self):
        if not rospy.get_param('/use_sim_time', False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('simulation diagnostics require the run wrapper')
        self.root=Path(os.environ['SIM_RUN_DIR'])
        self.limit=int(rospy.get_param('~max_bytes',10*1024*1024))
        if not 1024 <= self.limit <= 50*1024*1024:raise ValueError('invalid recording budget')
        self.count=self.bytes=0;self.truncated=False;self.closed=False
        self.lock=threading.Lock()
        self.file=(self.root/'trajectory_progress.jsonl').open('w',buffering=1)
        self.sub=rospy.Subscriber(rospy.get_param('~topic','/planning/progress'),
                                  TrajectoryProgress,self.callback,queue_size=100)
        rospy.on_shutdown(self.close)

    def callback(self,msg):
        with self.lock:
            if self.closed or self.truncated:return
            line=json.dumps(dict(receipt_ros_sec=rospy.get_time(),data=plain(msg)),separators=(',',':'))+'\n'
            size=len(line.encode('utf-8'))
            if self.bytes+size>self.limit:
                self.truncated=True
                rospy.logwarn('Planner diagnostics reached byte budget; flight is unaffected')
                return
            self.file.write(line);self.bytes+=size;self.count+=1

    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True;self.file.close()
            (self.root/'trajectory_progress_recording.json').write_text(json.dumps(dict(
                messages=self.count,bytes=self.bytes,max_bytes=self.limit,truncated=self.truncated,
                status='RECORDED' if self.count else 'EMPTY'),indent=2))


if __name__=='__main__':
    rospy.init_node('planner_progress_recorder');recorder=Recorder();rospy.spin()
