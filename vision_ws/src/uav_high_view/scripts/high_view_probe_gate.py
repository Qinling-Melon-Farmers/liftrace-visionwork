#!/usr/bin/env python3
"""Simulation-only observer; truth is used for this gate, never probe goals."""
import json
import os
from pathlib import Path
import time
import rospy
import math
import yaml
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class Gate:
    def __init__(self):
        if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('simulation gate only')
        self.root=Path(os.environ['SIM_RUN_DIR'])
        self.probe=None;self.contacts=None;self.truth_count=0;self.violation=''
        self.contact_seen=0.;self.truth_seen=0.;self.target_errors=None
        self.started=time.monotonic();self.last_status=None
        self.model=rospy.get_param('~truth_model','iris_mid360')
        self.max_height=float(rospy.get_param('~max_fc_height',3.0))
        self.wall_limit=float(rospy.get_param('~wall_timeout',1200.))
        self.subs=[rospy.Subscriber('/uav_high_view/probe_status',String,self.status,queue_size=1),
            rospy.Subscriber('/mission/gazebo_contact_status',String,self.contact,queue_size=1),
            rospy.Subscriber('/gazebo/model_states',ModelStates,self.truth,queue_size=1)]

    def status(self,msg):
        self.probe=json.loads(msg.data)
        if msg.data!=self.last_status:
            with (self.root/'high_view_probe_events.jsonl').open('a') as f:
                f.write(json.dumps(dict(t=rospy.Time.now().to_sec(),status=self.probe))+'\n')
            self.last_status=msg.data

    def contact(self,msg):
        self.contacts=json.loads(msg.data);self.contact_seen=time.monotonic()

    def truth(self,msg):
        if self.model not in msg.name:return
        p=msg.pose[msg.name.index(self.model)].position;self.truth_count+=1
        self.truth_seen=time.monotonic()
        if not (-4.8<=p.x<=4.8 and -.5<=p.y<=7.4 and p.z<=self.max_height):
            self.violation='truth_bounds_or_height'

    def run(self):
        reason='wall_timeout';passed=False
        while not rospy.is_shutdown() and time.monotonic()-self.started<self.wall_limit:
            if self.contacts and self.contacts.get('actual_collision_count',0)>0:
                reason='actual_collision';break
            if self.violation:reason=self.violation;break
            if self.probe and self.probe.get('done'):
                passed=bool(self.probe.get('succeeded') and self.probe.get('low_limits_applied')
                    and self.probe.get('slots_committed')==0 and self.truth_count>0
                    and self.contacts and self.contacts.get('ready')
                    and time.monotonic()-self.contact_seen<2
                    and time.monotonic()-self.truth_seen<2)
                if passed:
                    truth=yaml.safe_load((self.root/'random_field_truth.yaml').read_text())
                    selected=self.probe['selected'];low=self.probe['reacquired']
                    target=next((t for t in truth['targets'] if t['class']==selected['class_name']),None)
                    if target is None:
                        passed=False
                    else:
                        high_error=math.hypot(selected['xy'][0]-target['x'],selected['xy'][1]-target['y'])
                        low_error=math.hypot(low['xy'][0]-target['x'],low['xy'][1]-target['y'])
                        self.target_errors=dict(high_hint_error_m=high_error,low_reacquisition_error_m=low_error)
                        # Identity association checks, not a claim that P0's
                        # statistical 0.25m localization criterion has passed.
                        passed=high_error<=.6 and low_error<=.35
                reason='fresh_low_reacquisition' if passed else self.probe.get('failure','probe_failed')
                break
            time.sleep(.1)
        result=dict(status='PASS' if passed else 'FAIL',scope='HIGH_VIEW_SINGLE_REVISIT_NOT_FULL_MISSION',
                    reason=reason,probe=self.probe,truth_samples=self.truth_count,contacts=self.contacts,
                    target_errors=self.target_errors)
        (self.root/'gate_status.json').write_text(json.dumps(result,indent=2))
        return passed


if __name__=='__main__':
    rospy.init_node('high_view_probe_gate')
    success=Gate().run()
    rospy.signal_shutdown('PASS' if success else 'FAIL')
