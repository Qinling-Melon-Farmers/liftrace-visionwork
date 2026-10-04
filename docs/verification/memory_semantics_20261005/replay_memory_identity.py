#!/usr/bin/env python3
"""Replay recorded mapped observations through real memory callbacks, no ROS master.
The recorded trajectory/modes remain fixed; this is not a counterfactual flight.
"""
import argparse,contextlib,csv,importlib.util,json,math
from pathlib import Path
from unittest.mock import patch
import rospy,rosbag,yaml

class Sink:
    def __init__(self,*args,**kwargs): self.latest=None
    def publish(self,msg): self.latest=msg

def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--before',type=Path,required=True);ap.add_argument('--after',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--target-xy',type=float,nargs=2,required=True);ap.add_argument('--window',type=float,nargs=2,default=[70.,90.]);ap.add_argument('--reacquire-window',type=float,nargs=2,default=[79.2,79.45]);a=ap.parse_args()
    params=yaml.safe_load((a.run/'rosparams.yaml').read_text())['target_memory']
    clock=[rospy.Time(0)]
    old,new=module(a.before,'memory_before'),module(a.after,'memory_after')
    results={'scope':'Actual target_memory callbacks; original mapped observations and align modes. No detector rerun or simulated future trajectory.','frames':0,'samples':[],'near_original_reacquisition':[]}
    with contextlib.ExitStack() as stack:
        for name,func in [('init_node',lambda *a,**k:None),('get_param',lambda k,d=None:params.get(k.lstrip('~'),d)),('Publisher',Sink),('Subscriber',lambda *a,**k:None),('Service',lambda *a,**k:None),('loginfo',lambda *a,**k:None),('logwarn_throttle',lambda *a,**k:None),('logwarn',lambda *a,**k:None)]:stack.enter_context(patch.object(rospy,name,func))
        stack.enter_context(patch.object(rospy.Time,'now',lambda:clock[0]))
        nodes={'before':old.TargetMemory(),'after':new.TargetMemory()}
        with rosbag.Bag(str(a.run/'vision_metrics.bag')) as bag:
            for topic,msg,t in bag.read_messages(topics=['/uav_vision/detections_mapped','/uav_vision/align_mode']):
                clock[0]=t
                if topic.endswith('align_mode'):
                    for node in nodes.values():node._on_align_mode(msg)
                    continue
                results['frames']+=1
                for node in nodes.values():node._on_detections(msg)
                sec=msg.header.stamp.to_sec()
                if a.window[0]<=sec<=a.window[1]:
                    current=[dict(label=z.class_name,confidence=z.class_confidence,geometry=z.geometry_confidence) for z in msg.detections if z.class_name!='circle' and z.map_valid and math.hypot(z.map_point.x-a.target_xy[0],z.map_point.y-a.target_xy[1])<.6]
                    states={}
                    for name,node in nodes.items():
                        near=[z for z in node._targets_pub.latest.targets if z.class_name!='circle' and math.hypot(z.map_point.x-a.target_xy[0],z.map_point.y-a.target_xy[1])<.6]
                        states[name]=[dict(id=z.id,label=z.class_name,state=z.state,streak=z.consecutive_observe_count,map_valid=z.map_valid,last_seen=z.last_seen.to_sec()) for z in near]
                    item=dict(t=sec,observations=current,**states);results['samples'].append(item)
                    if a.reacquire_window[0]<=sec<=a.reacquire_window[1]:results['near_original_reacquisition'].append(item)
    def accepted(v):return v['state']==2 and v['map_valid'] and v['streak']>=3
    results['wrong_panzer_frames_in_window']={k:sum(any(accepted(v) and v['label']=='panzer' for v in s[k]) for s in results['samples']) for k in nodes}
    results['first_confirmed_pillbox_in_window']={k:next((s['t'] for s in results['samples'] if any(accepted(v) and v['label']=='pillbox' for v in s[k])),None) for k in nodes}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(results,indent=2))
    print(json.dumps({k:v for k,v in results.items() if k not in ['samples','near_original_reacquisition']},indent=2))
    print('AT_REACQUISITION',json.dumps(results['near_original_reacquisition'][:3],indent=2))
if __name__=='__main__':main()
