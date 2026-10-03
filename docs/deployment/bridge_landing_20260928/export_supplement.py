"""Export supplemental ROS1 bag topics without ROS master, publishers or inference."""
import argparse,json,sys
from pathlib import Path
import rosbag

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/bag_replay'))
from bag_replay import plain

def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 a.out.mkdir(parents=True,exist_ok=True)
 supplemental=['/mavros/extended_state','/mavros/statustext/recv','/mavros/battery','/mission/release_commitment_evidence','/rosout','/rosout_agg']
 extras=['/mission/command','/mission/release_permission','/navigation/planner_bridge_status','/uav_vision/alignment_target_context','/uav_vision/release_evidence_context','/uav_vision/drop_ready','/camera/camera_info']
 with rosbag.Bag(str(a.bag.resolve())) as bag:
  start=bag.get_start_time();topics={k:dict(type=v.msg_type,count=v.message_count) for k,v in bag.get_type_and_topic_info().topics.items()}
  supplemental.extend(t for t in topics if 'diagnostic' in t or 'high_view' in t or 'parameter' in t)
  sup={t:[] for t in supplemental if t in topics};ext={t:[] for t in extras if t in topics}
  for topic,msg,bt in bag.read_messages(topics=list(sup)+list(ext)):
   target=ext if topic in ext else sup
   if topic=='/camera/camera_info' and target[topic]:continue
   target[topic].append(dict(t=bt.to_sec()-start,m=plain(msg)))
  inventory=dict(bag=str(a.bag.resolve()),start=start,duration=bag.get_end_time()-start,topics=topics)
 for name,data in [('supplement',sup),('extra',ext),('inventory',inventory)]:
  (a.out/(name+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2 if name=='inventory' else None))
 print('Supplement export complete:',a.out)
if __name__=='__main__':main()