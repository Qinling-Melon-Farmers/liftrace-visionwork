import json,rosbag
from pathlib import Path
out=Path('/home/xhj/liftrace-deliverables/flight_review_20261012')
for key,path in json.load(open(out/'sources.json')).items():
 d=out/key;d.mkdir(exist_ok=True)
 logs=[];metadata=[];states=[];poses=[];structured=[]
 with rosbag.Bag(path) as b:
  t0=b.get_start_time(); inv={k:dict(type=v.msg_type,count=v.message_count) for k,v in b.get_type_and_topic_info().topics.items()}
  ts=['/rosout_agg','/board_trials/run_metadata','/mavros/state','/navigation/local_pose','/navigation/setpoint_mission','/mavros/local_position/pose','/mavros/setpoint_position/local','/mission/release_authorization','/uav_vision/release_evidence','/uav_vision/release_evidence_context','/uav_high_view/probe_status','/navigation/mission_status']
  last=None
  for topic,m,t in b.read_messages(topics=ts):
   rx=t.to_sec()-t0
   if topic=='/rosout_agg':logs.append(dict(t=rx,stamp=m.header.stamp.to_sec()-t0,node=m.name,text=m.msg))
   elif topic=='/board_trials/run_metadata':metadata.append(dict(t=rx,data=json.loads(m.data)))
   elif topic=='/mavros/state':
    state=[m.connected,m.armed,m.mode]
    if state!=last:states.append([rx,*state]);last=state
   elif hasattr(m,'pose'):
    p=m.pose.position;q=m.pose.orientation;poses.append([rx,topic,m.header.stamp.to_sec()-t0,m.header.frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
   elif hasattr(m,'data'):
    try:structured.append(dict(t=rx,topic=topic,data=json.loads(m.data)))
    except (ValueError,TypeError):pass
  json.dump(dict(source=path,start=t0,duration=b.get_end_time()-t0,topics=inv,states=states,metadata=metadata,logs=logs,poses=poses,structured=structured),open(d/'analysis.json','w'),ensure_ascii=False)
  print('\nBAG',key,'duration',b.get_end_time()-t0,'STATES',states)
  for r in logs:
   if any(v in r['text'] for v in ['External ALIGN','frozen slot','descent_started','async Servo submitted','positive Servo ACK','Following speed phase','Takeoff complete','raw decision','ABORT','Abort','abort reason']):print(round(r['t'],3),r['text'])
  for m in metadata:
   if m['data'].get('ground_reference'):print('META',json.dumps(m['data']['ground_reference'],ensure_ascii=False));break