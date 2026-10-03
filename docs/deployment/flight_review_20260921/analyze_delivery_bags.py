from pathlib import Path
import rosbag,json,zipfile,collections
R=Path(__file__).resolve().parents[3]
D=R/'logs/flight_delivery_20260921'
with zipfile.ZipFile(next((R/'试飞产物').glob('liftrace*.zip'))) as z:
    for n in z.namelist():
        rel=Path(*Path(n).parts[1:])
        if n.endswith('/') or not str(rel).startswith(('patrol_uav_ws-patrol_planner/src/','top_level_scripts/')):continue
        if Path(n).suffix not in ('.cpp','.h','.py','.yaml','.launch','.xml','.sh','.txt'):continue
        p=D/'source'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
for p in list(D.glob('corridor*/corridor*/navigation_0.bag'))+list((R/'试飞产物').glob('*.bag')):
    with rosbag.Bag(str(p)) as b:
        print(p.name, b.get_start_time(),b.get_end_time())
        print({k:(v.msg_type,v.message_count) for k,v in b.get_type_and_topic_info().topics.items()})
        if p.name!='navigation_0.bag':continue
        out={'path':str(p),'start':b.get_start_time(),'end':b.get_end_time(),'events':[],'pose':[],'command':[],'logs':[],'tf':[]}
        for topic,m,t in b.read_messages():
            sec=t.to_sec()-out['start']
            if topic in ('/fastplanner/goal','/planning/goal_status','/navigation/mission_result','/navigation/mission_command_raw','/planning/bspline'):
                out['events'].append([sec,topic,str(m)])
            if topic in ('/navigation/local_pose','/fastplanner/setpoint_position/local'):
                q=m.pose.position
                out['pose' if topic=='/navigation/local_pose' else 'command'].append([sec,q.x,q.y,q.z])
            if topic=='/rosout_agg' and any(s in m.msg.lower() for s in ('collision','occupied','fail','hold','adjust','replan')):out['logs'].append([sec,m.name,m.msg])
            if topic=='/tf_static':out['tf'].append(str(m))
        dest=D/(p.parent.name+'.json');dest.write_text(json.dumps(out,indent=2))
        print('EVENTS',json.dumps(out['events'])[:16000]);print('LOGS',json.dumps(out['logs'][-20:]))
