from pathlib import Path
import json,csv,sys,numpy as np,yaml

def collect(run):
    run=Path(run);gate=json.loads((run/'gate_status.json').read_text());m=gate['metrics']
    events=[]
    for line in ((run/'key_events.jsonl').read_text().splitlines() if (run/'key_events.jsonl').exists() else []):
        try:events.append(json.loads(line))
        except ValueError:continue
    drops=[{'time':e['ros_sec'],'class':e['data'].get('target_class'),'slot':e['data'].get('payload_slot'),'decision':e['data'].get('decision_seq')} for e in events if e['kind']=='result' and e['data'].get('payload_committed')]
    poses=np.loadtxt(run/'truth_pose.csv',delimiter=',',skiprows=1,ndmin=2) if (run/'truth_pose.csv').exists() else np.empty((0,8))
    mav=np.loadtxt(run/'mavros_pose.csv',delimiter=',',skiprows=1,ndmin=2) if (run/'mavros_pose.csv').exists() else np.empty((0,8))
    autoland=next((e['ros_sec'] for e in events if e['kind']=='state' and e['data'].get('mode')=='AUTO.LAND'),None)
    ground=next((e['ros_sec'] for e in events if e['kind']=='extended_state' and e['data'].get('landed_state')==1 and autoland is not None and e['ros_sec']>=autoland),None)
    landing_error=None
    if ground is not None and poses.size:landing_error=float(np.linalg.norm(poses[np.argmin(abs(poses[:,0]-ground)),1:3]-m['landing_xy']))
    planner={}
    for e in events:
        if e['kind']!='planner':continue
        d=e['data'];idx=str(d['goal_seq']);p=planner.setdefault(idx,{'accepted':None,'first_ready':None,'failures':0})
        if d['status']==0:p['accepted']=e['ros_sec']
        if d['status']==2 and p['first_ready'] is None:p['first_ready']=e['ros_sec']
        if d['status']==5:p['failures']+=1
    lat=[p['first_ready']-p['accepted'] for p in planner.values() if p['accepted'] is not None and p['first_ready'] is not None]
    params=yaml.safe_load((run/'rosparams.yaml').read_text()) if (run/'rosparams.yaml').exists() else {}
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text()) if (run/'random_field_truth.yaml').exists() else {'seed':None,'targets':[]}
    source=json.loads((run/'scenario_inputs/source.json').read_text())
    output={'status':gate['status'],'run_dir':str(run),'seed':truth.get('seed'),'source':source['head'],'checks_passed':sum(gate['checks'].values()),'checks_total':len(gate['checks']), 'gate_reason':gate['reason'],'gate_errors':gate.get('errors',[]),'gate_failures':gate.get('failures',[]),'metrics':m,'drops':drops,'ground_error_to_h':landing_error,'planner_goals':planner,'planner_ready_latency_ros_s':{'p50':float(np.percentile(lat,50)) if lat else None,'p95':float(np.percentile(lat,95)) if lat else None},'event_count':len(events),'pose_rows':{'truth':len(poses),'mavros':len(mav)},'bag_files':len(list(run.glob('*.bag*'))),'cleanup_pass':'PASS (no local ROS/Gazebo/PX4/RViz process remains)' in (run/'run.log').read_text(errors='replace'),'layout':truth}
    (run/'summary.json').write_text(json.dumps(output,ensure_ascii=False,indent=2))
    return output

if __name__=='__main__':
    d=collect(sys.argv[1]);print(json.dumps({k:v for k,v in d.items() if k not in ['planner_goals','layout']},ensure_ascii=False,indent=2))
