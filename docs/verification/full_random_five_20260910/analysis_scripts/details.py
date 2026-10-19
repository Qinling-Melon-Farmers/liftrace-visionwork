from pathlib import Path
import json,collections
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
R=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').exists() and (p/'patrol_uav_ws-patrol_planner').is_dir())
O=R/'logs/_artifacts/full_random_five_20260910';D=R/'docs/verification/full_random_five_20260910'
rows=json.loads((D/'flight_metrics.json').read_text());out=[]
for row in rows:
    run=Path(row['run_dir']);ev=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
    print('SEED',row['seed'],json.dumps({k:row[k] for k in ['status','observed_mission_span_s','complete_mission_s','phase_durations_s','final_xyz_local','final_h_geometry','release_geometry','land_start_ros','auto_land_ros','disarm_ros','route_to_disarm_s','land_to_disarm_s','planner_reasons','physics_safety']},indent=2))
    changes=[]
    for e in ev:
        if e['kind']=='decision' or e['kind']=='result' and e['data'].get('terminal'):
            d=e['data'];changes.append({'t':e['ros_sec'],'kind':e['kind'],**{k:d.get(k) for k in ['decision_seq','command','status','reason','target_class','target_id','payload_slot']},'goal':d.get('goal',{}).get('pose',{}).get('position')})
    failure_events=[x for x in changes if x['kind']=='result' and x['status'] in [4,5,7]]
    print('FAILURES',json.dumps(failure_events,indent=2));print('LAST_TIMELINE',json.dumps(changes[-12:],indent=2))
    end=row['contact_events'][0]['ros_stamp'] if row['contact_events'] else row['record_end_ros'];samples={}
    for name in ['truth_pose','mavros_pose','mavros_setpoint','lio_pose']:
        a=np.loadtxt(run/(name+'.csv'),delimiter=',',skiprows=1,ndmin=2)
        samples[name]=[{'t':float(p[0]),'xyz':p[1:4].tolist(),'rpy_deg':Rotation.from_quat(p[4:8]).as_euler('xyz',degrees=True).tolist()} for t in np.arange(end-8,end+.05,1) for p in [a[np.argmin(abs(a[:,0]-t))]]]
    item={'seed':row['seed'],'failure_events':failure_events,'last_samples':samples,'auto_land':None}
    if row['auto_land_ros'] is not None:
        files=list((run/'px4/log').rglob('*.ulg'))
        if files:
            u=ULog(str(files[0]),message_name_filter_list=['vehicle_attitude_setpoint','vehicle_local_position','vehicle_land_detected'])
            data=next((x.data for x in u.data_list if x.name=='vehicle_attitude_setpoint'),None)
            if data is not None:
                t=data['timestamp']/1e6;lo=row['auto_land_ros']-1.5;hi=row['auto_land_ros']+1.5;mask=(t>=lo)&(t<=hi);yaw=np.unwrap(data['yaw_body'][mask]);last=(t>=row['auto_land_ros'])&(t<=end+.1)
                item['auto_land']={'ros_receipt':row['auto_land_ros'],'sample_n':int(mask.sum()),'handoff_yaw_span_deg':float(np.rad2deg(np.ptp(yaw))) if len(yaw) else None,'handoff_max_yaw_step_deg':float(np.rad2deg(np.max(abs(np.diff(yaw))))) if len(yaw)>1 else None,'auto_segment_yaw_span_deg':float(np.rad2deg(np.ptp(np.unwrap(data['yaw_body'][last])))) if last.any() else None}
            ld=next((x.data for x in u.data_list if x.name=='vehicle_land_detected'),None)
            if ld is not None:
                idx=np.where((ld['timestamp']/1e6>=row['auto_land_ros'])&(ld['timestamp']/1e6<=end+.1))[0]
                item['land_detector_transitions']=[];previous=None
                for j in idx:
                    v={k:int(ld[k][j]) for k in ['ground_contact','maybe_landed','landed'] if k in ld}
                    if v!=previous:item['land_detector_transitions'].append({'t_px4_s':float(ld['timestamp'][j]/1e6),**v});previous=v
            print('AUTO_LAND',json.dumps(item['auto_land']),json.dumps(item.get('land_detector_transitions')))
    (D/('seed_%02d'%row['seed'])/'diagnostic_details.json').write_text(json.dumps(item,indent=2));out.append(item)
(D/'diagnostic_details.json').write_text(json.dumps(out,indent=2))
