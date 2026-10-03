"""Align archived PX4 clocks to ROS pose records; inspect landing, not control it."""
from pathlib import Path
import csv,json
import numpy as np
from pyulog import ULog
D=Path(__file__).resolve().parent
results=[]
for item in json.loads((D/'runs.json').read_text()):
    run=Path(item['run']);events=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
    land=next((e['ros_sec'] for e in events if e['kind']=='decision' and e['data']['command']==5),None)
    if land is None:continue
    with (run/'truth_pose.csv').open() as f:truth=np.array([[float(v[k]) for k in ('t','x','y','z')] for v in csv.DictReader(f)])
    with (run/'mavros_pose.csv').open() as f:pose=np.array([[float(v[k]) for k in ('t','x','y','z')] for v in csv.DictReader(f)])
    log=ULog(str(next((run/'px4/log').rglob('*.ulg'))),['vehicle_local_position','vehicle_land_detected','trajectory_setpoint','vehicle_local_position_setpoint','vehicle_status'])
    local=log.get_dataset('vehicle_local_position').data;t=local['timestamp']*1e-6
    # MAVROS ENU from PX4 NED. Fit only the final moving window; report residual.
    xyz=np.c_[local['y'],local['x'],-local['z']]
    samples=pose[(pose[:,0]>=land-10)&(pose[:,0]<=truth[-1,0])]
    choices=[]
    for offset in np.arange(-1.,1.0001,.001):
        pred=np.column_stack([np.interp(samples[:,0]+offset,t,xyz[:,i]) for i in range(3)])
        delta=samples[:,1:]-pred;bias=np.mean(delta,axis=0);rmse=np.sqrt(np.mean((delta-bias)**2))
        choices.append((float(rmse),float(offset),bias))
    rmse,offset,bias=min(choices,key=lambda v:v[0]);row=dict(label=item['label'],land_command_ros=land,px4_time_minus_ros_s=offset,clock_fit_rmse_m=rmse,constant_enu_bias_m=bias.tolist())
    near=truth[(truth[:,0]>=land)&(truth[:,3]+.22<.24)]
    row['first_near_support_ros']=float(near[0,0]) if len(near) else None
    row['min_true_agl_m']=float((truth[truth[:,0]>=land,3]+.22).min())
    row['ros_modes']=[dict(t=e['ros_sec'],mode=e['data']['mode'],armed=e['data']['armed']) for e in events if e['kind']=='state' and e['ros_sec']>=land]
    contacts=json.loads((run/'gazebo_contact_status.json').read_text())
    row['contact_events']=contacts.get('events',[])
    row['support_events']=contacts.get('support_events',[])
    row['guard_xy_m']=contacts.get('guard_xy_m')
    row['land_speed_parameter']=float(log.initial_parameters['MPC_LAND_SPEED'])
    detector=log.get_dataset('vehicle_land_detected').data;transitions=[]
    last=None
    for i,stamp in enumerate(detector['timestamp']):
        rt=float(stamp*1e-6-offset)
        if rt<land-1:continue
        flags={k:bool(detector[k][i]) for k in ('ground_contact','maybe_landed','landed','in_ground_effect','in_descend')}
        if flags!=last:transitions.append(dict(t=rt,**flags));last=flags
    row['px4_land_flags']=transitions
    row['setpoints']={}
    for name in ('trajectory_setpoint','vehicle_local_position_setpoint'):
        try:a=log.get_dataset(name).data
        except (KeyError,IndexError,StopIteration):continue
        ts=a['timestamp']*1e-6-offset;selected=np.where(ts>=land)[0]
        # One second samples plus last, retaining NaN semantics as JSON null.
        inds=[];last_t=-1e9
        for i in selected:
            if ts[i]-last_t>=.5 or i==selected[-1]:inds.append(i);last_t=ts[i]
        fields=[k for k in ('x','y','z','vx','vy','vz','position[0]','position[1]','position[2]','velocity[0]','velocity[1]','velocity[2]') if k in a]
        row['setpoints'][name]=[dict(t=float(ts[i]),**{k:float(a[k][i]) if np.isfinite(a[k][i]) else None for k in fields}) for i in inds]
    row['reset_counters_after_startup']={k:sorted(set(int(v) for v in local[k][t>10])) for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter')}
    results.append(row)
(D/'landing_sequence.json').write_text(json.dumps(results,indent=2))
for row in results:
    print(json.dumps({k:v for k,v in row.items() if k!='setpoints'},indent=2))
    print('last PX4 setpoints', {k:v[-4:] for k,v in row['setpoints'].items()})
