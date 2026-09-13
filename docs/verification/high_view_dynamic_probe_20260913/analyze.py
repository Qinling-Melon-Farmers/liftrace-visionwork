"""Read completed single-revisit trials; no ROS/Gazebo commands."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import yaml


def read_csv(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    return np.array([[float(r[k]) for k in ['t','x','y','z']] for r in rows])


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();run=Path(args.run);out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    gate=json.loads((run/'gate_status.json').read_text())
    records=[json.loads(s) for s in (run/'high_view_probe_events.jsonl').read_text().splitlines()]
    truth=read_csv(run/'truth_pose.csv');estimate=read_csv(run/'mavros_pose.csv');commands=read_csv(run/'mavros_setpoint.csv')
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    truth_offset=params['competition_key_recorder']['truth_world_offset'][2]
    ground=params['target_map_projector']['ground_z']
    status=gate['probe']
    transitions=[]
    for r in records:
        s=r['status'];stage=s['stage']
        if stage=='SURVEY' and not s.get('ascent_verified'):stage='ASCEND'
        if s.get('done'):stage='DONE'
        if not transitions or transitions[-1]['stage']!=stage:transitions.append(dict(stage=stage,t=r['t']))
    start=next(v['t'] for v in transitions if v['stage']!='IDLE')
    end=transitions[-1]['t']
    stages=[]
    for a,b in zip(transitions,transitions[1:]):
        if a['stage']=='IDLE':continue
        stages.append(dict(stage=a['stage'],start=a['t'],end=b['t'],seconds=b['t']-a['t']))
    distance=0.
    for a,b in zip(truth,truth[1:]):
        if a[0]>=start and b[0]<=end and 0<b[0]-a[0]<=.5:
            distance+=float(np.linalg.norm(b[1:3]-a[1:3]))
    result=dict(scope=gate['scope'],run=str(run),status=gate['status'],reason=gate['reason'],
        mission_observed_seconds=end-start,stages=stages,truth_xy_distance_m=distance,
        max_true_fc_agl=float(np.max(truth[:,3]+truth_offset)),target_errors=gate.get('target_errors'),
        selected=status.get('selected'),reacquired=status.get('reacquired'),
        observation_counts=status.get('observation_counts'),collisions=gate['contacts'].get('actual_collision_count'),
        payload_committed=status['slots_committed'])
    revisit=next((v['start'] for v in stages if v['stage']=='REVISIT'),None)
    low=truth[(truth[:,0]>=revisit)&(truth[:,0]<=end)] if revisit is not None else np.empty((0,4))
    result['low_stage_max_true_fc_agl']=float(np.max(low[:,3]+truth_offset)) if len(low) else None
    result['hint_age_at_reacquisition']=status['reacquired']['last_seen_ns']/1e9-status['selected']['last_seen_ns']/1e9 if status.get('reacquired') else None
    (out/'metrics.json').write_text(json.dumps(result,indent=2))
    (out/'gate_status.json').write_text(json.dumps(gate,indent=2))
    colors=dict(ASCEND='#636363',SURVEY='#326ca6',RETURN_COLUMN='#9467bd',DESCEND='#ff9a30',REVISIT='#219d72',REACQUIRE='#c74c5b',DONE='#000000')
    fig,axs=plt.subplots(1,2,figsize=(12,5))
    for stage in stages:
        rows=truth[(truth[:,0]>=stage['start'])&(truth[:,0]<=stage['end'])]
        if len(rows):axs[0].plot(rows[:,1],rows[:,2],color=colors.get(stage['stage'],'gray'),label=stage['stage'])
        axs[1].axvspan(stage['start']-start,stage['end']-start,color=colors.get(stage['stage'],'gray'),alpha=.1)
    cfg=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    for t in cfg['targets']:
        axs[0].scatter(t['x'],t['y'],s=25,marker='s',color='.4')
        axs[0].annotate(t['class'],(t['x'],t['y']),xytext=(4,4),textcoords='offset points',fontsize=8)
    for tree in params['random_field_spawner']['static_exclusions']:
        axs[0].add_patch(Circle((tree['world_x'],tree['world_y']),tree['radius'],color='forestgreen',alpha=.2))
    if status.get('selected'):
        axs[0].scatter(*status['selected']['xy'],marker='x',color='red',s=80,label='high-view hint')
    axs[0].set(xlim=(-4.8,4.8),ylim=(-.5,7.4),xlabel='X (m)',ylabel='Y (m)',title='Actual truth trajectory; tree circles are proxies')
    axs[0].set_aspect('equal');axs[0].legend(fontsize=7,loc='upper left')
    for values,label,style in [(truth,'true FC AGL','-'),(estimate,'estimated FC AGL','--'),(commands,'command FC AGL',':')]:
        mask=(values[:,0]>=start)&(values[:,0]<=end)
        axs[1].plot(values[mask,0]-start,values[mask,3]+(truth_offset if label=='true FC AGL' else -ground),style,label=label,lw=1)
    axs[1].axhline(2.6,color='.6',ls='--');axs[1].axhline(1.4,color='.6',ls='--')
    axs[1].set(xlabel='Simulation seconds after first probe status',ylabel='Height (m)',title='High survey -> retraced descent -> low revisit')
    axs[1].legend(fontsize=8)
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle(run.name+' | '+gate['status']+' | single revisit, no delivery',fontsize=10)
    fig.tight_layout();fig.savefig(out/'trajectory_height.png',dpi=160);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
