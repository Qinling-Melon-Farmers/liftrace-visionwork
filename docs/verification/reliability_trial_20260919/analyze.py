"""One-run report using the earlier complete flight-chart pipeline."""
import csv,importlib.util,json,subprocess
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D=Path(__file__).resolve().parent;R=D.parents[2]

def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def main():
    state=json.loads((R/'logs/reliability_trial_20260919_batch/state.json').read_text())
    if state['status']=='RUNNING':raise RuntimeError('Wait for the single flight and cleanup to finish')
    run=Path(state['run']);case=json.loads((D/'case.json').read_text())
    spec=importlib.util.spec_from_file_location('previous_fast_report',D.parent/'fast_full_random_20260914/analyze_fast.py')
    base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
    metrics=base.analyze(dict(seed=case['seed'],run=str(run),world=case['world'],label='reliability_high_fast'),D)
    start=metrics['start_ros_s'];end=metrics['observed_end_ros_s']
    records=read_jsonl(run/'trajectory_progress.jsonl')
    by_source={v:[row for row in records if row['data']['source']==v] for v in (0,1)}
    recovery_by_goal={}
    first_changes=[];last_reason={}
    for row in records:
        v=row['data'];source=v['source'];time=v['header']['stamp']/1e9
        if source==0:recovery_by_goal[v['goal_seq']]=max(recovery_by_goal.get(v['goal_seq'],0),v['recoveries'])
        key=(source,v['traj_id'],v['reason'])
        if last_reason.get(source)!=key:
            first_changes.append(dict(t=time,source=source,traj_id=v['traj_id'],goal_seq=v['goal_seq'],reason=v['reason'],
                                      projection_t=v['projection_t'],lookahead_t=v['lookahead_t'],
                                      tracking_error=v['tracking_error']))
            last_reason[source]=key
    diag=dict(messages=len(records),fsm_messages=len(by_source[0]),server_messages=len(by_source[1]),
              recovery_by_goal=recovery_by_goal,total_recovery_requests=sum(recovery_by_goal.values()),
              max_stagnant_motion_s=max((row['data']['stagnant_seconds'] for row in by_source[0] if row['data']['motion_intent']),default=0),
              budget_exhausted=any(v['reason']=='liveness_budget_exhausted' for v in first_changes),
              reason_changes=first_changes,recording=json.loads((run/'trajectory_progress_recording.json').read_text()))
    fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
    for source,label in [(0,'FSM'),(1,'Server')]:
        rows=by_source[source];t=np.array([row['data']['header']['stamp']/1e9-start for row in rows]);progress=np.array([row['data']['projection_t'] for row in rows])
        breaks=np.array([False]+[rows[i]['data']['traj_id']!=rows[i-1]['data']['traj_id'] or t[i]-t[i-1]>.5 for i in range(1,len(rows))])
        progress[breaks]=np.nan
        axes[0].plot(t,progress,lw=.7,label=label+' projection')
        if source==1:
            look=np.array([row['data']['lookahead_t'] for row in rows]);look[breaks]=np.nan
            axes[0].plot(t,look,lw=.5,alpha=.6,label='Server lookahead')
    rows=by_source[0];t=[row['data']['header']['stamp']/1e9-start for row in rows]
    axes[1].plot(t,[row['data']['stagnant_seconds'] for row in rows],lw=.8,label='Physical no-progress window')
    axes[1].axhline(4,ls=':',color='red',label='4s recovery threshold')
    axes[2].step(t,[row['data']['recoveries'] for row in rows],where='post',label='Recoveries (per goal)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(0,end-start)
    axes[0].set_ylabel('Curve parameter (s)');axes[1].set_ylabel('Stagnant (s)');axes[2].set_ylabel('Requests')
    axes[2].set_xlabel('Simulation seconds after first mission command')
    fig.suptitle('Projection, lookahead and bounded recovery | resets mark new trajectories/goals')
    fig.tight_layout();fig.savefig(D/'planner_progress.png',dpi=160);plt.close(fig)
    # Display estimates separately; the historical H failure involved estimator divergence.
    data={name:base.base.csv4(run/(name+'.csv')) for name in ('truth_pose','lio_pose','mavros_pose','mavros_setpoint')}
    truth=data['truth_pose'];truth=truth[(truth[:,0]>=start)&(truth[:,0]<=end)]
    heights={};fig,axes=plt.subplots(2,1,figsize=(13,7),sharex=True)
    for name,a in data.items():
        axes[0].plot(a[:,0]-start,a[:,3]+.22,lw=.7,label=name)
        if name not in ('truth_pose','mavros_setpoint'):
            z=np.interp(truth[:,0],a[:,0],a[:,3])-truth[:,3]
            axes[1].plot(truth[:,0]-start,z,lw=.7,label=name+' minus truth')
            heights[name]=dict(abs_error_p95_m=float(np.percentile(abs(z),95)),abs_error_max_m=float(abs(z).max()),final_error_m=float(z[-1]))
    axes[0].set_ylabel('FC AGL (m)');axes[1].set_ylabel('Z error (m)');axes[1].set_xlabel('Mission time (s)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(0,end-start)
    fig.suptitle('Truth, LIO, FC estimator and command height (same recorded frame convention)')
    fig.tight_layout();fig.savefig(D/'height_estimation.png',dpi=160);plt.close(fig)
    pilot=D.parent/'fast_full_random_20260914/pilot'
    for script,name in [('check_tree_overflight.py','tree_projection.json'),('check_body_projection.py','body_projection.json')]:
        subprocess.run([__import__('sys').executable,str(pilot/script),str(run),case['world'],'--out',str(D/name)],check=True,stdout=subprocess.DEVNULL)
    metrics['planner_diagnostics']=diag;metrics['height_estimation']=heights
    final=metrics.get('high_view_final',{})
    interrupted=next((e['time'] for e in final.get('events',[]) if e['stage']=='SURVEY_INTERRUPTED_TOP3'),None)
    metrics['top3_interrupt_mission_s']=interrupted-start if interrupted else None
    metrics['delivery_stage_entries']=sum(e['stage']=='DELIVERY' for e in final.get('events',[]))
    (D/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (D/'flight_state.json').write_text(json.dumps(state,indent=2))
    summary={k:metrics.get(k) for k in ('status','reason','completed_mission_s','third_commit_s','top3_interrupt_mission_s','xy_distance_m','collisions','speed_by_phase','height_estimation')}
    summary.update(commits=len(metrics['commit_times']),releases=len(metrics['releases']),
                   recovery_requests=diag['total_recovery_requests'],cleanup_pass=state['cleanup_pass'])
    (D/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
