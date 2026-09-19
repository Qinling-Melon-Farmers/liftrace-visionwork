from pathlib import Path
import json,csv
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');D=R/'docs/verification/history_31_40_20260920'
m=next(v for v in json.loads((D/'metrics.json').read_text()) if v['seed']==31);run=Path(m['run'])
events=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
decision=next(e for e in events if e['kind']=='decision' and e['data']['reason'].startswith('post_delivery_route:4/9:'))
start=decision['ros_sec'];stop=next(e['ros_sec'] for e in events if e['kind']=='decision' and e['data']['command']==7)
target=decision['data']['goal']['pose']['position'];goal=max(m['initial_planning'],key=lambda v:v['max_attempt'])['goal_seq']
def read(name):
    with (run/name).open() as f:return np.array([[float(v[k]) for k in ('t','x','y','z')] for v in csv.DictReader(f)])
truth=read('truth_pose.csv');pose=read('mavros_pose.csv');cmd=read('mavros_setpoint.csv');offset=yaml.safe_load((run/'rosparams.yaml').read_text())['competition_key_recorder']['truth_world_offset'][2]
fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
for i,key in [(1,'X'),(2,'Y')]:axes[0].plot(truth[:,0]-start,truth[:,i],label='True '+key)
axes[0].axhline(target['y'],color='red',ls='--',label='Requested Y');axes[0].set_ylabel('Field position (m)')
for arr,label in [(truth,'True FC AGL'),(pose,'MAVROS estimated AGL'),(cmd,'FC setpoint AGL')]:axes[1].plot(arr[:,0]-start,arr[:,3]+offset,label=label)
axes[1].axhline(1.1,color='gray',ls=':',label='Configured virtual ceiling AGL');axes[1].set_ylabel('Height (m)')
rows=[e for e in events if e['kind']=='planner' and e['data']['goal_seq']==goal]
axes[2].step([e['ros_sec']-start for e in rows],[e['data']['planning_attempt'] for e in rows],where='post',label='Planning attempts; no ready trajectory');axes[2].set_ylabel('Attempt counter')
distance=np.hypot(truth[:,1]-target['x'],truth[:,2]-target['y']);axes[3].plot(truth[:,0]-start,distance,label='True XY distance to requested goal')
records=[json.loads(l)['data'] for l in (run/'trajectory_progress.jsonl').read_text().splitlines()]
fsm=[v for v in records if v['source']==0];second=axes[3].twinx();second.plot([v['header']['stamp']/1e9-start for v in fsm],[v['stagnant_seconds'] for v in fsm],color='orange',alpha=.8,label='Tracking watchdog stagnant_seconds');second.set_ylabel('Tracking watchdog (s)');second.legend(loc='lower right',fontsize=8)
axes[3].set(ylabel='Remaining XY (m)',xlabel='Seconds since corridor segment 4 decision')
for ax in axes:ax.set_xlim(-2,stop-start+1);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
fig.suptitle('Seed31: first trajectory never became ready | 107 attempts, about 90 seconds\nProgress-on-an-active-trajectory monitoring does not cover this initialization wait',fontsize=12)
fig.tight_layout();fig.savefig(D/'seed31_initial_planning_stall.png',dpi=160);plt.close(fig)
print('Seed31 stall figure generated')
