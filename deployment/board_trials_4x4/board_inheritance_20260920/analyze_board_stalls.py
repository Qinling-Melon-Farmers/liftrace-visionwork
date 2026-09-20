from pathlib import Path
import json,collections,datetime
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;source=D.parents[2]/'logs/board_stall_review_20260920/board_nav_export_20260920'
files=sorted(source.glob('corridor*.json'))
if (D.parents[2]/'logs/board_stall_review_20260920/board_recovered_review/last_nav.json').exists():files.append((D.parents[2]/'logs/board_stall_review_20260920/board_recovered_review/last_nav.json'))
summaries=[]
def pos(m):
 if 'position' in m:return m['position']
 p=m.get('pose',{});return p.get('pose',p).get('position')
def arr(events,topic):
 rows=[]
 for e in events:
  if e['topic']==topic:
   p=pos(e['message'])
   if p:rows.append([e['t'],p['x'],p['y'],p['z']])
 return np.array(rows)
for n,p in enumerate(files):
 d=json.loads(p.read_text());ev=d['events'];status=[e for e in ev if e['topic']=='/planning/goal_status'];goals=collections.defaultdict(list)
 for e in status:goals[e['message']['goal_seq']].append(e)
 result=dict(run=d['run'],start=d['start'],end=d['end'],goals=[])
 states=[]
 for e in ev:
  if e['topic']=='/mavros/state':
   s=e['message'];v=(s['mode'],s['armed'])
   if not states or tuple(states[-1][1:])!=v:states.append([e['t'],*v])
 result['state_changes']=states
 for seq,es in goals.items():
  reasons=collections.Counter(e['message']['reason'] for e in es);failed=[e for e in es if e['message']['reason']=='new_trajectory_attempt_failed'];ready=[e for e in es if 'ready' in e['message']['reason']]
  row=dict(seq=seq,requested=es[-1]['message']['requested_goal']['pose']['position'],effective=es[-1]['message']['effective_goal']['pose']['position'],reasons=dict(reasons),failed=len(failed),ready=len(ready))
  if failed:
   row.update(first_failed=failed[0]['t'],last_failed=failed[-1]['t'],failure_span=failed[-1]['t']-failed[0]['t'])
   row['failures_in_offboard']=sum(next((s[1] for s in reversed(states) if s[0]<=e['t']),'')=='OFFBOARD' for e in failed)
  result['goals'].append(row)
 warnings=[e for e in ev if e['topic'] in ['/rosout','/rosout_agg'] and e['message']['name']=='/fast_planner_node'];result['planner_log_counts']=dict(collections.Counter(e['message']['msg'] for e in warnings if e['message']['level']>=4))
 poses=arr(ev,'/navigation/local_pose');cmd=arr(ev,'/planning/pos_cmd');setpoints=arr(ev,'/navigation/setpoint_mission')
 failedrows=[v for v in result['goals'] if v['failed']]
 if failedrows:
  r=failedrows[-1];active=[s[0] for s in states if s[0]>r['first_failed'] and s[1]!='OFFBOARD'];stop=active[0] if active else d['end'];q=poses[(poses[:,0]>=r['first_failed'])&(poses[:,0]<=stop)]
  if len(q):r['offboard_wait_pose_mean']=q[:,1:].mean(axis=0).tolist();r['offboard_wait_pose_span']=np.ptp(q[:,1:],axis=0).tolist()
  result['bsplines_after_first_failure']=sum(e['topic']=='/planning/bspline' and e['t']>=r['first_failed'] for e in ev)
 fig,axs=plt.subplots(2,2,figsize=(11,8));t0=d['start']
 for a,label,color in [(poses,'Measured mission pose','blue'),(setpoints,'Controller setpoint','orange')]:
  if len(a):axs[0,0].plot(a[:,1],a[:,2],color=color,label=label,alpha=.7);axs[0,1].plot(a[:,0]-t0,a[:,1],color=color,label=label)
 for g in result['goals']:
  point=g['requested'];axs[0,0].scatter(point['x'],point['y'],marker='x',color='red');axs[0,0].annotate('Goal '+str(g['seq']),(point['x'],point['y']))
  fs=[e for e in goals[g['seq']] if 'attempt_failed' in e['message']['reason']];axs[1,1].plot([e['t']-t0 for e in fs],[e['message']['planning_attempt'] for e in fs],'.',label='Goal '+str(g['seq']))
 if len(poses):axs[1,0].plot(poses[:,0]-t0,poses[:,3],label='Local Z (not AGL)')
 for row in result['goals']:
  if row['failed']:
   for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.axvspan(row['first_failed']-t0,row['last_failed']-t0,color='red',alpha=.08)
 axs[0,0].set(xlabel='Mission X (m)',ylabel='Mission Y (m)',title='Actual trace; no obstacle geometry in this bag');axs[0,0].set_aspect('equal',adjustable='datalim')
 axs[0,1].set(xlabel='Recording time (s)',ylabel='X (m)');axs[1,0].set(xlabel='Recording time (s)',ylabel='Local Z (m)');axs[1,1].set(xlabel='Recording time (s)',ylabel='Initial planning attempt')
 for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.suptitle(Path(d['run']).name,fontsize=11);fig.tight_layout();name='flight_stall_'+str(n+1)+'.png';fig.savefig(D/name,dpi=140);plt.close(fig);result['figure']=name
 summaries.append(result)
(D/'real_flight_stalls.json').write_text(json.dumps(summaries,indent=2))
print(json.dumps([{k:v for k,v in r.items() if k in ['run','goals','bsplines_after_first_failure']} for r in summaries],indent=2))
