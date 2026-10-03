"""Unified historical comparison in rotated coordinates; failures never become time savings."""
from pathlib import Path
import csv,json,importlib.util,collections
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');D=R/'docs/verification/history_31_40_20260920'
spec=importlib.util.spec_from_file_location('a',D/'analyze_current.py');a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
current=json.loads((D/'metrics.json').read_text());old_all=json.loads((D.parent/'high_fast_five_20260915/metrics.json').read_text())+json.loads((D.parent/'high_fast_new_seeds_20260915/metrics.json').read_text())
old_high={m['seed']:m for m in old_all if m['label'].startswith('fast_high')};old_low={m['seed']:m for m in old_all if m['label']=='slow_coverage'}
items=json.loads((D/'runs.json').read_text());pairs=[];diagnostics=[];release_samples=[]
def pose(path,quat=False):
    keys=('t','x','y','z','qx','qy','qz','qw') if quat else ('t','x','y','z')
    with Path(path).open() as f:return np.array([[float(row[k]) for k in keys] for row in csv.DictReader(f)])
def deliveries(m):return len({(e['data'].get('execution_id'),e['data'].get('payload_slot')) for e in m.get('releases',[]) if e['data'].get('success')})
def alignment(m,group):
    run=Path(m['run']);xyz=pose(run/'truth_pose.csv');targets={t['class']:np.array([t['world_x'],t['world_y']]) for t in yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets']};params=yaml.safe_load((run/'rosparams.yaml').read_text());offset=np.array(params['competition_key_recorder']['truth_world_offset'][:2]);seen=set()
    for e in m.get('releases',[]):
        event=e['data'];key=event.get('execution_id',event.get('payload_slot'))
        if not event.get('success') or key in seen:continue
        seen.add(key);stamp=event['header']['stamp']['stamp_ns']/1e9;idx=np.searchsorted(xyz[:,0],stamp)
        if idx==0 or idx==len(xyz) or xyz[idx,0]-xyz[idx-1,0]>.5:continue
        point=np.array([np.interp(stamp,xyz[:,0],xyz[:,i]) for i in (1,2)])+offset;delta=point-targets[event['target_class']]
        if group!='current':delta=np.array([delta[1],-delta[0]])
        release_samples.append(dict(seed=m['seed'],group=group,class_name=event['target_class'],error_m=float(np.linalg.norm(delta)),dx_m=float(delta[0]),dy_m=float(delta[1])))

for item,m in zip(items,current):
    old=old_high[m['seed']];row=dict(seed=m['seed'],old_high_status=old['status'],current_status=m['status'],old_high_completed_s=old['completed_mission_s'],current_completed_s=m['completed_mission_s'],old_high_third_s=old['third_commit_s'],current_third_s=m['third_commit_s'],old_high_drops=deliveries(old),current_drops=deliveries(m),completed_gain_s=None,completed_gain_pct=None,third_gain_s=None)
    if old['completed_mission_s'] is not None and m['completed_mission_s'] is not None:row.update(completed_gain_s=old['completed_mission_s']-m['completed_mission_s'],completed_gain_pct=100*(old['completed_mission_s']-m['completed_mission_s'])/old['completed_mission_s'])
    if old['third_commit_s'] is not None and m['third_commit_s'] is not None:row['third_gain_s']=old['third_commit_s']-m['third_commit_s']
    if m['seed'] in old_low:
        low=old_low[m['seed']];row['old_low']=dict(status=low['status'],completed_s=low['completed_mission_s'],third_s=low['third_commit_s'],drops=deliveries(low),gain_s=low['completed_mission_s']-m['completed_mission_s'] if low['completed_mission_s'] is not None and m['completed_mission_s'] is not None else None)
    pairs.append(row)
    run=Path(m['run']);gate=json.loads((run/'gate_status.json').read_text());contact=json.loads((run/'gazebo_contact_status.json').read_text());points=pose(run/'truth_pose.csv',True)
    corners=np.array([[x,y,z] for x in (-.275,.275) for y in (-.275,.275) for z in (-.22,.18)]);verts=np.einsum('tij,kj->tki',Rotation.from_quat(points[:,4:]).as_matrix(),corners)+points[:,None,1:4]
    clearance=np.minimum.reduce([verts[:,:,0].min(axis=1)+.5,7.4-verts[:,:,0].max(axis=1),verts[:,:,1].min(axis=1)+4.8,4.8-verts[:,:,1].max(axis=1)])
    tail=next((v['start']+m['start_ros_s'] for v in m['actions'] if v['command']==4 and v['reason'].startswith('post_delivery_route:')),m['observed_end_ros_s'])
    mask=(points[:,0]>=m['start_ros_s'])&(points[:,0]<tail)
    no_ready=[v for v in m['initial_planning'] if v['ready_delay_s'] is None and v['max_attempt']>5]
    last=next((v for v in reversed(m['actions']) if v['command'] not in (6,7)),None)
    final=m['high_view_final'];reason=m['actions'][-1]['reason'] if m['actions'] else m['reason']
    kind='PASS' if m['status']=='PASS' else ('WALL_CLOCK_CUTOFF' if m['reason']=='mission_wall_timeout' else 'SEARCH_BODY_BOUNDARY' if m['reason']=='search_envelope_outside_inner_region' else 'INITIAL_PLANNING_STARVATION' if no_ready else 'HIGH_VIEW_OR_TRANSACTION_FAILURE')
    body=json.loads((D/f'{m["seed"]}_current/body_projection.json').read_text());finite=[v['min_separating_axis_gap_m'] for v in body['trees'] if v['min_separating_axis_gap_m'] is not None]
    diag=dict(seed=m['seed'],category=kind,gate_reason=m['reason'],abort_reason=reason,last_action=last,high_failure=final.get('failure'),high_stage=final.get('stage'),no_initial_trajectory=no_ready,initial_planning_max_attempts=max((v['max_attempt'] for v in m['initial_planning']),default=0),liveness_recoveries=m['liveness_recovery_requests'],liveness_exhausted=m['liveness_budget_exhausted'],boundary_violations=gate['metrics'].get('search_envelope_violations',0),minimum_recorded_search_body_clearance_m=float(clearance[mask].min()) if mask.any() else None,tree_projection_overlap_samples=sum(v['overlap_samples'] for v in body['trees']),minimum_tree_projection_axis_gap_m=min(finite) if finite else None,actual_collision_count=contact.get('actual_collision_count'),post_route_successes=gate['metrics'].get('post_delivery_return_success_count'),door_crossings=gate['metrics'].get('door_crossings',[]))
    diagnostics.append(diag)
    alignment(m,'current');alignment(old,'old_high')
    if m['seed'] in old_low:alignment(old_low[m['seed']],'old_low')
    fig,ax=plt.subplots(figsize=(8,8));a.scene(ax,item['world'],yaml.safe_load((run/'random_field_truth.yaml').read_text()))
    previous=pose(Path(old['run'])/'truth_pose.csv');ax.plot(previous[:,2],-previous[:,1],lw=.9,label='Historical high '+old['status']);ax.plot(points[:,1],points[:,2],lw=.9,label='Current '+m['status'])
    if m['seed'] in old_low:
        lowpose=pose(Path(old_low[m['seed']]['run'])/'truth_pose.csv');ax.plot(lowpose[:,2],-lowpose[:,1],lw=.65,alpha=.55,label='Historical coverage '+old_low[m['seed']]['status'])
    ax.legend(fontsize=8);ax.set_title(f'Seed {m["seed"]}: same rotated tree/door/target layout');fig.tight_layout();fig.savefig(D/f'paired_seed{m["seed"]}.png',dpi=150);plt.close(fig)

def aggregate(data):
    return dict(n=len(data),full_pass=sum(m['status']=='PASS' for m in data),three_drops=sum(deliveries(m)==3 for m in data),drop_count=sum(deliveries(m) for m in data),collision_runs=sum(m.get('collisions',0)>0 for m in data))
summary=dict(current=aggregate(current),old_high=aggregate(list(old_high.values())),old_low_31_35=aggregate(list(old_low.values())),failure_categories=dict(collections.Counter(d['category'] for d in diagnostics)),paired_complete=[p for p in pairs if p['completed_gain_s'] is not None])
for name,data in [('comparison.json',pairs),('failure_diagnosis.json',diagnostics),('release_alignment.json',release_samples),('summary.json',summary)]: (D/name).write_text(json.dumps(data,indent=2))
fig,axes=plt.subplots(2,1,figsize=(14,9));xs=np.arange(10)
for i,group in enumerate(('old_high','current')):
    full=[p[group+'_completed_s'] if p[group+'_completed_s'] is not None else 0 for p in pairs];third=[p[group+'_third_s'] if p[group+'_third_s'] is not None else 0 for p in pairs]
    axes[0].bar(xs+(i-.5)*.35,full,.35,label=group);axes[1].bar(xs+(i-.5)*.35,third,.35,label=group)
    for x,p,value in zip(xs,pairs,full):
        if not value:axes[0].text(x+(i-.5)*.35,5,'FAIL',rotation=90,ha='center',fontsize=8)
for ax,title in zip(axes,['Completed mission only (s); FAIL is not a fast completion','Third successful mock ACK (s); absent means fewer than three']):ax.set_xticks(xs,[p['seed'] for p in pairs]);ax.set_title(title);ax.legend();ax.grid(axis='y',alpha=.2)
fig.tight_layout();fig.savefig(D/'time_comparison.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(13,5));ax.bar(xs,[d['initial_planning_max_attempts'] for d in diagnostics]);ax.set_xticks(xs,[d['seed'] for d in diagnostics]);ax.set(ylabel='Max initial/trajectory planning attempt counter',title='Repeated planning versus physical-progress recovery are separate mechanisms');ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(D/'planning_attempts.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(11,5));groups=['old_low','old_high','current'];values=[[s['error_m']*100 for s in release_samples if s['group']==g] for g in groups];ax.boxplot(values,tick_labels=groups,showmeans=True);ax.set(ylabel='True FC to target centre at mock ACK (cm)',title='Successful releases retained even when final Gate failed; not parcel impact error');ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(D/'release_alignment.png',dpi=160);plt.close(fig)
print(json.dumps(summary,indent=2));print(json.dumps([{k:p[k] for k in ('seed','old_high_status','current_status','old_high_completed_s','current_completed_s','completed_gain_pct')} for p in pairs],indent=2))
