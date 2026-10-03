"""Offline reporting only. Correct mutable-hint metrics and compare frozen runs."""
from pathlib import Path
import csv,importlib.util,json,math,shutil
import numpy as np,yaml,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2]
spec=importlib.util.spec_from_file_location('history',D.parent/'history_31_40_20260920/analyze_current.py')
history=importlib.util.module_from_spec(spec);spec.loader.exec_module(history)
def csv4(p):
 with p.open() as f:return np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])
def load(p):return json.loads(p.read_text())
ms=load(D/'metrics.json');old=load(D.parent/'columns_off_20260927/metrics.json')
summary=[]
for m in ms:
 run=Path(m['run']);state=m['high_view_final'];start=m['start_ros_s'];truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
 xy={t['class']:np.array([t['world_x'],t['world_y']]) for t in truth['targets']}
 interrupt=next(e for e in state['events'] if e['stage']=='SURVEY_INTERRUPTED_TOP3')
 support=interrupt['support'];reacq={h['class_name']:h for h in state['reacquisitions']}
 # final top_hints have already been rewritten by low confirmation: never label them high measurements.
 comparison={}
 for c in ('red_cross','bridge','panzer'):
  hs=support.get(c,[])
  if not hs:continue
  h=max(hs,key=lambda h:(h['low_verified'],h['source']!='bbox',h['source_stamp_ns']))
  comparison[c]=dict(high_xy=h['xy'],high_source=h['source'],high_source_stamp_ns=h['source_stamp_ns'],high_hint_error_m=float(np.linalg.norm(np.array(h['xy'])-xy[c])),
   low_xy=reacq[c]['xy'] if c in reacq else None,low_fused_point_error_m=float(np.linalg.norm(np.array(reacq[c]['xy'])-xy[c])) if c in reacq else None,
   high_to_low_delta_m=float(np.linalg.norm(np.array(reacq[c]['xy'])-np.array(h['xy']))) if c in reacq else None,
   source_age_at_reacquisition_s=reacq[c]['time']-h['source_stamp_ns']/1e9 if c in reacq else None,
   scope='High point frozen at TOP3 event; low point at first reacquisition. Not release accuracy.')
 m['target_hint_evaluation']=comparison
 body=load(D/f"{m['seed']}_balanced_columns/body_projection.json")['trees'];tree=load(D/f"{m['seed']}_balanced_columns/tree_projection.json")['trees']
 m['projection_summary']=dict(center_above_hull_samples=sum(t['center_above_footprint_samples'] for t in tree),whole_guard_above_tree_overlap_samples=sum(t['overlap_samples'] for t in body),
   whole_guard_above_tree_min_axis_gap_m=min(t['min_separating_axis_gap_m'] for t in body),fc_above_tree_nominal_disk_overlap_samples=sum(t['nominal_0_275m_radius_overlap_samples'] for t in tree),
   scope='Conservative full tree+pedestal footprint. Whole-guard and FC-above tests have different height conditions; not a complete swept-volume guarantee.')
 delivered=[x['target'] for x in m['commit_times']];m['high_weight_delivered']=len(set(delivered)&{'red_cross','bridge','panzer'})
 m['source_visual']='74ec4bd5185a7a3a56c40f32f6306ed17f8944dc';m['source_navigation']='d71b8fe1fdf83873c0b7a2be48167effb984acad'
 m['analysis_notes']=['Mission clock starts at first mission decision, after initial arming/takeoff.','ACK FC-to-target XY errors are not parcel-impact or slot-compensated errors.','Reacquisition hint_delta in raw telemetry may reference a recently updated low hint; use frozen TOP3 metrics here.']
 (D/f"{m['seed']}_balanced_columns/metrics.json").write_text(json.dumps(m,indent=2))
 summary.append({k:m[k] for k in ('seed','status','completed_mission_s','third_commit_s','top3_interrupt_mission_s','high_weight_delivered','xy_distance_m','collisions','milestone_durations_s','projection_summary','target_hint_evaluation')})
(D/'metrics.json').write_text(json.dumps(ms,indent=2));(D/'summary.json').write_text(json.dumps(summary,indent=2))
shutil.copyfile(R/'logs/balanced_columns_20260927_batch/matrix.json',D/'matrix.json')
fig,axs=plt.subplots(1,2,figsize=(14,7),layout='constrained')
for ax,m in zip(axs,ms):
 run=Path(m['run']);truth=yaml.safe_load((run/'random_field_truth.yaml').read_text());scene=R/f"docs/verification/history_31_40_20260920/seed_{m['seed']}/field.world";history.scene(ax,scene,truth)
 a=csv4(run/'truth_pose.csv');a=a[(a[:,0]>=m['start_ros_s'])&(a[:,0]<=m['observed_end_ros_s'])]
 stop=m['start_ros_s']+m['top3_interrupt_mission_s'];hi=a[:,0]<=stop
 ax.plot(a[:,1],a[:,2],color='#247aab',lw=1,label='Complete actual path');ax.plot(a[hi,1],a[hi,2],color='#d67e00',lw=2,label='High search before interruption')
 ax.annotate('',xy=(1,0),xytext=(.1,0),arrowprops=dict(arrowstyle='->',color='red'));ax.text(-.3,-.85,'initial heading +X',fontsize=8,color='red')
 for n,c in enumerate(m['commit_times'],1):ax.annotate(f"drop {n}",c['true_fc_xy_at_ack'],xytext=(6,-15),textcoords='offset points',fontsize=8)
 ax.set_title(f"Seed{m['seed']} | {m['completed_mission_s']:.1f}s | high-weight {m['high_weight_delivered']}/3")
 ax.legend(fontsize=8,loc='upper left')
fig.savefig(D/'paired_routes.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(12,5),layout='constrained');left=np.zeros(4)
combined=[old[0],ms[0],old[1],ms[1]]
phases=['start_to_third_ack','last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H','H_landing']
for phase in phases:
 vals=[m['milestone_durations_s'].get(phase) or 0 for m in combined];ax.barh(range(4),vals,left=left,label=phase);left+=vals
ax.set_yticks(range(4),['31 columns OFF (FAIL)','31 corrected columns ON','38 columns OFF (2/3 high)','38 corrected columns ON (2/3 high)'])
for i,m in enumerate(combined):ax.text(left[i]+3,i,'incomplete' if m['status']!='PASS' else f"{m['completed_mission_s']:.1f}s",va='center',fontsize=9)
ax.set_xlabel('Mission simulation seconds | changed column + inflation + evidence policy, not isolated A/B');ax.set_xlim(0,max(left)+100);ax.legend(fontsize=8,ncol=3,loc='upper center',bbox_to_anchor=(.5,-.22));ax.grid(axis='x',alpha=.2)
fig.savefig(D/'stage_comparison.png',dpi=150);plt.close(fig)
# Seed38 diagnostic: recorded local maps are cropped near goal, not evidence for the complete takeoff/landing area.
m=ms[1];run=Path(m['run']);s=m['start_ros_s'];snap=load(run/'local_map_failure_1.json');goal=np.array(snap['goal']);truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
fig,axs=plt.subplots(1,3,figsize=(17,5),layout='constrained');a=csv4(run/'truth_pose.csv');history.scene(axs[0],R/'docs/verification/history_31_40_20260920/seed_38/field.world',truth)
use=(a[:,0]>=76.8)&(a[:,0]<=177.52);axs[0].plot(a[use,1],a[use,2],lw=1,color='#237baa');false=np.array(m['target_hint_evaluation']['panzer']['high_xy'])
axs[0].scatter(*false,c='purple',marker='x',s=80);axs[0].annotate('false panzer hint',false,xytext=(10,-25),textcoords='offset points',fontsize=9)
axs[0].scatter(*goal[:2],c='red',marker='x',s=80);axs[0].set_title('38: failed bridge trip / false-hint detour')
map_measures={}
for key,col,lab in [('inflated_map','#eaa369','inflated map'),('static_map','#416c99','FreeDOM map')]:
 v=snap['clouds'][key];pts=np.array(v['points']);mask=np.abs(pts[:,2]-goal[2])<=.10;axs[1].scatter(pts[mask,0],pts[mask,1],s=3,c=col,label=lab)
 distance=np.linalg.norm(pts-goal,axis=1);idx=int(np.argmin(distance));map_measures[key]=dict(nearest_point_3d_m=float(distance[idx]),nearest_xyz=pts[idx].tolist(),voxel_centers_within_26mm=int(np.count_nonzero(np.max(np.abs(pts-goal),axis=1)<.026)))
axs[1].scatter(*goal[:2],c='red',marker='x',s=70,label='requested bridge goal');axs[1].set(title='Recorded goal-local slice: Z = 1.18 +/- 0.10m',xlabel='X (m)',ylabel='Y (m)');axs[1].axis('equal');axs[1].legend(fontsize=8,loc='lower left')
yfence=-4.8+.275*(math.cos(math.radians(10))+math.sin(math.radians(10)))
for name in ('truth_pose','lio_pose','mavros_setpoint'):
 p=csv4(run/(name+'.csv'));mask=(p[:,0]>=76.8)&(p[:,0]<=90);axs[2].plot(p[mask,0]-s,p[mask,2],label=name,lw=1)
axs[2].axhline(yfence,c='red',ls='--',label='planner search fence');axs[2].axhline(yfence+.03,c='orange',ls=':',label='command reserve');axs[2].axvline(88.886-s,c='grey',ls=':')
axs[2].set(title='Post-drop start near boundary; not beyond fence',xlabel='Mission seconds',ylabel='Y (m)');axs[2].legend(fontsize=8)
for ax in axs:ax.grid(alpha=.2)
fig.savefig(D/'seed38_diagnostic.png',dpi=150);plt.close(fig)
(D/'seed38_map_diagnostic.json').write_text(json.dumps(dict(snapshot=snap['ros_sec'],goal=snap['goal'],clouds=map_measures,planner_y_min=yfence,scope='Goal-local capture. Start cloud not present; nearest point alone is not traversability proof.'),indent=2))
fig,axs=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for ax,m in zip(axs,ms):
 vals=[x['fc_target_distance_at_ack_m']*100 for x in m['commit_times']];ax.bar([x['target'] for x in m['commit_times']],vals,color=['#bc5555','#358fb7','#d9ae55'])
 for i,v in enumerate(vals):ax.text(i,v+.3,f'{v:.2f} cm',ha='center')
 ax.set(title=f"Seed{m['seed']} mock ACK",ylabel='FC center to true target XY (cm)',ylim=(0,13));ax.grid(axis='y',alpha=.2)
fig.suptitle('Not slot-offset error or parcel-impact precision');fig.savefig(D/'release_center_errors.png',dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))