"""Compare same-layout seed38 before and after the targeted hypothesis repair."""
from pathlib import Path
import csv,importlib.util,json,math,os
import numpy as np,yaml,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2]
def load(p):return json.loads(p.read_text())
def csv4(p):
 with p.open() as f:return np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])
old=next(m for m in load(D.parent/'balanced_columns_20260927/metrics.json') if m['seed']==38)
new=load(D/'metrics.json')[0];run=Path(new['run']);start=new['start_ros_s']
truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
xy={t['class']:np.array([t['world_x'],t['world_y']]) for t in truth['targets']}
state=new['high_view_final'];ev=state.get('events',[])
interrupt=next((e for e in ev if e['stage']=='SURVEY_INTERRUPTED_TOP3'),None)
# Freeze source coordinates at high-view exit, not at mutable low-view top_hints.
statuses=[]
for line in (run/'high_view_full_events.jsonl').read_text().splitlines():
 try:statuses.append(json.loads(line))
 except json.JSONDecodeError:pass
exit_sample=next((s for s in statuses if s['status'].get('stage') in ('DESCEND','LOCAL_DESCENT_TRANSIT','RETURN_COLUMN')),None)
support=interrupt['support'] if interrupt else (exit_sample or {}).get('status',{}).get('navigation_support',{})
comparison={}
for c,hs in support.items():
 if c not in xy or not hs:continue
 best=max(hs,key=lambda h:(h['low_verified'],h['source']!='bbox',h['source_stamp_ns']))
 comparison[c]=dict(xy=best['xy'],source=best['source'],stamp_ns=best['source_stamp_ns'],error_m=float(np.linalg.norm(np.array(best['xy'])-xy[c])),high_hypotheses=hs)
new['frozen_high_hint_evaluation']=comparison
low={h['class_name']:h for h in state.get('reacquisitions',[])}
new['target_hint_evaluation']={c:dict(high_xy=v['xy'],high_source=v['source'],high_hint_error_m=v['error_m'],
 low_xy=low[c]['xy'] if c in low else None,
 low_fused_point_error_m=float(np.linalg.norm(np.array(low[c]['xy'])-xy[c])) if c in low else None,
 scope='High frozen at exit; low at first formal reacquisition, not release precision') for c,v in comparison.items()}
body=load(D/'38_panzer_location_repair/body_projection.json')['trees']
tree=load(D/'38_panzer_location_repair/tree_projection.json')['trees']
new['projection_summary']=dict(center_above_hull_samples=sum(t['center_above_footprint_samples'] for t in tree),
 whole_guard_above_tree_overlap_samples=sum(t['overlap_samples'] for t in body),
 whole_guard_above_tree_min_axis_gap_m=min(t['min_separating_axis_gap_m'] for t in body),
 nominal_0_275m_radius_overlap_samples=sum(t['nominal_0_275m_radius_overlap_samples'] for t in tree),
 scope='Conservative whole-tree plus pedestal XY hull; differs from authorized middle-contour columns. Not physical collision depth.')
new['source_visual']=load(D/'matrix.json')['source']
new['source_navigation']='738066995cb0212c7503653227b33aca33e065a8'
new['high_weight_delivered']=len({x['target'] for x in new['commit_times']}&{'red_cross','bridge','panzer'})
new['high_exit_mission_s']=(exit_sample['t']-start) if exit_sample else None
new['unconfirmed_location_retirements']=[e for e in ev if e['stage']=='UNCONFIRMED_LOCATION_RETIRED']
new['formal_delivery_degradations']=[e for e in ev if e['stage']=='DELIVERY_POINT_UNREACHABLE']
new['mock_ack_scope']='FC-to-true-target error only, not slot-compensated parcel-impact accuracy'
(D/'metrics.json').write_text(json.dumps([new],indent=2));(D/'38_panzer_location_repair/metrics.json').write_text(json.dumps(new,indent=2))
keys=('status','completed_mission_s','third_commit_s','top3_interrupt_mission_s','high_weight_delivered','collisions','xy_distance_m','milestone_durations_s')
summary=dict(before={k:old.get(k) for k in keys},after={k:new.get(k) for k in keys},new_high_hints=comparison,old_run=old['run'],new_run=new['run'],scope='Same scene/FOV/speed/columns/inflation, one run each. Different delivered classes; elapsed saving is not a distributional benchmark.')
if new['completed_mission_s'] is not None:
 summary['mission_seconds_saved']=old['completed_mission_s']-new['completed_mission_s']
 summary['mission_percent_saved']=100*summary['mission_seconds_saved']/old['completed_mission_s']
(D/'summary.json').write_text(json.dumps(summary,indent=2))
spec=importlib.util.spec_from_file_location('history',R/'docs/verification/history_31_40_20260920/analyze_current.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
fig,axs=plt.subplots(1,2,figsize=(14,7),layout='constrained')
for ax,m,title in zip(axs,(old,new),('Before: weak panzer ended high search','After: panzer requires refined support')):
 r=Path(m['run']);tr=yaml.safe_load((r/'random_field_truth.yaml').read_text());h.scene(ax,R/'docs/verification/history_31_40_20260920/seed_38/field.world',tr)
 a=csv4(r/'truth_pose.csv');a=a[(a[:,0]>=m['start_ros_s'])&(a[:,0]<=m['observed_end_ros_s'])]
 ax.plot(a[:,1],a[:,2],lw=1,color='#207eaa',label='Complete actual trajectory')
 stop=m.get('top3_interrupt_mission_s')
 if stop is not None:
  hi=a[:,0]<=m['start_ros_s']+stop;ax.plot(a[hi,1],a[hi,2],lw=2,color='#d28a16',label='High search before exit')
 for i,c in enumerate(m['commit_times'],1):ax.annotate(f'drop {i}',c['true_fc_xy_at_ack'],xytext=(5,-15),textcoords='offset points',fontsize=8)
 ax.annotate('',xy=(1.,0),xytext=(0.,0),arrowprops=dict(arrowstyle='->',color='red'));ax.text(.15,-.7,'initial heading +X',color='red',fontsize=8)
 ax.set_title(title+'\n'+str(m['status'])+f" | {m['completed_mission_s']:.3f} s | high {m['high_weight_delivered']}/3");ax.legend(fontsize=8,loc='upper right')
fig.savefig(D/'paired_routes.png',dpi=150,bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(figsize=(12,4),layout='constrained');left=np.zeros(2)
for phase in ('start_to_third_ack','last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H','H_landing'):
 vals=[m['milestone_durations_s'].get(phase) or 0 for m in (old,new)];ax.barh(range(2),vals,left=left,label=phase);left+=vals
ax.set_yticks(range(2),['Before: red_cross / bridge / tent','After: '+' / '.join(c['target'] for c in new['commit_times'])]);ax.set_xlabel('Mission simulation seconds');ax.grid(axis='x',alpha=.2);ax.legend(fontsize=8,ncol=3,bbox_to_anchor=(.5,-.25),loc='upper center')
fig.savefig(D/'stage_comparison.png',dpi=150,bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(figsize=(6,4),layout='constrained');vals=[c['fc_target_distance_at_ack_m']*100 for c in new['commit_times']];ax.bar([c['target'] for c in new['commit_times']],vals)
for i,v in enumerate(vals):ax.text(i,v+.1,f'{v:.2f}cm',ha='center')
ax.set(title='Mock release ACK: FC center to true target',ylabel='XY error (cm)');ax.grid(axis='y',alpha=.2);fig.savefig(D/'release_center_errors.png',dpi=150,bbox_inches='tight');plt.close(fig)
video=os.path.relpath(run/'presentation_review.mp4',D)
imgs=['paired_routes.png','stage_comparison.png','release_center_errors.png']+[str(p.relative_to(D)) for p in sorted((D/'38_panzer_location_repair').glob('*.png'))]
html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Seed38 修复复验</title><style>body{font:16px system-ui;background:#10171f;color:#e2edf7;margin:24px auto;max-width:1250px}a{color:#86ceff}img,video{width:100%;background:white;margin:10px 0}section{margin:24px 0}pre{white-space:pre-wrap;background:#1c2936;padding:16px}</style><h1>Seed38 高位中断与位置级复核修复</h1><p><a href="REPORT.md">详细报告</a> · <a href="summary.json">对比指标</a></p>'''
html+=f'<p>全程状态：{new["status"]}；投递类别：'+', '.join(c['target'] for c in new['commit_times'])+f'；完赛 {new.get("completed_mission_s")} 仿真秒。单轮对照，不代表普遍成功率。</p>'
html+=f'<h2>全程俯视＋跟随＋高度叠加（1×仿真时间）</h2><video controls preload="metadata" src="{video}"></video>'
html+='<p>红箭头为初始机头+X；Y为机身初始左侧。视频与bag/日志保留在本机run目录，不进入Git。</p>'
for p in imgs:html+=f'<section><h2>{Path(p).stem}</h2><img src="{p}"></section>'
(D/'index.html').write_text(html+'</html>')
print(json.dumps(summary,indent=2))