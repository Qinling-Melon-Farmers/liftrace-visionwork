from pathlib import Path
import json,os,re,subprocess,csv
import numpy as np,yaml,cv2,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[3];D=R/'docs/verification/failed_six_20260921';OLD=D.parent/'history_31_40_20260920'
new=json.loads((D/'metrics.json').read_text());old={m['seed']:m for m in json.loads((OLD/'metrics.json').read_text())};state=json.loads((D/'matrix.json').read_text())
phases=['start_to_third_ack','last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H','H_landing'];names=['to third ACK','last recovery','transfer','staging descent','corridor','H landing']
fig,ax=plt.subplots(figsize=(13,8));values=[];labels=[]
for m in new:
 for q,label in [(old[m['seed']],'old'),(m,'rerun')]:values.append(q);labels.append(f'{m["seed"]} {label} {q["status"]}')
left=np.zeros(len(values))
for phase,label in zip(phases,names):
 v=np.array([max(0.,q.get('milestone_durations_s',{}).get(phase) or 0.) for q in values]);ax.barh(np.arange(len(v)),v,left=left,label=label);left+=v
tail=np.array([max(0.,q['observed_duration_s']-x) if q['status']!='PASS' else 0. for q,x in zip(values,left)])
ax.barh(np.arange(len(tail)),tail,left=left,color='.85',hatch='///',label='observed unfinished interval')
ax.set_yticks(np.arange(len(labels)),labels);ax.invert_yaxis();ax.set_xlabel('ROS seconds after first mission decision');ax.grid(axis='x',alpha=.2);ax.legend(fontsize=8,bbox_to_anchor=(1.01,1));fig.tight_layout();fig.savefig(D/'stages.png',dpi=150);plt.close(fig)
errors={}
for label,items in [('old',[old[m['seed']] for m in new]),('rerun',new)]:
 errors[label]=[c['fc_target_distance_at_ack_m']*100 for m in items for c in m.get('commit_times',[]) if c.get('fc_target_distance_at_ack_m') is not None]
fig,ax=plt.subplots(figsize=(8,5))
for i,(label,v) in enumerate(errors.items()):ax.scatter(np.full(len(v),i)+np.linspace(-.13,.13,len(v)),v,label=f'{label}: n={len(v)}');ax.hlines(np.median(v),i-.2,i+.2,color='black')
ax.set_xticks([0,1],['old failed subset','rerun']);ax.set(ylabel='True FC-to-target distance at mock ACK (cm)',title='Release alignment; not parcel landing accuracy');ax.legend();ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(D/'release_accuracy.png',dpi=150);plt.close(fig)
stats={k:dict(n=len(v),median_cm=float(np.median(v)),p95_cm=float(np.percentile(v,95)),max_cm=float(max(v))) for k,v in errors.items()}
guards=[];sources=[]
for m in new:
 folder=D/f'{m["seed"]}_rerun';v=json.loads((folder/'body_projection.json').read_text());trees=v['trees']
 guards.append(dict(seed=m['seed'],overlap_samples=sum(t.get('overlap_samples',0) for t in trees),min_gap_m=min((t['min_separating_axis_gap_m'] for t in trees if t.get('min_separating_axis_gap_m') is not None),default=None)))
 run=Path(m['run']);mf=yaml.safe_load((run/'manifest.yaml').read_text());assert mf['uav_git_head']==state['source'] and mf['vision_git_head']==state['source']
 sources.append(dict(seed=m['seed'],git_head=mf['git_head'],uav_head=mf['uav_git_head'],vision_head=mf['vision_git_head'],uav_package=mf['resolved_uav_mission'],vision_package=mf['resolved_uav_vision']))
 (folder/'metrics.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
 video='presentation_review' if m['seed'] in (38,40) else 'presentation';meta=json.loads((run/(video+'.json')).read_text());cap=cv2.VideoCapture(str(run/(video+'.mp4')));idx=max(0,int(meta['frames'])-5);cap.set(cv2.CAP_PROP_POS_FRAMES,idx);ok,img=cap.read();cap.release();assert ok;cv2.imwrite(str(folder/'video_end.png'),img)
(D/'accuracy_summary.json').write_text(json.dumps(stats,indent=2)+'\n');(D/'body_projection_summary.json').write_text(json.dumps(guards,indent=2)+'\n');(D/'source_consistency.json').write_text(json.dumps(sources,indent=2)+'\n')
p=D/'REPORT.md';s=p.read_text().replace('实际碰撞','仿真接触事件')
intro='''
**优先结论：** 起飞组合修正使32完成整场、34完成升高，但34随后陷入占据起点；38/40暴露低空补搜未共享边界许可、以及恢复交接余量不足。不能据前三个PASS宣布停滞或边界问题全部根治。

[详细首因及下一步](DIAGNOSIS.md) · [视频和全部图表总览](index.html)

38/40的`actual_collision`来自55cm鲁棒代理包络与Wall_11接触，不是地面接触；首次采样深度不代表整段冲击峰值，也不证明真实裸机撞墙。严格Gate FAIL保持不变，详见诊断说明。
'''
s=s.replace('## 版本、场地和计时',intro+'\n## 版本、场地和计时',1)
s=s.replace('## 单轮指标和材料','''## 分阶段耗时与对准

任务计时从首条mission决策起，不含此前起飞准备；视频包含启动和少量收尾。相同阶段对比：31三投用时比旧轮增加约25.65秒，37增加约4.06秒。当前没有完整完赛节时的双成功配对，本批不能声称提速。

![分阶段耗时及未闭合观测段](stages.png)

斜线部分是失败轮尚未闭合阶段的已观察时间，不是成功完赛时间；静止时间还可能包含必要的对齐、投递和等待，不能全部称作停滞浪费。

![mock提交时的真实对准误差](release_accuracy.png)

| 样本 | ACK数 | 中位/cm | P95/cm | 最大/cm |
| --- | --- | --- | --- | --- |
'''+''.join(f'| {k} | {v["n"]} | {v["median_cm"]:.2f} | {v["p95_cm"]:.2f} | {v["max_cm"]:.2f} |\n' for k,v in stats.items())+'''
成功ACK集合不同，以上不是严格同目标配对精度改善结论；仅统计已成功提交的mock样本，不是包裹落点。保守树投影独立结果见[汇总](body_projection_summary.json)，与接触Gate分开解释。

## 单轮指标和材料''',1)
p.write_text(s)
p=D/'index.html';s=p.read_text().replace('<a href="comparison.json">配对数据</a>','<a href="comparison.json">配对数据</a> · <a href="DIAGNOSIS.md">失败首因与下一步</a>')
s=s.replace('<h1>历史失败六seed回归</h1>','<h1>历史失败六seed回归：3/6 PASS</h1><p>通过31/32/37；34任务失败；38/40鲁棒包络接触Wall_11。保留原Gate结果。</p><img src="paired_paths.png" alt="新旧航迹"><img src="stages.png" alt="阶段耗时"><img src="release_accuracy.png" alt="对准误差">')
p.write_text(s)
validation=json.loads((D/'validation.json').read_text());validation.update(all_overlay_heads_match=True,png_count=len(list(D.rglob('*.png'))),release_accuracy=stats,body_projection=guards)
for image_path in D.rglob('*.png'):assert cv2.imread(str(image_path)) is not None,image_path
broken=[]
for doc in D.rglob('*.md'):
 for target in re.findall(r'\]\(([^)]+)\)',doc.read_text()):
  if target.startswith(('http','/','#')):continue
  if not (doc.parent/target.split('#')[0]).exists():broken.append([str(doc),target])
validation['broken_markdown_links']=broken;assert not broken,broken
(D/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps({'accuracy':stats,'guards':guards,'pngs':validation['png_count'],'sources':len(sources)},ensure_ascii=False))
