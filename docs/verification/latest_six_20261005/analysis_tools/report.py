from pathlib import Path
import json,csv,math,html
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/latest_six_20261005';O=H/'docs/verification/route_speed_20261004'
def load(p,default=None):return json.loads(p.read_text()) if p.exists() else default
metrics=load(D/'metrics.json',[]);physical={(r['seed'],r['variant']):r for r in load(D/'physical_completion.json',[])};release={(r['seed'],r['variant']):r for r in load(D/'release_truth.json',[])};oldphysical={(r['seed'],r['variant']):r for r in load(O/'physical_completion.json',[])};oldmetrics={(r['seed'],r['label']):r for r in load(O/'metrics.json',[])};comp={(r['seed'],r['variant']):r for r in load(D/'comparison.json',{'rows':[]})['rows']};oldcomp={(r['seed'],r['variant']):r for r in load(O/'comparison.json',{'rows':[]})['rows']}
fmt=lambda v:'—' if v is None else f'{v:.2f}'
batch=load(H/'logs/latest_six_20261005_batch/matrix.json');rows=[];comparison=[];cards=[];centerlines=[]
for m in metrics:
 key=(m['seed'],m['label']);p=physical.get(key,{});rel=release.get(key,{});c=comp.get(key,{});name=f"{m['seed']}_{m['label']}";ok=sum(r['label_matches'] for r in rel.get('drops',[]));n=len(rel.get('drops',[]));run=Path(m['run']);ph=load(D/(name+'_centers')/'stages/phase_summary.json',{})
 rows.append(f"| {name} | {m['status']} | {ok}/{n} | {','.join(r['nearest_class'] for r in rel.get('drops',[]))} | {fmt(m.get('completed_mission_s'))} | {fmt(p.get('touchdown_mission_s'))} | {p.get('collision_count','—')} | {m.get('reason','')} |")
 for prev in [m['label'],'rectangle_baseline']:
  k=(m['seed'],prev);op=oldphysical.get(k,{});om=oldmetrics.get(k,{});oc=oldcomp.get(k,{})
  new_t=p.get('touchdown_mission_s');old_t=op.get('touchdown_mission_s');saved=None if new_t is None or old_t is None else old_t-new_t
  q=dict(seed=m['seed'],variant=m['label'],compare_to=prev,old_physical_touchdown_s=old_t,new_physical_touchdown_s=new_t,physical_saved_s=saved,new_correct_release_count=ok,new_release_count=n,new_collisions=p.get('collision_count'),old_collisions=op.get('collision_count'),old_post_delivery_s=oc.get('phases',{}).get('post_delivery_route'),new_post_delivery_s=c.get('phases',{}).get('post_delivery_route'),old_recovery_s=oc.get('recovery_sum_s'),new_recovery_s=c.get('recovery_sum_s'))
  q['quality_caveat']='Old seed38 rectangle has wrong-target release; not equal-quality time saving.' if k==(38,'rectangle') else 'One run per seed/config; vision and planning variability are included.';comparison.append(q)
 low='；'.join(e['class_name']+' '+fmt(e.get('error_m'))+'m' for e in m.get('low_reacquisition_truth_errors',[]));high='；'.join(e['class_name']+' '+fmt(e.get('same_class_distance_m'))+'m'+('（疑似异类）' if e.get('category_confusion_suspected') else '') for e in ph.get('first_high_survey',[]) if e.get('first_high_observation_of_class'))
 hmark=(m.get('gate_metrics',{}).get('valid_landing_h_mark') or {}).get('anchor_error_m');landerr=m.get('touchdown_truth_center_error_m');centerlines.append(f"| {name} | {high} | {low} | {fmt(None if hmark is None else hmark*100)} | {fmt(None if landerr is None else landerr*100)} |")
 video='../../../logs/'+run.name+'/presentation_review.mp4';valid=(run/'presentation_review.json').exists();native=f'<p><a href="../../../logs/{run.name}/downward.mp4">下视原始录像</a> · <a href="../../../logs/{run.name}/follow.mp4">跟随视角</a> · <a href="../../../logs/{run.name}/overview.mp4">场地俯视</a></p>';cards.append(f'<section><h2>{html.escape(name)} · {m["status"]}</h2><p>正确类别释放 {ok}/{n}；物理触地 {fmt(p.get("touchdown_mission_s"))} s；碰撞 {p.get("collision_count","—")}</p>'+ (f'<video controls preload="none" src="{video}"></video>' if valid else '<p>视频正在合成。</p>')+f'<p><a href="{name}/metrics.json">指标</a> · <a href="{name}_centers/stages/PHASE_REPORT.md">分阶段靶心</a></p>'+''.join(f'<a href="{name}/{im}.png"><img loading="lazy" src="{name}/{im}.png" alt="{im}"></a>' for im in ['route_stages','flight_charts','phase_target_timeline','landing_diagnostic'] if (D/name/(im+'.png')).exists())+native+'</section>')
(D/'historical_comparison.json').write_text(json.dumps(comparison,indent=2))
text='''# 最新公共补丁：三种高位航线六轮整机验证（2026-10-05）

本批固定版本 `87258798`，矩形、双扫描线、三扫描线各跑seed31/38。下表随已结束轮次生成；原始Gate与物理落地分别记录。仅模拟释放，没有包裹弹道或实机舵机动作。

## 版本与方法

- 已含跨类记忆连续帧/换类/物理ID合并修复，四项运动修复及None选档、pending就绪/超时和高度ACK回执修复。
- FAST-LIO三线程；内环净空10×10m、外墙5cm、原始均匀四树、障碍柱ON、膨胀0.25/0.20/0.10m、H80cm；相机AGL2.6m/FC AGL2.76m，local ground=-0.22m。
- 六轮与前八轮同seed/同航线的场景模型逐项相同，见scene_comparison.json。旧八轮版本2b0678b9不含最新公共修复。
- 全域直线权重2、运动衔接开启。每轮同版一次，不用失败重跑择优。
- 原始Gate与视频的区域高度判据仍为1.2m；新规划/控制共同走廊上限是FC参考点1.0m。该Gate不能单独证明实际机体严格不超过1.0m，需同时看轨迹指令、估计高度、真值和跟踪超调。
- “物理触地”取H支撑接触，再核对真值静止。落地静止后才超时/跳变可按用户口径计飞行完成，原FAIL仍保留；空中碰撞、错靶另计。
- 时间从任务接受算ROS秒，不含前置起飞；视频长度、仿真墙钟耗时不能当任务用时。每配置每seed仅一次，不能据此估计成功率。

## 六轮结果

| seed/航线 | 原始Gate | 类别正确释放/总释放 | 实际最近靶标 | COMPLETE秒 | 物理触地秒 | 碰撞数 | 原因 |
|---|---|---:|---|---:|---:|---:|---|
'''+ '\n'.join(rows)+'\n\n当前批次状态：'+batch['status']+'，已结束'+str(len(metrics))+'/6。\n\n## 与上一批比较\n\n“节省”为旧物理触地秒减本次；正数更快。旧seed38矩形含误投，不作为等质量完赛节时基准。旧同航线与本批同时有公共补丁差异，因此不把所有变化归因于路线或单一速度项。\n\n| 新配置 | 旧对照 | 旧触地s | 新触地s | 节省s | 旧/新投后航段s | 旧/新恢复合计s |\n|---|---|---:|---:|---:|---|---|\n'
for q in comparison:text+=f"| {q['seed']}_{q['variant']} | {q['compare_to']} | {fmt(q['old_physical_touchdown_s'])} | {fmt(q['new_physical_touchdown_s'])} | {fmt(q['physical_saved_s'])} | {fmt(q['old_post_delivery_s'])} / {fmt(q['new_post_delivery_s'])} | {fmt(q['old_recovery_s'])} / {fmt(q['new_recovery_s'])} |\n"
text+='\n## 靶心与实际靶位\n\n首个高位观测与低空复核来自各自事件快照，不用最终被低空更新的记忆冒充高空定位。低空误差单位m，H单位cm；高位误类别造成的跨靶距离不能当投影误差。释放核对使用机体中心最近真实靶标，不等同快递盒落点。\n\n| 配置 | 首次高位类别/同类中心距离 | 低空重新捕获事件 | H检测cm | H触地cm |\n|---|---|---|---:|---:|\n'+'\n'.join(centerlines)+'\n\n完整释放位置见release_truth.json，中心分阶段统计见各 *_centers/stages/PHASE_REPORT.md，图表与视频见[index.html](index.html)。\n\n## 配置和现场验收\n\n现场九组操作见视觉试飞分支 `docs/deployment/board_redeploy_20261001/NINE_TRIALS_20261005.md`，导航板端参考分支有同版手册。现场仍是FC最高2m、默认0.5m/s及狭小范围，不被本批光心2.6m和10×10配置覆盖。新运动衔接显式--motion-optimized开启。仿真通过不替代真实bag/ULog、舵机和落点验收。本次没有上板，不修改main。\n'
(D/'REPORT.md').write_text(text)
(D/'index.html').write_text('<!doctype html><html lang="zh"><meta charset="utf-8"><title>最新补丁六轮整机回放</title><style>body{font:16px sans-serif;background:#151a21;color:#ddd;max-width:1400px;margin:auto;padding:24px}a{color:#80caff}video{width:100%}img{max-width:49%;margin:.3%}section{padding:20px;background:#222a35;margin:20px 0;border-radius:8px}</style><h1>最新补丁六轮整机对比</h1><p>固定87258798；原始Gate、物理落地和正确靶标分别评价。<a href="REPORT.md">报告</a> · <a href="historical_comparison.json">历史对照</a></p>'+''.join(cards)+'</html>')
print('REPORT rows',len(metrics))