from pathlib import Path
import csv,json,math,shutil,subprocess,os,html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[4];D=R/'docs/verification/snake3_camera2m_20261005';O=D.parent/'latest_six_20261005'
metrics=json.loads((D/'metrics.json').read_text());old=json.loads((O/'metrics.json').read_text());physical=json.loads((D/'physical_completion.json').read_text());oldphysical=json.loads((O/'physical_completion.json').read_text());releases=json.loads((D/'release_truth.json').read_text())
rows=[]
fig,axes=plt.subplots(2,1,figsize=(12,8))
for i,m in enumerate(metrics):
 run=Path(m['run']);events=[json.loads(s) for s in (run/'high_view_full_events.jsonl').read_text().splitlines()]
 stamps=np.array([e['t'] for e in events]);data=list(csv.DictReader((run/'truth_pose.csv').open()))
 t=np.array([float(x['t']) for x in data]);z=np.array([float(x['z']) for x in data]);q=np.array([[float(x[k]) for k in ['qx','qy','qz','qw']] for x in data]);q/=np.linalg.norm(q,axis=1)[:,None]
 # CSV world z is shifted by -0.22m by competition_key_recorder. Camera mount is -0.16m body z.
 fc=z+.22;camera=fc-.16*(1-2*(q[:,0]**2+q[:,1]**2))
 ix=np.maximum(0,np.searchsorted(stamps,t,side='right')-1)
 mask=np.array([events[j]['status'].get('stage')=='SURVEY' and events[j]['status'].get('ascent_verified',False) for j in ix])
 mask&=t>=stamps[0]
 stats=lambda a:{'n':len(a),'p05':float(np.percentile(a,5)),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(max(a))} if len(a) else None
 p=next(x for x in physical if x['seed']==m['seed']);om=next(x for x in old if x['seed']==m['seed'] and x['label']=='snake3');op=next(x for x in oldphysical if x['seed']==m['seed'] and x['variant']=='snake3');rel=next(x for x in releases if x['seed']==m['seed'])['drops']
 row=dict(seed=m['seed'],source=json.loads((R/'logs/snake3_camera2m_20261005_batch/matrix.json').read_text())['source'],gate=m['status'],reason=m['reason'],physical=p,releases=rel,survey_fc_agl=stats(fc[mask]),survey_camera_agl=stats(camera[mask]),mission_complete_s=m.get('completed_mission_s'),physical_touchdown_s=p.get('touchdown_mission_s'),old_physical_touchdown_s=op.get('touchdown_mission_s'),old_gate=om['status'],actions=m.get('actions'),milestones=m.get('milestone_durations_s'),low_confirmation_errors=m.get('low_reacquisition_truth_errors'),run=str(run))
 if p.get('touchdown_mission_s') is not None and op.get('touchdown_mission_s') is not None:row['touchdown_delta_s']=p['touchdown_mission_s']-op['touchdown_mission_s']
 rows.append(row)
 ax=axes[i];ax.plot(t-m['start_ros_s'],fc,label='True FC AGL');ax.plot(t-m['start_ros_s'],camera,label='True optical center AGL');ax.axhline(2.16,color='C0',ls='--',alpha=.5);ax.axhline(2.,color='C1',ls='--',alpha=.5);ax.fill_between(t-m['start_ros_s'],0,4,where=mask,alpha=.10,color='green',label='Survey after ascent');ax.set(title=f"Seed {m['seed']} | camera target 2.00m / FC 2.16m | {m['status']}",xlabel='Mission ROS seconds',ylabel='Ground-referenced height (m)',ylim=(0,3.2));ax.grid(alpha=.2);ax.legend(loc='upper right')
fig.tight_layout();fig.savefig(D/'true_heights.png',dpi=140);plt.close(fig)
(D/'comparison.json').write_text(json.dumps(rows,indent=2))
fmt=lambda v:'—' if v is None else f'{v:.2f}'
lines=['# 镜头2m三扫描线两轮整机补验','', '本次按原设计恢复镜头AGL **2.00m**、FC中心AGL **2.16m**；三线X=1.0/3.7/6.4m，端点Y=-3.95/+4.10m。原六轮的镜头2.6m内收三线保留，不能混作2m结果。','', '两轮固定源码`'+rows[0]['source'][:8]+'`。生产代码与上一批87258798相同，只改本批高度、航线及绝对场景入口；模型、速度、10×10m净场地、四树、靶位、H80cm、障碍柱和膨胀保持一致。场景逐文件比较见geometry_comparison.json。','', '| seed | 原始Gate | 正确标签模拟释放 | 实体碰撞 | 物理落地任务秒 | 原2.6m三线落地秒 | 差值（新−旧） |','|---|---|---:|---:|---:|---:|---:|']
for x in rows:lines.append(f"| {x['seed']} | {x['gate']} | {sum(v['label_matches'] for v in x['releases'])}/{len(x['releases'])} | {x['physical']['collision_count']} | {fmt(x['physical_touchdown_s'])} | {fmt(x['old_physical_touchdown_s'])} | {fmt(x.get('touchdown_delta_s'))} |")
lines+=['', '按用户约定的落地后飞手切停口径，两轮均完成三类正确模拟释放、两扇门和H落地，实体碰撞为0。相较旧三线方案分别节省23.663s（10.37%）、40.444s（10.01%）。这仅是两个固定靶位的方案对照，不能据此宣称普遍提升10%。', '', '## 原始失败与物理落地分开记录', '', '- seed31：物理落地任务204.517s；落地7.732s后才出现 `probe_pose_exception:ValueError`。该时段仿真真值静止，估计高度跳变，原始Gate为FAIL。', '- seed38：物理落地任务363.540s；随后飞控报告ON_GROUND并自动解除解锁，任务仍留在LAND。落地236.467s后因 `landing_deadline_reached` / `safety_motion_timed_out` 结束。现场状态快照见seed38_landing_live_status.json（control_state_not_landing）。真值尾段Z范围仅3.5mm。', '- 本次没有修补或放宽终态Gate，也没有把这些尾段软件缺陷写成已解决；保留原始FAIL并另列物理完成。', '', '## seed38为何仍较慢', '', 'ROS52.990s出现的panzer粗线索位于碉堡附近；53.201s高位集齐中断发生在第二条扫描线上，第三条尚未飞完。低空61.110s已将panzer纠正为pillbox，61.206s将该位置延期，未在碉堡上错误投递。106.176s转入LOW_COVERAGE，之后寻找真正panzer并于298.914s释放；补搜启动至第三次ACK约192.7s，包含最终接近、对准。', '', '旧镜头2.6m三线seed38也经历了同样的高空误线索→低空纠正→补搜。此次降低镜头高度没有根除提前中断问题，但类别修复挡住了误投。后续可研究低空否定后接着飞未完成高位航段；本次只记录建议，不修改固定测试版本。完整事件见behavior.json。', '', '## 理想覆盖的边界', '', '完整飞完新三线时，按实测FOV比例、固定朝向和2cm网格估算：搜索净区84.5㎡的点覆盖99.2%，1×1m靶的合法中心覆盖100%，整张靶完整入镜覆盖99.11%。旧2.6m三线对应99.8%、100%、99.78%。84.5㎡是扣除走廊与隔墙后的搜索区域，整个场地仍为10×10m。', '', '这说明低一些、横向展开的三线几何覆盖基本够用，但不是保证识别成功。seed38提前中断，尤其不能把理想整圈覆盖当作该轮实际覆盖。']
lines+=['','原始Gate不改写；另列物理落地并依据用户落地后飞手切停口径判断。任何空中碰撞、错误靶位释放均不因落地而豁免。计时从任务接受开始，视频播放长度不是完赛秒数。','', '## 实际高度','', 'CSV真值已扣0.22m起飞参考偏移，以下加回得到FC离地，再按同帧完整姿态旋转相机−0.16m安装偏移求镜头高度。取高位SURVEY且ascent_verified之后的样本，包含巡航跟踪波动；并非直接抄YAML。','', '| seed | FC真高P05 / 中位 / P95(m) | 镜头真高P05 / 中位 / P95(m) |','|---|---|---|']
for x in rows:
 v=x['survey_fc_agl'];c=x['survey_camera_agl'];lines.append(f"| {x['seed']} | "+(' / '.join(fmt(v[k]) for k in ['p05','median','p95']) if v else '无有效高位样本')+' | '+(' / '.join(fmt(c[k]) for k in ['p05','median','p95']) if c else '无有效高位样本')+' |')
lines+=['', '注意：设定是镜头2.00m，但巡航实际镜头中位数为2.114/2.128m，仍比设定高约11–13cm。表中FC与镜头均来自同帧仿真真值；不能把配置值当作精确实测高度。这两轮纠正了原先2.6m镜头配置，尚不能宣称高度跟踪已零偏差。', '']
lines+=['','![实际镜头和FC高度](true_heights.png)','','## 投递位置与中心误差','', '释放位置核查使用仿真真值中的机体中心和最近靶标，不能当作包裹弹道落点精度。低空确认误差、逐帧中心投影和各阶段图表见各seed的centers目录。','']
for x in rows:
 lines.append(f"### seed{x['seed']}");lines+=['',f"终止原因：`{x['reason']}`。",'']
 for v in x['releases']:lines.append(f"- 槽{v['slot']}：任务标签{v['class_name']}，最近真实靶{v['nearest_class']}，机体中心距靶心{v['nearest_distance_m']*100:.1f}cm。")
 lines+=['',f"[低空/投递中心分析]({x['seed']}_snake3_centers/center_summary.json)",f"[完整运行目录]({os.path.relpath(x['run'],D)}/)",'']
lines+=['## 低空确认中心误差', '', '这是进入低空确认时的地图中心与同类靶真实中心的XY距离，不是释放点或包裹落点。seed38的panzer通过后续补搜完成，没有同名低空重访事件，留空而不填零。', '', '| seed | 红十字 | 桥梁 | 装甲车 |', '|---|---:|---:|---:|', '| 31 | 11.5cm | 15.4cm | 10.9cm |', '| 38 | 14.1cm | 10.8cm | 无该事件 |', '']
lines+=['## 比较范围与遗留','', '- 本次同时恢复低高度和原三线位置，是方案比较，不是只改变高度的消融实验。', '- 理想覆盖不含树遮挡、倾斜、绕行、动态模糊；实际检出/确认/释放须看本轮记录。', '- 旧2.6m seed38落地后终态超时仍保留，不能拿它的超时截尾时长冒充真实飞行时长。', '- 首个相对路径异常启动保留于STARTUP_DIAGNOSTIC.md，不计有效两轮；之后只有修正后的seed31/38各一轮，无替换失败结果。', '- 本次不上板、不改现场2m限高专项、不合入main或整机候选。','', '[视频与图表入口](index.html)']
(D/'REPORT.md').write_text('\n'.join(lines)+'\n')
page=['<!doctype html><meta charset="utf-8"><title>镜头2m三线补验</title><style>body{background:#141820;color:#eee;font-family:system-ui;max-width:1300px;margin:auto;padding:24px}a{color:#8dcfff}video{width:100%;max-height:780px}img{max-width:100%}table{border-collapse:collapse}td,th{padding:10px;border:1px solid #667}</style><h1>镜头2m三扫描线：seed31 / seed38</h1><p>FC目标2.16m，X=1.0/3.7/6.4，Y=-3.95/+4.10；固定四树、净空10×10m、最新公共补丁。</p><p><a href="REPORT.md">报告</a> · <a href="comparison.json">对比数据</a> · <a href="geometry_comparison.json">覆盖与场景一致性</a></p><img src="true_heights.png">']
for x in rows:
 rel=os.path.relpath(x['run'],D);page.append(f"<h2>Seed{x['seed']} · 原始软件Gate {x['gate']}</h2><p>按落地切停口径：{('完成' if x['physical'].get('physical_landing_accepted') else '未完成')}。软件终止原因：{html.escape(x['reason'])}</p><p>物理落地任务秒：{fmt(x['physical_touchdown_s'])}，旧2.6m三线：{fmt(x['old_physical_touchdown_s'])}</p><video controls preload='metadata' src='{rel}/presentation_review.mp4'></video><p>"+' · '.join(f"<a href='{rel}/{n}.mp4'>{n}</a>" for n in ['downward','follow','overview'])+'</p>')
 for f in sorted((D/f"{x['seed']}_snake3").glob('*.png')):page.append(f'<img loading="lazy" src="{f.relative_to(D)}">')
 for f in sorted((D/f"{x['seed']}_snake3_centers").rglob('*.png')):page.append(f'<img loading="lazy" src="{f.relative_to(D)}">')
(D/'index.html').write_text('\n'.join(page))
shutil.copy2(R/'logs/snake3_camera2m_20261005_batch/matrix.json',D/'BATCH.json')
print(json.dumps([{k:v for k,v in x.items() if k not in ['actions','milestones']} for x in rows],indent=2))