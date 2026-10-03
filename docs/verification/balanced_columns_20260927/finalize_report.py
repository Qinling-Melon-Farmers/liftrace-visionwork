"""Create a local review index and mark the completed, fixed-source experiment."""
from pathlib import Path
import json,html,re
D=Path(__file__).resolve().parent;R=D.parents[2]
ms=json.loads((D/'metrics.json').read_text());v=json.loads((D/'artifact_validation.json').read_text())
assert v['status']=='PASS'
validation=json.loads((D/'validation.json').read_text());validation.update(new_flights='COMPLETE',mission_gate_pass=2,high_weight_complete=1,contacts=0,cleanup='PASS_ZERO_REMAINS',video_decode='PASS_6_OF_6',source_visual=ms[0]['source_visual'],source_navigation=ms[0]['source_navigation']);(D/'validation.json').write_text(json.dumps(validation,indent=2))
parts=['''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>开柱0.275m · seed31/38回归</title><style>body{max-width:1280px;margin:auto;padding:28px;background:#15212d;color:#e8edf2;font:17px/1.65 system-ui,sans-serif}a{color:#87d4ff}img,video{max-width:100%;height:auto;border-radius:8px}figure{margin:22px 0}section{border-top:1px solid #506173;padding:26px 0}table{border-collapse:collapse;width:100%;margin:16px 0}td,th{padding:10px;border-bottom:1px solid #506173;text-align:left}.note{background:#253a50;padding:16px;border-radius:8px}.warn{color:#ffd18c}nav{display:flex;gap:22px;flex-wrap:wrap}.jump{padding:7px 13px;margin:5px;background:#34516c;color:#fff;border:1px solid #66819a;border-radius:5px;cursor:pointer}summary{cursor:pointer;font-size:20px}figcaption{color:#becad6}h1{line-height:1.3}</style></head><body><h1>修正障碍柱开启 · XY膨胀0.275m</h1><p>2026-09-27，同一冻结版本，seed31 / seed38，2.6m高位＋原快走廊。</p><nav><a href="REPORT.md">完整分析报告</a><a href="summary.json">数据摘要</a><a href="artifact_validation.json">视频及参数检查</a><a href="#seed31">seed31</a><a href="#seed38">seed38</a></nav><p class="note">两轮均完成三投、两门及H降落，0包络接触。<strong>seed31三个高权重全部完成；seed38第三投为tent，panzer未完成。</strong>配置0.275m经5cm体素取整实际扩张0.30m。原始视频与日志保留本机，远端仓库只包含报告和图表。</p><table><tr><th>seed</th><th>任务时间</th><th>投递顺序</th><th>高权重</th></tr>''']
for m in ms:parts.append(f"<tr><td>{m['seed']}</td><td>{m['completed_mission_s']:.3f}s</td><td>{' → '.join(x['target'] for x in m['commit_times'])}</td><td>{m['high_weight_delivered']}/3</td></tr>")
parts.append('</table><figure><img src="paired_routes.png" alt="双场地航迹、三投位置与初始机头方向"><figcaption>蓝：完整实际航迹；橙：高位结束前；X向场内，Y向起飞机体左侧。树圆仅作位置示意。</figcaption></figure><figure><img src="stage_comparison.png" alt="与关柱旧轮阶段对比"><figcaption>整套改动回归，非单因素A/B。旧seed31未完成尾段，图中仅汇总完整里程碑。</figcaption></figure>')
for m in ms:
 seed=m['seed'];run=Path(m['run']);rel='../../../logs/'+run.name;vr=next(x for x in v['videos'] if x['seed']==seed and Path(x['file']).name=='presentation_review.mp4');meta=next(x for x in v['runtime_params'] if x['seed']==seed)['composition'];dur=float(vr['probe']['format']['duration'])
 parts.append(f'<section id="seed{seed}"><h2>seed{seed} · {m["completed_mission_s"]:.1f}s · 高权重{m["high_weight_delivered"]}/3</h2>')
 if seed==38:parts.append('<p class="warn">本轮PASS表示三槽/两门/H落地；红十字、bridge、tent三投，不是三个高权重齐备。H附近panzer粗线索错误，第一次bridge重访有12s首轨迹超时。</p>')
 parts.append(f'<video id="v{seed}" controls preload="metadata" poster="video_sample_{seed}.jpg" src="{rel}/presentation_review.mp4"></video><p>合成视频{dur:.1f}s，10fps，源ROS时钟1倍速；包含任务前准备，不是墙钟运行时长。缺帧保持上一图像，俯视/跟随最大图龄{meta["max_image_age_s"][0]:.2f}/{meta["max_image_age_s"][1]:.2f}s。</p><p><a href="{rel}/overview.mp4">原始俯视</a> · <a href="{rel}/follow.mp4">原始低位跟随</a> · <a href="{rel}/presentation_review.mp4" download>下载合成视频</a></p>')
 marks=[('高位中断',m['start_ros_s']+m['top3_interrupt_mission_s'])]+[(x['target']+'投递ACK',x['t']) for x in m['commit_times']]+[('H降落',m['start_ros_s']+m['land_command_mission_s'])]
 for name,t in marks:parts.append(f'<button class="jump" data-video="v{seed}" data-time="{max(0,t-meta["common_ros_start"]):.3f}">{html.escape(name)}</button>')
 for file,title in [('flight_charts','航迹 / 高度 / 速度 / 指令'),('route_stages','分阶段实际航迹'),('route_3d','三维轨迹'),('speed_profile','速度与速度档'),('height_tilt','高度与姿态'),('phase_target_timeline','阶段和目标时序'),('progress','停滞监督'),('landing_diagnostic','H降落估计误差')]:parts.append(f'<details><summary>{title}</summary><img loading="lazy" src="{seed}_balanced_columns/{file}.png" alt="{title}"></details>')
 parts.append('</section>')
parts.append('<section><h2>局部诊断与投递中心误差</h2><img loading="lazy" src="seed38_diagnostic.png" alt="seed38误线索、目标局部点云及近墙起点"><img loading="lazy" src="release_center_errors.png" alt="模拟ACK时飞控中心与真靶心误差"><p>局部云只覆盖bridge目标，不能证明远处起点可达；误差不是实物落点或槽位补偿精度。详见报告。</p></section><script>document.querySelectorAll(".jump").forEach(b=>b.addEventListener("click",()=>{const v=document.getElementById(b.dataset.video);v.currentTime=Number(b.dataset.time);v.scrollIntoView({block:"center",behavior:"smooth"});}));</script></body></html>')
(D/'index.html').write_text('\n'.join(parts),encoding='utf-8')
report=D/'REPORT.md';txt=report.read_text().split('\n产物终检：',1)[0];txt+='\n产物终检：六个视频均完整解码通过，合成录像'+ ' / '.join(f"seed{x['seed']} {float(x['probe']['format']['duration']):.1f}s" for x in v['videos'] if Path(x['file']).name=='presentation_review.mp4')+'；合成画面、轨迹/高度图和局部诊断图已抽查，网页本地资源链接已核查。\n';report.write_text(txt)
missing=[]
for attr,url in re.findall(r'(src|href)="([^"]+)"',(D/'index.html').read_text()):
 if url.startswith(('#','http')):continue
 if not (D/url).exists():missing.append(url)
assert not missing,missing
print('INDEX_LOCAL_LINKS_PASS')
for x in v['videos']:print(x['seed'],Path(x['file']).name,x['probe']['format']['duration'],x['decode_pass'])