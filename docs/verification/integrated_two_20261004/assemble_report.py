#!/usr/bin/env python3
"""Assemble the closed-run report and local video entry without running ROS."""
from pathlib import Path
import html
import json
import os

D = Path(__file__).resolve().parent
R = D.parents[2]
batch = json.loads((R / 'logs/integrated_two_20261004_batch/matrix.json').read_text())
assert batch['status'] == 'COMPLETE' and len(batch['results']) == 2
metrics = json.loads((D / 'metrics.json').read_text())
centers = {seed: json.loads((D / f'{seed}_centers/center_summary.json').read_text()) for seed in (31, 38)}

def relative(path):
    return os.path.relpath(path, D)

rows = ['## 正式结果', '', '| seed | Gate | 模拟投递 | 完赛/ROS秒 | 三投完成/秒 | 高位中断/秒 | 接触数 | 落地中心误差/cm |',
        '|---|---|---|---:|---:|---:|---:|---:|']
for m in metrics:
    assert m['status'] == 'PASS'
    rows.append(f"| {m['seed']} | {m['status']} | 红十字→桥梁→装甲车，3/3 | {m['completed_mission_s']:.2f} | {m['third_commit_s']:.2f} | {m['top3_interrupt_mission_s']:.2f} | {m['collisions']} | {m['touchdown_truth_center_error_m'] * 100:.2f} |")
rows += ['', '两轮均完成九个投后航点、两扇门和 H 降落；Gate 记录零边界/高度违规，未耗尽无进展恢复预算，未进入整场低空补搜。结束后统一脚本确认 ROS/Gazebo/PX4/RViz 零残留。',
         '', '## 靶心与真值', '', '下表为有效、新鲜、按观测时间去重的映射点；为区分类别错误，定位列仅取“最近真值类别与预测类别相同”的样本。全部样本和疑似错类另见分组报告。数字为**中位数 / P95（cm）**，不是类别召回率。', '',
         '| 类别 | seed31 | seed38 |', '|---|---:|---:|']
for cls in ('red_cross', 'panzer', 'bridge', 'pillbox', 'tent', 'landing_pad'):
    values = []
    for seed in (31, 38):
        found = [s for s in centers[seed]['statistics'] if s['scope'] == 'fresh_nearest_class_agrees' and s['stream'] == 'mapped' and s['class_name'] == cls]
        assert len(found) <= 1
        values.append(f"{found[0]['median_m']*100:.2f} / {found[0]['p95_m']*100:.2f}（{found[0]['n']}帧）" if found else '缺测')
    rows.append('| ' + cls + ' | ' + ' | '.join(values) + ' |')
rows += ['', '低位重新捕获时的融合坐标误差（cm）：', '', '| seed | 红十字 | 桥梁 | 装甲车 |', '|---|---:|---:|---:|']
for m in metrics:
    values = {v['class_name']: v['error_m']*100 for v in m['low_reacquisition_truth_errors']}
    rows.append(f"| {m['seed']} | {values['red_cross']:.2f} | {values['bridge']:.2f} | {values['panzer']:.2f} |")
rows += ['', 'seed31 的高位支持中仍保留过 pillbox 附近的 panzer 竞争假设，但真正 panzer 也形成了粗框投影线索。中断时真实 panzer 粗线索误差约 34.6cm；低位复核后约 6.9cm，随后投递正确目标。高位“有线索”和“已精修确认”并不是同一门槛。模型误分类并未被本次 PASS 消除，不能据此撤去跨类别复核。',
         '', 'seed38 桥梁全部有效 mapped 观测的 P95 为 **69.41cm**，上表同类近邻子集的 63.03cm 已排除六个更偏离的点。ROS106.480 的同帧显示桥梁位于裁切边缘，记录的精修中心却在类别 ROI 外，仍被标为关联有效，支持圆心关联异常；具体代码根因尚未确定。这簇主要发生在桥梁投完后的 panzer 投递阶段。另有释放前的异常以及 panzer 极近距离裁切观测进入记忆融合，因此不能宣称所有坏点都被拒绝。任务最终未出现错误类别/实例的释放决策。详见 [逐帧长尾诊断](FINAL_CENTER_DIAGNOSTICS.md)。',
         '', '两轮按 source 时刻统计的 landing 模式映射输出均只有 H，没有非 H 类别框；轻量 bag 未录原始 detector 话题，不能仅据此证明模型推理进程必然停止。',
         '', '首次高位、冻结的中断快照、低位复核分别统计：[seed31 分阶段](31_centers/stages/PHASE_REPORT.md)、[seed38 分阶段](38_centers/stages/PHASE_REPORT.md)。完整统计：[seed31](31_centers/CENTER_REPORT.md)、[seed38](38_centers/CENTER_REPORT.md)。',
         '', '## 视频、图表和时间成本', '', '统一入口：[本地视频索引](index.html)。每轮保留跟随视角＋俯视多画面、下视视觉坐标叠加、原始相机视频及对应时间戳。视频显示录制结果，不重新跑检测器；没有同步观测时不虚构框或靶心。', '',
         '| seed | 三投前 | 最后投递恢复 | 转场＋进廊下降 | 走廊至H | H降落 |', '|---|---:|---:|---:|---:|---:|']
for m in metrics:
    v = m['milestone_durations_s']
    rows.append(f"| {m['seed']} | {v['start_to_third_ack']:.2f}s | {v['last_release_recovery']:.2f}s | {v['fast_transfer_to_staging']+v['staging_descent']:.2f}s | {v['corridor_to_H']:.2f}s | {v['H_landing']:.2f}s |")
rows += ['', '本轮没有落实新的连贯航点或提速改动。历史 9/27 通过轮 seed31 为 226.72s，修正后的 seed38 为 193.65s；本轮同时变更模型、若干链路修复、膨胀和 H 尺寸，不能把时差全部归功于某一个补丁，也不是严格模型 A/B。',
         '', '## 尚不能由这两轮证明的事项', '',
         '- 独立硬件入口与研究入口的 H 默认配置仍有差异，尤其是示例的 1.8m 与笔画兜底配置；本轮验证的是明确记录的研究场景参数，下一次上板需使用已核对的现场配置。',
         '- 零接触不等于机体投影完全不越过树木最大轮廓。seed38 的机体中心没有跨入离线树体＋底座水平凸包，但旋转后的55cm包络仍有72个插值采样与该凸包重叠，最深约23.6cm；seed31无此重叠。当前使用的是已约定的树冠中部建柱、允许经过下部外缘上方的口径，不能把PASS另行解释为最大树冠投影全禁飞验证。详细结果在各轮 body_projection.json。',
         '- 未模拟真实 LIO 延迟、外部定位融合停启及高度重置。日志存在 MAVROS 时间同步重置提示；不能把本轮完成任务解释为已根治实飞高度跳变。',
         '- 标准无反光 H 的两轮动态回归通过，不代表所有反光、遮挡、不同曝光场景都通过。H 离线反光修复证据与本次标准 H 回归应分别使用。',
         '- 真实舵机、RC 模式切换/接管、板端性能和现场外参没有在本次硬件验收。飞机断电后未部署新内容。',
         '- 原始逐帧导出和视频仅保留本地；Git 中保留脚本、场景、摘要、图表和可复现说明。', '']
if (D / 'CLEANUP.md').exists():
    rows += ['', (D / 'CLEANUP.md').read_text()]
report = D / 'REPORT.md'
text = report.read_text()
text = text.split('## 正式结果')[0]
text = text.replace('报告编制中，seed31 已完成，seed38 仍在运行；最终结果以本文完成后的结果表为准。', '两轮正式运行均完成并通过。')
text = text.replace('当前运行与视频清单、两轮结果、中心统计、清理结果将在两轮结束后补齐。', '')
report.write_text(text.rstrip() + '\n\n' + '\n'.join(rows))

cards = []
for m in metrics:
    seed, run = m['seed'], Path(m['run'])
    review = relative(run / 'presentation_review.mp4')
    overlay = f'{seed}_mapped_overlay.mp4'
    cards.append(f'''<section><h2>Seed {seed} · {m['status']} · {m['completed_mission_s']:.2f}s</h2>
<p>三投 → 两门 → 80cm H 降落；本次未连接实机。</p>
<h3>跟随＋俯视多画面</h3><video controls preload="metadata" src="{html.escape(review)}"></video>
<h3>下视视觉坐标与真值偏差</h3><video controls preload="metadata" src="{overlay}"></video>
<p><a href="{seed}_centers/CENTER_REPORT.md">中心统计</a> · <a href="{seed}_centers/stages/PHASE_REPORT.md">高/低位分阶段</a> · <a href="{seed}_integrated_two/metrics.json">任务指标</a></p>
<img loading="lazy" src="{seed}_integrated_two/route_stages.png" alt="分阶段航迹">
<img loading="lazy" src="{seed}_integrated_two/speed_profile.png" alt="飞行速度">
<img loading="lazy" src="{seed}_integrated_two/height_tilt.png" alt="高度与倾斜角">
<img loading="lazy" src="{seed}_centers/center_errors.png" alt="靶心误差"></section>''')
(D / 'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>整机两轮回归 20261004</title><style>body{background:#10171c;color:#e4edf1;font:16px/1.6 system-ui;max-width:1300px;margin:30px auto;padding:0 20px}section{background:#1b262e;padding:20px;margin:24px 0;border-radius:12px}video,img{width:100%;max-width:1200px}a{color:#82cfff}h1,h2,h3{line-height:1.3}p{max-width:1000px}</style><h1>最新整机链 · Seed31 / Seed38</h1><p><a href="REPORT.md">完整报告</a> · <a href="H_FRAME_DIAGNOSTIC.md">H笔画兜底诊断</a></p><p>H 80×80cm；观察本地 Z=0.68m（约0.90m AGL）；保留研究入口原有笔画兜底。记录帧按 ROS 时间对齐，缺帧保持会标注；播放用时不取代任务时间。</p>''' + ''.join(cards) + '</html>')
print('REPORT.md and index.html assembled')
