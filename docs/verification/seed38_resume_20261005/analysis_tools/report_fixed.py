from pathlib import Path
import json,importlib.util,types,csv,math,os,html
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/seed38_resume_20261005';O=D/'fixed_rerun'
spec=importlib.util.spec_from_file_location('comparison',D/'analysis_tools/compare.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
b=json.loads((H/'logs/seed38_resume_20261005_fixed_batch/matrix.json').read_text());r=b['results'][0];run=Path(r['run'])
a=m.analyze_case(r,H,O,types.SimpleNamespace(pose_max_age=.25));a['scene']=r['scene']
contact=json.loads((run/'gazebo_contact_status.json').read_text());support=contact['support_events'][-1];start=a['mission_start_ros_s'];touch=support['ros_stamp'];abort=222.884
poses=list(csv.DictReader((run/'truth_pose.csv').open()));stable=[p for p in poses if touch+1<float(p['t'])<abort]
ranges={k:max(float(p[k]) for p in stable)-min(float(p[k]) for p in stable) for k in ('x','y','z')}
last=poses[-1];error=math.hypot(float(last['x'])-8.75,float(last['y'])+4.2)
a['user_landing_criterion']=dict(continuous_support_start_ros_s=touch,stable_truth_from_ros_s=touch+1,truth_range_m=ranges,abort_ros_s=abort,contact_to_abort_s=abort-touch,contact_mission_s=touch-start,truth_h_center_error_m=error,ground_stable=True,autopilot_ground_and_disarm_verified=False,note='按用户落地数秒后才异常可记为物理完成口径；飞控最后仍AUTO.LAND/armed/LANDING，软件Gate不改写。')
(O/'diagnosis.json').write_text(json.dumps(a,ensure_ascii=False,indent=2))
truth=json.loads((O/'release_truth.json').read_text())[0]['drops'];third=truth[-1]['ros_s']-start
md=f'''# seed38 修正轮：三投、走廊与H触地

源码 **10120dfa**；相机目标AGL2m、FC目标AGL2.16m；10×10净场地、固定四树、80cm H。运行目录：`{run}`。

**三次模拟释放全部指向真实靶标，未发生许可拒绝或重复执行；通过两道门，零障碍接触。** 高位首次经过已得到真实panzer，本轮没有触发高位续扫，不能单凭本轮给续扫计算节时。

| 节点 | 任务ROS秒 |
|---|---:|
| 三槽全部模拟释放成功 | {third:.3f} |
| H板持续支撑接触开始 | {touch-start:.3f} |
| 触地后位姿保护ABORT | {abort-start:.3f} |

H中心真值偏差 **{error*100:.2f}cm**。216.206 ROS秒持续接触，222.884秒ABORT，相隔 **{abort-touch:.3f}秒**；触地一秒后至ABORT的真值XYZ变化范围分别为 {ranges} m。按用户确认的“触地后稳定数秒即可由飞手切停”口径，可记为物理任务完成。

原始软件Gate仍为 **FAIL**，原因`probe_pose_exception:ValueError`；日志结束前飞控仍报告armed、LANDING，没有ON_GROUND/解除武装证据。不能写成自动落地上锁闭环PASS，也不是实体碰撞或空中任务中断。位姿保护本轮没有关闭。

## 真实靶位与模拟释放

下表为释放ACK附近机体中心XY误差，不是快递盒实际落点误差。

| 槽 | 类别 | ACK任务秒 | 距真实靶心 |
|---|---|---:|---:|
'''
for v in truth:md+=f"| {v['slot']} | {v['class_name']} | {v['ros_s']-start:.3f} | {v['nearest_distance_m']*100:.2f}cm |\n"
md+='''
低空重新确认的记忆中心误差：panzer13.68cm、red_cross13.47cm、bridge11.92cm。高空碉堡附近仍出现类别混淆，但真正panzer也被看见并优先复访，三次实际模拟释放未投向碉堡。

详见[阶段中心分析](38_resume_on_fixed_centers/stages/PHASE_REPORT.md)、[轨迹/速度/高度](38_resume_on_fixed/metrics.json)、[执行与物理收尾结构化分析](diagnosis.json)。

## 对此前续扫轮的参考

旧A/B同为7a950204：不续扫首次确认后派发真实panzer接近为292.967任务秒；续扫为119.557秒，**提前173.410秒**。续扫新线索于107.758秒形成。最终释放分别304.982、161.098秒，提前143.884秒；误拒绝后的折返损失了一部分阶段优势。

这是同一投递阶段的观测差值，不宣称整场净节时，也不以本修正轮替换原始A/B。[原对照完整报告](../REPORT.md)。

## 修复与验证边界

- 本轮包含：同动作许可最多0.25墙钟秒刷新，ROS决策序号接线、许可记账消息顺序修复。
- 接触开关为`stop_on_collision=false`；保留接触数据，不因碰撞观测中止。本轮零接触，因此不是“发生接触仍继续”的动态反例。
- 重跑后追加：时序NOT_STARTED立即局部重试，跳过20秒冷却；仍须新鲜视觉、新许可、固定身份，最多两次且保留总deadline。完整532项中528通过4跳过；该补丁没有新的动态重跑，不冒称已实跑。
- 所有仿真进程已完成统一收尾并确认零残留，未上板。

## 离线复现

已有ROS环境及rl_drone下，从仓库根目录执行（只读录制，不启动仿真）：

```bash
python docs/verification/seed38_resume_20261005/analysis_tools/analyze.py --matrix logs/seed38_resume_20261005_fixed_batch/matrix.json --out docs/verification/seed38_resume_20261005/fixed_rerun
python docs/verification/seed38_resume_20261005/analysis_tools/centers.py --matrix logs/seed38_resume_20261005_fixed_batch/matrix.json --out docs/verification/seed38_resume_20261005/fixed_rerun
python docs/verification/seed38_resume_20261005/analysis_tools/release_truth.py --out docs/verification/seed38_resume_20261005/fixed_rerun
```
'''
(O/'REPORT.md').write_text(md)
video=os.path.relpath(run/'presentation_review.mp4',O)
page=f'''<!doctype html><meta charset="utf-8"><title>seed38 修正轮</title><style>body{{font:18px system-ui;max-width:1200px;margin:30px auto;background:#16202c;color:#eee}}a{{color:#8dcfff}}video{{width:100%}}</style><h1>seed38 修正轮 · 三投→走廊→H触地</h1><p>三次正确靶位模拟释放，零碰撞；触地6.68秒后位姿保护ABORT。原始Gate FAIL，按用户口径物理完成。本轮未触发高位续扫。</p><p><a href="REPORT.md">报告</a> · <a href="../index.html">旧A/B视频</a> · <a href="38_resume_on_fixed_centers/stages/PHASE_REPORT.md">中心误差</a></p><video controls preload="metadata" src="{html.escape(video)}"></video>'''
(O/'index.html').write_text(page)
p=D/'REPORT.md';s=p.read_text();head='> **修正轮已完成：** [三投、零碰撞、H触地后保护中止](fixed_rerun/REPORT.md) · [视频](fixed_rerun/index.html)。以下保留7a950204原A/B，不混入新源码结果。\n\n';p.write_text(head+s if not s.startswith(head) else s)
p=D/'index.html';s=p.read_text();p.write_text(s if '修正轮完整视频与报告（10120dfa）' in s else s+'\n<p><a href="fixed_rerun/index.html">修正轮完整视频与报告（10120dfa）</a></p>\n')
print(json.dumps(dict(start=start,third=third,touch=touch-start,H_error_m=error,stable_ranges=ranges),indent=2))