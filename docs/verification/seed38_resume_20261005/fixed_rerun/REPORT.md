# seed38 修正轮：三投、走廊与H触地

源码 **10120dfa**；相机目标AGL2m、FC目标AGL2.16m；10×10净场地、固定四树、80cm H。运行目录：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_on_fixed_seed38_20261005_173322`。

**三次模拟释放全部指向真实靶标，未发生许可拒绝或重复执行；通过两道门，零障碍接触。** 高位首次经过已得到真实panzer，本轮没有触发高位续扫，不能单凭本轮给续扫计算节时。

| 节点 | 任务ROS秒 |
|---|---:|
| 三槽全部模拟释放成功 | 132.507 |
| H板持续支撑接触开始 | 204.169 |
| 触地后位姿保护ABORT | 210.847 |

H中心真值偏差 **10.80cm**。216.206 ROS秒持续接触，222.884秒ABORT，相隔 **6.678秒**；触地一秒后至ABORT的真值XYZ变化范围分别为 {'x': 3.08911314306215e-08, 'y': 3.7509563455273565e-08, 'z': 2.2140079519727962e-08} m。按用户确认的“触地后稳定数秒即可由飞手切停”口径，可记为物理任务完成。

原始软件Gate仍为 **FAIL**，原因`probe_pose_exception:ValueError`；日志结束前飞控仍报告armed、LANDING，没有ON_GROUND/解除武装证据。不能写成自动落地上锁闭环PASS，也不是实体碰撞或空中任务中断。位姿保护本轮没有关闭。

## 真实靶位与模拟释放

下表为释放ACK附近机体中心XY误差，不是快递盒实际落点误差。

| 槽 | 类别 | ACK任务秒 | 距真实靶心 |
|---|---|---:|---:|
| 1 | panzer | 66.565 | 6.56cm |
| 2 | red_cross | 111.803 | 8.65cm |
| 3 | bridge | 132.507 | 7.14cm |

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
