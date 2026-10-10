# 2026-10-10 新增实飞 bag 离线分析

本轮首个装甲车对准已冻结并同步执行槽位补偿/下降，但未进入释放调用。下降高度与水平误差未同时满足新条件，随后记录 POSCTL、落地与解除武装；任务仍为首投 EXECUTING，未进入 H。本轮不构成三投、H 降落或实物投放成功验收。

## 输入、产物与处理范围

- 原 bag：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/flight_debug_0.bag`。
- 大小：315,953,470 字节，301.32 MiB。
- mtime：2026-10-10 21:41:11.204（北京时间）。
- 内部记录：2026-10-10 21:06:26.119 至 21:10:18.842（北京时间），232.723018 秒。日期由 bag 的记录时间确认，不能仅依文件名判断。
- 新输出：`试飞产物/analysis_flight_latest_20261010/`，与根 bag 同级。10 月 8 日两轮已有 analysis 未重新处理。
- [四视图播放器](../../../试飞产物/analysis_flight_latest_20261010/index.html) · [重点事件播放器](../../../试飞产物/analysis_flight_latest_20261010/focus_player.html) · [视频抽查拼图](../../../试飞产物/analysis_flight_latest_20261010/dashboard_review.jpg)。
- [补偿/许可时序图](compensation_timing.png) · [结构化摘要](summary.json) · [重点时间线](../../../试飞产物/analysis_flight_latest_20261010/focus_timeline.csv) · [逐样本补偿重建](../../../试飞产物/analysis_flight_latest_20261010/compensation_observed.csv)。

使用现有 `tools/bag_replay/run.sh` 导出、渲染、验证，并运行现有 `analyze_lio_flight.py`。新增分析脚本仅在本目录。没有启动 ROS Master、节点、实机、仿真或检测模型，没有修改原 bag、replay 工具、共享文档或用户未跟踪的 `merge_bags.py`，未 commit。

**本轮为 CPU 编码**：FFmpeg libx264、每路 2 threads、veryfast、CRF 23，10fps。GPU 编码由其他代理另行实现；本轮未重启渲染、不作为 GPU 编码验证。四个视频均为 232.8 秒（2328 帧，10fps 向上取整），已通过工具的全片 FFmpeg 解码与时长检查。

## 阶段时间线

以下统一以 bag 起点为 0 秒，话题和 rosout 使用录包接收时间。冻结 rosout 的源时间比接收时间早约 1.6ms；未额外做时钟校正。

| bag 秒 | 记录 | 解释 |
| --- | --- | --- |
| 0–124 | 初始化、一次短暂 armed 区间及再次准备 | 86.334–96.334 秒曾 armed，但 extended_state 未报告离地；不能将其算成另一轮完整任务 |
| 124.345 / 125.333 | armed / OFFBOARD | 后续主要飞行区间 |
| 126.129 | IN_AIR | 飞控状态回报 |
| 138.228 | SEARCH | 完整任务开始，motion_optimization 开启 |
| 176.662 | EXECUTING / APPROACH | 选中 panzer/4，decision=9、attempt=1、slot=1 |
| 177.560 | External ALIGN | 开始视觉对准 |
| 179.916 | frozen / enter_descend | 冻结靶心与 FC 补偿终点，切入下降 |
| 191.116 | permission_granted_from_commitment | 任务许可开始放行，不等于控制端几何就绪或释放 |
| 191.337 | 首个高度合格里程计样本 | 估算 AGL 0.4467m，但水平误差约 14.2cm |
| 196.816 | 高度窗口内最小重建水平误差 | AGL 0.3588m，FC/投口最大水平误差约 4.345cm，仍超过 4cm |
| 212.385 | 首个双水平误差 <4cm 样本 | FC 3.942cm、投口 3.973cm，但估算 AGL 0.3344m，低于 0.35m |
| 221.332 | OFFBOARD → POSCTL | 没有 H LAND 指令或 H 交接记录，不能归因为 H 自动交接 |
| 221.727 | 任务许可撤销 | release_altitude_invalid |
| 223.126 / 225.334 | ON_GROUND / disarmed | 飞控回报落地与解除武装；不证明 H 中心落地 |
| 228.436 | 最后任务状态仍 EXECUTING | committed_slots=0，槽 1 RESERVED，槽 2/3 FREE，无最终完成结果 |

## 对准冻结、补偿下降与释放耗时

### 已观察到的优化行为

ALIGN 到冻结 **2.3558 秒**。冻结日志为：

```text
[DropGeometry] frozen slot=1 center=(1.088,-1.299)
FC=(1.208,-1.303) stage=enter_descend decision=9
capture_z=1.1223 release_z=0.0723 observed_fc_z=1.2062
```

投口杆臂约 12cm，FC 终点偏向靶心另一侧；冻结后不要求继续用低空完整画面重新捕获。记录中视觉 stable_frames 达到 6，既有捕获证据形成后进入冻结。重复 evidence 消息数量不代表独立新图像数量，未从重复消息反推五张图片的精确回调顺序。

历史报告的 10 月 8 日两轮 ALIGN→冻结约 4.34 / 23.86 秒，本轮 2.36 秒更短；目标、姿态、路线及观测均不同，**不是同条件配对实验，不能给出归因于补丁的实测节时比例**。元数据的 deployment revision 仍登记 10 月 8 日来源，仅凭该项不能确定当前 ARM 二进制的完整 revision。当前在线 `DropGeometry waiting` 日志明确记录了新几何窗口的实际评估。

### 本轮首个失败阶段

冻结后至 POSCTL 约 **41.42 秒**；期间任务层许可放行约 **30.61 秒**，控制端连续 48 条释放等待日志，0 条 `release_ready`。首个未完成阶段是**冻结后的几何收敛/释放准入**，不是原本五帧捕获等待，也不是已经调用 Servo 后等待服务 ACK。

使用同轮 `ground_z=-0.2777390082` 换算，local Z 合法窗口为约 **0.072261–0.172261m**，对应 FC AGL **0.35–0.45m**。在 `/navigation/local_odom` 的同一样本中，按记录姿态旋转 slot 1 的 (-0.12,0,0)m 杆臂，分别计算：

1. FC 到冻结补偿 FC 终点的 XY 误差；
2. 当前投口到冻结靶心的 XY 误差；
3. 当前 FC 高度是否位于窗口内。

冻结到首次高度合格约 **11.42 秒**，冻结到首次双水平误差合格约 **32.47 秒**。在冻结至 POSCTL 的记录样本中，**两个水平误差与高度同时合格的样本数为 0**。高度合格时的最佳最大 XY 残差约 4.345cm；稍后 XY 合格时已低于高度下限。

在线控制日志也支持这一阶段判断：

```text
t=196.367 waiting: xy=0.049 fc_z=0.084 floor_z=0.072 ceiling_z=0.172
t=212.830 waiting: xy=0.036 fc_z=0.054 floor_z=0.072 ceiling_z=0.172
t=213.870 waiting: xy=0.030 fc_z=0.055 floor_z=0.072 ceiling_z=0.172
```

因此，本轮没有实测 ALIGN→释放调用、冻结→释放调用、服务耗时或 ACK 耗时可填写；这些项是**未发生/未观察到**，不能填 0 秒。没有新增释放回执，`raw_call_observed` 在已录 bridge 状态中保持 false，`payload_committed` 在最后任务结果中为 false，committed_slots 为 0。软件许可、软件 ACK、机构物理动作和包裹落点属于不同事实；此包连软件调用成功也未观察到，更不能声明物理投放成功。

## 视觉可观察项与新优化效果的边界

主要任务活动区间 138.228–221.332 秒：

| 指标 | 记录统计 |
| --- | --- |
| RKNN processing_ms | P50 50.761ms，P95 66.870ms，最大 95.212ms |
| RKNN inference_ms | P50 42.168ms，P95 53.518ms，最大 76.161ms |
| fps_ema | P50 18.971，范围 15.061–21.351 |
| target_detector 消息率 | 约 18.57Hz（录包接收时间计数） |
| 录制相机 | 1062 张，接收间隔 P50 0.2164s，最大 0.2839s |

诊断显示 native_fp16_input、latest_image_worker 为 True。本轮是整机实际并发数据，可描述其约 19FPS 表现；不能将独立检测链此前约 21.78FPS 直接当成本轮整机性能，也不能在缺同条件旧版对照时宣称帧率提升百分比。

对准后 drop_ready 理由计数包含 feedback_missing=1068、observation_stale=950、stale_observation=694；这些是重复状态消息计数，不是独立失效事件或漏检率。冻结后完整靶面离开视野、低空图像局部化，抽查 191.3/203.5/212.4 秒画面与此一致。此时冻结释放采用原承诺与当前运动几何，不能单凭 drop_ready=false 认定视觉失效是释放阻断的唯一根因。

本包未录 `/uav_vision/drop_alignment_feedback`，所以不能重建在线反馈所有拒绝字段及准确控制回调顺序。存在该话题的缺失与 stale 日志也不能直接证明线上发布节点始终未工作。有效任务许可和在线几何等待是已记录事实，错误根因还需原 run 配置/源码或补充诊断。

panzer、tent、bridge、red_cross 及辅助 circle 都有检测记录，无 pillbox 与 landing_pad。检测计数不是召回率；没有实例真值。泛化 circle 为圆环几何辅助，不能按两个 ID 算两块独立靶。24 条 incomplete fusion 日志表示融合源缺项，不能直接当相机丢帧或分类漏检。

motion_optimization 元数据为 enabled，巡航速度 1.2m/s、制动加速度 0.6m/s²；任务最终 speed_phase 为 BOUNDARY_REVISIT、following_lead_m=0.2。此次没有走完投后路线或 H，也无禁用优化的同场景对照，故不评价整场航路节时收益。

## 定位与 H / 任务结局

现有 LIO 离线工具按 armed 状态区间统计（状态约 1Hz，边界有约 1 秒量化）：

- LIO 接收年龄 P50 45.24ms、P95 73.12ms、最大 138.52ms；armed 区间 0 条超过 0.3s。
- 外源位姿接收年龄 P95 73.84ms、最大 139.62ms；armed 区间最大接收间隔约 179.87ms。
- 4 条 LIO header 未在外源位姿中匹配，均在未 armed 时；这只是时间戳集合匹配，不等于链路丢包。
- armed 区间 MAVROS/LIO 未触发工具的位姿跳变阈值（距离 >3m/s×dt+0.25m）。不能据此声称不存在所有 EKF 重置或具备厘米级定位精度。
- 初始化约第 3.38 秒，未 armed 的 MAVROS/navigation Z 曾跳变约 0.801m；按启动状态单列，不混入飞行故障。
- 主要任务区间 navigation odom 接收年龄 P95 72.72ms、最大 109.69ms；LIO 自报 output_age P95 74.955ms、最大 131.908ms，lidar_queue 最大为 0。未观察到本轮任务活动期间持续扫描积压。

H 模式从未进入：align_mode 只从 disabled 切到 drop_circle，无 landing 模式；任务没有 LAND 指令、H 捕获/下降或 H 交接。最后飞控落地解除武装，而任务仍保留首投槽位预留；这支持“首投未完成后结束飞行”，不支持“完整任务成功”或“H 检测失败”。没有 ULog 或外部位置、落点真值，无法分析 EKF 融合内部原因或声明 H/投放精度。

## 验证、遗留与下一步

日期：2026-10-10。改动范围：新 bag 独立 analysis 目录与本 verification 目录。生成四路回放、重点事件播放器、补偿重建 CSV、时序图、定位统计和本报告。四视频全片解码通过，播放器本地引用及摘要检查见 `artifact_validation.json`；原 bag 大小、mtime、ctime 对照保存在同文件。

只读抽查 158.0/179.9/191.3/203.5/212.4/223.2 秒 dashboard，冻结前完整靶面与下降后局部画面可见。控制残差只是相对于冻结估计点的误差，不是目标地图精度或物理落点精度。

遗留：未录控制反馈与 ReleaseResult；deployment metadata 未更新完整二进制 revision；没有同条件优化前后配对数据、飞控 ULog 或物理真值。下一步应先基于现有 `compensation_observed.csv`、在线 waiting 日志及现场配置，核对高度下限附近跟踪余量、XY 收敛与运动指令，再安排用户明确授权的新数据采集。分析过程不修改飞行阈值，不自动重跑飞行或仿真。
