# 来源、复核范围与计量说明

2026-09-19。本次结论来自本地源码、提交和既有实验，不借用外部通用无人机参数代替本工程。没有取得2025正式赛场逐阶段飞行日志，因此旧版只报告代码/配置事实与缺口。

## 1. 版本边界

| 对象 | 本次读取的版本/位置 | 使用方式 |
|---|---|---|
| 研究分支 | `liftrace-visionwork` / `feat/high-view-search-research@778121a100cfe549fdb797ac6d67299aee8d2395` | 本地与远端分支HEAD核对一致；本次文档改动以它为基点 |
| 新五场运行 | `92bd3eed45191a69fbbaa4356e9afc7292ce9f76`，36–40 | 原run/Gate/位姿和分析数据保留 |
| 下降解耦 | `a12750b`，后续保留 | 不能把暂时回退的过程条目误读为当前状态 |
| 主检出 | `/home/xhj/liftrace`，`main@2f405be` | 只读历史参考，未修改用户未跟踪文件 |
| 导航参考 | `/home/xhj/liftrace-controlwork-nav`，本地`main@a68925d` | 只读snapshot，不声明已与研究分支对齐，也不声称这是远端最新开发分支 |
| 旧机载主工作区 | `0ddc845` Git对象内的patrol_control/Fast-Planner | 含后来兼容代码，作为导入版参数旁证 |
| 2025桌面参考 | `/home/xhj/liftrace/Desktop_patrol_uav_ws-patrol_planner` | 现存原始资产，只读；没有用hash替代版本判断 |

研究分支从59c6215到778121a新增报告、实验工具和场景，`patrol_uav_ws-patrol_planner/src`、`vision_ws/src`对比无差异。该判断只针对已跟踪飞行源码，不把不同随机场景或机器负载当作相同运行状态。

## 2. 结果与机制依据

| 编号 | 资料 | 支持的结论 |
|---|---|---|
| S1 | [36–40报告](../../verification/high_fast_new_seeds_20260915/REPORT.md)、[summary](../../verification/high_fast_new_seeds_20260915/summary.json)、[metrics](../../verification/high_fast_new_seeds_20260915/metrics.json) | 2/5整场、3/5三投；复核loop_metrics函数，14是DELIVERY阶段进入次数，release与提交都是13 |
| S2 | [降落/碰撞/停滞深化](../../verification/high_fast_new_seeds_20260915/ANALYSIS_landing_collision_replan.md) | seed36完整轨迹与无续规划、38/40近墙预算、37/39落地事实 |
| S3 | [现有方案](../high_view_search_20260913/CORRIDOR_REPLAN_20260915.md) | 首次触发证据、A/B假设与观测缺口；本次补充与修正的基础 |
| S4 | [33复现](../../verification/high_fast_five_20260915/seed33_repro_20260915.md) | H阶段0.7114m与0.7m工程Gate冲突，不与墙高混淆 |
| S5 | [前轮修复报告](../../verification/high_fallback_repair_20260915/REPORT.md)、[走廊复盘](../../verification/high_fallback_repair_20260915/CORRIDOR_REVIEW.md) | 下降解耦保留、31/34同类停滞、离线反例并非现场唯一根因 |
| S6 | [本次重算摘要](measurements.json) | 相对中断时间、成功轮分阶段速度/耗时、旧配置摘要 |
| S7 | [旧速度诊断](../../verification/high_view_full_20260914/speed_review.json) | 0.4m跟随距离下实际约0.32–0.34m/s；旧样本移动阈值0.1m/s，不能和本次0.03阈值混成严格配对 |
| S8 | [主检出视觉组需求](//wsl.localhost/Ubuntu-20.04/home/xhj/liftrace/视觉组需求.md)（用户未跟踪文件，仅阅读） | 历史需求；原B100/C25/D11已向liftrace-sim交付，不重新当作从零需求 |
| S9 | [轻量仓上游合并](https://github.com/sakelier/liftrace-sim/commit/99cfa33cc448e62d4fcfdeace0669b553c91d888)、本地fork69e66c4的README/config/vision_performance/mission_sim | PR #1已接收原视觉包；后续R64工具尚在fork；新改造详情见LIFTRACE_SIM_ASSESSMENT |

本次原始读取路径为研究工作树下：

```text
logs/high_fast_new_seed36_20260915_180729
logs/high_fast_new_seed37_20260915_182936
logs/high_fast_new_seed38_20260915_184340
logs/high_fast_new_seed39_20260915_184841
logs/high_fast_new_seed40_20260915_190259
```

各目录的truth_pose、rosparams、key_events、gate_status和模型/场景记录是既有产物。本次未新建run目录，不重算或替换场景输入。

## 3. 当前代码定位

| 文件 | 关键位置（分析基点） | 用途 |
|---|---|---|
| [trajectory_progress.h](../../../patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/include/plan_manage/trajectory_progress.h) | projectProgress与boundedLookahead，约10–45行 | 单调局部投影、1.0s前向窗、0.02s采样及弧长/球限制 |
| [traj_server.cpp](../../../patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp) | 新轨迹复位约183–188；cmdCallback约283–319 | 保持锁、原始与最终命令的区分 |
| [kino_replan_fsm.cpp](../../../patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp) | 参数约66–68；EXEC_TRAJ约242–270 | 误差、耗尽、部分轨迹重规划条件 |
| [planner_manager.cpp](../../../patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/planner_manager.cpp) | 约221–231 | setPhysicalLimits与统一knot时间缩放，识别真正生效的上限 |
| [FollowingSpeed](../../../patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/execution_speed.py) | 类默认值与select | 阶段跟随距离与最后一投后的转场 |
| [fast_comparison.launch](../../../vision_ws/src/uav_high_view/launch/fast_comparison.launch) | 7–10、44–53附近 | 1.0m跟随、1.2m/s规划、1.2m命令准入 |
| [high_view_full.py](../../../patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py) | _all_top、_retreat、_finish_route、_start_fallback | 持久线索、下降解耦、高位无进展与低位兜底范围 |
| [低廊控制配置](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_full_low_corridor_control.yaml)、[桥接配置](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_planner_bridge.yaml) | 高度、稳定与到点参数 | 局部坐标与真值AGL需区分；以运行时覆盖后的值为准 |
| [36–40实际runtime样例](../../verification/high_fast_new_seeds_20260915/seed_37/experimental_runtime.yaml) | post_delivery_parameter_stages | 转场后才缩短lead、到点容差及走廊限高事务 |

行号只帮助定位；文档不会通过这些链接修改源文件。

## 4. 旧链调查入口

桌面参考路径均位于 `/home/xhj/liftrace/Desktop_patrol_uav_ws-patrol_planner/src/`：

- `patrol_control/src/node/patrol_control_node.cpp`：头部日期2025-05-16。
- `patrol_control/launch/patrol_control_real.launch`：加载patrol_real.yaml并include planner real入口。
- `patrol_control/config/patrol_real.yaml:18`：1.0m位置球；渐进降高、阈值与开关。
- `patrol_control/src/patrol_control.cpp:64–79, 350–540`：20Hz、位置接口、各模式及统一位置限幅；约760–880为对准退出，约1040–1064为发布循环。
- `patrol_control/src/alignment_control_converter.cpp:9–16, 145–183`：0.5m单次修正、5s高度插值及0.8/0.4目标；不能把声明值当实飞速度。
- `Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_real.launch:50–51,85`：3.0m/s、2.0m/s²、0.4m前视。
- `Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp:275–312`：全曲线最晚可用采样、终点fallback。
- `Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp:190–240`：时间推进与重规划起始速度0.1归一处理。

导入版旁证通过只读 `git show 0ddc845:<path>` 读取，未切换工作树：

- [patrol_real.yaml](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/0ddc845/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_real.yaml)：1.0m位置球、align_height1.2、land_height0.2、预设航路与停留。
- [patrol_planner_real.launch](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/0ddc845/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_real.launch)：3.0/2.0及0.6m前视。
- [patrol_control.cpp](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/0ddc845/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp)：独立align_height控制、0.10m投递与恢复、已有后来兼容代码，故不把整份快照称纯2025。

## 5. 统计口径与限制

本次相对中断时刻取高位事件 `SURVEY_INTERRUPTED_TOP3.time - start_ros_s`。XY速度由相邻真值点差分得到，Z单独计算；0<dt≤0.5s，移动阈值0.03m/s。阶段来自既有sampled_phase_changes（存在约一个采样间隔的边界误差）；移动P50不含静止样本，不能用于掩盖停滞。部分阶段样本很少，不作稳定分布结论。

测量文件保留每轮来源与所有五轮，包括FAIL。图中只用37/39说明已成功任务的时间构成，不把它们与其他seed相减声称改进。未提供2025各阶段实测速度、板端并发FPS或新代码性能结论。
