# 高位先搜与速度优化完整对照

## 2026-09-14补充：固定布局四方案效率对照已完成

[完整四方案报告](speed_comparison/REPORT.md) · [36张图表浏览](speed_comparison/index.html) · [逐轮指标](speed_comparison/summary.json)。原四轮记录继续保留于下文。

| 方案 | seed32完整耗时 | seed34完整耗时 |
|---|---:|---:|
| 提速前覆盖 | 529.224s | FAIL（降落碰墙） |
| 提速覆盖 | 348.954s | FAIL（首投恢复碰墙） |
| 提速前高位先搜 | 350.240s | 351.248s |
| 提速高位先搜 | 225.762s | 225.506s |

按飞行Gate，最快组合为提速高位先搜：两轮37/37 PASS、0碰撞，平均225.634s；相对低速高位平均350.744s缩短35.67%。seed32相对传统覆盖缩短57.34%；seed34传统覆盖未完成，不给虚构的成功完赛节时。

**最终合规选择仍需区分**：高位中心航迹没有进入障碍投影，但seed34旋转保守包络存在约15.1cm投影交叠指示。它不是实际碰撞深度，也不能自动套用用户已接受的seed32约1mm包络容差。最快候选尚不等于禁止整机越树条件下已最终验收；未替换正赛机载代码。详见[投影图与口径](speed_comparison/projection_review.png)。

全随机计划按用户选择暂不执行。本次同seed的四方案世界和靶位已核对一致；所有失败保留，不用重跑替代。

## 凌晨原冻结四轮记录

本批显示高位优先策略有明确提效价值：两布局三投均更快，高位组两轮整场均通过。基线存在未完成轮次，该布局不能计算成功整场节时；碰撞差异不能由单次对照归因于策略。

核心问题是先高位找齐三目标、再低位避障投递，是否减少整场时间。这里评估的是WSL笔记本上的真实PX4/Gazebo仿真链，不是实机结果。每种策略每布局仅一轮，两布局不能代表普遍成功率或统计显著性。

飞行源码冻结为 `5667bed`，四轮一致；159项相关离线测试和两个Catkin工作区构建通过。实际布局、共同任务/Gate参数和相机信息配对一致，见[报告一致性验证](validation.json)。

## 完整结果

| seed | 策略 | Gate | 投递槽 | 碰撞 | 首投/s | 第三投/s | 整场验收/s | 离地至落地/s | XY航程/m |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| 32 | baseline | PASS | 3/3 | 0 | 148.24 | 322.13 | 529.22 | 532.20 | 114.83 |
| 32 | strategy | PASS | 3/3 | 0 | 93.97 | 142.35 | 350.24 | 354.40 | 56.70 |
| 34 | baseline | FAIL | 3/3 | 1 | 10.95 | 260.81 | 未完成 | 未完成 | 95.45 |
| 34 | strategy | PASS | 3/3 | 0 | 94.30 | 148.24 | 351.25 | 356.59 | 58.83 |

baseline 为原低位覆盖，保留高权重中断、投后恢复，并在三投完成后立即转入走廊；strategy 为高位闭环观测、三目标齐备提前中断、就近下降和低位按序复访。两组共用模型、地图、低位释放门槛、速度跟踪参数、随机门导航及原整场Gate。

第三投和整场验收均从首条任务指令计时；整场验收包含最终降落事务及原Gate要求。另列离地至落地时长，落地到解除武装的等待见各轮metrics.json。失败轮的航程为已观察航程，不得拿截断航程当节省量。

## 同图差值

正数表示高位策略节省；任一未完成则不计算该项节时。
- seed32：第三投节省 179.78 s（55.81%）；整场节省 178.98 s（33.82%）。
  到第三投XY航程减少 57.90 m（61.52%）。
  完整XY航程减少 58.13 m（50.63%）。
- seed34：第三投节省 112.57 s（43.16%）；整场节时不可计算（存在未完成轮次）。
  到第三投XY航程减少 36.00 m（47.97%）。
  该组存在失败轮，不比较完整航程。

## 提效来自哪里

| seed | 基线覆盖搜索/s | 高位观察及升降/s | 高位组低位复访与等待/s |
|---|---:|---:|---:|
| 32 | 292.96 | 81.09 | 39.52 |
| 34 | 234.45 | 82.30 | 46.11 |

直接可见的主要变化是低位覆盖航程减少：高位观察和升降并非免费，但其成本小于本批基线继续覆盖搜索的开销。三投前航程也在两组中明显减少，说明收益不仅是计时口径差异。阶段边界来自采样状态和事务事件，释放ACK后至恢复完成计入RECOVERY。

首投并不总是更快：seed34基线先投了起点附近的bridge，高位组先等三目标齐备再开始投递；后者最终三投更早。应按完整三槽任务评价。

本批没有单独消融高位门槛和就近下降的贡献。六个实际高位线索的不确定度仍约0.20m；不能把全部收益归功于放宽到0.45m。低位新鲜重捕验证保持通过，线索只承担导航引导。

## seed34基线降落失败复盘

首发失败为机体保护圈与东侧边界墙Wall_11接触，场景时刻471.985s（任务约460.352s）。此前三次释放、三次恢复、9个投后导航点、走廊入口和两道门均已完成，H标志定位也已有效。其余落地/解除武装/任务完成相关未通过项是碰撞中止后的结果，不能解释成9个独立故障。

接近支撑高度后发生反弹。首次墙接触时，插值MAVROS相对真值的平面误差约0.447m、高度误差约0.859m；LIO对应误差约0.037m和0.016m。

这些记录将问题定位到近地阶段的估计分离与反弹现象，仍需PX4 ULog和接地动力学复核因果。该失败及原始飞控参数保留，尚未通过参数消融或重复试验确定因果。

![降落失败局部复盘](seed34_landing_failure.png)

[插值指标](landing_failure_metrics.json) · [原始时刻摘录与接触对象](seed34_landing_diagnostics.json)

![配对耗时和航程](comparison.png)

灰色叉号为失败停止时刻，不是完成时间。

![逐次投递时间](delivery_milestones.png)

![同地图完整航迹](paired_paths.png)

## 每轮航线、航迹、高度、速度与阶段图

### seed32 / baseline

原始run：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/high_view_full_frozen_baseline_seed32_20260914_015313`。Gate：PASS，原因：`all_checks_passed`。
投递顺序：bridge → panzer → red_cross。移动段实际水平速度中位数 0.28 m/s。
[完整指标与阶段分解](32_baseline/metrics.json) · [Gate](32_baseline/gate_status.json)

![完整飞行图](32_baseline/flight_charts.png)

![分阶段航迹](32_baseline/route_stages.png)

![阶段耗时和目标事务](32_baseline/phase_target_timeline.png)

### seed32 / strategy

原始run：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/high_view_full_local_strategy_seed32_20260914_012723`。Gate：PASS，原因：`all_checks_passed`。
投递顺序：bridge → panzer → red_cross。移动段实际水平速度中位数 0.13 m/s。
[完整指标与阶段分解](32_strategy/metrics.json) · [Gate](32_strategy/gate_status.json)

![完整飞行图](32_strategy/flight_charts.png)

![分阶段航迹](32_strategy/route_stages.png)

![阶段耗时和目标事务](32_strategy/phase_target_timeline.png)
下降提案：`{"candidates": 1, "classes": ["bridge", "panzer", "red_cross"], "cost_m": 20.519848480983512, "from_xy": [-2.7568907737731934, 4.899586200714111], "kind": "CURRENT_COLUMN", "map_stamp": 87.202, "scope": "SENSED_OCCUPANCY_PROPOSAL_REQUIRES_3D_PLANNER", "time": 88.3, "xy": [-2.7568907737731934, 4.899586200714111]}`。
高位坐标误差、低位新鲜重捕误差和线索年龄见指标文件的 target_hint_evaluation；这些不是实际快递落点得分。

### seed34 / baseline

原始run：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/high_view_full_frozen_baseline_seed34_20260914_025732`。Gate：FAIL，原因：`actual_collision`。
投递顺序：bridge → panzer → red_cross。移动段实际水平速度中位数 0.25 m/s。
[完整指标与阶段分解](34_baseline/metrics.json) · [Gate](34_baseline/gate_status.json)

![完整飞行图](34_baseline/flight_charts.png)

![分阶段航迹](34_baseline/route_stages.png)

![阶段耗时和目标事务](34_baseline/phase_target_timeline.png)
未通过项：contact_ready_zero, contract_errors_zero, final_landed_on_ground, final_vehicle_disarmed, land_success, manager_complete, mission_ros_within_limit, return_before_land, zero_collisions。

### seed34 / strategy

原始run：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/high_view_full_local_strategy_seed34_20260914_023010`。Gate：PASS，原因：`all_checks_passed`。
投递顺序：panzer → bridge → red_cross。移动段实际水平速度中位数 0.13 m/s。
[完整指标与阶段分解](34_strategy/metrics.json) · [Gate](34_strategy/gate_status.json)

![完整飞行图](34_strategy/flight_charts.png)

![分阶段航迹](34_strategy/route_stages.png)

![阶段耗时和目标事务](34_strategy/phase_target_timeline.png)
下降提案：`{"candidates": 1, "classes": ["panzer", "bridge", "red_cross"], "cost_m": 21.72167747730646, "from_xy": [-2.265604019165039, 5.699643611907959], "kind": "CURRENT_COLUMN", "map_stamp": 89.934, "scope": "SENSED_OCCUPANCY_PROPOSAL_REQUIRES_3D_PLANNER", "time": 90.2, "xy": [-2.265604019165039, 5.699643611907959]}`。
高位坐标误差、低位新鲜重捕误差和线索年龄见指标文件的 target_hint_evaluation；这些不是实际快递落点得分。

## 解释边界与后续

- 高位线索门槛与低位投递门槛分开：高位三次观测最短跨度0.2s、不确定度上限0.45m；低位仍须原连续确认、新鲜坐标和释放许可。
- 三点顺序使用感知占据栅格距离，包含最后目标至走廊入口；实际三维轨迹仍由Fast-Planner避障。栅格最短顺序不等于连续空间全局最短，也未按不同阶段实际速度优化总时间。
- 规划上限1m/s不是实际巡航速度。当前位置跟踪前视约0.4m，末段约0.15m；两组一致。提速后需重新比较识别和任务收益，不能直接推广本批结果。
- 图中树圆为布局示意，虚线为导航点顺序而非可飞直线；碰撞结论来自仿真接触监测。真值只用于评测，没有输入高位选靶或排序。
- 开发过程另保留随机门配置中止轮（excluded_runs.json）和返起飞柱旧版二投后失败轮（development_runs.json、development/）。旧版不是成功轮，不与冻结版混算；其返起飞柱阶段耗时21.657s。
- 后续先依据本批第三投与整场结果判断策略价值；独立对照2025跟踪链、分级提速，再检查避障、识别和净收益。研究分支不替换正赛部署。

建议继续推进这一研究方向；先独立复核近地失败并对照2025跟踪链，再在共同更高实际速度下做配对复测。当前结果支持提效潜力，尚不足以替换正赛部署。

[复现说明](REPRODUCE.md) · [运行索引](runs.json) · [配对差值](paired_differences.json) · [速度核对](speed_review.json) · [报告验证](validation.json)
