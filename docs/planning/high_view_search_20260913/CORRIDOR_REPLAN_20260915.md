# 走廊第二门停滞与近墙投递余量：观测与修复方案（设计稿）

2026-09-15 · **本文件仅设计**：未修改任何源码/launch/配置/模型，未启动仿真，未连接板端。用户本次选择"先出方案不动代码"，并已授权后续配套 **1–2 轮定向诊断跑**（留待实施观测补丁时使用）。

依据：[新场景 36–40 报告](../../verification/high_fast_new_seeds_20260915/REPORT.md)、[补充分析（降落/碰撞/重规划）](../../verification/high_fast_new_seeds_20260915/ANALYSIS_landing_collision_replan.md)、[走廊复盘](../../verification/high_fallback_repair_20260915/CORRIDOR_REVIEW.md)、[兜底设计](FALLBACK_20260915.md)、[修复状态](../../verification/high_fallback_repair_20260915/STATUS.md)。

## 1. 目标与本次审批范围

要解决两件互相独立的事：

1. **走廊第二门（投后路线第 8 段，Wall_22 x=+1.6）停滞**：一旦发生即跑满 90 s 段安全窗、终点前 0.54–0.67 m 零位移、`safety_motion_timed_out`；本批 1/3、归档 d463613 批 2/2 到达该段的 run 都发生，seed37/39 正常通过（15.9/40.0 s）。
2. **近墙投递碰撞**：本批 seed38/seed40 两次 `actual_collision` 均为第 3 投（red_cross）保护圈接触东侧边界墙 Wall_11；靶心距墙面仅 0.365/0.390 m，保护圈半宽 0.275 m，对准超调 0.08–0.11 m 即触墙。

本次建议批准：**先做只读观测补丁，再据观测结果改重规划**；近墙余量走离线校验先行。批准本设计不等于批准实机放飞、替换正赛代码，也不等于取消每轮仿真的显式授权要求。

## 2. 当前证据与主假设

已确认（代码级 + 实测）：

- 规划端 `kino_replan_fsm.cpp:260-267` 三个重规划条件：①机体到轨迹**投影点**误差 >0.45 m；②`execution_time_`（**投影进度**）≥ duration−0.03；③`partial`+已位移+间隔 0.75 s。
- 执行端 `traj_server.cpp:308-318`：前视点与实测偏差 > `target_dist+0.05`（走廊 0.20 m）时锁 `tracking_hold_active_` 并冻结指令，**只有新轨迹才解锁**；`trajectory_progress.h` 的 `projectProgress` 只向前搜索且进度单调。
- seed36 实测：203.62 s 规划器已给出**完整抵达目标 (2.3,8.35) 的轨迹**（9.22 s）；该段 90 s 内只有 ACCEPTED/PLANNING/TRAJECTORY_READY 三条事件，**无 TRAJECTORY_FINISHED、无 REPLANNING、无失败尝试**，也无 kinodynamic 失败；指令链自 203.81 s 冻结在 (0.933, 8.395, 0.230)；机体与轨迹偏差仅 0.019–0.152 m → 三条件结构性都不可满足 → 死锁。

本次新增判别测试（只读既有数据）：

| 检验 | 结果 | 指向 |
|---|---|---|
| 冻结指令到轨迹折线距离 | **1.6 cm** | 指令是轨迹求值点 |
| 冻结瞬间真值/LIO/FC 是否等于该指令 | 否（最近 3.3–4.5 cm，且在冻结后约 1 s 才收敛） | 不支持"保持锁把指令设成里程计位置" |
| 判别结论 | **主假设 = traj_server 前视/投影进度停止推进**（指令停在轨迹早期点，机体追上后一起静止） | 修法偏向"推进逻辑"，而非"保持恢复" |

**仍未决**：`tracking_hold_active_`、`best_t`、`execution_time_`、当时的 `target_dist`、指令速度均未记录（traj_server 目前只有 `[Traj server]: ready.` 一条日志；`planner_setpoint.csv`/`mavros_setpoint.csv` 只有 t/xyz/四元数）。因此第 3 节是必要条件，不能跳过直接改 FSM。

## 3. 方案一：执行链只读观测补丁（第一步，行为不变）

### 3.1 记录字段

| 字段 | 来源 | 用途 |
|---|---|---|
| `traj_id` | `traj_id_` | 关联规划事件与轨迹段 |
| `hold_active` | `tracking_hold_active_` | 直接判定是否触发保持锁 |
| `best_t` / `exec_time` / `traj_duration` | `cmdCallback` 内部量 | 判定前视/投影是否推进、是否接近耗尽 |
| `target_dist` | 当拍生效值 | 确认走廊 0.15 与 hold 阈值 0.20 的实际取值 |
| `dev_lookahead` = \|前视点−odom\| | 计算 | 触发判据的实测值 |
| `dev_projection` = \|投影点−odom\| | 计算 | FSM 条件①的实测值（应等于 FSM 内部量） |
| `cmd_speed` | \|vel\|（保持时置零） | **区分"保持锁(零速)"与"前视冻结(有速)"** |
| `fsm_tracking_error` / `fsm_partial` / `fsm_exhausted` | FSM 每拍计算 | 证明三条件为何不成立 |

### 3.2 接口形态

- 新增独立话题（建议 `~diagnostics`，消息只含 double/uint 字段），**10 Hz**，与 `/position_cmd` 完全解耦；
- 由既有 recorder 按现有 CSV 模式落盘为 `traj_progress.csv`（与 `planner_setpoint.csv` 同目录同规范）；
- `traj_server` 新增私有参数 `~diagnostics_enable`，**默认 false**；仅研究 launch 打开；
- FSM 侧同样默认关闭，避免影响其他运行。

### 3.3 改动清单（实施时）

| 文件 | 改动 |
|---|---|
| `plan_manage/src/traj_server.cpp` | 采集上表字段并按需发布；不加控制分支、不改任何阈值 |
| `plan_manage/src/kino_replan_fsm.cpp` | 发布本拍 `tracking_error/partial/exhausted` 与触发结果 |
| `plan_manage/msg/`（如需新消息） | 仅新增，不改既有消息字段 |
| `uav_mission/scripts/`（recorder） | 订阅并写 `traj_progress.csv` |
| 研究 launch | 打开 `diagnostics_enable`（默认入口保持关闭） |
| `test/` | 新增字段序列化/采集单测；既有 181/183 项回归必须保持通过 |

### 3.4 验收与退出条件

1. 离线：单测通过 + 两个 Catkin 工作区构建通过 + launch 静态展开确认默认关闭；
2. 定向跑（已授权 1–2 轮）：在易停滞场景抓到停滞瞬间的完整内部状态；
3. 退出条件：能明确回答"是保持锁还是前视/进度冻结"；否则不进入方案二。

### 3.5 风险与规避

- 观测本身可能扰动时序（本停滞对时序敏感）：频率限制 10 Hz、只发 double、默认关闭、必要时先跑一轮不开启的对照；
- 不改变 `/position_cmd` 的求解路径与参数读法；任何"顺手修"都不放进本补丁。

## 4. 方案二：重规划修复（依据观测结果二选一，不预先混做）

- **情形 A（确认前视/投影进度冻结）**：修推进条件本身——复核 `projectProgress` 的 1.0 s 前向窗与 `boundedLookahead` 的 `arc/偏差` 双重中断在"机体贴近轨迹但不再前进"时的行为；目标是让进度在机体静止时仍能给出可执行推进量，或显式上报"无进展"。**不放松 hold 判据来掩盖问题。**
- **情形 B（确认保持锁）**：保持锁增加可恢复条件（例如：偏差回到阈值内、或收到新轨迹、或有界超时），并把保持状态上报给 FSM。
- **共通项**：定义"**执行端无进展 N 秒 → 请求有界重规划**"，N 为待验证初值（建议 3–5 s，与 FALLBACK 中 8 s 换点阈值分开标定）；补全 `TRAJECTORY_FINISHED` 语义在"进度未尽但已无推进"时的表达；重规划后仍要求新轨迹在下发前退役旧动作。
- **明确不做**：延长 90 s 段等待；放宽走廊 0.7 m 工程阈值；回退 a12750b 下降解耦；无限扩大局部目标吸附半径。
- **验收设计**：先用观测跑确认复现场景与复现率（候选：新场景 seed36=LL；归档 seed31/34 场景），再在新代码上跑 2–3 轮同场景 + 1 轮已知通过场景（seed37/39 之一）防退化；通过判据 = 第 8 段出现推进/重规划且不再 90 s 空等、整场不新增碰撞与限高违规。

## 5. 方案三：近墙投递余量（离线先行，不需仿真）

1. **判据是预算方程而非单一距离**：`净空 ≥ 保护圈半宽(偏航) + 提示/目标点外偏 + 位置超调 + 余量`。实测（两次撞击，偏航均 0.2°）：半宽 0.275 m、提示外偏 0.042/0.065 m、超调 0.069/0.021 m，剩余余量只剩约 4 mm。偏航显著抬高第一项（0.275@0° → 0.352@20° → 0.376@30° → 0.389@45°）。建议分两层：**硬底线**（场景合法性，例如净空 < 0.30 m 直接拒绝）+ **软阈值**（净空 < 保护圈半宽 + 0.21 m 提示不确定度 + 0.10 m 跟踪误差时标记"需收紧对准"）。阈值参数化、需更多样本校准，不硬编码。
2. **飞行侧优先于场景侧**：本例外偏主要来自提示点本身偏向墙侧，因此**在近墙目标上限制悬停/对准点的前向余量**（不追着提示点越过安全线）比单纯拒绝场景更直接；两者互补。
3. **本仓实施点**：`simulation_tools/tools/export_r2026_scene.py` 与 `docs/verification/*/validate_scenes.py` 增加净空预算校验与逐靶净空表；纯离线、可单元测试。
4. **跨组**：运行时布设规则 `random_field_policy.footprint_clear_boxes` 属导航组（`liftrace-controlwork`），本仓只出提案与证据，不直接改其语义；按 AGENTS 3.3 记录 branch/commit 后再同步。
5. **兼容性**：已归档的 31–40 场景**不重算、不改写**，只输出净空预算表与风险标记（含偏航项）；新场景才施加硬底线并标注约束版本号，避免与历史批次混比。

## 6. 排期与产物（每阶段独立授权）

| 阶段 | 内容 | 产物 | 退出条件 |
|---|---|---|---|
| 1 | 只读观测补丁 + 单测 | 代码 + `traj_progress.csv` 样例 + 变更记录 | 构建/回归通过、默认关闭 |
| 2 | 定向诊断跑 1–2 轮（已授权） | run 目录 + 内部状态曲线 | 能判定情形 A/B |
| 3 | 依情形实施重规划修复 | 代码 + 离线单测 | 离线测试通过 |
| 4 | 修复验证跑 2–3 轮 + 1 轮防退化 | 报告与图表 | 第 8 段不再空等，无新增碰撞/限高 |
| 5 | 近墙净空离线校验（可与 1–2 并行） | 校验脚本 + 净空表 + 跨组提案 | 存量场景只标记、新场景受约束 |

## 7. 风险与边界

- 本停滞对时序敏感，单轮结果不构成稳定性结论；复现率需在阶段 2 用授权轮次确认。
- 保护圈尺寸取自本次运行机型 SDF（0.55×0.55×0.4 m）；换机型需重算余量阈值。
- 情形 A/B 的修法不同，**不先做观测就改 FSM 属于猜测**；本设计据此把观测列为第一步。
- 所有改动仅限研究分支，不替换机载部署；实机验证、板端资源与规则合规不在本方案范围。

## 8. 复现与数据入口

- 本次证据：`logs/high_fast_new_seed3{6,7,8,9}_20260915_*`、`logs/high_fast_new_seed40_20260915_190259`；
- 分析脚本：`docs/verification/high_fast_new_seeds_20260915/{corridor.py,analyze.py}`，产物 `corridor_evidence.json`、`corridor_history.png`；
- 离线反例：`docs/verification/high_fallback_repair_20260915/corridor_hold_reproducer.cpp`（g++ 离线编译，无 ROS）；
- 场景生成与校验：`simulation_tools/tools/export_r2026_scene.py`、`docs/verification/high_fast_new_seeds_20260915/validate_scenes.py`。