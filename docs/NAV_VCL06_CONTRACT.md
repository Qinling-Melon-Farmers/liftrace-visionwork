# NAV VCL06 导航侧合同与比赛 profile

## 1. 适用范围

本文冻结导航权威仓在 VCL06 联调中拥有的消息和比赛类别 profile。导航权威仓为
`https://github.com/sakelier/liftrace-controlwork.git`，本次基线是 `a68925d`；现有
任务管理器逻辑来自 `5144aa8`。

本阶段只新增合同、配置和测试，不改变 `target_search_manager_py.py` 的运行行为。
持久候选队列、结果 reducer、planner 回流和视觉集成适配由后续独立 PR 实现。

## 2. 话题和发布者所有权

- `/navigation/mission_command_raw`：导航 manager 唯一发布，类型
  `uav_mission/NavigationDecision`。
- `/navigation/mission_result`：执行桥唯一发布，类型
  `uav_mission/NavigationResult`。
- 正式联调时导航 manager 不直接发布 `/fastplanner/goal`；视觉/集成仓的 adapter
  保持该话题的唯一发布权。
- 导航单仓测试可以启用独立 goal bridge。它必须把 `decision_seq` 原样写入
  `PoseStamped.header.seq`，因此序号类型固定为 `uint32`。

## 3. NavigationDecision v1

`schema_version` 必须为消息常量 `SCHEMA_VERSION=1`。`header.stamp` 是决策产生时间，
`deadline` 是执行租约的 ROS 绝对时刻；执行桥不得在该时刻后新进入下一执行阶段，超时后
只能上报终态/不可逆 ACK 并停止继续执行。`mission_id + decision_seq` 在一次任务内唯一。
命令编号固定为：

| 命令 | 编号 |
|---|---:|
| SEARCH | 0 |
| APPROACH | 1 |
| ALIGN | 2 |
| RESUME | 3 |
| RETURN_HOME | 4 |
| LAND | 5 |
| HOLD | 6 |
| ABORT | 7 |

字段约束：

- `class_profile` 标识产生决策时使用的 profile。
- `deadline > header.stamp`；执行桥必须按 ROS time 检查租约。晚到但真实的 guarded release
  ACK 仍须原样回报，manager 负责把不确定槽位隔离，禁止因消息延迟复用 payload。
- `has_goal=false` 时接收方不得读取 `goal`；`has_target=false` 时不得用 `target_id`
  判断目标是否存在。`target_id=0` 是合法 ID。
- 一个目标实例由 `mission_id + target_id + target_first_seen` 唯一标识，防止视觉 reset
  后复用 ID。
- `target_observation_stamp` 是支撑本次决策的视觉观测时间。
- `attempt` 从 1 开始；`payload_slot=0` 表示当前决策不占投递槽，正式槽位为 1～3。
- `reason` 使用稳定、可检索的 ASCII reason token，并可在冒号后附加说明。

## 4. NavigationResult v1

`header.stamp` 是事件产生时间。执行桥在单次启动期间使用稳定的 `executor_id`，并令
`event_seq` 单调递增。manager 使用 `(mission_id, executor_id, event_seq)` 去重，并按
`mission_id + decision_seq + target_id + target_first_seen + attempt + payload_slot` 关联
决策。重复、旧任务或乱序结果不能二次改变队列或槽位。

状态编号固定为：

| 状态 | 编号 | 语义 |
|---|---:|---|
| ACCEPTED | 0 | 执行端已接收决策 |
| STARTED | 1 | 对应阶段已开始 |
| PROGRESS | 2 | 非终态进展或不可逆提交证据 |
| SUCCEEDED | 3 | 阶段或决策成功终止 |
| FAILED | 4 | 执行失败 |
| REJECTED | 5 | 执行端拒绝合同或前置条件 |
| CANCELLED | 6 | 决策被显式取消 |
| TIMED_OUT | 7 | 执行超时 |

阶段编号固定为 `DISPATCH=0、PLANNER=1、CAPTURE=2、ALIGNMENT=3、RELEASE=4、
RECOVERY=5、LANDING=6`。

唯一不可逆的 payload 提交事件同时满足：

```text
status=PROGRESS
stage=RELEASE
payload_committed=true
reason=release_ack_success
```

除上述组合外，`payload_committed` 必须为 false。payload 一经提交，即使后续恢复失败，
也不得回滚、重投或复用槽位。`terminal` 表示本 decision 生命周期终止，`retryable`
仅在未提交 payload 的失败终态上允许为 true。`evidence_source` 标识 planner、视觉证据、
mock ACK 或真实安全代理等结果来源。

## 5. 比赛 profile

配置文件是 `uav_mission/config/competition_profiles.yaml`，固定结构为：

```yaml
profiles:
  <profile_name>:
    classes: {<class_name>: <weight>}
    interrupt_top_k: <positive integer>
    required_deliveries: <positive integer>
```

当前正式 `r2026` 只包含：

| 靶标 | 权重 |
|---|---:|
| tent | 1.0 |
| pillbox | 1.5 |
| bridge | 2.0 |
| panzer | 2.5 |
| red_cross | 10.0 |

`interrupt_top_k=3`，因此固定的立即中断集合是 `red_cross、panzer、bridge`。
`tank` 暂不属于比赛 profile，任何 competition profile 都不记录它；正式模式收到它必须
以 `profile_excluded` 拒绝。未知 profile 必须导致任务启动失败，禁止静默回退。

## 6. 原始需求资产

导航组提供的两份原始资产按原字节复制到 `docs/handoff/original/`：

- `近期工作说明_2026-08-25.md`
- `视觉组需求.md`

它们只用于记录来源，不在原文中追加回复或决策。后续阶段回复、接口裁定和 Gate 证据必须
另建文档，避免把集成侧结论伪装成上游原始要求。

## 7. 2026-08-31 执行桥同步状态

导航 PR #6 已转为 Ready，当前 HEAD `d95377c`；planner bridge 的四个代码提交截至
`3864a7c`，已在视觉集成分支等价导入为 `98cb587/83e796b/c7c1d8f/933eb78`。本仓后续只把既有
`MissionCommand/AlignmentTargetContext/ReleaseEvidenceContext/ReleaseResult` 接入同一个
executor，并增加正式 launch 与只读 Gate。

接口集合保持不变：没有新增 msg/srv/action、legacy/deprecated schema、第二 planner-goal
adapter 或并行任务 manager。正式图中 clean manager 是唯一 raw decision 发布者，
`/navigation/planner_bridge` 是 `/fastplanner/goal` 唯一发布者。PR 正文已按 motion-only 范围、
默认关闭开关和当前验证证据更新；target transaction、LAND 与系统地图 Gate 仍明确属于集成层。

仿真地图已由真实 LiDAR→FAST-LIO→FreeDOM 链恢复：三段点云非空，
`/freedom/static_pointcloud` 为新鲜 `camera_init` frame 并持续约 10 Hz。90 秒 Gate v3 未再出现
`map_missing/map_stale`，合同错误、碰撞和越界均为 0；但仍因 `wall_timeout` FAIL，仅产生
3 个 decision/5 个 result，没有 selected/APPROACH/投递/返航/LAND。地图 readiness 成立不等于
`P_interrupt` 或完整任务 PASS，后续仍必须使用真实地图合同，不能由视觉伪造或放宽年龄阈值。
