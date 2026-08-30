# NAV VCL06 Fenced Planner Bridge

## 当前交付边界

本分支把导航任务层的 `NavigationDecision` 转成可审计的 planner motion intent，并把
`plan_manage/PlannerStatus + nav_msgs/Odometry` 归约成同一 executor 的
`NavigationResult`。它不包含 Servo、PWM、解锁、释放或降落实现，也不把
`TRAJECTORY_FINISHED` 直接当作到达。

默认 launch 设置：

- `execution_enabled=false`；
- planner goal 隔离输出为 `/navigation/fastplanner_goal`；
- `allow_live_goal_output=false`；

因此默认启动不会发布规划目标。只有同时显式设置 `execution_enabled=true` 与
`allow_live_goal_output=true`，并把目标话题配置为 `/fastplanner/goal` 时才创建 live publisher；
隔离话题仍可用于合同级联调。启用 live 输出只表示允许发布规划目标，不代表
cancel/hold、目标事务或降落链已经具备。

## Fencing 与到达证据

- 冷启动接受任意仍在 deadline 内的当前 decision，支持 manager latched 决策后的晚启动；
  同 mission 的旧序号只作为 stale 忽略，不要求 bridge 观察到中间所有序号。
- raw decision 的 `decision_seq` 原样复制到 planner goal `header.seq`；不生成第二套目标序号。
- planner goal 成功发布后，才发布同一 executor 的非终态 `ACCEPTED/DISPATCH`，使 Mission
  Manager 尽早冻结唯一 `executor_id`；目标阶段后续结果也必须由同一协调器代理。
- decision receipt 与 source stamp 均采用排他 deadline；等于 deadline 即过期。
- 旧 decision、未知旧 goal telemetry、planner event 回退和幂等重复均无副作用；仅当前
  decision/goal 的身份或生命周期冲突才 fail-closed。
- 只有当前目标先后见到 `ACCEPTED`、`TRAJECTORY_READY`、`TRAJECTORY_FINISHED`，且同
  mission frame 的新鲜 odom 对 effective goal 同时满足 3D 距离、速度和连续驻留，才产生
  motion success。
- SEARCH、RESUME、RETURN_HOME 到达产生终态 `SUCCEEDED/PLANNER`。
- APPROACH 到达只产生非终态 `PROGRESS/PLANNER` 和 target-transaction intent；绝不伪造
  payload commit 或目标成功。
- LAND 只产生 landing intent；ABORT/HOLD 只产生 safety-stop intent，不发布替代 planner
  goal。

## 尚未解除的 live Gate

Fast-Planner 当前没有通用的带 ACK hold/stop 接口。`planning/replan` 只截短轨迹时长，轨迹
服务器仍可能继续发布 setpoint。因此在补齐 planner hold/stop ACK、单一 target-stage
协调器和 landing executor 前，本分支不能表述为完整可飞闭环，也不得 ACK ABORT。
live goal 开关只供明确配置的规划运动联调使用。

这也是默认关闭 live 输出的原因，而不是启动便利性选项。

## 运行顺序与重启限制

- raw decision 是 latched；bridge 晚启动可接收任意有效当前 decision。bridge 只保留当前
  decision、当前/待取消 goal 与最近 planner event，不维护无界任务历史。
- 到达证据的 odom `header.frame_id` 必须实际等于 `camera_init`。MAVROS/仿真里程计若不是
  该 frame，必须先接显式坐标转换节点，不能只 remap 话题名。
- `PlannerStatus.event_seq` 没有 planner 进程实例 ID；回退或未知旧 goal telemetry 会被忽略，
  但当前 goal 的同序号内容冲突仍会闭锁当前执行。

## 导航—视觉对齐边界

视觉 `TargetCandidateArray` 与导航准入所需的稳定 ID、`first_seen/last_seen`、地图点、
`camera_init`、TF 年龄和质量字段已经逐项对齐；视觉 stable ID 从 0 开始，bridge 通过
`has_target` 区分“目标 0”和“无目标”。感知模型可以保留额外类别，但正式 `r2026` 决策
只允许 `tent/pillbox/bridge/panzer/red_cross`，不含 `tank`。

末端接口尚未对齐：当前 `ReleaseEvidence` 缺少冻结实例的 `first_seen` 与精确 observation
stamp，`drop_aligner` 也未消费导航选定的完整目标键。因此它不能直接转成 payload commit。
后续单一 target-transaction coordinator 必须以 `NavigationDecision` 为唯一目标上下文，
代理 CAPTURE/ALIGNMENT/RELEASE/RECOVERY，并沿用本 bridge 的同一 `executor_id` 与全局递增
`event_seq`。
