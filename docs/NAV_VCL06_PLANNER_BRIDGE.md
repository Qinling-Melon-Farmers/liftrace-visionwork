# NAV VCL06 Fenced Planner Bridge

## 当前交付边界

本分支把导航任务层的 `NavigationDecision` 转成可审计的 planner motion intent，并把
`plan_manage/PlannerStatus + nav_msgs/Odometry` 归约成同一 executor 的
`NavigationResult`。它不包含 Servo、PWM、解锁、释放或降落实现，也不把
`TRAJECTORY_FINISHED` 直接当作到达。

默认 launch 同时设置：

- `execution_enabled=false`；
- planner goal 隔离输出为 `/navigation/fastplanner_goal`；
- `allow_live_goal_output=false`；
- kino planner 与 manual-target 两项人工确认均为 `false`。

因此默认启动不会向 `/fastplanner/goal` 发布。本版即使显式打开执行、live 输出以及 kino
planner（`planner=1`）/manual target（`flight_type=1`）确认，只要解析后的目标话题是
`/fastplanner/goal`，仍会硬关闭输出且不创建该 publisher。当前只允许向隔离话题
`/navigation/fastplanner_goal` 做合同级联调；本轮没有启动该输出。

## Fencing 与到达证据

- 冷启动只接受 `decision_seq=1` 的 SEARCH；同任务后续序号必须连续。
- raw decision 的 `decision_seq` 原样复制到 planner goal `header.seq`；不生成第二套目标序号。
- planner goal 成功发布后，才发布同一 executor 的非终态 `ACCEPTED/DISPATCH`，使 Mission
  Manager 尽早冻结唯一 `executor_id`；目标阶段后续结果也必须由同一协调器代理。
- decision receipt 与 source stamp 均采用排他 deadline；等于 deadline 即过期。
- 旧 decision、旧 goal telemetry 和幂等重复均无副作用；同序号内容冲突、planner event
  回退或未知 goal 表示所有权丢失并 fail-closed。
- 只有当前目标先后见到 `ACCEPTED`、`TRAJECTORY_READY`、`TRAJECTORY_FINISHED`，且同
  mission frame 的新鲜 odom 对 effective goal 同时满足 3D 距离、速度和连续驻留，才产生
  motion success。
- SEARCH、RESUME、RETURN_HOME 到达产生终态 `SUCCEEDED/PLANNER`。
- APPROACH 到达只产生非终态 `PROGRESS/PLANNER` 和 target-transaction intent；绝不伪造
  payload commit 或目标成功。
- LAND 只产生 landing intent；ABORT/HOLD 只产生 safety-stop intent，不发布替代 planner
  goal。

## 尚未解除的 live Gate

Fast-Planner 当前没有带 ACK 的 cancel/hold 接口。`planning/replan` 只截短轨迹时长，轨迹
服务器仍持续发布 setpoint，不能作为安全停止。因此在补齐 planner cancel + traj_server
hold/stop ACK、单一 target-stage 协调器和 landing executor 前，本分支不能表述为可飞闭环，
也不得 ACK ABORT，更不得接真实 `/fastplanner/goal`。planner status 又没有进程实例 ID，
活动期 `event_seq` 回退只能闭锁。

这也是默认关闭 live 输出的原因，而不是启动便利性选项。

## 运行顺序与重启限制

- bridge 必须在 `/navigation/start_mission` 前启动并在本次任务内常驻。raw decision 是
  latched；晚启动或中途重启若首次看到 `decision_seq > 1`，会按冷启动 fence 闭锁。
- 当前一个 bridge 进程只承载一个 mission，不接受下一任务重新从 `decision_seq=1` 开始；
  第二次任务需重启 bridge，后续再用显式 mission reset/epoch 合同替代。
- 到达证据的 odom `header.frame_id` 必须实际等于 `camera_init`。MAVROS/仿真里程计若不是
  该 frame，必须先接显式坐标转换节点，不能只 remap 话题名。
- `PlannerStatus.event_seq` 没有 planner 进程实例 ID；活动任务中 planner 重启导致序号回退
  时，bridge 会永久闭锁，这是当前预期的 fail-safe 行为。

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
