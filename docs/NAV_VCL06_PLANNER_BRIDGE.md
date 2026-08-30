# NAV VCL06 Fenced Planner Bridge

## 当前交付边界

本分支把导航任务层的 `NavigationDecision` 转成单一 planner motion 生命周期，并把
`plan_manage/PlannerStatus + nav_msgs/Odometry` 归约成同一 executor 的
`NavigationResult`。当前集成继续在这一个 executor 内协调既有
`MissionCommand/AlignmentTargetContext/ReleaseEvidenceContext/ReleaseResult`，形成
APPROACH→CAPTURE→ALIGNMENT→RELEASE→RECOVERY 与 LAND 的事实回流；bridge 本身仍不调用
Servo/PWM、不解锁、不分配槽位、不实现任务队列或重试，也不把 `TRAJECTORY_FINISHED`
直接当作到达。

默认 launch 设置：

- `execution_enabled=false`；
- planner goal 隔离输出为 `/navigation/fastplanner_goal`；
- `allow_live_goal_output=false`；

因此默认启动不会发布规划目标。只有同时显式设置 `execution_enabled=true` 与
`allow_live_goal_output=true`，并把目标话题配置为 `/fastplanner/goal` 时才创建 live publisher；
隔离话题仍可用于合同级联调。正式 `navigation_search_delivery_vcl06.launch` 才同时打开这两个
开关、绑定 `/fastplanner/goal` 并启用 strict 目标事务；默认 bridge launch 仍保持安全关闭。

## Fencing 与到达证据

- 冷启动接受任意仍在 deadline 内的当前 decision，支持 manager latched 决策后的晚启动；
  同 mission 的旧序号只作为 stale 忽略，不要求 bridge 观察到中间所有序号。
- raw decision 的 `decision_seq` 原样复制到 planner goal `header.seq`；不生成第二套目标序号。
- planner goal 成功发布后，才发布同一 executor 的非终态 `ACCEPTED/DISPATCH`，使 Mission
  Manager 尽早冻结唯一 `executor_id`；目标阶段后续结果也必须由同一协调器代理。
- decision receipt 与 source stamp 均采用排他 deadline；等于 deadline 即过期。
- 旧 decision、未知旧 goal telemetry、planner event 回退和幂等重复均无副作用；仅当前
  当前 decision/goal 的身份或生命周期冲突只终止当前 action，不锁死 bridge 进程。
- 只有当前目标先后见到 `ACCEPTED`、`TRAJECTORY_READY`、`TRAJECTORY_FINISHED`，且同
  mission frame 的新鲜 odom 对 effective goal 同时满足 3D 距离、速度和连续驻留，才产生
  motion success。
- planner `ACCEPTED` 映射为 `STARTED/PLANNER`；`TRAJECTORY_READY` 与
  `FAILED_ATTEMPT` 映射为非终态 `PROGRESS/PLANNER`，timer 不周期制造结果事件。
- SEARCH、RESUME、RETURN_HOME 到达产生终态 `SUCCEEDED/PLANNER`。
- APPROACH 到达先产生非终态 `PROGRESS/PLANNER`，随后只有旧控制实际进入目标阶段、strict
  视觉上下文与 guarded release ACK 依次成立，才按 CAPTURE→ALIGNMENT→RELEASE→RECOVERY
  回流；payload commit 只认既有 `ReleaseResult` 的成功 ACK。
- LAND 使用返航完成后的新鲜同 frame odom 冻结落点，等待旧控制 Land、水平/高度/速度 dwell
  与 MAVROS `ON_GROUND` 后才成功。ABORT 通过同一 planner publisher 发当前位姿 goal 并等待
  typed planner 结果；没有可靠契约的 HOLD 明确 REJECTED，不另造 stop 接口。

## 当前 live Gate

单一 target transaction、LAND 和 ABORT 已收进现有 bridge，HOLD 明确拒绝；没有增加第二
executor、compat adapter 或新消息类型。2026-08-31 的 ROS 实跑确认正式图只有
`/navigation/planner_bridge` 发布 `/fastplanner/goal`，bridge 健康、随机场/anchor/contact
READY，0 碰撞、0 越界、0 超高。

联合 Gate 仍未通过：manager 在升空后持续 `map_missing`，未收到
`/freedom/static_pointcloud`，所以没有产生任何 decision/result，也没有真实
`P_interrupt`。当前阻断是导航/建图输入而非 bridge 接口；不得关闭 require-map、放宽新鲜度
或增加视觉地图代理绕开。默认关闭 live 输出仍是独立启动 bridge 的安全边界。

## 运行顺序与重启限制

- raw decision 是 latched；bridge 晚启动可接收任意有效当前 decision。bridge 只保留当前
  decision、当前/待取消 goal 与最近 planner event，不维护无界任务历史。
- mission frame 由 `execution/mission_frame` 参数唯一决定且必须非空；decision、planner
  telemetry 与 odom 必须一致。若实际里程计不是所配 frame，必须先接显式坐标转换节点。
- goal 接受任意有限非零四元数并规范化，发布 planner goal 时保留该姿态，不要求逐位等于
  identity。
- `PlannerStatus.event_seq` 没有 planner 进程实例 ID；回退或未知旧 goal telemetry 会被忽略，
  但当前 goal 的同序号内容冲突会生成当前 action 的 typed FAILED；后续新 decision 仍可执行。
- 单条 malformed 输入只记录并忽略；只有内部 goal/result 发布合同不一致等非输入异常才关闭
  bridge 输出。

## 导航—视觉对齐边界

视觉 `TargetCandidateArray` 与导航准入所需的稳定 ID、`first_seen/last_seen`、地图点、
`camera_init`、TF 年龄和质量字段已经逐项对齐；视觉 stable ID 从 0 开始，bridge 通过
`has_target` 区分“目标 0”和“无目标”。感知模型可以保留额外类别，但正式 `r2026` 决策
只允许 `tent/pillbox/bridge/panzer/red_cross`，不含 `tank`。

末端接口已通过既有 `AlignmentTargetContext` 与 `ReleaseEvidenceContext` 对齐：
`NavigationDecision` 是唯一语义目标上下文，strict 模式核对 decision、attempt、slot、profile、
语义—几何身份、map validity 与 observation stamp；最终 payload commit 仍只来自 guarded
`ReleaseResult` ACK。所有阶段沿用同一 `executor_id` 与全局递增 `event_seq`，没有让
selected 代替 `P_interrupt`，也没有复制 arbiter 的准入策略。
