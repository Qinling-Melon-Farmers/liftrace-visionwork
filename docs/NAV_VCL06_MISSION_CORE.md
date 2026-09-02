# NAV VCL06 确定性任务核心

## 范围

`uav_mission.mission_core` 是不依赖 ROS 的导航任务策略层。它只接收显式候选快照、
当前位置、任务时钟和 `NavigationResult` 对应的事实，不直接发布规划目标，也不调用投递
机构。ROS 消息转换、话题所有权及执行桥留给独立运行层。

本核心使用 `competition_profiles.yaml/r2026`，正式类别固定为 `tent、pillbox、bridge、
panzer、red_cross`；`tank` 和未知类别以 `profile_excluded` 拒绝。高权重搜索中断集合由
profile 确定为 `red_cross、panzer、bridge`。

## 候选准入和身份

候选只有同时满足下列条件才进入持久队列：

- `state == CONFIRMED(2)`，连续有效观测不少于 3 帧；
- 地图点、关联和 map frame 有效，且无视觉拒绝原因；
- 候选年龄和 TF 年龄均不超过 0.5 秒；
- 置信度/地图质量位于 `[0, 1]`，位置与时间均为有限值；
- 高度不超过 4 m，类别属于当前 profile。

目标实例键为 `(target_id, first_seen_ns)`。`target_id=0` 按合同是合法 ID；纳秒时间键避免
ROS epoch 浮点秒精度丢失。队列按“权重、可达性、地图质量、类别置信度、距离、首次出现
时间、ID”稳定排序，同一类别只让最佳实例竞争槽位，已投类别不重复投递。

派发 `APPROACH` 时，核心把 profile、目标身份、类别、地图点、最后观测时间、attempt、
payload slot 和目标上方安全航点冻结进不可变 `CoreAction`。执行期间到达的更新只刷新队列
中的最新快照，不能改变本次投递归因；提交时仍使用派发快照的类别。

## 搜索、时限和槽位

- 只在 `SEARCH` 阶段允许高权重目标中断；执行中的新候选只入队，不抢占当前任务。
- 低权重目标在覆盖路线结束，或进入动态时间兜底窗口后才参与投递。
- 动态估时包含保守路径系数、单槽服务时间、返航路径、返航/降落储备和决策 guard。
- 预计无法在 510 秒前完成当前投递，或无法在 600 秒内完成返航降落的候选不启动。
- 510 秒是硬返航边界；三槽一经提交完毕立即返航。
- 执行器明确报告的未提交失败释放槽位；可重试候选冷却 20 秒，最多尝试 2 次。
- 每个 target/search/return/land decision 都有明确 deadline。目标事务未见提交 ACK 就超时
  时，核心不能臆测载荷仍在：对应槽位转入 `QUARANTINED`、候选禁止重试并立即返航；延迟
  到达且身份匹配的释放 ACK 仍可把该槽位收敛为 `COMMITTED`。提交后的 recovery 超时保留
  槽位并返航，返航或降落超时进入 ABORT，禁止无限等待越过任务上限。
- 比赛配置和派发动作均为不可变对象，构造后不能把 510 s、600 s 或 4 m 硬边界改大。

`CoverageRoute` 独立持有 nominal waypoint cursor、route revision 和活动 decision。目标
中断只清除活动搜索派发，不推进 cursor；恢复时重新派发同一 waypoint。只有匹配的成功
终态才正常推进，重复/错序结果不改变 cursor；同一航点连续失败达到上限后会显式记入
`skipped_indices` 再推进，避免规划器永久卡在单点。

## 结果 reducer 与不可逆边界

结果按 `mission_id + executor_id + event_seq` 去重，并逐字段核对 `decision_seq、command、
target_id、target_first_seen、target_class、attempt、payload_slot`。执行器在任务中途改变、
重复/乱序事件或身份不一致均不能改变队列；结果携带的事件时间不得超出 manager 接收时钟
容差，防止未来时间戳提前改变状态。decision deadline 为排他边界：事件时间或 manager
接收时间任一处于边界或其后即按租约超时归约，不依赖 timer 与 result 回调先后；迟到释放
ACK 仍提交载荷事实，但任务标记失败并立即返航，其他迟到目标结果不能释放槽位重试。
隔离事务收到否定终态后仍保留有限 tombstone，因为否定结果不能证明载荷未被物理释放。
超期的 `SUCCEEDED` 若没有释放 ACK，与正常路径一样拒绝且不消耗 `event_seq`，允许同序号
的真实释放证据纠正该事件。

`SEARCH、RESUME、APPROACH、RETURN_HOME、LAND、ABORT` 共用同一个全局单调
`decision_seq`。targetless motion 也必须经过同一 result reducer；返航成功才产生 LAND，
返航失败不得直接降落。`APPROACH` 在本合同中是一次复合目标事务，执行桥通过 result 的
`PLANNER→CAPTURE→ALIGNMENT→RELEASE→RECOVERY` stage 报告内部进度，manager 不另建
第二套阶段状态机。

payload 的唯一提交事件为非终态 `PROGRESS/RELEASE`，并同时满足：

```text
payload_committed=true
retryable=false
reason=release_ack_success
evidence_source 非空
```

提交后槽位和类别不可回滚。后续必须以 `RECOVERY` 终态收尾；恢复失败会保留已提交槽位、
标记任务失败并立即返航。未见提交 ACK 的成功终态以 `success_without_payload_commit`
拒绝。语义非法的事件不会消耗 event sequence，因此同序号的修正事件仍可被接收。

未提交失败只有在可证明仍处于 `DISPATCH/PLANNER/CAPTURE/ALIGNMENT` 时才能释放槽位。
一旦收到 `RELEASE/RECOVERY` 阶段事件，核心锁存“可能已释放”；后续即使 stage 回退，失败
也只能把槽位转为 `QUARANTINED` 并返航，不能重试或复用。该 tombstone 接受同一 decision
后续身份匹配的真实释放 ACK，并把槽位收敛为 `COMMITTED`。

## 验证边界

`test/test_mission_core.py` 与 `test/test_coverage_route.py` 以系统 Python 执行确定性回归，
不启动 ROS master、Gazebo、PX4 或执行机构。它们覆盖准入拒绝、权重排序、冻结派发、
搜索中断边界、覆盖/时间兜底、cursor 恢复、冷却重试、全局序号、targetless result、
decision timeout、返航/降落失败、释放阶段槽位互锁、三槽完成，以及释放提交后恢复失败等
核心状态转换。

尚未在本阶段完成：ROS manager 运行桥、planner 遥测消费、统一仿真与真实执行代理。
它们必须在合同和本核心之上独立实现，不能回退为直接调用舵机。
