# 释放事务与 Bridge 后续修复（2026-10-05）

基线：共享工作树 `r2026-high-view-search`，任务分配时 HEAD `8a48500c`。本报告覆盖 P1-2、P2-1、P2-3、P3-1、P3-3。修改仅在本地共享工作树，由主代理统一构建、注册测试、提交及同步；没有启动 ROS、Gazebo、真实舵机或飞行。

## 1. 释放事实与槽位处理

| 已知事实 | 代理/Bridge 回报 | Core 与控制 worker 处理 |
| --- | --- | --- |
| 确定没有进入 raw 调用 | `NOT_STARTED=1`，`terminal=true`，完整固定身份 | 撤销本次动作；本次预飞拒绝不消耗槽位，可按既有冷却/复核机制建立新动作 |
| 进入 raw RPC；正在等待 | `RAW_CALL_STARTED=2`，`terminal=false` | 动作与槽位锁定，不允许重入 |
| raw bool 失败、异常，或可能执行后仍缺 ACK | `RAW_CALL_STARTED` 或未知事实；缺 ACK 单独超时 | 隔离槽位；不能从 `false`、无话题、超时推断没有执行 |
| 明确成功 | `COMPLETED=3`，`terminal=true`，`success=true` | 仅按对应固定动作提交一次投递事务 |

`RAW_CALL_STARTED` 表示进入真实服务调用边界，并不宣称已经精确观察到 PWM 使能。旧 raw `patrol_control/Servo` 仅返回 bool，无法区分使能前失败与动作末端关闭失败。因此一旦进入该 RPC，失败一律按结果不明锁住。

在严格释放证据尚未成立、也未观察到 raw 调用的阶段，Core 超时/abort 释放槽位。严格证据已经使释放成为可能后，缺少 ACK 保持隔离；这不是“没有信号就算没开始”。随后只有对应动作的代理原子撤销证明能回收尚未被观察为 raw 已开始的隔离槽位。raw 开始事实即使在超时后才到，也会记录进 tombstone，之后的迟到 NOT_STARTED 不能将其解锁。

Core 普通终态、总截止、显式 abort 与本地 lease 超时采用上述同一事实分类。地图过期导致 Manager 不再 tick 时，显式 abort 也不会凭空消耗确定尚未许可的槽位。本改动不负责 Manager 的独立总截止调度。

## 2. 固定身份与服务契约

`ReleasePermission.msg`、`ReleaseResult.msg` 追加固定动作字段：

```text
mission_id, decision_seq, attempt, payload_slot,
target_id, target_first_seen, target_class
```

`align_mode` 也必须匹配。本次 raw 开始和终态结果共享 `execution_id`；终态以 `terminal` 标识。新增 `execution_state` 枚举：`EXECUTION_UNKNOWN=0`、`NOT_STARTED=1`、`RAW_CALL_STARTED=2`、`COMPLETED=3`。`evidence_stamp` 保留作为对应观测信息，不用持续更新的最新图像时间挡住固定动作的成功回执。

### 正式控制调用：ServoAction

Pauli 在 `patrol_control/srv/ServoAction.srv` 中提供伴随接口，代理本轮已经实现：

```text
/mission/servo_action  patrol_control/ServoAction
```

代理参数 `~action_service_name` 可覆盖服务名。请求包含 controller-local `request_id`、槽位、上述完整固定身份和 `align_mode`。`request_id` 不是任务 `decision_seq`，不能互换。代理在 raw 准入锁内核对当前许可与全部身份后才进入 raw；响应回显该次 `request_id/payload_slot`、`res`、`execution_state`、`terminal`、`reason`。

C++ worker 只根据同一 RPC 的完整响应分类，由 Pauli 的 `classifyServoAction()` 和 `AsyncServo::releaseNotStarted()` 完成：仅匹配本次 request、槽位、终态 NOT_STARTED 且 `res=false` 可以解锁；未知、身份不符、传输错误及 raw 已开始不能解锁。单纯 CANCEL 或超时不构成 no-call 证明；若之后有同动作的明确终态 NOT_STARTED，应按该证明完成对应锁的回收。旧响应不能解除新 worker action 的锁。

原 `/Servo` (`patrol_control/Servo`) 和 `/legacy/Servo_raw` 服务格式未修改。旧 bool 服务调用者即使收到 `false`，仍不能把它当 NOT_STARTED 证明。代理发布的 `ReleaseResult` 是另一条带身份的事实通道；不能将旧 bool response 与其字段混为一谈。

最终定向合计 177 项全部通过；包括旧兼容开关与缺身份 NOT_STARTED/COMPLETED 的反例。

消息追加字段改变 ROS wire MD5。主代理必须重建 `uav_mission`/`patrol_control` 消息，以及相关 C++/Python 发布者、订阅者。语义上的旧字段兼容不等于新旧生成消息可以混跑。正式硬件 raw 服务不依赖新增 `ServoAction` 字段，接口仍旧。

### 原子撤销与 raw 准入

代理订阅 `~alignment_context_topic`（默认 `/uav_vision/alignment_target_context`）。匹配动作的 `active=false` 在同一把准入锁下：

1. raw 尚未准入：记录撤销 tombstone，发布终态 NOT_STARTED；该动作后续新许可也不能让 raw 启动。
2. raw 已准入：不发布 NOT_STARTED，保持该动作/槽位锁。
3. 等待服务发现期间收到撤销：真正进入 raw 前再次核查，阻止调用。

服务发现和约 1 秒 raw RPC 均在共享锁之外运行；并发请求仍由保留槽位和锁定动作防重入。收到 CANCEL 不会因“没看见开始话题”就直接释放槽位。Bridge 进入 `CANCEL_PENDING`，持续发布 inactive context，等待代理的明确证明；若 raw 实际已开始，最终成功仍绑定原动作提交，失败保留不确定。

节点进程内锁与 tombstone 尚未持久化到磁盘。不得在一次载荷状态不明的任务中，通过重启代理/arbiter 自动清掉隔离状态后继续释放；重启后的机构/槽位状态需现场重新建立。

## 3. Bridge ACK 与刚复核目标交接

- Bridge 默认只按完整固定身份匹配 `ReleaseResult`，不再用不断增加的视觉时间戳拒绝同动作的迟到成功。缺失 mission、decision_seq、first_seen 均拒绝，不退化匹配。`~allow_legacy_release_results` 默认 `false`；仅隔离旧链显式启用后，才允许 `execution_state=EXECUTION_UNKNOWN` 的旧 bool 形式按冻结证据匹配；任何新 NOT_STARTED/COMPLETED 缺身份仍拒绝。strict arbiter 同样拒绝缺身份回执。
- 拒绝错误任务、decision、attempt、槽、ID、first_seen、class、mode，以及早于动作或来自未来的结果。
- raw 开始与终态共享执行编号；开始后另一执行编号的拒绝不能撤销原调用事实。
- `PlannerMotionExecutor` 为释放结果超时/不确定失败保留有限晚到回执入口：同动作成功可补记已释放，原子 no-call 证明可回收；普通旧任务结果仍拒绝。
- arbiter 对 raw 开始/未知锁住当前槽，失败不自动跳下一槽；对应成功才推进，重复成功不重复推进。开始后的 NOT_STARTED 不解除锁。

公共目标 API：

```python
core.choose_confirmed(candidate, now, current_xy) -> Optional[CoreAction]
```

只选择这次传入且身份有效、新鲜、可准入的候选。冷却、终态、失效、不可达、时间预算等检查保留；不回退选整个缓存中的高分历史候选。主代理已经在 `high_view_full` REACQUIRE 接入该方法，completion record 使用实际 action 的 frozen `target_snapshot`；本子任务没有修改高位状态机文件。

## 4. 对准锁定后的窄反证撤销

Bridge 消费正式 TargetArray；原 drop_aligner 无需修改。默认门槛可通过 Bridge 参数覆盖：

| 参数 | 默认值 |
| --- | --- |
| `~target/contradiction_radius` | 0.35 m，距本次冻结目标坐标 |
| `~target/contradiction_confidence` | 0.90 |
| `~target/contradiction_min_frames` | 3 个独立、递增观测时间戳 |
| `~target/contradiction_min_span` | 0.15 s |
| 新鲜度/间隔 | 沿用 CAPTURE 最大年龄 0.50 s |

仅 confirmed (`state=2`)、有效投影/关联、当前任务坐标、对准锁定后新观测、持续同一矛盾正式类别，且尚未观察到 raw 准入时撤销本次许可并局部复核。正确类别重新得到支持或同帧竞争类别会清空反证窗口；重复帧、陈旧框、未确认框、邻近但不在半径内的目标、低分框、类别框消失均不触发撤销。

`0.35 m` 邻域在极近靶标或投影误差下仍可能涵盖其他靶标；三帧、confirmed、置信度及冻结坐标约束已保留。本轮不声称彻底解决近靶实例关联，应随实拍中心误差检查该参数。不会因单帧异类直接 ABORT。

## 5. 落地完成事实

同一次 LAND 已被控制器接受后，Bridge 允许新鲜 ON_GROUND + connected + disarmed + 新鲜独立位置稳定样本完成任务，不再要求上锁后控制器继续处于 LAND，也不再仅因 FC 地面高度偏移卡住。

仍要求地面状态、飞控状态、里程计的源时间及接收时间均新鲜，晚于该次 LAND 交接，位置在原落地半径内并满足原稳定窗口。未接受 LAND、陈旧地面状态、断连、仍在空中、位置移动不能完成。armed 情况继续要求 LAND 控制状态及原高度条件。空中已切 POSCTL 等人工模式会取消同次 LAND；其后手动落地不能自动转换为本次自主成功。

新订阅参数 `~flight_state_topic` 默认 `/mavros/state`，只读，不调用解锁或模式服务。

## 6. 回归记录

2026-10-05，本子任务最终定向验证：

- 156 项：`test_release_transaction_followups`、`test_mission_core`、`test_release_commitment`、`test_planner_execution`、`test_navigation_planner_bridge_contract`，全部通过。
- 21 项：`test_mission_runtime`、`test_planner_mission_contract`，全部通过。
- 新用例直接加载生产 proxy/arbiter/Bridge 类方法，配合同步/并发 transport doubles、真实 Core 与 PlannerMotionExecutor；不启动 ROS master。包含约 1 秒慢 raw 替身，证明代理共享锁不跨 RPC 持有；C++ 设定点连续性由异步舵机 agent 的回归与主代理构建另验。
- 新测试覆盖撤销先到/raw 先到/发现期间撤销、失败后禁止第二次 raw、固定身份 service、旧 NOT_STARTED 不解锁新动作、开始后及迟到开始后不解锁、许可前/许可后的无 ACK 超时、runtime 迟到成功对账、老高分候选不抢新目标、持续反证与非 confirmed/陈旧/重复帧反例、落地后控制状态退出，以及空中接管与断连反例。
- 旧 runtime timeout fixture 原先没有提供“可执行”事实就预期隔离；本轮补明确 RAW_STARTED，另增加许可开放但无开始 ACK 的迟到成功用例，以及尚未许可时超时释放槽位、恢复巡航的用例。未回退新的事实分类迎合旧断言。
- owned 文件 `git diff --check` 通过；全套 `uav_mission` unittest、message/full C++ build 由主代理汇总执行，本报告不提前代签其结果。

定向命令（工作树根目录）：

```bash
export PYTHONPATH="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/src:$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/scripts:$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/test"
/home/xhj/miniconda3/envs/rl_drone/bin/python -m unittest \
  test_release_transaction_followups test_mission_core test_release_commitment \
  test_planner_execution test_navigation_planner_bridge_contract \
  test_mission_runtime test_planner_mission_contract
```

## 7. 交接文件与尚需验证

代码/消息：`scripts/guarded_servo_proxy.py`、`scripts/navigation_planner_bridge.py`、`scripts/release_permission_arbiter.py`、`src/uav_mission/release_transactions.py`、`src/uav_mission/mission_core.py`、`src/uav_mission/planner_execution.py`、`msg/ReleasePermission.msg`、`msg/ReleaseResult.msg`。

测试：`test/test_release_transaction_followups.py`（新增）、`test/test_mission_core.py`、`test/test_mission_runtime.py`、`test/test_navigation_planner_bridge_contract.py`。CMake 测试注册、共享变更记录、git 提交和分支同步由主代理负责。本子任务未修改 CMake、patrol_control、高位状态机、vision drop_aligner 或共享文档。

合入前主代理需完成同版消息生成和 Python/C++ 全定向 suite，确认 C++ 正式外部入口服务名为 `/mission/servo_action`、固定身份来自当前 ALIGN context。之后用实际 ROS transport 验证慢 raw 调用时设定点连续、异步取消与失败回收；新完成判定及反证撤销还需动态整机场景/实机验收。本子任务的离线通过不等同实体载荷释放或飞行通过。

### C++ 撤销后的锁回收接线已完成

已只读核对 Pauli 12:16 后的最终实现：

- context inactive 仅关闭后续准入，不再直接取消已有 worker 或丢掉该 RPC 的终态响应。
- `AsyncServo::submit()` 在检查新动作槽位锁之前，对已完成旧 job 调用 `releaseNotStarted()` 回收证明。
- 回收同时匹配 job action、slot 和当前 slot owner，并要求 `done` 与已分类 `kNotStarted`；未知、传输失败、已开始 raw 或尚未结束的 RPC 都不能据此解锁。
- 单纯 cancel/timeout 不解锁，但其后该固定 RPC 返回确切终态 NOT_STARTED 可证明没有 raw 调用；同一旧 action 不会因此重新执行，也不会解除新 owner 的锁。
- `classifyServoAction()` 保留 request_id、slot、terminal 及 enum/bool 一致性检查。bool-only false 始终是不确定。

此前“Core 已回收但 C++ cancelled job 仍锁槽”的接线缺口已处理，不再作为待办。主代理回报 build/相关测试 62、42、11 项通过，EV agent 另完成 worker 22 项和 proxy 19 项评审/回归；这些是协作方的验证记录。本子任务在最终共享版本又复跑 177 项 Core/Bridge/proxy/arbiter/runtime 定向回归，全部通过。

最终本子任务没有剩余生产代码接线项。主代理可完成全套汇总、提交与跨分支同步；同版 ROS transport、动态仿真/实机验证仍按第 7 节执行，离线测试不替代实际飞行验收。
