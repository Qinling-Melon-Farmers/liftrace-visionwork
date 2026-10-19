# P1-1：Servo 异步执行，控制回调继续发布

日期：2026-10-05。基于共享工作树 `r2026-high-view-search`、HEAD `8a48500c4e9444ca91032921d6fc84d681826055`。本项仅修改 `patrol_control` 和本报告，未修改 mission_core、guarded proxy、Bridge、AGENTS 或共享变更记录；未提交、推送、部署，未启动 ROS master、Gazebo 或实机。

## 改动与行为

原先 DynamicProcess、WayPointDetectDone、CrossDetectionDone 在控制线程内同步等待 `/Servo`，正常约 1 秒的硬件服务会阻塞单线程回调。三处现在统一使用 `AsyncServo`：控制线程提交一次请求即返回 `kPending`，不声明成功；worker 等待真实服务结果，原 20 Hz 定时器继续工作。结果只在控制线程按相同本地动作编号和槽位领取一次，再更新成功及投后恢复状态。worker 不访问控制器成员，不使用 AsyncSpinner。

- 一次最多一个 RPC，无等待队列；三个槽位各自有锁，已提交、失败、结果不确定、接管取消或超时后均不自动重复调用同槽。
- ALIGN 关联现有 `MissionCommand.header.seq`、target_id 和类别；同一 decision 重发不会重置检测/投递事务。seq=0 的旧发布者在有效 header.stamp 下继续兼容。内部动作 generation 在显式检测/投递状态重置时递增。
- 飞控退出 OFFBOARD、解除武装或断连的状态回调会取消在途结果应用；非对准模式、legacy 对准超时、显式路线切换/状态重置也取消。发起调用前复用已有新鲜、connected、armed、OFFBOARD 检查。
- 成功必须同时具备真实 RPC transport 成功及 `Servo.res=true`。失败、异常、取消、超时及旧 generation 结果均不会写入成功状态。未携带身份的 legacy `/servo/complete` 不可确认已发起的异步 RPC。
- `drop_system/servo_call_timeout_sec` 默认 10.0 秒，用 steady_clock 判定 RPC 的逻辑等待期限；不改变视觉、位置、飞行速度或释放允许阈值。
- ROS1 同步 service call 没有可保证立即中断的 transport timeout。本实现把逻辑等待和资源数量限制住：超时后停止应用结果和自动重调；尚未返回的唯一 RPC 仍占 worker，不再启动其他 RPC。shutdown 不等待该阻塞 RPC，后台只持有 client 副本和共享 Job，退出后不会回写已销毁控制器。已经进入物理调用的动作无法被本地取消撤回，此时按结果不确定保留槽位锁。

## 旧链快照

首次改动前已在来源开发工作树创建独占目录，未覆盖既有目录：

`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/legacy_baseline/20261005_async_servo/`

其中 `patrol_control/` 保存当时当前包 43 个文件，`manifest.json` 记录来源工作树、HEAD、文件路径、大小及 SHA256，`SHA256SUMS` 提供清单。快照不复制回精简研究树，不提交此处共享日志。

## 修改文件

1. `patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/async_servo.h`：独立单 RPC worker、槽位锁、动作匹配、逻辑 deadline、取消及生命周期。
2. `.../include/patrol_control/drop_action.h`：新增 `kPending`，保持原成功分类和 Servo 服务契约。
3. `.../include/patrol_control/patrol_control.h`：worker 和控制线程事务状态。
4. `.../src/patrol_control.cpp`：异步提交/领取，控制定时器接线，接管/路线/超时取消，重复 ALIGN 防重置，旧 Bool 完成信号隔离。
5. `.../CMakeLists.txt`：Threads 链接及定向测试注册。
6. `.../test/test_async_servo.py`：生产 worker、生产提交/领取/取消及模式回调方法；执行生产定时器的异步前缀与发布尾段，用 mock RPC 和 Publisher 验证控制连续性。
7. `.../test/test_external_landing_handoff.py`：补足标准 Header 和新增控制器依赖的 mock，增加重复 ALIGN decision 不重置的生产方法测试；原有 LAND 验收断言未放宽。
8. 本报告。

## 验证

仅运行离线 mock 和编译，无真实服务调用。

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
/home/xhj/miniconda3/envs/rl_drone/bin/python -m unittest discover \
  -s patrol_uav_ws-patrol_planner/src/patrol_control/test -p 'test_*.py' -v
```

**50 项 Python 定向/相关回归全部通过**，含 11 项新异步测试、16 项生产 LAND/命令回调检查。RPC mock 正常阻塞 1.000 秒；同一动作调用次数 1，生产定时器异步前缀和发布尾段在 20 Hz mock 时钟内发出 **23 次设定点**，最大间隔 **0.0502214 秒**。这是离线 mock 出口，未测量实际 MAVROS/PX4 的调度间隔；几何及其他定时器分支由原控制包回归覆盖。

覆盖：成功前没有假 ACK、三槽各调用一次、transport/执行失败和异常不重复、逻辑超时后不重调、POSCTL 接管取消迟到 ACK、新动作/槽位不消费旧结果、完成只领取一次、阻塞时 shutdown 不等待且不使用销毁对象、无效槽及状态不发起调用。

```bash
g++ -std=c++14 -pthread -I patrol_uav_ws-patrol_planner/src/patrol_control/include \
  patrol_uav_ws-patrol_planner/src/patrol_control/test/test_drop_action.cpp \
  -lgtest -o /tmp/patrol_async_drop_action_test
/tmp/patrol_async_drop_action_test
```

**11 项 C++ drop-action 原接口回归通过**。

经主代理确认暂不并行编译导航/其他 C++ 后：

```bash
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
cd patrol_uav_ws-patrol_planner
catkin_make --pkg patrol_control -j2 -l2
```

**实际控制包构建通过**，`[100%] Built target patrol_control`，已通知主代理释放构建资源。此前另使用已有 flags/link 信息在临时目录编译、链接完整控制器成功，没有写共享 build/devel。验证临时输出：`/tmp/patrol_async_servo_build.log`、`/tmp/patrol_async_servo_tests.log`。`git diff --check -- patrol_uav_ws-patrol_planner/src/patrol_control` 通过。

## 与主代理 / guarded proxy 的协调

1. 两层互锁各有职责：控制器限制一个在途 RPC、同槽不重调及旧结果不应用；proxy 必须在进入 `/legacy/Servo_raw` 前用锁固定 permission 的动作/decision、槽位及目标身份。许可更新不能替换在途调用的身份，raw 调用应在其锁外执行，完成后按被冻结的身份回报；不能只依赖 Python 回调或 ROS spinner 串行化。
2. proxy 进入 raw 后若返回失败、transport 不明或关闭 PWM 失败，必须保留动作/槽位不确定锁；不得因新许可到达而再次 raw call。控制器的锁也保留。
3. 只有 proxy 明确证实 raw 未进入，才允许局部刷新许可后重试。现有 `Servo.req` 仅槽号、`Servo.res` 仅 bool，无法区分未开始和不确定失败。本项因此采取保守锁；另一 agent 的 P1-2 执行事实消息完成后，主代理需要把“同一动作确定未开始”的事实接到控制器的明确解锁/新动作准入流程。当前没有开放自动解锁 API，不以 `res=false` 解锁，也不宣称 P1-2 已完成。
4. 本地 generation 不是跨包消息中的 decision/action_id；跨包完成事实仍应由 Bridge/proxy 按固定身份关联。需要同时验证 RPC 启动与许可切换竞态：旧 req 不能被后来的新许可重新解释为新动作。若保留无身份的 legacy Servo 请求，proxy 应拒绝不能确定所属动作的请求；不能靠新许可覆盖槽位锁解决。
5. 本项未改服务消息和跨包执行事实。后续整体测试应使用 1 秒延迟的 raw mock，并验证 proxy raw call 数量、Bridge 固定动作 ACK 与控制出口的连续性；不能以快速模拟响应替代这一轮整链验收。

## 遗留与下一步

- 未做 ROS/SITL 或实机动态验收；没有发布新的板端版本。
- 失败后槽位保守锁可能阻止“proxy 明确未开始”的可恢复重试，需由 P1-2 事实接线完成后定向开放。当前这样处理是为了满足不确定失败不得重调的要求。
- 无期限挂起的服务会占住唯一 worker；需要上层已有失败处置/人工恢复，不能在同进程另起 worker 假装调用已取消。
- 本次只消除 Servo 服务等待对控制回调的约 1 秒阻塞；旧链其他同步服务及 legacy stopDropAction 内的 sleep 未重构。
- 主代理负责统一变更记录、提交、分支同步及后续整链验证；本子任务按要求保留本地修改，不提交、不 push。


## 2026-10-05 12:16 收尾补充：NOT_STARTED 兼容及异步回执竞态

本节替代上文“未改服务消息、未开放解锁”的历史限制。仍未修改共享记录、提交、推送或上板。新增文件为控制包 `srv/ServoAction.srv`、`include/patrol_control/servo_action_result.h`、`docs/SERVO_ACTION_INTERFACE.md`；既有 Servo.srv 不变。companion 服务由控制包生成，使用现有 uav_vision AlignmentTargetContext 冻结任务身份，不依赖 uav_mission C++ 消息，保持 uav_mission → patrol_control 依赖方向。

控制器 external 使用参数 `drop_system/servo_action_service`（默认 `/mission/servo_action`），legacy 使用旧 bool 服务。request_id 是独立本地 worker generation，不是 mission decision_seq；RPC 请求包含完整 mission/decision/attempt/target/slot，响应必须回显 request_id/slot，且 terminal COMPLETED + res=true 才成功。terminal NOT_STARTED + res=false 才是确定零 raw 调用证明；普通 bool false、transport 错误、unknown、raw-started 失败不会解锁。

匹配 inactive context 只撤销未来提交资格，不取消已经入队 RPC。这样 COMPLETED/NOT_STARTED topic 先触发 Bridge 清除 context、RPC 稍后返回时，控制器仍消费本次终态。接管、disarm、断连、新 ALIGN 等真正切换仍取消旧动作。worker 新 submit 会独立回收旧 job 匹配 owner 的 terminal NOT_STARTED，旧结果无需经过控制器 poll；即使旧任务已经取消或超时，确定零调用证明仍有效。但旧成功/不确定结果不解锁也不被应用到新动作，旧 generation 不得重调。

验证命令：

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
/home/xhj/miniconda3/envs/rl_drone/bin/python -m unittest discover \
  -s patrol_uav_ws-patrol_planner/src/patrol_control/test -p 'test_*.py' -v
PYTHONPATH=patrol_uav_ws-patrol_planner/src/uav_mission/src:patrol_uav_ws-patrol_planner/src/uav_mission/test \
  /home/xhj/miniconda3/envs/rl_drone/bin/python -m unittest test_release_transaction_followups -v
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
cd patrol_uav_ws-patrol_planner
catkin_make --pkg patrol_control uav_mission -j2 -l2
```

结果：控制包 Python **62/62 PASS**（异步服务 **23 项**）；读取 Singer 当前生产方法的 release followups **42/42 PASS**；旧 C++ drop-action **11/11 PASS**。合计 115 项不同测试。1 秒 mock RPC 期间 23 次 mock 设定点，最大间隔约 **0.0501712 秒**，同一请求调用一次。新增顺序覆盖成功/未开始 topic 先到、inactive context 后到、RPC 最后返回；新 ALIGN 在旧 proof 前到；取消及超时后确定未开始可回收；unknown 仍保持锁。

实际顺序构建成功，exit=0，完成于北京时间 **2026-10-05 12:16**；生成 ServoAction 及 uav_mission Release 消息，无循环依赖。日志 `/tmp/patrol_async_servo_final_build.log`；控制包回归 `/tmp/patrol_async_servo_proof_tests.log`；跨包方法回归 `/tmp/patrol_servo_release_joint_tests.log`。未运行 ROS 节点、仿真或真实舵机。

协调遗留：Singer 已在共享 proxy 中提供 companion endpoint，但截至本次检查，其若干 NOT_STARTED 拒绝/发现服务失败路径尚未在同一互锁中撤销本次固定 action fence。只有“本动作不会随后进入 raw”成立，worker 取消后确定未开始的回收才具有完整工程契约。主代理须让 Singer 补齐及验证旧请求不得因新许可再次进入 raw；原测试允许 legacy 服务发现失败后刷新许可重试，应区分新 companion 固定动作与 legacy 兼容语义。42 项通过不代表这个尚未加入的检查已通过。最终整链验收由主代理合并 Singer 后补测；控制包本轮不越权编辑 proxy/core/Bridge。
