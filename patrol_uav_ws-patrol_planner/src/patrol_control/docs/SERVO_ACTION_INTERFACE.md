# 给 Singer / release agent 的 ServoAction 最小接口约定

接收人：Singer `01a10a2a-3643-72e2-9b94-54a5f7f91837`。2026-10-05 本地共享工作树接口需求；当前工具没有跨 agent send_message，本文件与 commentary 提供主代理转交，不声称消息已直接送达。

## 依赖与入口

`uav_mission -> patrol_control`，保持该方向。控制包不引入 uav_mission ReleaseResult 的 C++ header、反向 find_package 或 package.xml 依赖。

新增 `patrol_control/srv/ServoAction.srv`；请 proxy 另外提供 `/mission/servo_action`（参数可配置）。`/Servo` 及 Servo.srv 原样兼容。控制器 external 任务默认使用新服务，legacy 继续使用旧 bool 服务；新服务不可用按不确定 transport 失败锁住槽位，不偷偷回退旧服务。

## 请求与响应

请求字段以 srv 文件为准：`request_id`、payload_slot、mission_id、decision_seq、attempt、target_id、target_first_seen、target_class、align_mode。

- `request_id` 是 controller 本地 worker generation，**不是** mission decision_seq，也不是 proxy execution_id。例如 request_id=1、decision_seq=900 是有效组合。
- 固定任务身份来自既有 `uav_vision/AlignmentTargetContext`；控制器在提交 worker 前冻结全部请求字段，验证其 decision_seq/target 与当前 ALIGN 一致。
- proxy 入口必须在同一互锁中固定并核对上述完整任务身份与当前许可；许可改变时旧请求不可被新许可重新解释。
- 响应回显本次 `request_id`、payload_slot；其 RPC 已绑定完整请求，因此不用复制 uav_mission msg 创建反向依赖。
- execution_state 与 ReleaseResult 一致：UNKNOWN=0、NOT_STARTED=1、RAW_CALL_STARTED=2、COMPLETED=3。仅 `terminal=true && !res && state=NOT_STARTED` 是解锁证明；仅 terminal COMPLETED + res=true 是成功。
- 响应还带 reason。错误 request_id/slot、非终态、未知状态、transport 失败、bool 失败或 RAW_CALL_STARTED 失败均保持锁。

## proxy 必须提供的不变量

1. NOT_STARTED 只能表示本次固定请求没有进入 raw 调用，而且此任务 fence 已被消费/撤销，不会因新许可到达随后再 raw call。
2. 不得把“重复请求被拒绝”解释成以前没有 raw 调用。槽位或动作已经执行、不确定、在途时不得返回可解锁的 NOT_STARTED；应返回保守事实并保持 proxy 自身锁。
3. 许可缺失、过期、已撤销或 raw 服务不可用可以给出 NOT_STARTED，但其身份必须源自本次请求，而不是碰巧存在的另一条许可。
4. raw 调用前保留 action/slot 锁；RPC 在锁外等待；成功或失败的 ReleaseResult 按原冻结身份发布。旧 /Servo 入口仍按现有兼容语义工作。
5. 同一 request_id/action fence 再来不能执行第二次。确定未开始后的合法重试是**新 ALIGN / 新 request_id**，由 core/Bridge 给出新许可。

## 控制器已经实现

- worker `releaseNotStarted(local_action,slot)` 仅在对应 job 完成、结果为可靠 kNotStarted、锁 owner 匹配时解锁；保留旧 owner 拒绝再次提交相同 action。
- 控制线程 poll 领取事实后解锁，不返回释放成功；旧 ALIGN 不再次 call。新 ALIGN 重置本地 generation，允许相同槽位的新任务调用。
- 前后两种身份不混淆：响应匹配 RPC request_id/slot；proxy 验证请求里的 mission/decision/attempt/target fence。
- 匹配 inactive context 只撤销未来提交能力，不取消已经入队的 RPC；避免 topic 先到而丢弃 COMPLETED 或 NOT_STARTED 回执。接管、停机及新 ALIGN 仍取消旧任务。新 submit 可独立回收已结束旧 job 的 terminal NOT_STARTED，包括取消/超时后晚到的确定零调用证明；不会消费旧成功为新动作成功，也不会放开 unknown/raw-started 锁。仅新动作允许重试，旧动作不重调。

## 已通过的离线测试

23 项生产 worker/回调定向测试通过（1 秒 mock 调用期间 23 次 mock 设定点，最大间隔约 50.1ms）：新增 NOT_STARTED 后新 ALIGN 可重试、原动作不重调、错误 request_id/slot/nonterminal 不解锁、mission context 不符不调用、legacy bool false 仍锁、旧 proof 不可解除新 owner。

请 release agent 在 proxy 补 companion 服务并新增上述不变量测试。主代理在该实现完成后运行完整构建和延迟 1 秒的整链 mock；当前尚未宣称完整 P1-2 跨包链路已经通过。所有改动只保留本地，未提交、push 或上板。

## 终态 topic 先于 RPC 的复现顺序

新增生产回调测试覆盖 COMPLETED→inactive context→RPC success、NOT_STARTED→inactive context→RPC NOT_STARTED，以及 new ALIGN 先取消旧任务、旧 RPC proof 后到、新 submit 独立回收。取消或超时后的 unknown/started 仍不得解锁。当前 companion endpoint 已在 Singer 的共享 proxy 文件中出现；仍需其 NOT_STARTED 路径在同一互锁中撤销本次固定 action fence，然后由主代理验证跨包生产顺序。
