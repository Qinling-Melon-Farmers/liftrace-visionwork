# 10月3日现场冻结回收与舵机验收

当前板端：`orangepi@192.168.3.126`，Orange Pi 5；工程 `~/liftrace_board_trials_20260928`。本轮只进行已获确认的地面三槽动作验收，没有解锁、起飞、运动设定点或仿真任务。工作台服务继续关闭。

## 1. 位姿与录包

现场高位probe仍保留 `dt>0 && 三维位移>3*dt+0.25m` 拒绝检查。已按实际代码恢复到视觉板端/高位研究和导航liveness；保留开发树中独立的原始ABORT原因上报修复，不把旧文件整份覆盖。源码/边界回归见[公共保护记录](../../verification/field_freeze_20261003/REPORT.md)。123项相关本地测试通过；本轮没有重新验收飞行定位质量。

现场 `mission_core.py`、`trial_bag.py`、`test_area.yaml` 与本地板端对应文件相同。轻量bag保留 `/sdf_map/occupancy_inflate`，不常开FreeDOM全图和占据图；下视图走原压缩/节流链，未恢复独立MP4编码，也未改图像/点云频率和范围。不要用10月1日历史“完全不录点云”覆盖这台机。

## 2. 现场舵机来源与修复

本机的舵机实体包位于 `patrol_uav_ws-patrol_planner/src/actuator_pwm`，没有 `hardware_ws`。现场映射为槽1/2/3→pwmchip4/5/0、设备febf0020/febf0030/fd8b0010，沿用原初始/释放脉宽。已保存为[Orange Pi 5参考包](../../../deployment/onboard_actuator_reference_20261003/actuator_pwm/README.md)，不能用5 Plus的2/3/4替换。

现场原代码即使PWM写失败仍返回True，并在初始化未完成前开放服务。本次仅补回此前已经实现的返回检查：sysfs写入/读回检查、逐步CheckedPulse、初始化全部成功后开放raw服务、失败返回False、退出只禁用PWM不unexport。板端目标 `pwm_node1` 重新编译通过，假PWM故障注入通过。原包已按旧链规则保存在板端 `legacy_baseline/20261003_servo_feedback/`，含清单和SHA256。

## 3. 两轮验收结果

实际飞控心跳均 `connected=true, armed=false`。使用独立 `/ground_servo_chain_check/*` 话题喂入**合成的严格视觉/对齐上下文**，不伪造全局MAVROS位姿或任务话题；真实 `release_permission_arbiter` 生成短期许可，真实 `guarded_servo_proxy` 调用 `/legacy/Servo_raw`，由现场PWM驱动实际舵机。

|项目|结果|
|---|---|
|无投递阶段/无许可请求|拒绝|
|持1号许可请求2号槽|拒绝：payload_slot_mismatch|
|1→2→3号许可释放|两轮均返回True，三个raw_actuator_ack|
|每槽成功后重复请求|拒绝：payload_slot_already_used|
|真实机械动作|用户第二轮确认三槽均正常作动|
|间隔后回位|用户确认填充物顶回，保持现有时序，不作软件复位故障|
|结束状态|三路enable=0，占空比留在释放值；测试ROS、MAVROS、arbiter/proxy/driver均退出|

首次没看清后，按用户再次授权重做一轮，各次释放之间留3秒。每轮只在driver启动时依次复位一次，随后每槽释放一次；日志没有再次启动/复位记录。PWM反馈不具备实际载荷离机传感能力；本轮是地面机构/许可链验收，不是带载飞行验收。

本地产物：`logs/field_freeze_20261003/ground_servo_chain_20261003/`、`ground_servo_chain_20261003_repeat_145048/`，含driver/arbiter/proxy/MAVROS日志、results与最终PWM状态；用户反馈单独记入 `operator_observation.json`，不改写原机器记录。

## 4. 失败返回之后的行为

- 初始化失败：driver退出，不开放raw服务，实投入口检查应拒绝继续。
- 运行中PWM操作失败：raw返回False；代理发布 `success=false / raw_actuator_rejected`，不会当成投递成功。
- 当前MissionCore收到已进入RELEASE阶段的失败：隔离当前目标和槽位（QUARANTINED），原因 `candidate_release_state_uncertain`，转RETURN_HOME收尾；**不自动重发同槽，不自动换下一槽补投**。软件失败也可能发生在机构已经动作之后，不能推定盒子仍在。
- 对准尚未触发释放就失败，是另一路候选失败/延期处理，不等同于释放结果不确定。

代理本身只负责许可与执行结果，不负责飞行返航；返航/30cm交接/降落由当前任务档案与任务层处理。上述失败路径依据现场相同的MissionCore源码及49项核心回归；这两轮真实硬件均成功，没有人为在实机上制造断电/卡滞。

## 5. 当前入口与分支

工作台默认地址更新为3.126，保留历史地址、自定义及密码入口。PWM初始化/二进制路径改指当前主工作区实体包，修正“5a会复位”的旧提示：5a不输出脉冲，5b服务启动才复位；34项离线工作台回归通过。工作台没有被启动。

公共保护推送视觉研究、视觉板端和导航liveness（origin+fork）；本台舵机参考包、入口与本报告推视觉板端及导航 `板端参考分支`。不覆盖试飞组 `板载代码`。main、旧main-integration/competition-integrated、根目录main与旧VCL06工作树不移动；完整冻结链同版实飞验收及独立正赛交付仍待后续明确安排。速度仅计划，未实现。
