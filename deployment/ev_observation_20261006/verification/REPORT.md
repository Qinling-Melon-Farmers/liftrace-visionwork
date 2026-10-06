# 本轮套件验证（2026-10-06）

所有本轮编写文件位于deployment/ev_observation_20261006内。没有连接板端、init真实ROS节点/master、运行仿真或录制，没有修改其他包/共享changelog或commit。并行主代理的包/launch修改不计入本套件代理的修改。

| 检查 | 最终结果 | 范围 |
|---|---|---|
| 原样预测组件 | 19 PASS | 上游纯NumPy回归 |
| 原样reset组件 | 22 PASS | 上游纯Python回归，合成事件 |
| suite隔离/补丁/默认模式 | 20 PASS | 全部输出含status、防内部/外部remap、false标定、无默认启动、补丁不改线程 |
| suite轻量录制 | 4 PASS | 独立精确topic/type配置、preview不创建输出、原低空录制器白名单不变 |
| 纯数值/隔离合计 | **65/65 PASS** | offline_result.json，无master |
| 原样ROS reset适配 | 10 PASS | 使用真实0928最新ReleasePermission定义生成的离线序列化fixture |
| ROS/bag预测往返 | 2 PASS | 实际ROS消息写入/读回临时bag，预测输入与拒绝MAVROS重映射 |
| ROS wrapper | 6 PASS | 新事务字段保留、真实适配构造、显式--start、disable_rosout、graph集合、fixture被正式preflight拒绝 |
| ROS schema离线合计 | **18/18 PASS** | schema_fixture_result.json；不是Catkin/ARM构建或真实运行图 |
| 合成预测/0.524m reset用例 | PASS | synthetic_demo与fc_reset_case.json；没有向在线节点喂假健康/假事件 |
| bash语法、默认preview、record-preview | PASS | 无node/master/输出目录创建 |
| 快照/低空候选patch适用性 | PASS（主代理集成前） | git apply --check；patch随后冻结，不能重放 |
| 当前本机devel检查 | 正确拒绝旧ReleasePermission | 没有用EV旧消息、临时fixture或其他worktree冒充0928当前生成消息 |

轻量录制最初subscriber_queue_size=200超过原低空实现的100上限，回归发现后已改100，最终65项全过。未放宽原实现的队列边界。

ROS序列化fixture位于本suite忽略的verification/.tmp/generated_schema，使用Noetic genpy从实际消息定义生成，仅供离线验证，不是新Catkin工程；正式message_checks检查来自当前0928 devel且拒绝此路径。没有将fixture加入source.sh或启动入口。

主代理已集成4个FAST-LIO快照文件及同LIO低空入口prediction_state_enabled参数（默认false），报告ARM新消息/LIO编译完成，正在执行真实production matching。第一轮ARM三线程flags/链接与105项专项结果由主代理提供，本代理未连板复核。第二轮matching、板端preflight/真实运行路径/图和记录内容由主代理验收，不把本报告写成已完成实飞或在线reset控制连续性。

**未接正式EV或控制；仍无已验收authoritative FC reset生产者和独立LIO健康生产者，标定强制false。** 观察READY不能放行；低层HOLD握手、实机故障/负载、真实bag录制仍未验证。
