# FSM停滞与起飞首轨迹修复报告

## 状态

修复代码已在导航权威仓既有分支`feat/high-view-liveness-20260919`提交为`151eeba`，并精确同步到当前整机研究分支。未新开分支，未替换正赛部署。

本轮闭合三个已确认缺口：

- traj_server进入`tracking_hold`后，FSM按当前目标和轨迹身份在0.25s持续窗后直接重规划；不再等待偏差从服务器保持阈值约0.20m继续增长到FSM旧阈值0.45m。
- 起飞首段先在1.4m AGL前往固定起飞系场内`(0.60, 0.05)`，再原地升到2.6/3.0m；A*首层同时增加经过完整安全检查的目标方向原语。
- `initial_plan_timeout=12s`监督所有从未产生有效轨迹的规划动作，覆盖seed31曾遗漏的RETURN_HOME/走廊段；旧search-only参数保持兼容且本入口两者均为12s。

## 安全与事务边界

- A*新增原语仍逐20ms检查膨胀占据；没有允许从占据起点穿出、关闭障碍柱或缩小30cm膨胀。
- SERVER进展必须匹配`goal_seq + traj_id + traj_start`，旧目标、旧样条、超龄消息和瞬时保持不会触发恢复。
- 服务器保持恢复和4s/4cm物理无进展共用每目标两次预算。耗尽时任务桥收到终端`server_hold_budget_exhausted`并取消当前动作，不再静默等待总动作期限。
- 低空进场观测不进入高位记忆；进场或升高失败均终止该高位段，不能跳过后继续。

## 验证

- 整机Catkin完整构建通过，包含driver2、FAST-LIO、FreeDOM、patrol_control、Fast-Planner和uav_mission。
- uav_mission完整回归333项通过。
- uav_high_view回归71项通过。
- plan_manage/path_searching Catkin结果汇总65项，0错误、0失败；其中新增9项A*测试全部通过，SERVER保持监控4项、MotionWatchdog共享预算5项通过。
- 三份launch XML通过；seed32入口离线展开核对见[launch_validation.json](launch_validation.json)。

## 动态验收边界

本轮用户要求修复，但没有在当前请求中明确要求启动或重跑仿真。按仓库单实例规则，本报告没有启动SITL。后续动态验收至少应包含：

1. seed32和34各跑一轮，确认低空staging与高位升高均及时获得首轨迹；
2. seed31走廊第4段，确认被占据目标在12s内形成明确失败/语义恢复，不再停90s；
3. 注入0.25m跟踪偏差，确认出现`server_tracking_hold_replan`，恢复身份正确且次数不超过2；
4. 保留seed33或35成功布局作无回归对照。

没有这些动态结果前，准确结论是“实现、编译和离线契约回归通过”，不是“十轮故障已动态根治”。

## 首次动态尝试（后续已按用户指令停止）

seed32首轮回归在任务启动前以`startup_wall_timeout`结束：Gazebo车辆spawn冷启动约269秒，ROS时钟刚到0.785秒，MAVROS未连接、任务/规划事件均为0。该轮只证明基础设施未在默认180秒墙钟内就绪，不评价修复。包装器零残留收尾。回归入口随后增加默认行为不变的`presentation_recording`和`gate_startup_wall_timeout`参数，供下一轮关闭汇报录像并仅放宽宿主启动墙钟；ROS任务600秒限制不变。

第二次seed32在ROS时间0再次未启动：`/gazebo/model_states`在90秒内不可用，随机场节点以`field_status_fail`结束，车辆spawn调用被shutdown中断。WSL内核同期记录`dxg`图形设备通信失败，首轮YOLO CUDA进程还出现SIGABRT；仍不是导航结果。入口因此再增加默认值不变的模型状态墙钟和检测设备参数，最终诊断轮使用CPU推理、Mesa软件渲染及600秒两级启动墙钟。若该轮仍未进入任务，本次回归停止。

2026-09-21复核：上述冷缓存及GPU根因尚未证实，只能确认spawn延迟、model_states缺失及同期GPU告警；CPU/软件渲染第三轮未运行。Catkin65含容器重复和两项未提供FrozenR54外部点云的空返测试，不代表65个独立有效回放。新增通知/序号0等问题已修正，详见板端工作树flight_review_20260921报告。
