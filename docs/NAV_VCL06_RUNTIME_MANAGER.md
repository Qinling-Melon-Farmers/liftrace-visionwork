# NAV VCL06 ROS Mission Manager

## 运行边界

`navigation_mission_manager.py` 是确定性 `MissionCore + CoverageRoute` 的 ROS1 运行壳。
它只发布导航合同 `uav_mission/NavigationDecision` 到
`/navigation/mission_command_raw`，消费执行桥回传的
`/navigation/mission_result`。它不发布 `/fastplanner/goal`，不调用 Servo、PWM、解锁、
起飞或投递机构；planner goal 与 guarded release 必须由独立 execution bridge 持有。

节点没有自动启动路径。人工调用 `/navigation/start_mission` 前必须已有：

- 新鲜且 frame 为 `camera_init` 的本地位姿；
- 新鲜、非空、布局完整且 frame 为 `camera_init` 的
  `/freedom/static_pointcloud`；
- 非零 ROS 时钟；
- 可加载且名称为 `r2026` 的冻结比赛 profile。

任一条件不满足时 start fail-closed，不发布飞行 decision。任务运行中位姿/地图失鲜或高度
超过 4 m、ROS 时钟回退或回调发生未预期异常时，只生成一次合同级 `ABORT` raw decision，
仍不直接操作飞控。`r2026` profile 不允许关闭地图 readiness 门。

launch 中的 `/mavros/local_position/pose` 只是可覆盖的接线默认值。启动前必须实测其
`header.frame_id`；若不是 `camera_init`，必须通过 `pose_topic` 接入已有的 mission-frame
`PoseStamped` 转换输出。管理器不会猜测或静默改写 frame，默认不匹配时会拒绝启动。

## 话题和服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav_vision/targets` | `uav_vision/TargetCandidateArray` | 输入 | 全量候选，逐项转成不可变快照 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 输入 | 当前 mission-frame 位姿 |
| `/freedom/static_pointcloud` | `sensor_msgs/PointCloud2` | 输入 | 地图就绪、新鲜度、非空和数据布局门控 |
| `/navigation/mission_result` | `uav_mission/NavigationResult` | 输入 | execution bridge 的相关、幂等结果 |
| `/navigation/mission_command_raw` | `uav_mission/NavigationDecision` | 输出 | latched raw decision，含 deadline |
| `/navigation/mission_status` | `std_msgs/String` | 输出 | latched JSON 诊断，不是控制接口 |
| `/navigation/start_mission` | `std_srvs/Trigger` | 服务 | 满足 readiness 后创建新 mission |
| `/navigation/abort_mission` | `std_srvs/Trigger` | 服务 | 发布合同级 ABORT；不直接控制飞控 |

所有候选、result、pose、map、timer 和服务回调在同一 `RLock` 事务边界内调用运行核心，避免
“选择目标覆盖活动 SEARCH，但 CoverageRoute 尚未退休旧 decision”的交织窗口。
运行核心同时记录最后有效时刻；时钟回退会以该时刻原子退休当前路线并生成幂等 ABORT，
不会延后 510 s 硬返航。硬返航中断当前搜索时不累计 waypoint 失败，也不推进覆盖游标。

## 路线与投递语义

- 搜索使用非靶标坐标蛇形覆盖路线；成功才推进 cursor，失败达到上限才显式跳过。
- 只有 `red_cross、panzer、bridge` 能在 SEARCH 中断；目标事务结束后恢复同一 waypoint。
- `tent、pillbox` 仅在路线结束或时间预算兜底时参与排序。
- 三个槽位提交后立即 RETURN_HOME；510 s 强制返航，600 s 任务上限，所有 goal 高度不超过
  4 m。
- raw decision 的 `header.seq`、`decision_seq` 与 `deadline` 一起构成 execution fencing；
  execution bridge 必须拒绝旧序号或过期 decision，但真实 guarded release ACK 必须回报。

## 启动（只生成 raw decision）

```bash
roslaunch uav_mission navigation_mission_manager.launch
rosservice call /navigation/start_mission
```

该启动本身不会解锁、起飞或投递。没有 execution bridge 时 raw decision 不会变成 planner
goal；因此可用于合同/诊断接线，但不能表述为闭环飞行验收。
