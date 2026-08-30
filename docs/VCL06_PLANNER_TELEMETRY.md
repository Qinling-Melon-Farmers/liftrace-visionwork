# VCL06 Fast-Planner 目标状态遥测

## 目的

`plan_manage` 在不改变规划行为的前提下发布 `/planning/goal_status`，让任务层能够把
目标请求、实际规划目标和每次规划尝试关联起来。本变更不增加超时、有限重试或新的
到达判定，也不修改目标、阈值、轨迹生成和 FSM 状态迁移。

节点使用私有参数 `~goal_status_topic` 配置输出话题，默认值为
`/planning/goal_status`。launch 中可按需设置：

```xml
<param name="goal_status_topic" value="/planning/goal_status" />
```

## 消息契约

话题类型为 `plan_manage/PlannerStatus`：

- `goal_seq` 原样取自 `/fastplanner/goal` 的 `PoseStamped.header.seq`。
- `event_seq` 是当前 planner 进程内单调递增的遥测事件序号；进程重启后从 1 开始。
- `planning_attempt` 在每个新目标到来时清零，每次真实调用 kinodynamic replan 前递增。
- `requested_goal` 保存收到的目标；`effective_goal` 反映 preset 解析或避障调整后的目标。
- `distance_to_goal` 使用最新 odom 到 `effective_goal` 的欧氏距离；尚无 odom 时为 NaN。
- `FAILED_ATTEMPT` 只表示一次规划调用失败，明确为非终态；原 FSM 仍按既有逻辑重试。
- `TRAJECTORY_FINISHED` 只表示当前局部轨迹时间走完，不等价于任务层的速度/驻留到达判定。
- 活跃目标被新目标覆盖时，旧 `goal_seq` 先发布 `CANCELLED`，随后新目标发布 `ACCEPTED`。

唯一受支持的跨节点关联方式是：执行桥把导航决策的 `decision_seq` 原样写入
`/fastplanner/goal` 的 `PoseStamped.header.seq`，planner 再把它回传为 `goal_seq`。
旧发布者通常写入 0 或非唯一序号，不满足该关联契约；在执行桥完成适配前，消费方不得
把旧发布者产生的 `goal_seq` 当作唯一任务标识。

状态序列通常为：

```text
ACCEPTED
  -> PLANNING -> TRAJECTORY_READY
  -> REPLANNING -> TRAJECTORY_READY
  -> TRAJECTORY_FINISHED
```

单次失败会产生 `PLANNING/REPLANNING -> FAILED_ATTEMPT`，随后是否继续尝试完全沿用原 FSM。

## 验证

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace-controlwork-worktrees/vcl06-planner-telemetry/patrol_uav_ws-patrol_planner
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES=plan_manage -j1
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES=plan_manage \
  run_tests_plan_manage_gtest_planner_status_schema_test -j1
catkin_test_results build/test_results/plan_manage
```

该验证只构建消息、节点和 schema/序列化测试，不启动 ROS master、Gazebo、PX4 或执行机构。

## 初始化安全修正

`KinoReplanFSM::init()` 显式把 `trigger_` 初始化为 `false`。原实现会在收到首个 goal 前由
INIT 状态读取该成员，未初始化值可能导致未定义分支；本修正不改变目标、轨迹或控制输出，
只固定启动状态。

## Catkin 下游依赖修正

`plan_manage` 只生成消息并构建可执行节点，没有名为 `plan_manage` 的库目标。因此
`catkin_package()` 不再导出不存在的同名库。该修正确保 `uav_mission` 等下游包可以通过
`find_package(catkin COMPONENTS plan_manage)` 使用 `PlannerStatus`，不会在 CMake 配置阶段
因虚假库导出而失败。
