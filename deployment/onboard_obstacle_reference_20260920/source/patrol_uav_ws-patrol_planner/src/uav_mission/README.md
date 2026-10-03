# uav_mission

导航任务、规划执行和投递安全层。正式 VCL06 入口为：

```text
navigation_search_delivery_vcl06.launch
  -> random_field_spawner + planner_anchor_spawner
  -> navigation_mission_start_gate
  -> navigation_mission_manager（唯一任务策略）
  -> navigation_planner_bridge（唯一 /fastplanner/goal 发布者）
  -> patrol_control 外部任务模式
  -> guarded release / mock Servo
```

该入口实现起飞完成后启动、覆盖搜索、候选中断、同 stable ID 的
APPROACH/CAPTURE/ALIGN/RELEASE/RECOVERY 事务、三次投递、三门走廊路线和最终 H 降落。
旧 `coverage_search_manager.py`、视觉侧临时 manager 和
`navigation_visual_delivery_adapter.py` 均不在正式图中。

为隔离前段搜索/投递波动，提供只用于 SITL 联调的后段入口：

```text
navigation_post_delivery_vcl06.launch
  -> MissionCore post_delivery 显式启动模式（payload commit 仍为 0）
  -> 同一版本化走廊路线、Fast-Planner、H 视觉和 LAND executor
  -> gate_scope=post_delivery 专用断言
```

该 Gate 只验收“走廊首段 → 三门 → H 新鲜证据 → 对齐 → 落地并上锁”，并拒绝任何
APPROACH/投递动作；其 PASS 不能替代正式入口的三投整场 PASS。正式入口的
`mission_start_mode` 和 `gate_scope` 默认值均为 `full`。

## 比赛 profile

正式 profile 固定为 `r2026`：

```text
red_cross=10.0
panzer=2.5
bridge=2.0
pillbox=1.5
tent=1.0
interrupt_top_k=3
```

因此 red_cross、panzer、bridge 都可中断搜索。tank 不进入比赛候选/投递队列。
`coverage_toudi3_random.yaml` 的历史文件名仅为 launch 兼容，内容只负责随机场生成几何；
搜索、权重和中断策略只由 `competition_profiles.yaml`、
`vcl06_random_field_runtime.yaml` 和 MissionCore 管理。

## 投递边界

```text
/uav_vision/release_evidence + frozen context + align mode + UAV pose
  -> release_permission_arbiter
  -> /mission/release_permission
  -> guarded_servo_proxy (/Servo)
  -> /legacy/Servo_raw
  -> /mission/release_result
```

`patrol_control` 仍调用 `/Servo`，顺序槽位 `1/2/3` 和服务返回语义保持兼容。
仿真使用 `release_guard_sim.launch` 的纯软件 mock；本包不启动真实 `actuator_pwm`、PWM 或舵机。

## 视觉运行依赖

导航仓不再复制或维护 `uav_vision` 源码。构建导航工作区前必须先 source 视觉组的
devel/install；正式任务链使用下列视觉消息：

```text
uav_vision/TargetDetectionArray
uav_vision/TargetCandidate
uav_vision/DropOffset
uav_vision/DropReady
uav_vision/AlignmentTargetContext
uav_vision/ReleaseEvidenceContext
```

正式 external launch 直接消费 `/uav_vision/detections_mapped` 中经过几何验证的
`landing_pad`，并以 `start_legacy_compat:=false` 关闭旧 `/detect/*` 视觉桥；legacy launch
仍默认保留兼容桥。当前精确双仓基线、overlay 顺序和最终 Gate 条件见仓库根目录
`docs/NAVIGATION_FINAL_BASELINE_HANDOFF_20260905.md`。

## 轻量策略仿真

导航组另设纯 Python 仓 `https://github.com/sakelier/liftrace-sim.git`，用于批量设计和比较牛耕搜索、
障碍/A*、Cue 中断、有限载荷投递及动态阈值策略。它不属于本 ROS 工作区，也不进入正式 launch；
其参数只能作为本仓 feature 分支的候选输入，仍须经过 ROS/PX4/Gazebo 完整 Gate 验证。记录时版本为
`main@18f6ee8`，详细模型边界见 HANDOFF 第 12 节。

## 当前验收边界

`r41` 已在视觉 `8e53bd0` + 导航 `3557215`、KS2A543、seed 11 上完成三投、三恢复、
三门、H、AUTO.LAND、落地和解除武装，ROS 任务时钟 `429.875 s`。后继收口候选修正了计时
口径并去除正式链旧视觉桥，但精确最终候选仍须在两个仓库收口后只运行一次完整 Gate；不得把
分阶段或旧 revision 的 PASS 自动继承给新组合。

上述结果不是全工作区构建 PASS。全量 Catkin 配置仍会报告仓内 `tool/cv_bridge/src` 缺少
`CMakeLists.txt`，以及 `local_sensing` 的 `cmake_modules` 依赖在当前环境不可用；这两项属于独立的
仓级构建缺口，不影响本任务链四包的定向验证结论。跟踪见导航上游 Issue #9：
`https://github.com/sakelier/liftrace-controlwork/issues/9`。
