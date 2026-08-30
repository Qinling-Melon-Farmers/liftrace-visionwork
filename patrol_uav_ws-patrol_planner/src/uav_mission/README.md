# uav_mission

任务/安全层最小包。当前第一项职责是把视觉释放证据与原有投递服务隔离：

```text
/uav_vision/release_evidence + align_mode + UAV pose
  -> release_permission_arbiter
  -> /mission/release_permission
  -> guarded_servo_proxy (/Servo)
  -> /legacy/Servo_raw
  -> /mission/release_result
```

`patrol_control` 仍调用原来的 `/Servo`，顺序槽位 `1/2/3` 和服务返回值语义不变。
原始 `actuator_pwm` 不在本包内启动。实机迁移时必须把原始服务重映射到
`/legacy/Servo_raw`，并让网关独占 `/Servo`；没有许可节点时应保持 fail-closed。

仿真使用 `release_guard_sim.launch`，其 raw 端是纯软件 mock，不访问 PWM。

## 导航组搜索 manager 集成

导航组 `liftrace-controlwork@5144aa8` 的 Python manager 和纯策略模块已经作为原始基线接入：

```text
target_search_manager_py.py
  -> search_policy / candidate_policy / search_types
  -> /navigation/goal_raw（集成 launch 中由 remap 得到）
```

`navigation_search_delivery_toudi4.launch` 不修改导航组状态机，而是在外围启动
`navigation_visual_delivery_adapter.py`，由适配器独占 `/fastplanner/goal`，并把新视觉候选接到
既有 ALIGN/释放安全链。旧临时 `coverage_search_manager` 不在该入口启动。

当前候选策略只保证地图点合法和 ID 未接受；类别权重只在视觉 `target_memory` 对当前新鲜
候选选择时生效。它不是持久全局队列，失败目标也不会自动重试。接口、headless 结果和
接续任务见 `docs/导航组任务链与新视觉联调HANDOFF_20260826.md`。

当前正式模型必须通过 `UAV_VISION_MODEL_PATH` 或 launch 参数传入；空路径会进入发布空检测
的 dev/sim 兼容模式，不能作为识别实跑。

## V-CL-06 全随机场正式入口

`navigation_search_delivery_random_field.launch` 以导航上游 manager
`liftrace-controlwork@5144aa8` 为冻结源，使用外围 profile selector 和 adapter 接入
全随机场；地图实验 `a68925d` 只通过 `nav_feature_profile` A/B，不代表 manager 更新。
正式 Gate 必须要求结构化报告存在，baseline 首轮命令为：

```bash
SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 \
top_level_scripts/sim_run.sh vcl06_random_seed11_baseline \
roslaunch uav_mission navigation_search_delivery_random_field.launch \
field_seed:=11 class_profile:=r2026 nav_feature_profile:=baseline \
enable_debug_image:=false record_debug:=false
```

缺少或无法解析 `gate_status.json`、状态不是 `PASS`，统一入口都返回非零。候选
`a68925d` 只在同 seed 的 30 秒预检与 90 秒固定路线 A/B 达标后再用于 600 秒 Gate；
未达标时默认保持 `baseline`。

30 秒预检不要求路线到点或完成投递，只检查随机场 footprint/真值、anchor、模型文件、
CameraInfo、`camera_init -> camera optical frame` TF、图像/地图/pose/ModelStates、实际接触
监测和 raw/planner goal 发布者所有权。单次 baseline 预检可运行：

```bash
SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 \
top_level_scripts/sim_run.sh vcl06_preflight_seed11_baseline \
roslaunch uav_mission navigation_random_field_preflight.launch \
field_seed:=11 class_profile:=r2026 nav_feature_profile:=baseline \
target_model_path:="$UAV_VISION_MODEL_PATH" gui:=false rviz:=false
```

完整 A/B 入口按 `baseline 预检 -> baseline 90 s -> a68925d 预检 -> a68925d 90 s -> 比较`
顺序执行，每一段都走独立 `sim_run.sh` run 目录：

```bash
export UAV_VISION_MODEL_PATH=/absolute/path/to/model.pt
bash top_level_scripts/run_navigation_ab.sh
```

90 秒样本关闭 candidate selector，实际执行导航组固定搜索路线，并记录 pose 最大断流、
场界、高度漂移、规划响应/失败、实际接触和共同路线进度墙钟。比较器要求 seed/world/真值/
模型/路线一致，双方 0 碰撞/越界且高度不超过 4 m；候选至少改善规划失败或高度漂移之一，
共同进度墙钟退化不超过 10%。`promote_candidate=true` 只是报告事实，不会自动修改默认值；
设置 `a68925d` 为默认仍须独立审查提交。

## 新 VCL06 manager 的一次性启动 gate

`navigation_mission_start_gate.launch` 是供后续 execution bridge include 的安全边界，
默认 `enabled:=false`。它本身既不启动 manager，也不发布 planner goal 或执行机构命令；
只有显式 `enabled:=true` 且以下 latched 合同同时成立，才会调用一次
`/navigation/start_mission`：

- random field 为 `READY/ready=true`，seed/profile 与 `11/r2026` 一致；
- `tent/pillbox/bridge/panzer/red_cross` 五个 target 的 expected/spawned/verified 清单一致，
  footprint 已验证，`random_field_truth.yaml` 位于本 run 目录并能解析出同一 seed/profile、
  五类与正 footprint；
- planner anchor 的 READY profile 与 `nav_feature_profile` 一致，三份模型清单一致；
- 新 `navigation_mission_manager.py` 状态为 `IDLE` 且 profile 为 `r2026`。

manager 仍独立执行 pose/map readiness 的第二道门；启动 gate 不订阅 pose/map，也不复制其
新鲜度策略。服务尚未注册或 manager 因自身 readiness 拒绝时，gate 使用 0.5--5 s 有界
指数退避；第一次成功后永久闭锁。观测状态为 latched JSON
`/navigation/mission_start_gate_status`，其中 `service_call_count=0` 是 READY 前的硬约束。

当前 `navigation_search_delivery_random_field.launch` 仍启动冻结的旧
`target_search_manager_py.py`，因此没有把新 manager/gate 塞入该入口。待 execution bridge
接好 `NavigationDecision/NavigationResult` 后，应在新的 VCL06 launch 中分别 include
`navigation_mission_manager.launch` 与本 gate，并显式传入：

```xml
<include file="$(find uav_mission)/launch/navigation_mission_start_gate.launch">
  <arg name="enabled" value="true" />
  <arg name="field_seed" value="$(arg field_seed)" />
  <arg name="class_profile" value="r2026" />
  <arg name="nav_feature_profile" value="$(arg nav_feature_profile)" />
</include>
```
