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
