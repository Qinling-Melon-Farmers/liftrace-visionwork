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
