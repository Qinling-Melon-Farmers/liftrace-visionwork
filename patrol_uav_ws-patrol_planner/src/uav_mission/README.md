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
