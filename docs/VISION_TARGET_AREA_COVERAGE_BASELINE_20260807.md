# 靶标区域覆盖与视觉候选基线（2026-08-07）

## 1. 验证范围

运行：`logs/target_area_navigation_20260807_190817/`
场景：原始 `toudi3.world` 靶标区域
模式：无 GUI、关闭实际投递
结论：`PASS`

本 Gate 验证非靶标坐标覆盖、视觉发现、地图投影、stable ID、候选排序和安全返航。北区
走廊、末端对准、投递执行和降落不属于本 Gate。

## 2. 结果

```text
route_total: 12
visited: 12
skipped: 0
collision_count: 0
boundary_violations: 0
raw_servo_calls: []
planner_goal_publishers: [/coverage_search_manager]
mission_elapsed: 246.6 s
```

候选按权重输出：

```text
tank     id=9  weight=5.0
panzer   id=6  weight=2.5
bridge   id=5  weight=2.0
pillbox  id=0  weight=1.5
tent     id=1  weight=1.0
```

## 3. 地图坐标误差

真值来自 `uav_vision_eval/config/sim_target_catalog.yaml`，估计值来自该运行最终
`coverage_status.json`。以下为 XY 平面欧氏误差：

| 类别 | 真值 `(x,y)` m | 估计 `(x,y)` m | 误差 |
| --- | --- | --- | ---: |
| tank | `(0.2827,3.8559)` | `(0.2803,3.8332)` | 2.28 cm |
| panzer | `(-1.5886,3.0217)` | `(-1.5542,3.0424)` | 4.02 cm |
| bridge | `(-1.9031,-0.0227)` | `(-1.9212,-0.0348)` | 2.18 cm |
| pillbox | `(-0.6020,-1.0413)` | `(-0.4660,-0.8562)` | 22.97 cm |
| tent | `(1.0156,0.2563)` | `(0.9805,0.2111)` | 5.72 cm |

平均误差 7.43 cm，中位误差 4.02 cm，RMSE 10.83 cm。阶段 4 自动 Gate 未设置地图真值
误差阈值，因此 `pillbox` 离群值仍需在后续只对准、不投递的联调中复核。仿真投影当前使用
`ground_z=0.0`，而靶面真值高度为 0.115 m，也应作为误差来源单独验证。

## 4. 参考实现边界

本次验证使用仓库内临时 Mission Manager 驱动 `/fastplanner/goal`。其候选过滤、权重排序、
覆盖航线和恢复逻辑可供导航组参考，但不属于纯视觉包正式 API，导航组可自行决定采用方式。
视觉正式交付只承诺消息和话题字段语义，不承诺参考 manager 的节点名或内部状态机。
