# 九组专项运动优化选项（2026-10-05）

九组共享最新规划/控制限高与任务位姿过期处理、恢复速度换算。现场高度、速度、手动OFFBOARD和投递许可不变。适配代码已本地验证并同步Git；未部署机载电脑，未完成新补丁同版本动态验收。

## 选用方式

沿用各组的 run_trial.py 入口，可以附加：

- --motion-optimized：开启与高位研究相同的投后移动恢复、近墙按距离降速。仅有明确实测走廊几何时合并入口/共线中继；H扫描、转角、降高点不删。
- --survey-pattern rectangle、snake2、snake3：高位类专项用矩形、双扫描线、三扫描线。按该组现有 survey_xy 外接范围生成，不自动搬入正赛10×10配置；低空/H专项拒绝此参数。
- 不加这些参数沿用现场原航线和运动衔接配置。global直线偏好独立为 settings.yaml 的 planner_line_preference_weight，默认2.0、显式0关闭；不受 motion_optimization.enabled 影响。软偏好仍可为避障绕行。

仅校验示例（不启动节点）：

    python3 deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py high_priority preview --root "$PWD" --check-config --motion-optimized --survey-pattern snake2

YAML也可设置 motion_optimization: {enabled: true} 和 survey_pattern: snake2。各组原实投入口和 --real-release 用法保持不变；记忆/高速采集/H专项不会因此启用释放或投后恢复。

## 走廊补充

corridor_geometry需明确wall_axis、wall_coordinates、entry_waypoints，缺失时保留现有逐点航线。限制为0.90m目标、1.00m FC中心AGL上限；local Z=ground_z+AGL。到低入口后检查当前高度确实低于限高，并等待规划侧回执，再发走廊下一目标。H后缀恢复原高度上限。
示例中继优化不改变障碍膨胀，实机生成配置仍为水平0.25/上0.20/下0.10；仓库旧通用水平规划模板0.275由现场覆盖，并非本次回退。

## 已完成校验

- 实际板端分支九组、19种launch展开组合通过，未启动硬件；三种高位路线全部限于现场范围、保持原固定机头方向。
- 6项专项配置测试通过；任务层397项，393通过/4项整机专属跳过；控制27项在同源编译工作树通过。
- 整机入口另有413项通过及三种候选配置、6种启动组合通过。
- 编译/离线检查不能替代飞行验收；当前八轮SITL冻结在2b0678b9，没有混入这次约束修复。
