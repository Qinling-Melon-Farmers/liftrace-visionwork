# 09_high_speed_capture：高速飞行拍摄

2026-09-29已实现并完成离线测试/构建，9月29日已部署到飞机并完成构建/离线检查，未实飞或新增SITL实跑。本专项只巡航、记录图像和粗记忆，不中断去目标、不重访、不调用模拟或真实舵机。原八组默认0.5m/s保持不变。

## 航线与继承

- 沿用现场前方6m、左右±1.5m范围，固定起飞坐标+X向前、+Y向左。端点默认(0.8,0)和(5.2,0)，端点避开边界，机头方向保持原配置。
- 低空1.4m入场到(0.8,0)→原地规划升到2m→两次直线往返，共4条4.4m长边→返回已验证升降位置→规划下降到1.4m→原有出口降至30cm悬停→飞手落地。往返方向分别统计；每个方向各经过两次。
- 继承已知相机/槽位外参、自动地面基准、定位稳定后空白FreeDOM、四路视觉READY、静态TF、虚拟顶棚关闭、25/20/10cm膨胀、现场关闭附加障碍柱的诊断档。真实三维避障仍启用。该无柱场地配置不作为比赛禁越树的验收。
- READY后人工解锁，监督器沿用现场OFFBOARD/低空稳定后自动开始任务；不自动解锁。正常结束30cm悬停，接管后不抢回。未检出靶也能完成采集；航线不可达/超时仍会报未完成，不能跳过失败段假装采完。

## 操作

在完成源码更新、按部署总览构建并接好原设备后，从工程根目录执行。预览和flight不能同时运行。

```bash
# 只校验配置，不启动ROS节点
bash deployment/board_trials_4x4/09_high_speed_capture/start.sh preview --capture-speed 1.0 --capture-lighting normal --check-config
# 预览，沿用相同模型/外参/地图入口
bash deployment/board_trials_4x4/09_high_speed_capture/start.sh preview --capture-speed 1.0 --capture-lighting normal
# 现场飞手确认后：先0.5m/s对照，结束并退出本轮后再运行1m/s
bash deployment/board_trials_4x4/09_high_speed_capture/start.sh flight --capture-speed 0.5 --capture-lighting normal
bash deployment/board_trials_4x4/09_high_speed_capture/start.sh flight --capture-speed 1.0 --capture-lighting normal
```

较暗光照使用`--capture-lighting dim`；该参数只标记条件，不自动调相机曝光或灯光。`--model`/`--metadata`继承公共入口，必须成对匹配。`--capture-speed`只接受0.5或1.0，只能用于本专项；`--real-release`被拒绝，没有start_real.sh。

现场快捷入口也支持：`bash deployment/site_20260928/start_test.sh capture preview --capture-speed 1.0 --capture-lighting dim --check-config`；`capture`可写`6`。编号6是原现场五组之后的新增采集，并不是八组中的06_high_priority。

## 速度与结果

规划器速度约束及任务名义速度同步选0.5/1m/s；两档加速度均0.35m/s²。巡航前视/末级跟随距离同步0.5/1.0m，控制器轨迹命令接纳距离相应0.75/1.25m，避免下游仍按旧0.75m拒绝1m前视；精调/终止档沿用原值。跟随距离单位是米，设置1m不保证实速达到1m/s，需回放评价。

`capture_line_xy`和`capture_round_trips`集中在settings.yaml。每端至少内收中心边界0.30m、往返1–4次，单边至少满足v²/a+v×1秒且不短于3.9m；不能把这个6m配置直接塞进旧4×4场地。现场`--site-config`仍可继承范围/高度约束，但不覆盖本专项的直线航线；冲突配置会在启动前拒绝。

正常采完并人工落地后结果为`CAPTURED`，仅代表采集航线和结束流程完成，速度/识别验证为`PENDING_OFFLINE`。它不等于高速识别PASS，不要求凑够三类才能结束。原八组结果语义不变。

bag保留压缩图、CameraInfo、全视觉链、位姿/速度、规划、记忆及terminal_hover_status；板端独立MP4脚本已移除，视频留本机合成。运行目录`logs/board_high_speed_capture_<时间>/`；在本机用`tools/bag_replay`生成原片、叠加、轨迹和多画面视频，按[试验计划](../../../docs/planning/high_speed_capture_20260929/PLAN.md)只统计有效速度窗口。仿真工具仍只提供原八组4×4场景，本次没有为第09组伪造SITL通过。

[实现检查及阈值本地回放](../../../docs/verification/high_speed_capture_20260929/REPORT.md)
