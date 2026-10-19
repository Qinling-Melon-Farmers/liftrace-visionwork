# 09_high_speed_capture：现场第六组高速拍摄

2026-10-03更新。18:11旧版直线往返已完成采集，实际速度/航迹另见本次[复盘](../../../docs/deployment/board_redeploy_20261001/GROUP5_GROUP6_UPDATE_20261003.md)。新版改用第五组的场内巡航路线和现有正赛速度档；本次只通过离线回归，尚未实飞。只采集、形成粗记忆，不中断去靶、不重访、不调用舵机。

## 路线和高度

现场入口`start_test.sh 6`和第五组共读`deployment/site_20260928/test_area.yaml`的`flight_area.staging_xy/survey_xy`，不再被直线往返配置覆盖。修改这份现场航线会同时作用于两组。

当前相对起飞点的巡航点（米）：

```text
低位入场 (0.6,0) → 原地升到2m
(0.8,-1.1) → (3.2,-1.1) → (5.5,-1.1) → (5.5,0)
→ (5.5,1.1) → (3.2,1.1) → (0.8,1.1) → (0.8,-1.1)
→ 原升降点 → 规划降至1.4m → 返实际起飞点 → 30cm悬停 → 飞手落地
```

第五组可在满足三类支持条件后提前中断；第六组会完整走完这些巡航点。它模拟全场搜索的转向和短航段，不等于执行三投、走廊和H降落。仍使用现场前方6m、左右±1.5m；保留2m高位/指令高度上限、1.4m低位、0.3m终点悬停。实际估计高度的超调/重置须从bag另外检查。

定位稳定后建图、已知外参、静态TF、虚拟顶棚关闭、25/20/10cm三维膨胀、附加障碍柱关闭、轻量bag与人工解锁后自动任务均继承。只有本专项速度调整，其他八组仍是0.5m/s。

## 速度

来源为整机候选分支`feat/r2026-competition-integrated`的`deployment/competition/field.example.yaml`（31c741d）：

| 参数 | 新默认 |
|---|---:|
| 规划速度上限 | 1.2m/s |
| 规划加速度上限 | 1.0m/s² |
| 巡航前视/末级跟随距离 | 1.0m |
| 精调/普通返航前视 | 0.4m |
| 终止/通道前视 | 0.15m |

`--capture-speed 0.5/1.0`保留对照入口（同一新路线、加速度1.0m/s²）。前视取min(速度数值,1.0)m；不要把前视距离当速度。现有到点验收、刹停和驻留没有删除，1.1m短航段未必达到1.2m/s，必须统计实测速度窗口。路线完成只报CAPTURED，速度/识别仍PENDING_OFFLINE。

## 操作

设备、定位和飞手按部署手册准备。以下校验不会启动ROS节点：

```bash
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
bash deployment/site_20260928/start_test.sh 6 preview --check-config
bash deployment/site_20260928/start_test.sh 6 preview --capture-speed 1.2 --capture-lighting normal --check-config
```

现场获准后使用`bash deployment/site_20260928/start_test.sh 6 flight --capture-speed 1.2 --capture-lighting normal`。仍等待飞手手动解锁；不自动解锁。光照标签可选normal/dim/unspecified，不调曝光。结束返起飞点30cm悬停，飞手接管落地。`--real-release`禁止，机构不参与本项。

第09模块的独立`start.sh`使用settings.yaml内同形路线；现场优先用上述第6组入口，保证始终引用与第5组相同的现场参数。旧`capture_line_xy/capture_round_trips`已移除；残留这些字段会在启动前明确拒绝，防止混用旧路线。

工作台第6卡片也支持1.2/1.0/0.5，默认显示1.2。录制仍只留轻量bag和状态文本，本机`tools/bag_replay`合成视频。本次没有启动工作台服务、仿真或实机飞行。
