# 2026 无人机竞赛整机工程

**R56 联合全流程 PASS（37/37）：三投、三次恢复、11 航段、三门、H 对准、AUTO.LAND、落地/disarm，零碰撞、零越界、零超高。** Gate 任务时长 182.924 ROS 秒；这是笔记本 SITL 功能验收，板端实时链和实机仍需独立验收。

[完整报告与 R4x 改动历史](docs/verification/r56_final/REPORT.md) · [全部参数/阈值](docs/verification/r56_final/PARAMETERS.md) · [Nodes only 图](docs/verification/r56_final/topology/index.html) · [成功记录下载](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/releases/tag/sim/r2026-r56-contact-map-pass)

本分支保留原始工程资产，通过明确的比赛包清单构建和运行；机械 PWM 与历史工作区不参与比赛仿真入口。只需今年整机代码时使用 feat/r2026-competition-integrated 分支。

| 目录 | 当前用途 |
|---|---|
| vision_ws/src/uav_vision | 目标检测、几何精修、记忆、投递对齐与证据 |
| vision_ws/src/uav_vision_eval | 仿真模型、评测、接触代理插件和图表工具 |
| vision_ws/src/camera_sdk | 相机输入和标定 |
| patrol_uav_ws-patrol_planner/src | LIO、FreeDOM、Fast-Planner、任务与控制 |
| top_level_scripts | 构建、统一仿真启动与收尾 |

在 Ubuntu 20.04 / ROS Noetic 中执行。Windows 宿主使用 `wsl -e bash -c '...'`；本机日志、bag、PX4 工作目录、分析和归档统一留在 WSL 项目 `logs/` 内，不再写到其他盘。

```bash
top_level_scripts/build_competition.sh
SIM_STORAGE_GUARD_PATH=/mnt/f UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1   top_level_scripts/run_competition_sim.sh
```

只有收到当前明确仿真授权后才执行启动命令。包装器独占 ROS/Gazebo/PX4，成功、失败与中断均收尾。`SIM_NO_RECORD=1` 关闭桌面录屏，原始相机话题仍进入本轮 bag。权重、bag、视频及大日志不入 Git。

搜索高度 1.40 m，投递/H 取景 1.60 m；正装机顶 MID360 的 IMU 到相机为下方 21 cm。三槽满后结束搜索并走廊返程，当前不做槽位偏差补偿。实际成功源码 cc899f2；合入 main 保留分叉和功能分支，验收 tag 为 gate/vcl06-r56-full-competition。

- [环境和依赖](docs/ENVIRONMENT.md)
- [相机与飞行参数](docs/CAMERA_AND_FLIGHT.md)
- [节点/机械接口](docs/INTERFACES.md)
- [实机入口与待验收项](docs/HARDWARE.md)
- [代码来源](docs/SOURCE_REVISIONS.md)
- [任务优先级](VISION_2026_ROADMAP.md)
