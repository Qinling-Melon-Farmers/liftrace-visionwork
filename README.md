> R59唯一一轮已收尾：8/8航点、两通口、低空H对准和AUTO.LAND触垫完成；终点平面垫名称遗漏导致原始Gate FAIL，ON_GROUND/disarm未确认。评测分类已修正但未重跑，十seed暂停。见[完整结果](docs/verification/r59_corridor_landing/REPORT.md)。

# 2026 无人机竞赛整机工程

接触代理的雷达过滤有建模前提，见[工程适用性复核与当前飞机图](docs/verification/r56_model_review/REVIEW.md)。真实护圈遮挡尚未验收。

**R56 联合全流程 PASS（37/37）：三投、三次恢复、11 航段、三门、H 对准、AUTO.LAND、落地/disarm，零碰撞、零越界、零超高。** Gate 任务时长 182.924 ROS 秒；这是笔记本 SITL 功能验收，板端实时链和实机仍需独立验收。

[完整报告与 R4x 改动历史](docs/verification/r56_final/REPORT.md) · [全部参数/阈值](docs/verification/r56_final/PARAMETERS.md) · [Nodes only 图](docs/verification/r56_final/topology/index.html) · [成功记录下载](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/releases/tag/sim/r2026-r56-contact-map-pass)

精简整机分支包含今年实际运行的导航、视觉和仿真代码；不含机械 PWM 实现及原始参考工作区。保留 Servo 服务定义和仿真 mock。

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

只有收到当前明确仿真授权后才执行启动命令。包装器独占 ROS/Gazebo/PX4，成功、失败与中断均收尾。`SIM_NO_RECORD=1`关闭桌面录屏，当前默认不录全场bag；紧凑记录器保留JSONL、CSV、参数与失败局部地图。权重、bag、视频及大日志不入 Git。

默认全场配置仍为搜索/走廊1.40m、投递/H取景1.60m；独立R59专项目标0.45m、H取景0.50m，已完成通口/H对准并触垫，但最终ON_GROUND/disarm未确认。早期0.65m草案未替换默认实机配置。正装机顶MID360的IMU到相机为下方21cm；当前不做槽位补偿。R56完整成功源cc899f2仍保留，main未合本轮FAIL。

- [环境和依赖](docs/ENVIRONMENT.md)
- [相机与飞行参数](docs/CAMERA_AND_FLIGHT.md)
- [节点/机械接口](docs/INTERFACES.md)
- [实机入口与待验收项](docs/HARDWARE.md)
- [代码来源](docs/SOURCE_REVISIONS.md)
- [任务优先级](VISION_2026_ROADMAP.md)
