# 2026 RoboCup 无人机导航与视觉集成

本分支准备合入 `liftrace-visionwork/main`，使用与 `feat/r2026-competition-integrated` 相同的比赛运行代码，同时保留原始工程资产。`build_competition.sh` 只编译当前导航和视觉依赖，旧工作区与机械 PWM 源码不参与本入口。需要只含今年工程的 checkout，请使用远端 `feat/r2026-competition-integrated` 分支。


本分支集成导航、建图、规划、视觉、任务管理及投递/降落执行逻辑。比赛入口保留机械组对接服务定义和仿真 mock，历史舵机驱动源码不参与此入口的编译或启动。

**最新 R56：同轮重跑三投/三恢复完成，第一门前第 4 航段超时，整场 FAIL；main 未合并。** [完整报告](docs/verification/r56/REPORT.md) · [Nodes only 图](docs/verification/r56/topology/index.html)。

当前验收状态见 [实跑记录](docs/VALIDATION.md)。历史成功记录和本场景的新验证分开列出；整场通过前不标记为比赛验收基线。

| 目录 | 用途 |
|---|---|
| `vision_ws/src/uav_vision` | 检测、几何精修、目标记忆、投递对准 |
| `vision_ws/src/uav_vision_eval` | 场景、相机模型、只读评测工具 |
| `vision_ws/src/camera_sdk` | 相机接入与标定参数 |
| `patrol_uav_ws-patrol_planner/src` | LIO、FreeDOM、Fast-Planner、任务及控制 |
| `top_level_scripts` | 编译、仿真、收尾、板端工具 |
| `docs` | 环境、接口、参数与验收说明 |

在 Ubuntu 20.04 / ROS Noetic 中执行；Windows 宿主命令使用 `wsl -e bash -c '...'`。

```bash
top_level_scripts/build_competition.sh
SIM_STORAGE_GUARD_PATH=/mnt/f \
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 \
  top_level_scripts/run_competition_sim.sh
```

示例中的 F 盘为本机 WSL 宿主盘；本机日志、PX4 工作目录及打包统一在 WSL 项目 logs 内，不再切到 E 盘或其他盘。异机按实际位置配置，原生 Linux 可省略宿主盘参数。权重单独提供，不放进 git。`SIM_NO_RECORD=1` 用于没有桌面录屏能力的环境。运行默认生成 `logs/<场景>_<时间>/`，保存 Gate、manifest、任务时间线和 ROS 日志，并在成功、失败或中断时停止本轮全部仿真进程。

流程为自动起飞、低空搜索、候选接近/升高取景、下降投递、恢复搜索、三投后走廊穿门、H 对准与自主降落。当前不启用槽位偏差补偿。

- [环境与外部依赖](docs/ENVIRONMENT.md)
- [相机安装与飞行参数](docs/CAMERA_AND_FLIGHT.md)
- [节点与机械接口](docs/INTERFACES.md)
- [代码来源](docs/SOURCE_REVISIONS.md)
- [任务与验收状态](VISION_2026_ROADMAP.md)

笔记本 SITL 的结果不代表香橙派实时链或实机已经验收。本分支保留原始机载目录和历史参考，当前比赛入口只使用上述编译依赖。

实机应用入口与设备接线见 [HARDWARE.md](docs/HARDWARE.md)。单独保留的 `feat/r2026-competition-integrated` 精简分支不包含旧搜索管理器、旧视觉节点或机械 PWM 实现。
