# 2026 无人机竞赛整机工程

本分支集成导航、建图、规划、视觉、任务管理及投递/降落执行逻辑。机械组的舵机驱动实现不在本分支；保留对接服务定义和仿真 mock。

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
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 \
  top_level_scripts/run_competition_sim.sh
```

权重单独提供，不放进 git。`SIM_NO_RECORD=1` 用于没有桌面录屏能力的环境。运行自动生成 `logs/<场景>_<时间>/`，保存 Gate、manifest、任务时间线和 ROS 日志，并在成功、失败或中断时停止本轮全部仿真进程。

流程为自动起飞、低空搜索、候选接近/升高取景、下降投递、恢复搜索、三投后走廊穿门、H 对准与自主降落。当前不启用槽位偏差补偿。

- [环境与外部依赖](docs/ENVIRONMENT.md)
- [相机安装与飞行参数](docs/CAMERA_AND_FLIGHT.md)
- [节点与机械接口](docs/INTERFACES.md)
- [代码来源](docs/SOURCE_REVISIONS.md)
- [任务与验收状态](VISION_2026_ROADMAP.md)

笔记本 SITL 的结果不代表香橙派实时链或实机已经验收。原始机载目录和历史参考保留在来源分支及 git 历史中。

实机应用入口与设备接线见 [HARDWARE.md](docs/HARDWARE.md)。原始历史工程留在来源功能分支，精简分支不包含旧搜索管理器、旧视觉节点或机械 PWM 实现。
