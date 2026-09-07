# 环境与构建

运行环境为 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11、PX4 SITL。构建使用 GCC 9、Catkin、Eigen、PCL、OpenCV 4、nlopt、yaml-cpp 和 MAVROS。先安装系统 ROS 开发依赖，执行仓库根目录的 `top_level_scripts/build_competition.sh`；脚本先编译视觉工作区，再编译导航工作区，并只引用当前 checkout 的 overlay。

本机外部仿真底座为 `/home/xhj/PX4-Autopilot` 和 `/home/xhj/AstraDroneOpen`。异机通过 `PX4_ROOT`、`ASTRA_MODEL_ROOT`、`ASTRA_SIM_LIB` 指定位置。需要编译 PX4 Gazebo 插件及 Astra 的 MID360 插件；本仓库不复制整套飞控和仿真器。场景中的基础模型由这些模型目录提供。

模型推理使用已有 conda `rl_drone` 环境，默认 Python 位于 `/home/xhj/miniconda3/envs/rl_drone/bin/python`，异机设置 `VISION_PYTHON`。系统 Python 仅运行 ROS 节点，不安装 ML 包。板端主路径为 RKNN/NPU；YOLO/PyTorch 仿真推理在笔记本运行。

权重通过 `UAV_VISION_MODEL_PATH` 指定。数据集、模型权重、build/devel、bag、视频及大日志不进入 git；联合实跑的报告与归档索引见 VALIDATION.md。

仿真统一由 `run_competition_sim.sh` 调用 `sim_run.sh`。调用方在获得启动授权后，仅在该命令设置 `SIM_RUN_AUTHORIZED=1`。包装器独占本机 ROS/Gazebo/PX4 资源，并检查启动前与收尾后的进程。无桌面时设置 `SIM_NO_RECORD=1`。不得在另一终端直接再起 roslaunch。


WSL 启动前还须检查 VHDX 宿主盘，使用 `SIM_STORAGE_GUARD_PATH`；本机按用户约定将 run、PX4 工作目录、分析和归档统一放在 WSL 项目 logs 内，不再写到其他盘。相机录图与本机示例见 [日志存储说明](verification/r55/RECORDING_FIX.md)。

VHDX 已通过离线 compact 实际回收 68.06 GiB。两次 WSL 服务恢复经用户 UAC 允许完成。R56 同轮原生目录重跑越过启动并完成三投，记录缓冲溢出 0；第一次外盘尝试的失败记录保留，不能据此承诺任意记录负载均稳定。

uav_vision_eval 现构建仿真接触代理插件，需要 Gazebo 开发包；它属于笔记本 SITL 评测工具，不属于板端飞行运行链。全量 build_competition.sh 面向本机联合仿真；板端包选择与实时验收按 HARDWARE.md 单独执行。

最终根因修复完整仿真 PASS；整机与 main 集成分支均独立构建通过。回放、全量归档和验证目录见 [最终报告](verification/r56_final/REPORT.md)，物理接触代理光学语义与竞赛建图 profile 见根因修复报告。
