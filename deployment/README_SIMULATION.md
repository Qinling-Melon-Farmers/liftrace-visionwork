# R64仿真工程与模型包

包含今年整机源码、当前9.6m内净地图、55×55×40cm保守机架、相机/雷达装配、五个靶标、起降H、树与所需通用网格/材质、R64先导、十seed报告与11轮图表，并保留R60历史报告，附笔记本推理权重`runtime_models/merged_standard.pt`。`BUNDLE_MANIFEST.json`给出精确源码和权重来源。该包不自动启动仿真。

当前资源：`vision_ws/src/uav_vision_eval/models`与`simulation_assets/models`；旧D435i/fpv/tank对照资产仅在`simulation_assets/optional_models`，不参与今年五靶默认场景。历史报告中的world按历史布局保留，不能用它们替换新场地后继续沿用旧PASS。

外部运行环境仍需Ubuntu20.04、ROS Noetic、Gazebo Classic、PX4 SITL及其Gazebo插件、MID360激光插件及`mid360-real-centr.csv`、已有Python推理环境。不会将WSL系统、PX4完整源码/二进制或板端驱动塞进模型包。参考本机底座版本：PX4 `99c40407`、AstraDroneOpen `82dce3e2`，须包含`10020_gazebo-classic_iris_mid360`自启动配置。插件路径由`ASTRA_LIB`和`ASTRA_SIM_LIB`同时指定为对应已构建目录；扫描CSV由该插件自身寻找，移动插件二进制前需重新构建/核对其源码路径。

复现R64 seed11 PASS必须在PX4源码应用deployment/px4_patches/0001-initialize-task-reset-counters.patch并重新构建px4_sitl_default；单纯替换ROS源码不会修复飞控内部AUTO.LAND航向跳变。补丁基线、应用和板型约束见同目录README。

解包到WSL/Linux后在包根目录编译：

```bash
BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
export PX4_ROOT=/path/to/PX4-Autopilot
export ASTRA_LIB=/path/to/astra/sim_workspace/devel/lib
export ASTRA_SIM_LIB="$ASTRA_LIB"
export VISION_PYTHON=/path/to/existing/inference/environment/bin/python
export UAV_VISION_MODEL_PATH="$PWD/runtime_models/merged_standard.pt"
export GAZEBO_MODEL_PATH="$PWD/simulation_assets/optional_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

统一入口自动加入包内的当前机架和模型目录。仅在明确获得本轮启动授权后执行：

```bash
SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 bash top_level_scripts/run_competition_sim.sh field_seed:=11 record_camera_video:=true record_overview_video:=true
```

WSL宿主VHDX位于F盘时，附`SIM_STORAGE_GUARD_PATH=/mnt/f`只检查该盘剩余空间；日志仍全部写包根目录`logs/`。不录全场bag/录屏，包装器单实例并强制收尾。

R62已经实跑完成seed11建图、起飞、搜索、首个panzer投递确认并恢复搜索；仍未取得全场PASS。当前SITL关闭420秒提前返航，保留600秒总限时。机架物理尺寸/相机外参已按用户数据修正，通用动力学仍待标定。当前两门开口固定错列，尚未覆盖规则中未知随机开口。

R64完整seed11记录见docs/verification/r64_seed11/REPORT.md。overview.mp4是Gazebo服务端相机，无需gzclient GUI；对照时必须保持同一飞行源码/固件。当前固定俯视视角可能受隔墙遮挡，R64实跑于投后只移动无碰撞观测相机至(0,8.35,5)，具体时刻和姿态在报告记录；不移动飞机，不改变场内障碍。机载相机独立录制。失败删除俯视视频，保留机载视频及轻量诊断，不全量发布失败release。

本最终包补入矩阵报告、物理墙排除修复和当前随机场景工具。矩阵原始7/10完整PASS，5/7/8有靶板压墙；修复保持seed11坐标，但新随机布设没有重跑SITL矩阵。随机门/树箱仅离线几何和launch展开已验证。

投递完成后，如需与R64先导一样将无碰撞观察相机移到走廊视角，可在同一ROS环境运行：`python top_level_scripts/pan_overview_camera.py --run-dir /path/to/current/run`。只移动观测相机，脚本不控制飞机。
