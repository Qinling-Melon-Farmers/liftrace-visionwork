# 环境与构建

当前运行源码9cfb3e5的整机Catkin构建、239项任务/执行/Gate回归通过；先导完整PASS、十seed2/10，不能据构建成功宣称飞行鲁棒。见[验收](VALIDATION.md)。本轮未启动或改动实机。

环境：WSL Ubuntu20.04、ROS Noetic、Gazebo Classic11、PX4 SITL；GCC9、Catkin、CMake、Eigen/PCL、OpenCV4、nlopt、yaml-cpp、MAVROS。`top_level_scripts/build_competition.sh`先构建视觉再构建导航，引用当前checkout的overlay。

外部仿真底座为`/home/xhj/PX4-Autopilot`及`/home/xhj/AstraDroneOpen`，异机通过`PX4_ROOT`、`ASTRA_MODEL_ROOT`、`ASTRA_SIM_LIB`指定。仓库不复制完整飞控/仿真器和模型权重，权重由`UAV_VISION_MODEL_PATH`指定。

非ROS Python分析/仿真推理使用已有conda `rl_drone`；系统Python只运行ROS脚本，不安装ML包。离线飞控分析另用该conda中的pyulog。板端采用RKNN/NPU，不把笔记本PyTorch时延当作板端性能。

仿真必须由`run_competition_sim.sh`→`sim_run.sh`包装，调用方在明确授权后临时设`SIM_RUN_AUTHORIZED=1`。禁止同时再起第二套ROS/Gazebo/PX4；包装器正常/失败/中断均清零进程。

日志、PX4运行目录、分析和归档统一在WSL项目`logs/`；`SIM_STORAGE_GUARD_PATH=/mnt/f`用于宿主VHDX所在盘空间预检。默认关闭全场bag；`SIM_NO_RECORD=1`关闭桌面录屏。保留紧凑TXT/JSON/CSV、参数、局部故障地图和原生ULog。

历史离线VHDX压缩曾实际回收68.06GiB；本轮没有再次压缩VHDX，也不把删除GitHub Release说成释放宿主磁盘。R60全批次无I/O中断、全部收尾通过。真实设备入口与差异见[HARDWARE.md](HARDWARE.md)。
