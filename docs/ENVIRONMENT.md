# 环境与构建

R62 seed11已建图、起飞、搜索、首投并恢复；保留原生相机视频、JSON/CSV/ULog。当前是阶段运行验证，不是完整比赛或板端验收。参见[验收](VALIDATION.md)。

环境：WSL Ubuntu20.04、ROS Noetic、Gazebo Classic11、PX4 SITL；GCC9、Catkin/CMake、Eigen/PCL、OpenCV4、nlopt、yaml-cpp、MAVROS。`top_level_scripts/build_competition.sh`先视觉后导航，引用当前checkout，不复制笔记本build/devel上板。

外部PX4通过PX4_ROOT指定；MID360插件目录同时通过ASTRA_LIB/ASTRA_SIM_LIB指定。run_competition_sim.sh默认把当前checkout的机架模型与simulation_assets/models加入GAZEBO_MODEL_PATH，靶标根目录默认使用随仓资源，可由ASTRA_MODEL_ROOT覆盖。PX4/Gazebo/plugin与扫描CSV仍需安装，资源包不是系统镜像。更换主机的步骤见[BUNDLES.md](BUNDLES.md)。

非ROS Python使用已有conda rl_drone；ROS脚本使用系统Python，不安装ML包。SITL权重由UAV_VISION_MODEL_PATH指定，板端采用RKNN/NPU。两份本地包附明确选定的相应权重，但权重不进入git。

获当轮明确授权后才可由run_competition_sim.sh→sim_run.sh启动，单实例并在成功/失败/中断均收尾。日志统一留本项目logs；SIM_STORAGE_GUARD_PATH=/mnt/f只用于宿主VHDX所在盘空间预检。默认不录全场bag/录屏。没有再次进行磁盘清理或VHDX压缩。
