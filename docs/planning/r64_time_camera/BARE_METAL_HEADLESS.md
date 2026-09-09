# 基于本机AstraDroneOpen的裸机无头环境方案

2026-09-09。用户确认后续采用无头仿真；没有提供服务器连接信息，本次仅核对现有入口及安装来源，不连接远端或运行安装器/仿真。

## 无头运行约定

当前`navigation_horizontal_search_vcl06.launch`已默认`gui=false`、`rviz=false`。保留gzserver、PX4 SITL、ROS、相机渲染和视觉处理；不启动gzclient、RViz或QGC。`SIM_NO_RECORD=1`关闭桌面录屏，不会关闭独立的机载相机录像。机载录像用`record_camera_video:=true`；俯视观察相机按任务选择`record_overview_video`，它同样在服务端渲染，不需要GUI。无头不等于纯CPU或没有图形运行库。

正式运行仍只用本工程`sim_run.sh`包装器和单实例收尾，不用Astra示例直接拉起多套进程。取得具体轮次授权后，显式携带`gui:=false rviz:=false`即可；本次没有启动任何轮次。

## 核对到的本机来源

项目实际名称/位置是`AstraDroneOpen`、`/home/xhj/AstraDroneOpen`，本机基线82dce3e2c8bdf09fb1aa2f2fb84ecc37886f7510。它可作为依赖和插件来源，再叠加当前竞赛工程；不是直接运行其示例就等价于R64。

- `scripts/env_sh/00_env_ubuntu_init.sh`包含ROS与桌面工具配置；`01_env_px4_init.sh`拉取PX4 v1.15.4、配置MAVROS/GeographicLib和PX4资源；`02_env_third_party_init.sh`包含第三方构建步骤。
- 本机`docs/01-安装脚本详解.md`、`docs/02-仿真源码详细介绍.md`实际为空文件；README列出的部分等价安装`.sh`在本地不存在，只有`.bin`。因此裸机安装不能只依赖README目录说明。
- 现有PX4环境脚本会清理build、打开gnome-terminal、编译并等待Gazebo启动，还直接调用pip、修改shell环境和配置QGC；`pc_example.sh`会启动roscore、仿真、自动解锁示例和QGC。它们不适合作为当前无人值守服务器入口，须在后续部署时提取所需依赖/构建步骤，避免安装阶段自动飞行。
- 本地Astra还有已有改动/额外目录，当前未触碰。远端应记录基线及实际必需的插件/补丁，不能声称仅clone同一commit就复制了本机全部状态。

## 裸机配置顺序（计划）

1. 确认服务器OS、架构、CPU/RAM、GPU及OpenGL能力、磁盘、管理员/容器权限。优先x86_64；宿主不是Ubuntu20.04时，可用Ubuntu20.04用户态容器复现ROS Noetic和Gazebo Classic11，不要求远端安装WSL。
2. 准备Git、GCC/CMake、ROS Noetic、MAVROS/GeographicLib、Gazebo Classic11开发库，以及当前工程用到的OpenCV4、PCL、Eigen、nlopt、yaml-cpp等。按当前工程版本构建，不为旧Astra参考模块另外全量安装OpenCV3或重复旧任务链。ROS系统Python和PyTorch环境分开管理；远端新环境创建在实际部署时确认，不在系统Python随意安装ML包。
3. 安装匹配宿主内核的GPU驱动及容器图形/计算能力；配置Classic所需离屏/X显示路径，确认实际OpenGL渲染设备，不能只检查CUDA。没有物理显示器不妨碍使用虚拟显示，但Xvfb本身并不保证GPU加速。不要把新版Gazebo的OGRE2无头参数直接套到Classic11。
4. 准备与本机R64对应的PX4 99c40407基线及子模块，核对其v1.15.x/Astra配置；应用`deployment/px4_patches/0001-initialize-task-reset-counters.patch`并构建`px4_sitl_default`，安装阶段只编译，不运行make启动仿真的目标。准备`10020_gazebo-classic_iris_mid360`等所需配置；不能只拉最新PX4替代记录版本。
5. 从Astra的`simulation/sim_workspace`构建需要的仿真插件，特别是`sensors/Mid360_simulation_plugin/livox_laser_simulation`；带上`mid360-real-centr.csv`及其路径依赖。在服务器重新编译并核对共享库，不直接搬本机devel二进制。
6. 解包当前R64仿真工程及模型/权重，先视觉后导航构建。当前地图、相机和机架由竞赛包提供；Astra提供仿真底座，不能用其原示例世界或旧相机覆盖当前9.6m地图和16/21/6cm外参。
7. 在每个作业环境显式设置`PX4_ROOT`、`ASTRA_LIB`、`ASTRA_SIM_LIB`、`VISION_PYTHON`、`UAV_VISION_MODEL_PATH`，不沿用本机`/home/xhj`默认路径；日志写该作业目录，不把本机WSL的`/mnt/f`空间检查直接搬到服务器。
8. 先做版本/库加载/包路径静态检查。得到新实跑授权后再进行单实例seed11检查：图像非黑屏、真实传感器频率、建图/起飞/巡航/投递、Gate、两类录像可解码及收尾。单作业达到本地口径后，按[远程隔离方案](REMOTE_SERVER_PLAN.md)逐步增并发。

轻量几何服务器无需安装这一整套ROS/PX4/Gazebo。它可先在纯Python环境按seed/策略分发CPU作业；完整视觉仿真另使用具有图形渲染和CUDA能力的GPU节点。多路CPU并非必需，高性能核心数量、单核速度、内存带宽和任务隔离更直接影响吞吐。

本次确认无头默认值、补充计划，没有修改飞行代码、启用安全返航或部署服务器。需要实际落地时再结合服务器地址/权限和配置细化命令。
