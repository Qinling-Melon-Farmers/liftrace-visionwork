# 远程轻量计算与多seed完整仿真部署评估

2026-09-09。可使用远程Linux服务器；本次只评估，不连接服务器、安装环境或启动新仿真。沿用[时间与辅助相机计划](PLAN.md)，不能将几何演算当完整飞行验收。

用户本轮确认后续恢复无头策略。当前正式入口本就默认`gui=false`、`rviz=false`，无需改飞行算法或重跑；无头保留服务端相机渲染、机载录像及关键日志。[以本机AstraDroneOpen为来源的裸机配置方案](BARE_METAL_HEADLESS.md)。

## 资源如何选

| 工作负载 | CPU | GPU | 建议起步预算（工程估算，非已测最低配置） |
|---|---|---|---|
| 轻量仓路线/遮挡/视场/参数扫描、多seed策略比较 | 主要瓶颈；按独立seed/策略多进程并行 | 现有Python/A*代码不使用GPU | 16–32个高性能CPU核心、32–64GB RAM、本地SSD；已有较小服务器也可先低并发跑 |
| 图像回放、真实YOLO比较、以后训练/视觉参数扫描 | 解码、预处理、后处理仍用CPU | CUDA GPU适合当前PyTorch链 | 根据模型、批量及分辨率再定显存；不混用板端RKNN时延 |
| 当前Gazebo Classic11＋PX4＋LIO/FreeDOM＋Planner＋视觉完整seed | 单核性能和可用核心都重要，物理/建图/规划有CPU热点 | 建议NVIDIA GPU同时支持图形渲染与CUDA | 先用8–16个高性能CPU核心、32–64GB RAM、12–24GB显存试跑一套，再测可增加的并发 |

这些是用于试配的资源预算，不是供应商型号或并发保证。云服务器vCPU通常是逻辑线程，不能直接当同数量物理核心。只做轻量优化时优先增加CPU并发，不必为现有纯Python代码购买GPU；若只租一台兼顾两类任务，可从16个高性能核心/64GB RAM/一张16–24GB显存且支持OpenGL的NVIDIA GPU开始评估。正式采购/租赁前应在候选机器测单任务峰值与每单位成本吞吐。

## 为什么“不打开GUI”仍可能需要GPU

当前headless入口不启动Gazebo GUI，但机载相机图像仍由gzserver渲染，YOLO仍处理图像；因此关闭窗口不等于关闭图形管线。Gazebo Classic官方说明也要求带相机的gzserver具有渲染支持。[Classic说明](https://classic.gazebosim.org/tutorials?cat=gzweb&tut=gzweb_install&ver=7%2B)

远程主机不必接物理显示器，可以配置离屏/虚拟显示环境；对于现用Gazebo Classic11，应核对其OGRE/OpenGL及X显示路径，不能直接照搬新版Gazebo的OGRE2 `--headless-rendering`参数。纯CPU软件渲染和CPU推理可作为兼容性探针，但大规模完整飞行通常会付出显著时间成本，本项目没有该路径的完整基准。

GPU云主机需要确认图形能力，不能只看到CUDA或nvidia-smi成功就认定相机可渲染。容器默认compute/utility并不等同graphics/display；具体驱动挂载按实际OpenGL/X11路径配置。编码若要硬件加速另需video能力及合适编码器，本项目现有录像不能自动假定已使用它。[NVIDIA驱动能力说明](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)；[PyTorch GPU环境说明](https://pytorch.org/get-started/locally/)。

## 多seed如何并行

轻量仓现有几何脚本主要是Python循环。应按seed或参数组分发到独立进程/作业；仅给一个串行脚本更多核心不会自动加速。控制每作业内部BLAS/OpenMP线程，避免多进程各自占满全机；保留固定随机种子、独立输出目录和最终统一JSON/CSV汇总。

完整SITL现有sim_run.sh使用flock及进程名检查，stop脚本也按仿真进程收尾，属于单实例运行契约。不能删除互斥后在同一个宿主进程/网络空间同时启动多轮，否则可能串话题、争用MAVLink端口或相互停止。

未来宜采用“一作业一隔离容器/虚拟机、一容器一套sim_run”。每作业隔离PID、网络、ROS_MASTER_URI、GAZEBO_MASTER_URI、PX4/MAVLink UDP端口、ROS_HOME、临时目录/锁、PX4工作目录和日志挂载。避免host PID或所有任务共用同一个可写/tmp/ROS_HOME；收尾只能作用于该作业。容器GPU、图形显示连接和显存仍是共享资源，需要限制并发。没有容器/VM权限时，可在单服务器串行跑多seed，或每台机器运行一套；当前脚本尚未实现远程多worker调度。

建议分层流程：CPU节点大量轻量几何预筛 → 留出未用于调参的合法holdout → GPU节点少量完整SITL候选 → 固定版本多seed统计。所有失败计入，不能只把轻量预测可成功的布局当作随机验收集。

## 迁移内容与一致性

复现ROS Noetic/Ubuntu20.04用户态、Gazebo Classic11、匹配PX4基线与R64补丁、MID360仿真插件和扫描CSV、模型/材质/相机标定/权重；主机系统可以不同，但容器内依赖应冻结。优先使用当前仿真包，源码在服务器重新构建，不复制本机x86 build/devel作为通用安装。

轻量仓pyproject声明Python≥3.12；本地既有rl_drone Python3.9运行61测试已通过，是当前脚本的实际兼容记录。远端需明确选择受支持解释器/锁定依赖并跑同样测试，不能把这两个环境描述成完全一致。

保持16/21/6cm外参、实际机头/相机刚性关系、相同地图生成器及seed。图像、推理、ROS回调和规划竞争的改变会影响异步时序，所以同seed不保证轨迹逐点相同。完整验收需保存版本、参数、驱动/硬件、实际相机频率、实时因子RTF、推理时延、CPU/RSS/显存峰值、事件日志和Gate。

先单作业复现seed11，检查图像非黑屏、内外参与本地一致、LIO/规划/任务链时钟正常、录像可解码及收尾无残留，再试2、4作业逐级测量。保持物理步长、传感器频率、控制器频率和600s仿真任务时限；不能为跑得快降低感知频率或关掉相机后沿用同一PASS口径。墙钟watchdog应按实际RTF留足，而非用600墙钟秒杀掉尚未达到600仿真秒的慢作业。

总墙钟时间可粗估为 `任务数 × 单轮仿真秒 / (实测并发数 × 实测RTF) + 启动/构建/归档开销`。例如100轮、平均450仿真秒、4个独立作业且各RTF=1，纯运行理想约3.125小时；若各RTF=0.5约6.25小时。当前本地运行较慢不意味着GPU服务器一定达到RTF=1，上述只用于容量规划。

常规矩阵保留txt/json/csv与必要的视频，不恢复全场bag；每轮录像若大小为B，N轮预算至少N×B再加日志及余量，按首轮实测决定磁盘配额。板端RKNN并发与实机降落问题仍需在目标硬件验收，远程SITL不能代替。
