# 前线 FAST-LIO 对比与匹配容器最小修复（2026-10-04）

本轮以EV专用分支 `feat/ev-continuity-20261004@cc46ca00` 为修改基线，主代理已完成review；源码与构建事实由本轮SSH取回后本地分析。EV代码未部署，未启动ROS/仿真。

## 来源与实际构建入口

文中B为 `/home/xhj/liftrace-worktrees/r2026-board-vision-tests`。原始证据留在B的 `logs/field_update_20261004/`，不把大日志或编译产物加入Git：

- `board_snapshot/`：11套FAST_LIO的mapping、CMake、IMU处理、预处理、ikd-tree及启动配置。
- `board_build_inventory.json`：实际CMake cache、flags、二进制大小/mtime、动态依赖；这是对初次只读结论中“缺板端构建资料”的补充。
- `board_snapshot/.ros/log/`：六次 `fastlio_mapping` 启动均为 `/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/devel/lib/fast_lio/fastlio_mapping`。例：`6d663ab0-bfd9-11f1-8555-0000a40bff7d/roslaunch-orangepi5-7180.log:90`（17:53:39）；同组rosout的源码路径也在根目录R64。

| 工作区 | 实际构建宏 | 二进制mtime（Asia/Shanghai） | 本轮判断 |
|---|---|---|---|
| `liftrace_board_trials_20260928` | `MP_EN`、`MP_PROC_NUM=3`、`-fopenmp` | 2026-10-03 22:54:24.778880 | 确有三线程匹配构建；不能据此归因于当晚R64运行 |
| 根目录 `liftrace_r64_onboard_405bda42` | 仅 `MP_PROC_NUM=1`，无 `MP_EN`；仍有 `-fopenmp` | 2026-09-25 15:51:33.583009 | 当晚实际启动路径对应单线程匹配构建；不应称已运行0928并行版 |
| `patrol_uav_ws` | 仅 `MP_PROC_NUM=1` | 2026-09-13 11:07:30.924818 | 旧单线程匹配构建 |
| `old/patrol_uav_ws-patrol_planner` | 仅 `MP_PROC_NUM=1`，cache和ROOT_DIR仍指向根目录R64 | 2026-09-10 18:11:45.320999 | 目录名不代表独立源码构建对应关系，不作为候选入口 |

mtime只是文件修改时间，不等于可追溯Git版本或完整构建时间。两种匹配配置都链接 `libgomp.so.1`：串行路径也使用 `omp_get_wtime()`；仅检查动态依赖不足以证明启用并行循环。快照未给出其余目录的构建资料，不把“未采集”写成“未编译”。

## 实际源码差异

各套CMake均编译 `src/laserMapping.cpp`、`include/ikd-Tree/ikd_Tree.cpp`、`src/preprocess.cpp`；mapping直接包含同目录 `IMU_Processing.hpp`。

| 快照相对目录 | mapping及入口差异 | 按源码在ARM64重新配置 |
|---|---|---|
| `liftrace-controlwork`、`patrol_uav_ws`、`Downloads/patrol_uav_ws` | mapping相同、driver1，旧构建依赖 | 匹配串行 |
| `Downloads/liftrace_r64_onboard_405bda42`、`old` | mapping与上行相同；补消息构建依赖和仿真XYZ预处理 | 匹配串行 |
| `liftrace-visionwork` | driver2及预处理测试入口，无地图导出改进 | 匹配串行 |
| `newfly/liftrace_r64_onboard_405bda42`、`newfly_push/...`、0928内`deployment/onboard_obstacle_reference_20260920/source` | 三者mapping相同；地图发布开关、订阅检查和限频；driver1 | 匹配串行 |
| 根目录 `liftrace_r64_onboard_405bda42` | 地图发布改进＋driver2 | 匹配串行 |
| `liftrace_board_trials_20260928` | mapping与根目录R64只有空白差异；CMake加入ARM64自动分支 | 8核时3线程 |

11套板端 `IMU_Processing.hpp` 内容完全相同；11套加EV的 `ikd_Tree.cpp` 也相同。没有新增并行IMU、异步回调或并行EKF。已有OpenMP只覆盖逐点变换、最近邻、平面拟合与残差；汇总/Jacobian/滤波更新及地图插入仍串行，ikd-tree后台重建是各版本共有的实现。

0928另在 `localization.launch` 设置 `OMP_WAIT_POLICY=PASSIVE`、`OMP_PROC_BIND=close`、可配置 `OMP_PLACES`；`run_trial.py:94` 在RK3588且允许CPU集合满足时选 `{4},{5},{6}`，否则用`cores`。这是绑定策略，不是CPU独占或运行时线程数证明。

## EV 已支持前线三线程

`cc46ca00`已有 `FAST_LIO_MATCH_THREADS=0/1/2/3/4`。显式 `-DFAST_LIO_MATCH_THREADS=3` 即产生 `MP_EN/MP_PROC_NUM=3`，并要求OpenMP可用。无需将0928的ARM自动三线程CMake片段覆盖回来，也无需加入AsyncSpinner。默认0仍保持ARM匹配串行；该值是编译配置，不是设置 `OMP_NUM_THREADS` 或ROS参数即可替代。当前分支的driver2和消息生成依赖必须保留。

## 本轮最小处置与生产回归

- `laserMapping.cpp`：将固定100000项标志/残差换为按实际扫描点数resize的 `vector<uint8_t>` / `vector<float>`，避免 `vector<bool>` 打包位并发写；在并行区前完成尺寸调整。缓存最近邻的迭代保留同一索引的选择状态，新元素默认未选中。
- 匹配汇总点云先resize到扫描点数，再按索引压紧，最后resize到有效点数。删除原先clear后越界写、固定点云预分配及两组不适用于动态容器的memset；不改滤波、匹配阈值、时钟reset或线程模型。
- `matching_buffers_test` 编译包含实际 `laserMapping.cpp` 并调用生产 `h_share_model()`，链接真实ikd-tree；重命名节点main且从不调用它，无ROS master或硬件。测试使用 `_GLIBCXX_ASSERTIONS` 捕获“capacity仍在但size为0”的vector下标越界。
- 10次生产匹配覆盖：空扫描、小扫描、缓存迭代、100017点、全拒绝、缩到3点、清空、重新增长到100019点，以及外参估计开关。已知z=1平面与交替离群点验证实际有效点数、压紧顺序、残差、Jacobian尺寸/有限值/法向；并检查实际OpenMP team大小。

编译及执行结果见本目录 [REPORT.md](REPORT.md) 的本轮追加节；所有结果均为笔记本WSL，不是板端提速或完整线程安全验收。

## 地图导出默认与剩余风险

源码 `map_pub_en=false` 且参数默认false；但当前 `uav_mission/config/mid360_hardware.yaml:36` 为true。board_trials `localization.launch` 读入该配置，`board_load.yaml`未覆盖这个开关。故不能笼统写“硬件入口已关闭”；实际还受 `/Laser_map` 订阅者与限频条件控制。本次未连接参数服务器，未确认当晚实际参数/订阅状态。

完整导出直接调用 `flatten(Root_Node, ...)`，没有参与ikd-tree后台替换/释放树节点的同步，内部Push_Down也会改节点；放在map_incremental后面不足以消除后台竞争。本轮按最小范围只记录风险，不改锁、不改配置、不启用导出。后续若需要完整地图，单独设计树内快照接口并验证锁顺序；默认关闭且无人订阅的路径不会执行该遍历。

## 纳入、拒绝与后续修复

| 分类 | 处置 |
|---|---|
| 纳入 | 保留EV可配置匹配线程及已有IMU有效性、预处理边界、协方差、输出队列和地图步长修复；本轮仅增加匹配容器安全和生产回归 |
| 可选待验证 | 0928绑核/等待策略；同输入比较1/2/3线程下的质量、P99年龄与整机负载，不能承诺三线程必然更快 |
| 拒绝 | 整包覆盖newfly/driver1；把0928构建当成当晚R64并行实跑；只凭libgomp判断并行；直接替换正式EV输入 |
| 后续修复 | 时间回退后的完整状态恢复、重复/缺口/覆盖检查；200000输入订阅队列与无界deque；完整地图导出同步；等待阶段诊断 |
| 本轮不实施 | AsyncSpinner、共享EKF多线程、时钟reset大改、板端部署和飞行参数调整 |

本轮diff与本机回归已检查；后续按现场安排处理运行入口和受控同输入负载对照。当前保留原EV输入及全部任务保护。
