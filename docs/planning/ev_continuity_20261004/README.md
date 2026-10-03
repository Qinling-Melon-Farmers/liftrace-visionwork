# EV 连续性与高度跳变专项

本分支 `feat/ev-continuity-20261004` 从最新整机分支 `feat/r2026-competition-integrated@e5fb382d` 派生，工作区为 `/home/xhj/liftrace-worktrees/r2026-ev-continuity`。基线已完成 seed31/38 两轮整机 SITL，但这不等于复现或解决实机 EV 超时重置。本分支用于定位性能、独立 IMU 预测和恢复控制的集中研发；当前尚不能替换机载 EV 输入。

参考资料：用户提供的 `r2026-high-view-search/无人机高度跳变与整机恢复技术说明_20261003.pdf`，重点采用第 4–6 章及附录 B。原 PDF 留在原目录；未重新打包入 Git。实测依据见 [LIO 诊断](../../deployment/board_redeploy_20261001/LIO_POSE_DIAGNOSIS_20261003.md) 与其中链接的 ULog 复盘。

## 前线情况与分支边界

**2026-10-04 用户明确告知：试飞队员已将板端 FAST-LIO 改为并行处理。** 目前未取得对应提交或实际源码；不知道是 OpenMP 最近邻循环、异步订阅、独立 IMU 传播还是其他线程拆分，不能认为已经审查通过，也不能用本分支直接覆盖。收到版本后按下述对照表迁入并检查。

本次只修改新分支。未连接或部署板端，未改变 PX4 参数、任务 ABORT/接管/位姿保护、正式 EV 话题或飞行速度。`main`、研究、整机、板端参考等已有分支保持原头；新分支验证成熟后再向视觉整机与导航活动分支回流。导航侧承接对象仍为 `feat/high-view-liveness-20260919`，不恢复 VCL06。

## 对“平缓 EV”的具体理解

本轮实测问题的主要链条是 LIO/转发迟到 → EV 融合停止 → 恢复融合时 PX4 高度重置 → 旧高度设定点引出下降响应。原始 LIO 没有相同幅度阶跃。因此仅给 LIO 高度做低通，不能解决这个链条；重复旧位置并改成当前时间也会错误表达新鲜度。

采用两条配合的路线：

1. 降低 LIO 的无效工作和尾延迟，记录输入年龄、等待、计算及发布耗时。
2. 以慢路径真实完整状态为校正、MID360 原始 IMU 为连续输入，独立推算到最新实际 IMU 时刻；新的慢路径校正先重放到同一时刻，再平缓接续小校正。真实运动本身不做低通。

第三条必需的配套是坐标与控制恢复：即使连续性改善，长失援、坐标重建和 EKF reset 仍可能发生。需统一目标、地面参考、限高和恢复状态；不能删除位姿保护代替处理，不能机械地给所有永久目标加 `delta_z`。

## 本轮已实现

### FAST-LIO 确定性修复与观测

- 修复 `disA` 重复赋值、`disB` 未初始化；修复全盲区扫描线、Ouster 空/单点扫描、空 PointCloud2，以及平面搜索耗尽后的越界终点。
- IMU 处理显式返回是否推进到扫描末；空 IMU/初始化帧清空旧点云并跳过发布，避免将上一帧状态打上新时间戳。初始化阶段明确设置状态时间锚点，消除未初始化时间变量。
- 地图盒保留 20 m、`det_range=6`。将原移动步长限制在不触发区宽度的 90% 内：原 3 m 改为 1.8 m，避免移到另一侧触发带。24/30 m 等原本合理的步长不变；无有效内部区的参数在启动时明确拒绝。此处 **1.8 m 是地图盒平移步长，不是飞行或 H 检测高度**。
- Odometry 在发布前填协方差；按真实状态顺序取位置/旋转块，并将右扰动旋转协方差转为 ROS 固定世界轴。没有借此启用新的 PX4 速度/协方差融合。
- 发布队列从 100000 改为可配置 `publish/output_queue_size`，默认 1；原始雷达/IMU 接收队列暂未简单缩为 1，避免丢失去畸变所需 IMU 区间。
- 无订阅者时跳过 world/body 可视化点云构造；保留 PCD 路径和实际 FreeDOM body-cloud 消费。FreeDOM 有订阅时照常发布。
- 雷达回退时成对清理点云/时间队列及已推入标志；任何输入时钟回退会使新预测状态失效。没有声称这已实现旧滤波器的完整时钟重建。
- `/laserMapping/realtime` 输出完成扫描的回调、IMU/地图分割/下采样、ICP、地图插入、输出墙钟耗时，以及主线程 CPU、队列长度、最老扫描年龄、IMU 覆盖时刻。它不覆盖未完成扫描的等待，也不是全进程 CPU/所有 OpenMP 线程总耗时。
- `FAST_LIO_MATCH_THREADS=1/2/3/4` 可显式设置最近邻工作线程；0 保留原平台默认（x86 按核数、ARM 为 1）。未直接上全核或引入 `AsyncSpinner`，不冒充已复现前线并行版。

### 独立 IMU 预测原型（仅观察）

新增 `fast_lio/PredictionState`，在慢路径校正后输出同一时刻的姿态、位置、速度、两种 IMU 偏置、重力、18 维误差协方差、过程噪声、加速度单位缩放、IMU 时间偏移和源代次。S2 重力协方差用雅可比映射到三维；不能从旧 Odometry 的零速度字段猜测这些状态。

生产者默认 `~prediction_state_enabled=false`。独立 `ev_shadow.py` 订阅快照及原始 IMU，不读取 PX4 融合后位姿，也不持有 ICP 锁。输出仅限 `/ev_shadow/raw_pose`、`/ev_shadow/smooth_pose` 和状态；代码拒绝将这两个输出重映射到该命名空间之外。

- 时间戳来自最新已积分 IMU，绝不使用 `now` 冒充测量时刻；相同采样时刻不重复发布。
- 有界 IMU 缓存；延迟校正从其真实扫描时刻重放至最新 IMU，含区间边界插值。独立订阅导致校正先于对应 IMU 到达时，保留一条最新 pending 校正，覆盖到齐后接入。
- 小校正仅衰减同一时刻的校正偏移，真实惯性运动保留。分别检查原始校正跳差与累计平滑偏移，防止相反方向的校正绕过门槛。
- 同时检查最新输出年龄与最近雷达校正年龄。IMU 断档、大校正、时钟/坐标代次变化和长时间失去校正会停止输出并锁存故障；当前只允许地面明确重启，不提供空中自动恢复。
- 传播位置/姿态/速度/偏置/重力误差的局部协方差近似；对平滑的位置和角度偏移增加误差代理。该近似尚未做实测一致性标定，也未建模连续高频观测的时间相关性。
- 只导出位姿；没有把原始预测速度冒充平滑轨迹的导数，没有启用 EV 速度融合。

候选参数均集中在 `FAST_LIO/config/ev_shadow.yaml`，只是离线实验起点：输出 50 Hz、最大 IMU 间隔 25 ms、输出年龄 100 ms、校正年龄 300 ms、小校正位置 20 cm/角度 10°、接续时间常数 150 ms、缓存 0.8 s/最多 1000 条。**这些不是经实飞验证的保护值，不用于放宽原有拒收或 ABORT 阈值。**

## 后续板端并行版对照

| 项目 | 必须比较的内容 |
|---|---|
| 源码与入口 | 实际提交、编译宏、启动 yaml、现场 20 m 盒/特征模式、是否仍使用同一 EV 桥 |
| 并行方式 | 最近邻局部并行还是回调/ICP 并行；线程数与大小核分配；ikd-tree 后台线程重叠 |
| 共享状态 | `lidar_buffer/time_buffer/imu_buffer`、预处理器、`kf`、map 树、发布状态是否成组加锁；锁是否阻塞快路径 |
| 时间与丢帧 | 重复/乱序/回退、扫描与 IMU 完整覆盖、队列最老年龄；不能只删除旧雷达帧 |
| 坐标与参考点 | LIO 世界竖直、MID360 到机体的完整旋转/平移、重力、速度和协方差一致变换；不得猜装机角度 |
| 输出 | 是否真实新 IMU 推算；是否复用旧位置改 stamp；慢校正是否先对齐到相同时刻 |
| 负载 | 同一输入/相同地图质量，分别 LIO 单独、导航、视觉、录制；主/工作线程 CPU、墙钟、温度、P99 与最大 age |
| 飞控/任务 | EV 融合停启、全部 Z reset、最终设定点/限高、下沉响应、接管及恢复；零 ABORT 不是通过标准 |

至少保留四个独立配置：基线 A、仅前线并行 B、仅本分支性能修复 C、性能修复+预测观察 D。先对照原始/预测输出，不同时改变体素、EKF 高度参考和阈值。再决定正式 EV 输入的替换以及同一时间只能有一个发布者。

尚未完成的关键工作：原始传感器回放/板端负载验证；前线源码合并和线程安全核查；有界雷达扫描策略；完整世界/机体系标定；MAVROS/PX4 协方差/相关性及 reset 契约；低层 HOLD 生效与最终控制目标恢复；实飞复测。不能据当前原型宣告“根治”。

## 操作与复现

先在本机新工作树构建 `fast_lio`。此次借用已编译整机工作区的依赖 overlay，编译输出仍在新树；不启动 ROS：

```bash
source /home/xhj/liftrace-worktrees/r2026-board-frame-fix/patrol_uav_ws-patrol_planner/devel/setup.bash
cd /home/xhj/liftrace-worktrees/r2026-ev-continuity/patrol_uav_ws-patrol_planner
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES=fast_lio -j2
cmake --build build --target preprocess_inputs local_map_shift_test imu_process_validity_test -j2
devel/lib/fast_lio/preprocess_inputs
devel/lib/fast_lio/local_map_shift_test
devel/lib/fast_lio/imu_process_validity_test
```

纯数值测试和故障注入不运行 PX4/Gazebo：

```bash
cd /home/xhj/liftrace-worktrees/r2026-ev-continuity
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
OPENBLAS_NUM_THREADS=1 python tools/ev_continuity/test_predictor.py
OPENBLAS_NUM_THREADS=1 python tools/ev_continuity/offline_demo.py --out /tmp/ev_demo
```

后续获准进行地面观察时，在新编译 LIO 节点显式加入私有参数 `prediction_state_enabled=true`，核对 `prediction_imu_frame` 与原始 IMU frame 一致；再运行 `roslaunch fast_lio ev_shadow.launch`。这不是起飞命令，也不会代替既有 `lio_external_pose`。**不要在空中重启定位节点来加参数。**

观察录包只需小消息：`/livox/imu`、`/laserMapping/prediction_state`、`/laserMapping/realtime`、`/ev_shadow/raw_pose`、`/ev_shadow/smooth_pose`、`/ev_shadow/status`、原 EV、FC 位姿和 mode；保留现场轻量点云方式，不额外录全场大点云。慢路径真实性能 A/B 另需原始雷达输入，不能从预测观察包计算 ICP 的提速收益。

新包的离线重放无需 roscore（source 本分支已生成消息的 devel 后，用系统 ROS Python）：

```bash
/usr/bin/python3 tools/ev_continuity/replay_bag.py <new.bag> --out /tmp/ev_replay
/usr/bin/python3 tools/ev_continuity/bag_timing.py <old-or-new.bag> --out /tmp/ev_timing.json
```

`replay_bag.py` 缺少 IMU/完整状态会明确拒绝，不能自动给旧 bag 填零偏置。具体验证结果见 [本轮报告](../../verification/ev_continuity_20261004/REPORT.md)。
