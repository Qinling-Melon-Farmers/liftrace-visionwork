# LIO—外部定位—PX4 位姿跳变排查步骤

**18点进展：** 新一轮 17:14:24 实飞 ULog 已取得并按 606 对位置数据对齐，确认 EV 超时停融→高度/位置先恢复并重置→航向稍后恢复，以及约 4.4° 姿态差造成的水平—高度耦合。见 [同固件 ULog 分析](PX4_JUMP_ULOG_171424_20261003.md)。下文前三轮的 ULog 仍缺，不用下午日志代替它们；当前板端已换回 `192.168.43.59`，日志地址不代表当前连接地址。

适用对象：10月3日板端地址 `192.168.3.126`，115933/122529/124630 三轮现有日志。结果来源与数值见 [实飞分析](LIO_POSE_DIAGNOSIS_20261003.md)。本流程先分析已有数据，再安排地面观测；本文与离线工具不启动 ROS、仿真或实机动作。

要分别回答两个问题：**飞控位置为什么不连续；LIO 数据为什么变旧并被桥接拒收。** 今天确定的是 LIO/EV 位置连续、飞控位置出现大差分，且三次差分均紧随 EV 中断恢复。不能由此排除 LIO 实时性，也不能在没有 ULog 时断定 PX4 执行了哪种重置。

## 1. 先取得今天三轮飞控 ULog

飞机停桨、保持未解锁，通过 QGroundControl 连接飞控。在左上菜单进入 **Analyze Tools / 分析工具 → Log Download / 日志下载 → Refresh / 刷新**，选择今天对应三轮日志，点击 **Download / 下载**。菜单文字随 QGC 版本变化，功能是列出并下载飞控日志；见 [QGC 官方操作说明](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/log_download.html)。保留原始 `.ulg`，不要点击 Erase All。

| ROS run | 本地解锁段（北京时间，状态采样边界约1秒） | 重点事件（飞控消息被 bag 收到） |
|---|---|---|
| 115933 | 12:01:17–12:02:34 | 12:02:04.135，ENU Z 增加 0.4351 m |
| 122529 | 12:25:49–12:28:49 | 对照轮：EV 最大接收间隔 476 ms，没有飞行中大差分 |
| 124630 | 12:49:43–12:50:45 | 12:50:22.491 / 12:50:25.664，ENU Z 增加 0.3715 / 0.4572 m |

按解锁、接管、落地及飞行时长匹配日志，不能仅凭 `.ulg` 文件名认定北京时间；它可能用 UTC，也可能没有有效日期。ULog 内一般是飞控启动后的微秒，bag 为 ROS epoch 时间，必须先用解锁/接管等共同事件估计偏移，再用两端高度曲线精细对齐。状态采样和链路延迟使“解锁对齐”不足以直接比较几十毫秒先后顺序；同时检查 MAVROS/PX4 时间同步记录。

本流程初稿时只有 14 个9月27–28日仿真 `.ulg`；后来取得的真实 `px4_log781.ulg` 对应下午 17:14:24 新一轮，前三轮仍待匹配。QGC 的 `.tlog` 也不能保证具有 EKF reset/fusion 字段。

## 2. 确认重置，再查触发重置的条件

查看事件前后各 3–5 秒，而不是只看全程平均值。优先检查下列字段；以 ULog 内记录的固件 revision 和实际字段为准，不套用其他版本的参数名/枚举。

| 检查项 | 重点字段或曲线 | 能回答什么 |
|---|---|---|
| 飞控位置重置 | `vehicle_local_position.z_reset_counter`、`delta_z`、`xy_reset_counter`、`delta_xy`、`z/vz`、有效性标志 | reset counter 同时变化且 delta 对应，才能确认位置重置；非零初值不是事件 |
| 估计器切换 | `estimator_selector_status.primary_instance`、`instance_changed_count` | 位置变动是否来自不同 EKF 实例切换；多实例的 aid/flags 必须与当时主实例匹配 |
| 飞控收到的 EV | `vehicle_visual_odometry.timestamp`、`timestamp_sample`、位置、`reset_counter`、`pose_frame` | 中断是否到达飞控；输入是否变旧、未来时间戳、坐标系/重置标记发生变化 |
| EV 高度融合 | `estimator_status_flags.cs_ev_hgt`；`estimator_aid_src_ev_hgt.fused/innovation_rejected/innovation/test_ratio/time_last_fuse` | EV 高度融合是否停止、恢复；恢复前观测与预测差异有多大 |
| 重置/停融事件 | 主实例的 `estimator_event_flags.vision_data_stopped/reset_hgt_to_ev/starting_vision_yaw_fusion`，与高度/位置/航向 flags 对照 | 确认恢复顺序与重置来源；局部位置降采样时优先参考事件时刻 |
| 其他高度来源 | `cs_baro_hgt/cs_rng_hgt/cs_gps_hgt`、气压高度及其 aid-source | 是否有参考来源变化、气压观测偏差或融合故障 |
| 传感器健康 | IMU clipping/振动、`fs_bad_acc_vertical` 等；固件对应 IMU 状态数据 | 是否有 IMU 饱和、振动或加速度估计问题；启动时和飞行时分开 |
| 当轮参数 | ULog 内 `EKF2_EV*`（含 `EVP/EVA/EVV`）、`EKF2_HGT_REF` 或旧版 `HGT_MODE/AID_MASK`、气压/测距/GPS 设置及飞行中参数变更 | 只采用当次日志配置；今天或昨天事后读回不能当成飞行参数快照 |

PX4 的局部位置是 NED，Z 向下；MAVROS 的位置为 ENU，Z 向上。因此今天的三次高度跃变，若确为同一次垂直 reset，ULog 的 `delta_z` 应大致为 **-0.435、-0.372、-0.457 m**，允许事件内正常运动和采样延迟差异。[PX4 字段定义](https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition)

计数变化超过1时，只能拿到最新一次 `delta_z`，不能用它解释中间多次重置的总位移。日志缺字段、被降采样或发生 logger dropout 时，应写“无法确认”，不能写“没有融合/没有输入中断”。旧固件可能只有 `estimator_status` 位掩码和 `estimator_innovations`，需要按该 revision 定义解码。

| 事件窗口内的观测 | 判断与下一步 |
|---|---|
| Z counter 变化，delta 对应，EV 融合在中断后重新启动 | 确认高度重置，支持 EV 恢复触发的解释；继续查为何中断、为何预测与观测偏离 |
| Z counter 变化且主 EKF 实例改变 | 先查实例健康和选择原因，不能统一归因于 LIO 中断 |
| 飞控 EV 输入连续、新鲜，但创新拒绝/高度源状态异常 | 优先查时间同步、坐标/安装外参、观测噪声和多高度源一致性 |
| LIO 新鲜、ROS EV 连续，只有飞控 EV 断流 | 查桥接、MAVROS/MAVLink 传输及飞控日志采样/丢失 |
| 飞控 ULog 位置连续，只有 MAVROS/导航位置跳 | 查消息转换、时间同步、坐标变换或记录错配 |
| counter 未变化但飞控 Z 仍大差分 | 不能简单排除融合修正；检查日志采样、创新、融合连续修正及发布链 |

第二轮无大差分是重要对照：比较相似 EV 间隔时的融合状态、创新、气压/预测高度差和实际配置，区分“有间隔”和“足以触发重置的状态”。

## 3. 现成的只读离线工具

新增 [analyze_px4_ev.py](../../../tools/bag_replay/analyze_px4_ev.py) 使用已有 `rl_drone` 中的 `pyulog/numpy`，读取封闭日志，汇总 reset/delta、EV 时间间隔与采样年龄、融合/选择器状态变化、当轮参数和日志 dropout。所有输出时间明确标为 **PX4 boot seconds**；它不自动推断根因、不自动把 ULog 对齐到 ROS。

把今天 `.ulg` 放在忽略的 `logs/lio_diagnosis_20261003/ulogs/` 后，在 WSL 内运行：

```bash
cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python tools/bag_replay/analyze_px4_ev.py \
  logs/lio_diagnosis_20261003/ulogs/*.ulg \
  --output logs/lio_diagnosis_20261003/px4_ev_summary.json
```

从 Windows shell 执行时按仓库规定用 `wsl -e bash -c '...'` 包装上述命令。已用保存的仿真 ULog 验证解析，这只验收工具，不是今天实飞的诊断结果。原有 [analyze_lio_flight.py](../../../tools/bag_replay/analyze_lio_flight.py) 可重新计算今日 bag 统计；需要查看 reset 窗口的原始曲线时，使用支持所有 uORB 字段的离线工具，而不只看 Flight Review 概览。[PX4 日志工具说明](https://docs.px4.io/main/en/log/flight_log_analysis)

## 4. 为 LIO 增加分阶段观测，定位延迟起点

今日 bag 缺原始雷达、IMU、队列长度和逐阶段耗时，不能继续仅凭 `/Odometry` 平均10 Hz区分雷达传输、IMU等待、计算与发布开销。后续在导航链权威源码增加轻量统计，与现有输出一起低频汇总；本次没有实施或部署这项代码改动。

| 观测位置 | 最少记录的量 | 判别方法 |
|---|---|---|
| 雷达/IMU 回调入口 | 消息采样时间、接收 age、点云数量、IMU 相邻 dt/最长间隔、回调预处理耗时 | 到回调入口就已变旧或存在缺帧：查驱动、网口、调度/回调队列；不能直接怪 ICP |
| 同步器/内部 deque | lidar/IMU/time 三队列长度、最旧扫描 age、扫描终点与末条 IMU 的时间差、等待原因/时长 | IMU 未覆盖扫描结束：同步等待；覆盖足够但旧扫描越积越多：消费能力或回调调度不足 |
| 每帧滤波与地图 | 去畸变、ICP/迭代次数、有效点/残差、地图盒移动/删除、增量插入耗时 | 确定哪段耗时与 age 尖峰同步；地图盒数学缺陷本身不能证明删除计时就是主因 |
| 输出发布 | world/body/map 点云变换和发布耗时、订阅者数量、Odometry 采样与发布时间 | 发布造成尖峰：只减少确认无消费者的输出；`cloud_registered_body` 可能供 FreeDOM，不能盲目关闭 |
| 板端资源 | 各进程/线程 CPU、运行频率/温度、内存、IO、网口错误/丢包增量 | 对齐上述窗口后判断算力竞争、降频、IO或传输问题；飞后空闲状态不能说明飞中负载 |
| EV 桥接/MAVROS | 接收/转发时间、拒收原因累计数、header、timesync offset/RTT | 分清 LIO 旧数据拒收与下游传输损失；节流警告条数不是拒收帧数 |

执行耗时使用单调时钟，ROS 消息 age 使用一致的 ROS 时基；两者不能直接混减。统计采用有界内存和低频汇总，避免逐帧 flush 反过来引入卡顿。雷达/IMU 全量 bag 只在短地面诊断窗口按需采集，例如10–30秒，并比较录制开/关的负载；不把长时全量录包设为飞行默认。

网口检查同时记录 `eth0` 链路、静态雷达子网/路由、`ip -s link` 计数与驱动点云/IMU时间戳；ping正常仅证明可达，不能替代持续传感器数据验证。雷达直连网口与 `wlan0=192.168.3.126` 的SSH地址分别核对。只有错误计数随运行增长、并与输入间隔对应，才支持网络丢包判断。

## 5. 地面单因素比较，最后才复核飞行

后续经现场安排的停桨地面测试，从定位链基线开始，依次加入规划/地图、视觉、现有录制，保持传感器安装、场地、配置一致。静止与受控小幅平移都要看：延迟尖峰、最旧扫描年龄、队列是否持续增长、匹配质量及实际 CPU 竞争。不要主动注入断流来测试尚在解锁状态的飞机。

首个算法候选是地图盒 `20→24 m`，保持 `det_range=6` 和其他参数不变。先确认往返移动消失、删除/插入耗时降低，再看 LIO age、点数/内存及空间精度是否改善；候选24 m尚未上板验收。没有观测支持前，不同时改体素、线程、特征模式和飞控融合参数。

`EKF2_EV_DELAY` 是测量时间相对 IMU 时基/实际采样时间的偏差，不能把75 ms接收 age直接填入；先核实时间同步和采样时间戳。[PX4 外部定位说明](https://docs.px4.io/main/en/ros/external_position_estimation) 也推荐30–50 Hz输入，但提频必须生成带正确时间的新估计，不能反复发送旧位姿或将旧 header 改为 now。

地面验收至少要求：时间戳无回退/未来错误；缓冲不持续增长；EV无成串超300 ms而被拒的输入；坐标与轴向响应正确；匹配质量没有因减负恶化。地面通过不保证飞行中不会重置，后续实飞需同步保留 ULog/轻量诊断并专门核对重置事件。先解释今日跳变，再讨论任务有界恢复；当前连续性保护与300 ms陈旧数据门槛不以“避免ABORT”为由放宽。
