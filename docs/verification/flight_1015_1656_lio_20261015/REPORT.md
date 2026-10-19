# 2026-10-15 16:56 轮：LIO积压、EV转发缺口与位姿跳变

本目录仅由已关闭bag离线生成；未启动ROS节点、SITL、硬件或服务，未修改公共文档和运行代码。

## 结论

本轮LIO积压复发有直接诊断证据，不能只归因于bag写入延迟。尚无证据能够判定OpenMP三线程或大核绑定失效。持续飞行时存在4.90秒EV发布缺口；后续FC位置跳变触发任务ABORT。缺少ULog，不能把位置跳变进一步认定为某种EKF reset，也不能证明前一次EV缺口是后一次跳变的唯一原因。

## 文件和时间

- 输入：`1015-1656-投递H2D2-H降落vhD2-外场1702.bag`，121,510,586字节，mtime 2026-10-15 17:18。
- 内容时段：2026-10-15 **16:56:44.975～17:00:23.493（UTC+8）**，218.518秒。文件名“外场1702”不能代替包内时间。
- 已排除先前分析的1015-1501文件。
- 本报告时间均为bag起点后的秒数；header时间、bag接收时间单独计算，未用文件mtime推算飞行事件。
- 包中未记录`/clock`；LIO、EV、FC时间戳为相容的epoch秒，FC收包年龄中位约7.3ms。缺少完整参数转储，不能仅凭缺少`/clock`证明use_sim_time参数值。

## 飞行时间线

| 相对时间 | 事件 |
|---|---|
| 2.016s | laserMapping记录IMU Initial Done；初始未解锁时FC位置有大跳变，不计为空中事件 |
| 78.44s起 | FreeDOM开始连续处理点云的日志；仅能证明该时刻起有记录 |
| 83.94～86.13s | 视觉、控制、许可、任务节点陆续就绪；此时发生第一次短暂积压 |
| 84.18～85.43s | LIO自报输出年龄超过300ms，峰值540.8ms；雷达队列最多4帧 |
| 122.182s | 状态记录armed=true、MANUAL |
| 128.180s | 状态记录进入OFFBOARD |
| 135.888s | 控制器记录起飞完成 |
| 138.369s | 开始高位SURVEY，随后执行多段搜索 |
| 165.869～170.770s | EV消息间隔4.9006秒；LIO仍输出，出现明显积压及时间戳无效警告 |
| 168.398s | LIO自报输出年龄最大690.832ms，lidar_queue=5、imu_queue=108；全包imu_queue最大118 |
| 170.182s | FC状态进入ALTCTL；仅凭bag不能判断是飞手拨档还是飞控模式处理 |
| 172.183s | FC状态重新进入OFFBOARD |
| 191.930s | 任务下发high_view_full:DESCEND |
| 195.676s | 任务下发high_view_full:REVISIT |
| 199.625～202.712s | 又一段LIO输出年龄超过300ms，峰值415.4ms |
| 200.367～202.104s | EV消息又出现1.7365秒间隔 |
| 203.053s | FC位置在40ms header间隔内变化1.0174m，主要为Y+0.6781m、Z+0.7584m；LIO无相应跳变 |
| 203.062s | 任务报probe pose discontinuity |
| 203.135s | 控制器报height violation并保持/失效轨迹，current_z约1.946m、limit_z约1.649m |
| 203.146s | 下发ABORT，原因board_pose_exception:ValueError |
| 203.185／204.188s | FC状态分别记录ALTCTL、POSCTL |
| 210.181s | MANUAL、未解锁 |
| 216.465s | laserMapping记录catch sig 2，晚于落地上锁；不能写成空中LIO进程崩溃 |

本轮确实飞行并执行了高位搜索、下降和重访。记录中没有ALIGN/DROP命令，最终committed_slots=0；未进入走廊/H。不能用这份包评估本轮投递机构或H降落是否工作。

## LIO直接诊断

`/laserMapping/realtime`有2143条，包含输出年龄、各阶段wall时间、主线程CPU时间、雷达/IMU队列。没有worker数、线程数、CPU频率、温度或各线程CPU亲和性字段。

| 项目 | 正常待机2～80s | 160～171s |
|---|---:|---:|
| 输出年龄P50 | 30.7ms | 183.3ms |
| 输出年龄P95 | 38.5ms | 651.0ms |
| 单次迭代wall P50 | 26.3ms | 86.1ms |
| 单次迭代wall P95 | 33.8ms | 151.4ms |
| 主线程CPU时间P50 | 18.1ms | 41.8ms |
| ICP wall P50 | 21.7ms | 61.4ms |
| 雷达队列最大 | 0 | 5 |

全包自报输出年龄P50=37.7ms、P95约270.9ms、最大690.8ms；最大oldest_scan_age约691.2ms。仅看全包中位数会被较长地面待机稀释。

雷达源header相邻间隔中位约100ms、最大123.9ms；`/Odometry`在最严重的4.9秒EV缺口内仍持续输出。现象符合处理追赶输入、队列积压，不能说雷达停止供数。未记录原始`/livox/lidar`和IMU输入，不能统计驱动原始丢包。

`iteration_wall_sec`包含spinOnce及输出，`main_thread_cpu_sec`仅统计主线程，不包含OpenMP工作线程总CPU；wall与主线程CPU差值不能直接当作“CPU被抢占时间”，也不能用于反推并行线程数。较高ICP wall及间歇超过100ms的整轮wall会侵蚀10Hz处理余量；具体原因还需运行时线程/负载/频率证据。

## EV缺口与超时筛除

- `/Odometry`2143条，`/mavros/vision_pose/pose`2057条。
- 按相同header纳秒匹配，**86条LIO没有对应EV**，其中解锁状态期间72条。86条的bag接收年龄全部超过290ms，85条超过300ms；唯一略低于300ms者约299.6ms。
- EV最大消息间隔4.9006秒，发生于165.869～170.770s；其header间隔也是4.9000秒。
- 其它明显EV间隔：地面83.877～85.497s为1.6199秒；飞行200.367～202.104s为1.7365秒。
- `/lio_external_pose`在84.194、166.063、168.137、170.203、180.312、200.026、202.036s记录`LIO external pose has an invalid frame or timestamp`。

同header缺失集中在老数据、并伴随桥接告警，因此**强烈支持EV桥因时间有效性检查不再转发老LIO**。bag接收延迟与桥回调检查时间不同，不能用299.6ms边界样本否定300ms筛除。包中没有逐条drop reason计数，不能把86条全部写成已经被逐条确认的超时丢弃。没有证据显示FAST_LIO内部主动丢弃了86帧雷达输入。

## 位姿跳变及ABORT

203.053s的FC跳变为：ENU Δ=(-0.0083,+0.6781,+0.7584)m，header间隔40ms，长度1.0174m。相同时段LIO无相应离散跳变。其后约8ms任务回调记录位姿不连续，约92ms后下发ABORT。

位姿跳变直接触发任务退出有日志支持。此前EV中断可能影响飞控外部源融合，但bag没有PX4 reset计数、创新量、融合状态，因果链只能作为待查方向，不能确认“EKF reset已发生”。同样不能把跳变后的reported height越界当作飞机在40ms内真实爬升76cm。

## 优化是否生效：bag能证明的范围

- 包内历史deployment manifest写有`fast_lio_match_threads=3`、`lio_match_threads=3`、`omp_places={4},{5},{6},{7}`，但记录日期为**2026-10-08**；它是历史部署说明，不是此次运行时线程或绑核采样。
- 本轮rosout确有`target_detector_rknn detector CPU affinity: [4,5,6,7]`；这是**检测器**，不能移作LIO绑核证据。
- LIO realtime诊断证明本轮运行了带阶段性能诊断的版本；没有实际并行worker字段。
- 不能据此宣布“三线程未生效”，也不能宣布“三线程和大核绑定肯定正常”。主代理应核对正在使用的二进制、CMakeCache中的FAST_LIO_MATCH_THREADS、OpenMP链接及LIO实际线程CPU允许列表，并结合整机负载/热降频观察。
- 不建议用扩大EV年龄门槛掩盖已证实的LIO积压；先定位运行配置或算力瓶颈。

## 记录中的其它配置

run_metadata记录：full_mission/high_view_full；高位FC AGL2.0m、低位1.4m；速度上限1.2m/s、加速度1.0m/s²，前视1.0/0.6m；运动优化与续扫开启，虚拟柱开启、额外顶棚关闭，软件最大AGL3.2m。

投递仍为H2/D2、目标0.15m、窗口0.10～0.45m、下降目标变化速度1.5m/s；这不是本地正在准备的0.25m候选。模型为flight_5cls_new_video_v8_20261011_fp16.rknn。元数据中的历史source_head不能直接作为此次所有板端源码的精确revision。

## 产物和方法

- `inventory.json`、`run_metadata.json`：包信息和运行配置。
- `timeline.txt`／`timeline.csv`：稀疏事件时间线。
- `timeline_records.jsonl`、`pose_records.json`、`rosout.txt`：离线消息提取。
- `lio_basic.json`：复用现有tools/bag_replay/analyze_lio_flight.py输出。
- `lio_summary.json`、`lio_detail.json`：队列、年龄、wall统计、EV缺口及跳变。
- `lio_ev_timeline.png`：年龄、队列、耗时和位姿变化总图。
- `replay/index.html`：现有bag_replay工具生成的四路218.6秒／5fps回放。全部完整解码、尺寸、帧数、时长校验通过，详见replay/validation.json。
- `replay/REPORT.md`是工具通用报告，本文件是本轮LIO专项结论。

所有receipt age均为bag记录时间减header源时间，不能当作LIO计算时长；计算阶段数据来自LIO自身realtime诊断。没有额外启动任何在线ROS链路。

## 导航源码复核

0928 af599680仍保留显式OMP_PLACES={4},{5},{6},{7}、OMP_PROC_BIND=close、OMP_WAIT_POLICY=PASSIVE。top_level_scripts/build_competition.sh默认FAST_LIO_MATCH_THREADS=3；FAST_LIO的CMake缓存默认0在ARM上回到单线程。因此必须核查实际编译缓存和当前/proc/PID/exe、各线程Cpus_allowed_list，不能以历史manifest代替。2026-10-15本轮尝试SSH至192.168.3.126在握手阶段被远端关闭，未进入板端，未确认实时线程亲和性。

完整回放与提取文件保留在[本地分析目录](../../../试飞产物/flight_20261015_1656_lio_analysis/REPORT.md)，[播放器](../../../试飞产物/flight_20261015_1656_lio_analysis/replay/index.html)。本轮没有调整EV年龄门槛或恢复冻结EV预测。


## 关键积压阶段的耗时分解

165.5～168.5秒队列增长段，平均每次迭代121.7ms，超过约100ms的源帧间隔；其中ICP匹配/滤波平均88.6ms，回调10.4ms、IMU/地图预处理7.9ms、插图9.8ms、输出2.3ms。该段ICP P95为137.8ms，整轮P95为183.6ms。对比20～70秒地面段ICP均值22.1ms、整轮26.5ms，飞行负荷下处理余量明显下降。当前最大项是匹配/滤波阶段，数据不足以区分场景点数增长、调度争用、温度频率或并行构建/亲和性；不能仅凭此归咎JPEG编码、网络或检测器。

168.5～171.1秒平均迭代回落82.7ms后才逐渐清队列；即使当下单帧已变快，旧队列仍使EV年龄长时间超门槛。分段数据见[timing_breakdown.csv](timing_breakdown.csv)。原始livox输入未录制，所述100ms是LIO结果源时间戳间隔，不是驱动完整收包率。用户随后确认板端不在线，本轮停止连接，不宣称运行时三线程/大核已复核。
