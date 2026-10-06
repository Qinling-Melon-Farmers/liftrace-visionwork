# 历史 PX4 ULog 补录准备（离线，不连接飞控）

权威固件为 `30e763b6780061d70a14894e3e8b06e6a656f9b8`（历史 FMUv6C 日志），
不是本机 `/home/xhj/PX4-Autopilot` 的新版本。本目录只准备文件；不改参数、不重启、
不连接板端，不执行任何飞行命令。本轮未启动 ROS、仿真或实机。

## ROS 子进程交接

主入口使用已有 ROS 环境中的 Python3：

```bash
python3 deployment/low_hover_observation/record_diagnostics.py \
  --output <run>/recording \
  --config deployment/low_hover_observation/recording.yaml
```

`<run>` 可存在，`recording` 自动创建；`recording` 若已存在必须为空。
旧输出不会被覆盖。`--preview` 只验证 YAML 并打印计划，既不导入 ROS，也不创建输出。
`--duration 0` 默认由父进程结束；随仓 `max_duration_s: 900` 保留十五分钟上限。
如需纯外部截止，可配置 `max_duration_s: 0`；正数 duration 与正数 ceiling 取较小值。

父进程以 `Popen` 启动，轮询 `poll()` 和 `recording/recording_ready.json` 中的
`ready: true`。ready 文件是原子写入的，包含 `pid`、bag 文件名、开始墙钟和实际截止时间；
写入前 bag 已打开、writer 线程存活、ROS master 查询成功。
stdout 仅输出一次 `READY <ready文件路径>`，并立即 flush；没有周期 stdout。
推荐父进程将 stdout/stderr 重定向到该 run 的独立文件，避免无人读取的 PIPE。

这个 READY **只表示录制服务就绪**。不代表已在地面、定位稳定、解锁或切到 OFFBOARD。
录制器不判定/放行飞行；主入口自行门控并发布状态。
`recording.yaml` 的 `status_topic` 默认 `/low_hover_observation/status`，作为 JSON String
被动记录；可配置名称，并自动加入白名单。summary 留存最后一条 JSON 状态。

结束时父进程只向该记录器 PID 发送 SIGINT（SIGTERM/SIGHUP 同样处理），等待退出。
记录器注销自己的订阅，排空自己的有限队列，关闭自己的 bag，然后写 summary。
不要对整个进程组发信号，不要调用 rosnode kill、停止定位/控制或启动新的 ROS。
记录器的时间截止、磁盘错误、master 临时断连都没有控制指令或停止 LIO 的路径。
磁盘错误或关闭失败返回 1；参数/依赖/入口错误返回 2；正常信号或截止返回 0。
父进程始终检查 returncode，不能只凭历史 ready 文件判断还活着。

输出：`diagnostics.bag`、`recording_ready.json`、`bag_topics.json`、`summary.json`、
`diagnostics.jsonl`、`recording_config.yaml`、`rosparam_start/end.yaml`、
`rosnode_start/end.txt`、`rosgraph_start/end.json`。
参数 dump 和节点列表来自只读 ROS master API，与 rosparam dump / rosnode list 同源。
停止后 ready 文件更新为 false。磁盘完全不可写时 summary 写入也可能失败，需结合退出码。

bag 以消息到达回调时的 ROS 时间写入，原消息 header source stamp 原样保留。
JSONL 至多 1 Hz，含最近 source stamp、ROS/墙钟接收时间、source age、连续两次接收
的 monotonic gap、沉默时间、计数、全程最大 gap 和 age 极值、CPU/温度与状态。
source age 用相同时基的 ROS receive minus source；负值、零 stamp 和源时间倒退明确保留。
没有 header 的消息 age 为 null；不是把接收时间冒充源时间。
这些是 ROS 时序诊断，不能当作 PX4 延迟融合时域内的 EV 超时判定。

只订阅 YAML 的精确 topic 名称，并同时核对实际 wire type 与固化轻量消息类型白名单。
拒绝 image/compressed/JPEG/PNG/camera/cloud/pointcloud/lidar/livox/depth/scan 等名称，
拒绝图像、点云、Livox custom 及未知自定义类型，即使名称是 `/Odometry` 也会拒绝。
不使用 AnyMsg、正则、全 topic 录制、自动推断类型或 ROS remapping 参数。
可选 ESC 等话题缺失不阻止就绪，缺项/类型拒绝写入 manifest 与 summary。
队列上限同时限制消息数和序列化字节，单条消息也限长；溢出只丢录制数据并计数。
summary 的 written 是成功 bag 写入数；received 是本订阅收到数，不能证明上游没有丢包。
`/laserMapping/realtime` 按本树 FAST_LIO 生产源码的 `diagnostic_msgs/DiagnosticArray`
录制；`/rosout_agg` 按 `rosgraph_msgs/Log` 捕获 LIO/EV 时间戳丢弃等 warning。
`min_free_space_bytes` 默认 1073741824（1 GiB），在启动与每次 1 Hz 诊断时检查。
启动不足返回 1，不初始化 ROS、不写 READY；运行中不足则排空队列关闭 bag、正常返回 0，
summary 的 `stop_reason: low_disk_space` 明确说明。都不停止控制或 LIO。
父进程发现记录器退出后继续按自身逻辑保持、等待飞手；不得由录制器退出触发降落。

离线测试（已有环境，无安装）：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python deployment/low_hover_observation/test_recording.py
python deployment/low_hover_observation/record_diagnostics.py --output /tmp/hover-preview --preview
```

## 官方历史源码核对

- [logger 命令与启动逻辑](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/logger/logger.cpp)：
  `custom_command` 支持 `on`、`off`，没有 `topic` 子命令，因此 **不要使用 `logger topic add ...`**。
- [话题列表与文件解析](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/logger/logged_topics.cpp)：
  存在非空 `logger_topics.txt` 时直接加载文件，不再追加 `SDLOG_PROFILE` 默认/扩展列表。
  `logger on` 也不会重新加载 SD 卡文件；logger 初始化时才读列表。
- [255 订阅容量](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/logger/logged_topics.h)
  和 [uORB 多实例上限](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/platforms/common/uORB/uORB.h)。

QGC MAVLink Console 的人工地面操作：先 `ver all` 核对 revision，再 `logger status`。
确认已有 logger 正在运行、SD 文件后端可用后，可使用 `logger on` 开始地面记录，
随后 `logger status` 核对文件路径/运行状态；结束地面记录用 `logger off`。
这只覆盖日志是否开始，不补话题。飞行时不要发送 `logger off`，在默认 while-armed
模式它只撤销 override，仍由解锁状态决定记录；不能把它当作无条件 stop。
若 logger 未运行、配置未知或版本不同，停在检查阶段，不自动 `logger start/stop`、
改 `SDLOG_MODE`/`SDLOG_PROFILE`、重启或刷固件。

## 完整默认话题加诊断 profile

`default_topics_30e763b.cpp.txt` 是官方 BSD 源码默认函数的参考摘录，保留版权，
不参与编译。`logger_topics.txt` 由它生成，包含全部默认 topic 名称和默认
sensor/control 多实例配置，再加入/提频诊断；不是只有额外 EKF 话题的短列表。
每行严格 `topic interval_ms instance`，全部显式写 instance。
**只写两列会展开至 uORB 上限，多占订阅槽。** 此版本无 optional 行语法；不存在的
已编译话题也可能预占槽，不能照抄所有理论默认实例再无限追加。

随仓 profile 为 **232 个订阅，预留 4 个 mission 槽**。保留所有默认名称、传感器/
actuator 默认实例，EKF 仅明确覆盖 **0、1 两个实例**，与历史 primary=1 日志相容。
这不是宣称六个理论 EKF 槽都已保留：`profile_manifest.json` 明确列出未展开的
默认可选 estimator 2–5 槽。全展开硬件默认函数有 292 个理论订阅，已超 255；
不能在实际没有这些 optional 实例时强行全部预占。
**地面必须先核对实际 EKF 实例数量；超过两实例不能直接使用随仓 profile。**
生成器支持 `--estimator-instances` 并拒绝超过容量的方案，不会静默截断。
其他 `SDLOG_PROFILE` 位（如 replay/high-rate）也被文件覆盖；如果现场启用了这些位或
已有定制 logger 文件，先离线把原列表并入并核算容量，不直接覆盖现场。
SITL、SYS_HITL groundtruth、FIFO 高速数据不是本 profile 的范围。

补录项（记录整个 uORB 消息，字段不是单独 logger 命令）：

| 项目 | 话题及关键字段 |
|---|---|
| EV yaw/height 与其他高度来源 | `estimator_aid_src_ev_yaw/ev_hgt/baro_hgt/rng_hgt/gnss_hgt`；`timestamp_sample`、`time_last_fuse`、`observation`、innovation/variance/test_ratio、`fused`、`innovation_rejected` |
| 航向其他来源 | `estimator_aid_src_gnss_yaw`、`estimator_aid_src_mag`，结合 `estimator_status_flags` |
| EV 输入 | `vehicle_visual_odometry`：`timestamp_sample`、pose/velocity frame、position/q/variance、`reset_counter`、`quality` |
| 姿态 reset | `vehicle_attitude` 及各 `estimator_attitude`：`delta_q_reset`、`quat_reset_counter` |
| 位置/速度/航向 reset | `vehicle_local_position` 及各 `estimator_local_position`：`delta_xy/z/vxy/vz/heading`、对应 reset_counter |
| 融合与选主 | 默认 `estimator_selector_status`、status/status_flags、event_flags、innovations/test_ratios/variances/bias；EV pos/vel aid-source 两实例 |
| 控制/电源 | 保留默认 actuator 输出、姿态/位置/速度目标、RC、battery/system_power、ESC/timesync/telemetry（是否有样本取决于已存在的驱动/配置） |

名称/字段来自该版 [AidSource1d](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/msg/EstimatorAidSource1d.msg)、
[AidSource3d](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/msg/EstimatorAidSource3d.msg)、
[VehicleOdometry](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/msg/VehicleOdometry.msg)、
[VehicleAttitude](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/msg/VehicleAttitude.msg)、
[VehicleLocalPosition](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/msg/VehicleLocalPosition.msg)。
record ROS esc topic 不能自动开启 ESC 遥测，ULog 无 ESC 样本也不能证明电机健康。

## 离线准备、人工应用和恢复

生成器不下载源码、不安装依赖、不访问飞控。对 **已有精确 revision 的独立 checkout**：

```bash
python deployment/low_hover_observation/px4_logging/prepare_profile.py \
  --source <exact-historical-px4-checkout> --output <new-local-staging-directory>
```

HEAD 不等于完整 revision 会拒绝。也可将该 revision 官方原始
`logger.cpp`、`logged_topics.cpp`、`logged_topics.h` 下载至本地独立目录，用
`--source-files <local-directory>`；该模式由调用者确认来源 revision，不能使用本机新版本。
输出 profile/manifest 和三份原始源码到新 staging 目录，目标已存在则拒绝。

后续经用户明确授权，由操作者在**断电卸下的 SD 卡**上人工执行以下顺序：

1. 拷走原 `etc/logging/logger_topics.txt`（若有）与 SD 上历史 ULog，记录原文件是否存在；
   原文件缺失表示之前使用固件 profile，不能拿空白默认文件冒充原配置。
2. 核对实际 EKF 实例/原定制话题/原 profile 位后，将准备文件放到
   `etc/logging/logger_topics.txt`。**不改任何飞控参数**；安全弹出 SD 卡。
3. 已经运行的 logger 不会加载新文件，等待操作者之后按现场流程安排启动，不由脚本重启。
   在授权的地面准备时使用上面的 `logger status/on/status` 检查，再核对生成 ULog 的
   话题实例及字段、dropout/写盘负载。未核对 ULog 前不能宣称补录已生效。
4. 恢复时再断电卸 SD：原文件存在则完整恢复备份；原文件原本不存在则移走本次添加的文件
   （可改为 `logger_topics.low_hover.disabled`），恢复由固件 profile 选择话题。
   保留此次 ULog 与 manifest，不删除历史日志。恢复生效同样等待人工安排的下次启动。

本目录没有自动 SD 应用/恢复、SSH/SCP、MAVLink 发送或 reboot 脚本。
