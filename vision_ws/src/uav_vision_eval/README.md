# uav_vision_eval

`uav_vision_eval` 只负责独立 Gazebo 真值、记录和评分，不进入实机运行依赖。V-SIM-04
最小框架在单个 Gazebo 会话中执行固定 `seed=11` 的 23 个 trial：

- 静态：`tent/pillbox/bridge/panzer/red_cross × {1.2, 2.4, 3.6} m`，共 15 个；
- 动态：`panzer/red_cross × {1.8, 3.0} m × {0.5, 1.5} m/s`，共 8 个；
- 当前 `class_profile=r2026` 不含 tank；tank 资产继续由 `full` profile 和历史回归保留。
- 五类靶在同一世界中共存，属于 clutter 评测；`frames.csv` 记录每帧共同完整入画类别。
  recorder 额外记录所有类别的 CONFIRMED/selected 观测及 stable ID；unexpected/禁用类
  分开汇总。禁用类 confirmed 只作为误检诊断，任何禁用类 selected（包括 r2026 tank）
  以及任何不满足当前连续帧/地图/关联/年龄/拒绝原因合同的 selected 都是硬失败。

## 指标语义

- `P_confirm`：目标从完整入画到离开期间，`/targets` 出现满足当前连续帧、地图、关联、
  拒绝原因、年龄和 profile 完整准入的候选；
- `P_selected`：上述同一个 stable ID 在离开前发布到 `/selected_target`；
- `P_interrupt`：visual-only 固定为 `null`，只有导航 adapter 实际接受同 ID 并发生
  `SEARCH→APPROACH` 才能评分，不能用 selected 代替；
- `confirmation_exposure_sec` 使用图像/仿真源时间；`confirmation_processing_ms` 使用
  monotonic 墙钟；若 recorder 的图像回调晚于候选回调则显式记录 receipt reorder 并把该样本
  作为 0 ms 下界，不再因跨话题回调顺序丢样本。地图无效、TF 失败和地图误差分别报告。
- `frames.csv` 的相机位置和 ZYX yaw 使用与真值投影相同的 stamped pose 零阶保持规则：取不晚于
  图像时间戳的最近位姿并记录 pose 来源时间与年龄。动态线速度为相邻有效 pose 的三维位移除以
  严格递增的 pose 来源时间差，yaw 角速度使用最短有符号角差；静态、首个有效 pose、缺 pose、
  重复或倒退时间戳均保留空值并写明原因，不以 `0` 冒充样本。
- 动态横向偏移是相机在水平面相对计划起终点直线的有符号垂距；归一化值再除以整段计划路径
  长度，为无量纲量，路径左侧为正。静态、无效路径和缺 pose 时保持空值。

每次运行固定输出：`manifest.json`、`frames.csv`、`events.csv`、`summary.json`、
`report.md`、`vision_search_performance.csv`。manifest 记录 profile、world、模型、阈值、
CameraInfo、相机模型/RPY、内嵌场景/真值模型/anchor catalog、外参来源、视觉/导航 revision、
轨迹期望/实测时长与速度，以及 monotonic 接收 FPS 和图像源/仿真时间 FPS。
`summary.json` 与 `report.md` 同时按类别、高度和请求速度给出分层统计；
`vision_search_performance.csv` 仍保持一 trial 一行、原字段顺序不变，仅在尾部追加当前 trial
所属三类分组的统计列，便于直接比较 `sparse30` 各层结果。
`summary.json`/report/性能 CSV 明确分开 artifact set、trial measurement completeness 和
algorithm performance verdict：`MEASURED` 只表示正式测量完整，不等于性能 PASS。当前只按
已冻结合同检查处理 P95 `<=200 ms` 和地图误差 P95 `<=0.25 m`；P_confirm、P_selected 与
TF failure 门槛未冻结时写 `NOT_GATED`，不得自行补阈值。diagnostic 子集固定写
`DIAGNOSTIC_ONLY`，不能成为 Gate PASS。

recorder 先以 latched JSON 状态等待 CameraInfo、连续图像、完整真值、mapped detections、
targets 心跳，以及 `/uav_vision/perf` 中 detector 的 `OK`、backend 和模型路径一致；runner
同时要求 mapped 消息的 `completed_sources` 包含 target/circle/cross 三个正式分支，并等待
Gazebo 与 memory reset。每个 trial 内持续监控这些心跳，任何超过阈值的恢复性
断流也会留下 infrastructure gap 并令终态失败。trial_end 还必须等 image/truth/mapped/
targets/perf 的源时间 watermark 越过离场帧并完成短 quiet drain，runner 收到 recorder 完成
握手后才进入下一项；握手预算由 drain/quiet/status/write margin 同源推导。Gazebo/reset 服务
调用、`/clock` 停滞和过慢 ROS 时间均有 monotonic 截止时间，abort 也等待 recorder 写盘 ACK。
分类器采用 `queue_size=1`，负载下允许跳过部分相机帧；因此仅含几何分支的 partial fusion
bucket 会计入 manifest 诊断并退出评分，但不会冒充完整 heartbeat。只有含上述三个正式分支的
complete bucket 才能推进 mapped heartbeat 和离场 watermark，complete bucket 超时仍硬失败。
缺任一链路不会开跑。最后只有
23/23 trial 均真实进入并离开完整入画窗口、六项产物齐全且 manifest 关键值非空时才写
`MEASURED`；部分运行写 `INCOMPLETE/INVALID`，runner 非零退出。

## 无 Gazebo schema 验证

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
source /home/xhj/liftrace/vision_ws/devel/setup.bash
python vision_ws/src/uav_vision_eval/scripts/vsim04_dry_run.py \
  --matrix vision_ws/src/uav_vision_eval/config/vsim04_trial_matrix.yaml \
  --output-dir /tmp/uav_vision_eval/vsim04_dry_run
```

dry-run 只证明矩阵为 15+8 且六类产物 schema 可生成，报告明确不是 Gate PASS。

## 单会话仿真入口

必须通过统一 `sim_run.sh` 启动，并显式记录模型及两个仓 revision：

```bash
SIM_NO_RECORD=1 \
VSIM04_VISION_REVISION=<vision_commit> \
VSIM04_NAVIGATION_REVISION=<navigation_commit> \
UAV_VISION_MODEL_PATH=<best.pt> \
bash top_level_scripts/sim_run.sh vsim04_seed11 \
  roslaunch uav_vision_eval vsim04_stability.launch gui:=false
```

正式 `output_dir` 默认取 `$SIM_RUN_DIR/vsim04`，由 `sim_run.sh` 一并归档；没有该环境变量时
才回退 `/tmp/vsim04`。
`sim_run.sh` 对 `vsim04*` 场景还会二次检查六项产物、`summary.status=MEASURED` 和
`performance_verdict.hard_failure=false`；即使 `roslaunch` 在 required runner 失败后正常
关停并返回 0，统一入口仍会对缺产物、完整性错误和禁用类 selected 返回非零。普通性能
FAIL 仍保留完整测量产物，但 formal 入口返回非零；diagnostic 只有
`DIAGNOSTIC_ONLY` 且无硬失败时返回零。由于当前三项门槛未冻结，formal 即使已知阈值通过
也会以 `NOT_GATED` 非零结束，避免把命令成功冒充算法 Gate PASS。

扩展运营域矩阵可通过命名切片顺序执行。第一个参数固定为 `static25` 或 `sparse30`，其后
参数会显式透传给 `roslaunch`；不含 `:=` 的参数、未知 launch 参数以及试图覆盖
`matrix_file`/`trial_slice`/`trial_selector` 都会非零失败，不会静默忽略：

```bash
UAV_VISION_MODEL_PATH=<best.pt> \
VSIM04_NAVIGATION_REVISION=<navigation_commit> \
bash top_level_scripts/run_vsim04_surface.sh sparse30 \
  enable_failure_capture:=true failure_capture_max_frames:=30 \
  failure_capture_output_dir:=pillbox_surface
```

### 失败 trial 原始帧采集

采集器默认不启动，因此正式 23 项运行没有额外图像订阅或写盘负载。只有 diagnostic
selector 非空时才能显式启用，例如为 `static_pillbox_h3p6` 保存最多 30 帧：

```bash
SIM_NO_RECORD=1 \
VSIM04_VISION_REVISION=<vision_commit> \
VSIM04_NAVIGATION_REVISION=<navigation_commit> \
UAV_VISION_MODEL_PATH=<best.pt> \
bash top_level_scripts/sim_run.sh vsim04_diag_pillbox_capture \
  roslaunch uav_vision_eval vsim04_stability.launch gui:=false \
  trial_selector:=static_pillbox_h3p6 \
  enable_failure_capture:=true failure_capture_max_frames:=30
```

输出位于该 run 的 `vsim04/failure_capture/`：无叠加的 lossless PNG、同名逐帧 JSON 和
`dataset_manifest.json`。`failure_capture_output_dir` 可改为 run 内的相对子目录；绝对路径、
`..` 逃逸和非法路径片段都会被 launch 链与统一入口拒绝。逐帧图片统一以 `bgr8` 保存，JSON
同时记录 `source_encoding`/`saved_encoding`、active truth 以及同帧全部完整入画的
`scene_targets`，防止转 YOLO 标签时把共视靶标误当背景。原图和真值只接受
`(secs,nsecs)` 完全一致的配对。

采集可由非空 `trial_selector` 或命名 `trial_slice` 启用，二者严格互斥。多个 trial 共用
总帧数上限，预算按 trial 顺序确定性均分（余数分给靠前 trial），且总上限至少等于 trial
数；每个 trial 的样本按图像源时间覆盖预计时长：单样本位于 45%，多样本覆盖 0%～90%，
动态时长和目标中心时刻使用 runner 实际裁剪后的路径计算。启用采集后，runner 必须先等
CameraInfo 与一组 exact-stamp 图像/真值使采集器 READY；每个 trial 又通过独立 control
话题预告未来的 source-time `sampling_start`，收到 ACK 后才开始运动，recorder 的正式
trial event 契约不受影响。首个可投影帧作为可见窗口入口，动态出口由实际目标中心时刻
镜像并受轨迹末端约束，因此 0%/45%/90% 不再在低高度首帧处挤在一起。非末端 fallback
样本默认最大迟到 `0.25 s`，超限立即失败；末端 fallback 会显式标记且只允许最后一个
样本豁免迟到上限。

逐 trial 配额未填满、CameraInfo 首个 profile 非法或中途发生 frame/尺寸/内参/畸变变化、
任一声明 PNG/JSON 缺失、manifest 未
`run_complete`，均 fail closed 并使 `sim_run.sh` 非零。CameraInfo 在固定仿真相机中按
latched profile 使用，不要求其 header stamp 与每帧图像相同，但 frame_id、尺寸、有限且
正的 fx/fy 必须与图像契约一致。新握手、实际路径窗口和迟到字段使用 capture schema v3；
旧 schema v2 产物不会被新 checker 冒充通过。该数据只标记为 `sim-small-target` 诊断输入，不得当作实拍
数据或未经独立验证直接触发阈值/正式模型切换。

schema-v3 实跑 `vsim04_diag_pillbox_multih_v3_seed11_20260830_215821` 完成 pillbox 三高度
3/3 trial、9/9 帧，最大非 fallback 迟到 0.0544 s 且无 trial-end fallback；转换结果为
train 6/val 3，并强制 `training_ready=false`。该证据只证明采集和转换合同，不证明数据量足以
训练或模型泛化已经改善。

入口只启动 Gazebo、评测相机、视觉链、真值、recorder 和 trial runner；不会启动 PX4、
MAVROS、旧控制、`actuator_pwm` 或真实投递。历史合并态
`logs/vsim04_seed11_current_20260829_195506/` 的 13/23 和更早 9/23 继续保留作阈值基线；
当前证据为：

- `logs/vsim04_formal23_latest_seed11_20260830_220302/`：23/23、六产物完整，
  P_confirm=P_selected=22/23；processing P95=195.3 ms、地图 P95=0.0792 m、TF failure=0，
  唯一失败为 `static_pillbox_h3p6/raw_classifier`。终态为 `MEASURED`，性能 verdict 为
  `NOT_GATED`；
- `logs/vsim04_diag_static25_seed11_20260830_220647/`：25/25，P_confirm=P_selected=24/25，
  1.8/3.0 m 五类全通过，唯一失败仍为 pillbox 3.6 m；
- `logs/vsim04_diag_sparse30_retry3_seed11_20260830_223136/`：30/30，
  P_confirm=P_selected=25/30，processing P95=153.9 ms、地图 P95=0.1103 m、TF failure=0，
  motion/lateral 有效样本分别为 4045/4075，terminal errors=0。五个失败均为
  `target_memory_admission`：pillbox h1.2/v2、h3.6/v0.5、h3.6/v2，bridge h1.2/v2 和
  panzer h1.2/v2；tank/disallowed/policy-rejected selected 均为 0；
- `logs/vsim04_diag_pillbox_h3p6_1280_rerun1_seed11_20260830_223958/`：1280 输入使 pillbox
  3.6 m 达到 P_confirm=1，但共视 bridge 被 selected，故 P_selected=0；processing
  P95=449.3 ms，超过 200 ms。该 A/B 不推广，默认继续使用 640。

上述 formal/static/sparse 均为单 seed 一次覆盖，不能据此估计概率；visual-only 的
P_interrupt 仍为 `null`。2026-08-31 已完成 raw classifier/target-memory admission 六个边界点
固定 seed 重复和 camera-only 10 min，结果见本文末尾；采用门槛冻结前仍不扩 30-seed。

## 边界点独立重复与跨 run 聚合

`run_vsim04_repeats.sh` 用于重复验证单个边界 trial。每次重复都单独调用
`top_level_scripts/sim_run.sh`，使用唯一 scene 名并等待该 Gazebo 会话结束后再启动下一次；
不会在一个 Gazebo 会话内伪造重复样本。模型路径、矩阵、输入尺寸和视觉/导航 revision 都必须
显式给出。wrapper 自行 source `/opt/ros/noetic/setup.bash` 和当前 worktree 的
`vision_ws/devel/setup.bash`，调用者不需要预先 source；若当前 worktree 尚未完成视觉工作区
编译、overlay 不存在，wrapper 会在启动任何仿真前明确非零退出。先执行：

```bash
source /opt/ros/noetic/setup.bash
cd vision_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=uav_vision\;uav_vision_eval -j1
cd ..
```

然后运行：

```bash
bash top_level_scripts/run_vsim04_repeats.sh \
  --repeats 3 \
  --trial-selector static_pillbox_h3p6 \
  --matrix vision_ws/src/uav_vision_eval/config/vsim04_operating_surface_matrix.yaml \
  --imgsz 640 \
  --model-path <best.pt> \
  --vision-revision <vision_commit> \
  --navigation-revision <navigation_commit>
```

先加 `--dry-run` 可只打印三条完整命令、独立 scene 和环境元数据，不启动 ROS/Gazebo。实际执行
会尽量完成全部 repeats，即使中间一次失败也保留其 run，再输出：

逗号分隔的多 trial selector 会先生成可读短前缀，再限制为 48 字符并附加 8 位 SHA-256
短哈希；因此 6 个以上边界 ID 不会把 scene/path 无界拉长，截断前缀相同的 selector 也不会
静默生成同名 scene。

runner 会在启动首个 Gazebo 前实际加载 matrix，并按正式 `select_trial_matrix()` 合同拒绝未知、
重复或空 selector；视觉/导航 revision 也必须是 7～40 位 git SHA。`batch-id` 同样经过有界净化，
默认及显式 `--output-dir` 解析后都必须位于当前 worktree 的 `logs/` 内。已有批次目录默认拒绝，
避免覆盖上一次聚合；需要重跑时使用新的 batch ID。

每次 `sim_run.sh` 启动前后分别记录对应 scene 的目录集合，只接受集合差中唯一的新 run，不再按
mtime 猜测。批次目录中的 `batch_checkpoint.json` 在启动前及每个 repeat 结束后原子更新；若
runner 收到中断或编排异常，会为尚未执行项写失败占位，仍对已完成项生成聚合，并把 checkpoint
中的对应 repeat 标成 `UNFINISHED`、将批次终态写成 `INTERRUPTED`。当前未实现自动 `--resume`
和编排层单 run 墙钟超时；中断后可按 checkpoint 中的 run 路径使用独立聚合 CLI 恢复报告，
后续再补自动续跑。

- `repeat_summary.json`：每个源 run 的六产物、测量终态和算法 verdict，以及逐 trial 聚合；
- `repeat_trials.csv`：完成数、P_confirm/P_selected、failure_stage 分布、processing/map P95 样本；
- `repeat_report.md`：便于组间 review 的 Markdown 摘要。

聚合层再次检查六项产物、`run_complete`、完成 trial 数和视觉-only 的
`P_interrupt=null`。只有所有源 run 都为正式 `MEASURED + PASS` 时总判定才是 PASS；
`DIAGNOSTIC_ONLY`、`NOT_GATED`、缺产物及非终态均不会被命令成功掩盖，聚合命令返回非零。
因此单 trial diagnostic 重复即使测量完整，预期总判定仍是 FAIL/DIAGNOSTIC_ONLY，而不是 Gate
PASS。

`measurement_eligible` 与 `source_pass_eligible` 分开：六产物和测量终态完整、但算法硬失败或
`sim_run` 非零的 diagnostic 仍保留 trial 指标，同时源 verdict 与总判定保持 FAIL。聚合还从每个
`manifest.json` 核对视觉/导航 revision、模型路径、imgsz、profile、matrix、seed 和 selector；
配置元组不一致不得合并为 PASS。这里的 repeats 始终复用 matrix 中同一个固定 seed，只衡量相同
设计点的运行时波动，不是多 seed 独立样本，报告会显式写 `repeats_are_multi_seed=false`。

已有 run 也可在不启动 Gazebo 的情况下重新聚合：

```bash
PYTHONPATH=vision_ws/src/uav_vision_eval/src \
python3 vision_ws/src/uav_vision_eval/scripts/vsim04_repeat_aggregate.py \
  --output-dir /tmp/vsim04-repeat-aggregate \
  logs/<run-1> logs/<run-2> logs/<run-3>
```

## 600 秒 camera-only 稳定性入口

`vsim04_camera_soak.launch` 复用 V-SIM-04 的单 Gazebo、评测相机、独立真值和正式 Phase-D
视觉链，但不启动 PX4、MAVROS、导航/control 或 actuator。相机以固定高度沿场内确定性椭圆
循环，每圈只产生一次新的 `soak_loop_NNNN` ID；运行期间不反复 reset memory，避免把长时间
记忆与积压问题切碎成短 trial。camera 与 red-cross 不再由两个并发 `spawn_model` 进程碰运气，
而是由单节点串行调用 Gazebo。每个模型只提交一次不可取消的 spawn 请求，逐个核验 model state，
并通过 world properties 排除 `_0` 等同名前缀副本后才发布 latched ready；runner 未收到 ready
时禁止设置相机位姿或进入预热。

入口默认以 monotonic 墙钟连续运行 600 秒；ROS 源时间只驱动可复现路线，并继续执行倒退和
停滞硬检查。预热阶段先等完整流水线和真值，再执行一次 post-warm-up reset、等待全流新鲜，
随后在正式计时前再次 reset；正式计数、memory、selected 和 rolling window 均不含预热样本。
相机循环位姿通过 latest-only Gazebo set-state topic 发布，不再为每帧创建 service 线程；报告同时
记录命令位姿和 Gazebo camera link 实际位姿，按实际 pose 的 ROS 时间对齐路线命令，并对实际
跟随误差、年龄和相机姿态相对启动基线的漂移 fail closed。入口还自动检查：有效且全程不变的
CameraInfo 快照、场景中全部目标的真值存在且 `pose_valid`、测量期至少一次有效投影和完整入画、预期 ROS 进程、image/truth/camera pose/
complete-mapped/targets/detector perf 心跳、各流源时间单调、最大心跳间隔、输入与完整映射
吞吐、连续窗口 partial-only 和源时间积压趋势，以及 selected 的年龄、连续帧、地图、关联、
拒绝原因与 r2026 profile。任何 tank/禁用类/陈旧或不满足当前准入的 selected 都硬失败。
终态额外要求每个 required stream、有效真值和实际相机位姿在正式计时 epoch 内至少出现一次，
不能只复用 reset 前缓存。
`P_interrupt` 固定为 `null`，不订阅也不伪造导航接受事件。

必须通过统一入口启动：

```bash
SIM_NO_RECORD=1 \
VSIM04_VISION_REVISION=<vision_commit> \
VSIM04_NAVIGATION_REVISION=<navigation_commit> \
UAV_VISION_MODEL_PATH=<best.pt> \
bash top_level_scripts/sim_run.sh vsim04_soak600_seed11 \
  roslaunch uav_vision_eval vsim04_camera_soak.launch gui:=false
```

调试时必须使用 `vsim04_soak_smoke*` scene（例如
`vsim04_soak_smoke35_seed11`）并传 `duration_sec:=35`；成功结果只写
`status=SOAK_MEASURED + qualification_status=SMOKE_ONLY + soak_600s_pass=false`，不得作为
600 秒 PASS。只有请求且 monotonic 墙钟实际跑满至少 600 秒、全程无终态错误时才写
`qualification_status=SOAK_600S_MEASURED` 和 `soak_600s_pass=true`。运行固定输出同名六产物；
缺产物、墙钟未跑满、CameraInfo 无效/变化、进程/心跳/源时间/吞吐/积压/partial/selected
审计失败均非零退出。manifest 保存 width/height、distortion_model、K/D/R/P 和 frame_id；
CameraInfo 是启动准入与固定 profile，不作为周期心跳。`sim_run.sh` 会按 scene 严格匹配
`SMOKE_ONLY` 或 `SOAK_600S_MEASURED`，并额外核验 `artifact_set_complete=true`、空 errors、
`P_interrupt=null` 和实际墙钟；`vsim04_soak600*` 搭配短 duration 会明确失败。终态先冻结回调并
生成快照，`summary.json` 最后一次性落盘，避免半套产物被误收为成功。

soak 入口把 Gazebo `LinkStates` 派生的最新相机位姿限到 20 Hz、发布和订阅队列均为 1；该频率
高于 10 Hz 路线与相机评测所需频率，并写入 manifest。通用 `ground_truth.launch` 默认仍为 0
（不限频），不会改变既有矩阵入口。限频只移除冗余评测回调，不放宽图像/真值/位姿心跳、实际
pose 误差、完整映射吞吐或 selected 准入门槛。
`selected_target` 同样按最新状态使用 queue 1，并在 events 中记录接收源时间、`last_seen` 和实际
年龄；因此评测器不会把自己队列中已被新消息替代的历史 selected 当成当前导航输入。

无 Gazebo 的纯函数回归：

```bash
python3 vision_ws/src/uav_vision_eval/scripts/vsim04_soak_assertion.py
```

## 2026-08-31 最终实跑证据与适用边界

### Camera-only 600 秒

正式 run 为：

```text
logs/vsim04_soak600b_seed11_20260831_011055/
```

- 视觉 revision：`8b3b88cd321469e3b61b6127ec2574d770848109`；导航来源字段仍冻结为
  `5144aa8f536bdcd214aea2f39ada558383b3bcb0`，但本入口没有启动导航；
- `SOAK_600S_MEASURED`、wall 600.024 s、ROS source 564.863 s、六产物非空、
  `artifact_set_complete=true`、`errors=[]`、`P_interrupt=null`；
- 输入 9012 帧（15.019 FPS），complete mapped 8002 帧（13.336 FPS），partial bucket 1003；
- 59 个完整 10 s 健康窗口：input 9.906–19.549 FPS、complete mapped 7.730–17.969 FPS、
  最大 lag 0.207–0.587 s、partial 比例 2.312%–24.837%，四类坏窗口 streak 始终为 0；
- 各必需流最大 heartbeat gap 为 0.1346–0.4478 s；实际相机位姿 10289 条，最大路线跟随误差
  0.0782 m、相对起始姿态漂移 0；有效真值 9010 条、有效投影 3668 条、完整入画 1756 条；
- selected 共记录 1702 次（panzer 756、red_cross 942、tent 4），这里是状态重复发布次数，
  不是 1702 个独立目标；最大 selected age 0.467 s，tank/disallowed/stale selected 均为 0。

该 PASS 只关闭“固定五目标、不含 tank、Gazebo 合成 D435i、笔记本 Ultralytics 的 camera-only
视觉链 10 min”项。run 名含 `seed11`，但 soak manifest 没有 seed 字段，场景也不是随机场；
tank=0 只说明本路线没有选中 tank。它不证明 OrangePi/RKNN、新实物相机、导航接受、
`SEARCH→APPROACH`、三投、返航或落地，不能写成 V-CL-06 PASS。

### 六个边界点三次独立重复

聚合目录：

```text
logs/vsim04_repeat_aggregate_boundary6-seed11-r3-final-307ac5c4/
```

三次源 run 均 `exit_code=0`、六产物与 terminal measurement 完整、配置一致，源 verdict 均为
`DIAGNOSTIC_ONLY`。聚合的 `is_gate_pass=false` 是预期诊断语义，不是 wrapper、Gazebo 或产物
编排失败。三次重复始终复用 matrix 的固定 `seed=11`，`repeats_are_multi_seed=false`。

| 边界 trial | `P_confirm/P_selected` 成功次数 | 当前结论 |
| --- | ---: | --- |
| `dynamic_bridge_h1p2_v2p0` | 0/3 | 低空 2 m/s 不准入 |
| `dynamic_panzer_h1p2_v2p0` | 0/3 | 低空 2 m/s 不准入；一轮 map P95 0.305 m |
| `dynamic_pillbox_h1p2_v2p0` | 2/3 | 存在运行波动，不能承诺稳定工作点 |
| `dynamic_pillbox_h3p6_v0p5` | 1/3 | 高空低速仍不稳定 |
| `dynamic_pillbox_h3p6_v2p0` | 0/3 | 高空高速不可采用 |
| `static_pillbox_h3p6` | 0/3 | 三次均停在 `raw_classifier` |

因此当前默认继续使用 imgsz 640，不把 2 m/s 作为跨类别通用速度，也不承诺 3.6 m 工作域。
本轮只改评测/稳定性工具，算法、模型和阈值没有变化，所以没有重跑
`formal23/static25/sparse30`；引用这些历史结果时必须同时保留其原运行目录和 revision 边界。
