# uav_vision_eval

`uav_vision_eval` 只负责独立 Gazebo 真值、记录和评分，不进入实机运行依赖。V-SIM-04
最小框架在单个 Gazebo 会话中执行固定 `seed=11` 的 23 个 trial：

- 静态：`tent/pillbox/bridge/panzer/red_cross × {1.2, 2.4, 3.6} m`，共 15 个；
- 动态：`panzer/red_cross × {1.8, 3.0} m × {0.5, 1.5} m/s`，共 8 个；
- 当前 `class_profile=r2026` 不含 tank；tank 资产继续由 `full` profile 和历史回归保留。
- 五类靶在同一世界中共存，属于 clutter 评测；`frames.csv` 记录每帧共同完整入画类别。
  seed=11 最小矩阵暂不评分 false positive，不能据此报告 FP 指标。

## 指标语义

- `P_confirm`：目标从完整入画到离开期间，`/targets` 出现满足当前连续帧、地图、关联、
  拒绝原因、年龄和 profile 完整准入的候选；
- `P_selected`：上述同一个 stable ID 在离开前发布到 `/selected_target`；
- `P_interrupt`：visual-only 固定为 `null`，只有导航 adapter 实际接受同 ID 并发生
  `SEARCH→APPROACH` 才能评分，不能用 selected 代替；
- `confirmation_exposure_sec` 使用图像/仿真源时间；`confirmation_processing_ms` 使用
  monotonic 墙钟；若 recorder 的图像回调晚于候选回调则显式记录 receipt reorder 并把该样本
  作为 0 ms 下界，不再因跨话题回调顺序丢样本。地图无效、TF 失败和地图误差分别报告。

每次运行固定输出：`manifest.json`、`frames.csv`、`events.csv`、`summary.json`、
`report.md`、`vision_search_performance.csv`。manifest 记录 profile、world、模型、阈值、
CameraInfo、相机模型/RPY、内嵌场景/真值模型/anchor catalog、外参来源、视觉/导航 revision、
轨迹期望/实测时长与速度，以及 monotonic 接收 FPS 和图像源/仿真时间 FPS。

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
`sim_run.sh` 对 `vsim04*` 场景还会二次检查六项产物和 `summary.status=MEASURED`；即使
`roslaunch` 在 required runner 失败后正常关停并返回 0，统一入口仍会返回非零。

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
动态时长按 `2*path_half_length/speed` 计算。逐 trial 配额未填满、CameraInfo 首个 profile
非法或中途发生 frame/尺寸/内参/畸变变化、任一声明 PNG/JSON 缺失、manifest 未
`run_complete`，均 fail closed 并使 `sim_run.sh` 非零。CameraInfo 在固定仿真相机中按
latched profile 使用，不要求其 header stamp 与每帧图像相同，但 frame_id、尺寸、有限且
正的 fx/fy 必须与图像契约一致。该数据只标记为 `sim-small-target` 诊断输入，不得当作实拍
数据或未经独立验证直接触发阈值/正式模型切换。

入口只启动 Gazebo、评测相机、视觉链、真值、recorder 和 trial runner；不会启动 PX4、
MAVROS、旧控制、`actuator_pwm` 或真实投递。当前合并态证据
`logs/vsim04_seed11_current_20260829_195506/` 已完成 23/23 并写出 `MEASURED`；
P_confirm/P_selected 均为 13/23。该终态只表示运行与产物有效，不代表红十字、高空和动态
召回已经达标；合并前 `logs/vsim04_seed11_20260829_192515/` 的 9/23 仅保留作旧阈值基线。
