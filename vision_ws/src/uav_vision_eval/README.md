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

入口只启动 Gazebo、评测相机、视觉链、真值、recorder 和 trial runner；不会启动 PX4、
MAVROS、旧控制、`actuator_pwm` 或真实投递。完整 23 trial 尚需在独占仿真窗口实跑验收。
