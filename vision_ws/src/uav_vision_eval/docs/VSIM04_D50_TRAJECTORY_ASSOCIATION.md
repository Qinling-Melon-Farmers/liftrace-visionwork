# V-SIM-04 D50 轨迹与多目标关联诊断面

## 边界

D50 是 `formal23` 之后的定向诊断面，不替代正式 Gate，也不与 C25 横向偏移矩阵
合并计数。当前实现冻结 schema、逐采样相机姿态和 dry-run，并提供复用现有
pixel/Phase-D/recorder 的单 trial Gazebo 入口；尚未执行 Gazebo，因此不能产出算法
PASS，所有 dry-run 摘要都明确记录 `gazebo_execution_status=NOT_RUN`。

实际相机入口当前只允许 `single_pairwise + center/quadrant`。`edge/partial` 不能直接
套用 recorder 的 fully-in-frame 或 C25 partial 窗口：前者可能裁靶，后者要求中心仍
在画内，而 D partial 的中心特意在画外。独立 center-in-frame 观察窗口落地前，这两类
由 runner fail-closed 为 NOT_RUN。`multi10` 在第二靶/H spawner 与动态真值接通前同样
保持 NOT_RUN。center/quadrant 还必须满足完整轨迹坐标不超过 4.8 m；例如
`d_single_39` 会达到 5.2898 m，因此不在 supported slice，runner 和 wrapper 都会拒绝。
同一 readiness 还用冻结靶尺寸、CameraInfo 和 0.0325 m color-sensor 光轴偏移投影四角，
要求轨迹内确实出现 fully-in-frame 且随后离开；因此 d12/d32（从未完整入画）和 d16
（进入后未离开）也会在启动前排除，不能被记成视觉算法失败。

## 50 个 trial

- `d_single_01`～`d_single_40`：指定一个 expected target 的 pairwise 设计；实际仍
  使用既有五靶 clutter 场，并非物理隔离的单靶世界。高度固定 2.4 m，平均速度固定
  1.0 m/s，完整覆盖相对角 `{0,45,90,135}`、轨迹
  `{constant,accel_decel,turn}` 和入画位置
  `{center,quadrant,edge,partial}` 的三组两两组合。
- `d_multi_41`～`d_multi_50`：同类近邻、异类近邻、红十字优先、部分入画高优先级、
  同类重叠、target+H、red_cross+H、双 target+H、三对象重叠和交叉转弯十个定向场景。
  每个物理对象都有独立 `target_id`、是否可选和权重。

相对角不是下视相机的 ZYX yaw。实现先用世界光轴和目标朝向构造
`image_right/image_down` 基向量，再由目标朝向在该基上的投影计算角度。Gazebo 相机
姿态同样由这组三维基向量生成，因此在俯视奇异姿态下仍有明确定义。
仿真 D435i color sensor 的 `horizontal_fov=1.211 rad`，640 像素宽对应
`fx=fy=462.266337 px`；matrix loader 会重新计算并断言，禁止用其他相机的 554 px
焦距悄悄改变 framing。

## 轨迹和关联契约

`generate_d50_pose_samples()` 返回 20 Hz 的 source-time pose 序列：

- `constant`：4 m 直线、恒定 1.0 m/s；
- `accel_decel`：相同几何路径，位置使用 `3u²-2u³`，平均速度保持 1.0 m/s；
- `turn`：4 m、90° 圆弧，位置和绕光轴姿态都逐采样变化，中点满足矩阵相对角与
  入画位置。

多目标观测必须同时保留真值 `truth_target_id` 与视觉链给出的
`associated_truth_target_id`，否则无法区分错误关联、一个真值拆成多个 stable ID
（duplicate）以及多个真值合成一个 stable ID（merge）。SEARCH 阶段还独立统计：

- 高权重、已确认且可见目标连续未获选择的最大帧数；
- 是否至少选择过一个准入 target；
- H 或其他不可选对象是否被选择。

关联记录缺少任一必填字段、stable ID 为空或观测帧覆盖不足都会失败。当前 4 帧覆盖
仅是 dry-run/接口自测的诊断下限，状态为 UNFROZEN，不是 multi-target Gate 的冻结
性能阈值；实跑前仍需两组共同确认统计窗口。

## 本地 dry-run

```bash
export PYTHONPATH=$PWD/vision_ws/src/uav_vision/src:$PWD/vision_ws/src/uav_vision_eval/src
python3 vision_ws/src/uav_vision_eval/scripts/vsim04_d50_dry_run.py \
  --matrix vision_ws/src/uav_vision_eval/config/vsim04_trajectory_d50_matrix.yaml \
  --output-dir /tmp/vsim04_d50_dry
python3 -m unittest -v \
  vision_ws/src/uav_vision_eval/test/test_vsim04_d_matrix.py
```

dry-run 输出：

- `d50_manifest.json`
- `d50_trials.csv`
- `d50_trajectory_samples.csv`
- `d50_coverage.json`
- `d50_association_contracts.json`
- `summary.json`

## 单 trial Gazebo 入口

以下命令只能在统一仿真队列空闲时执行；wrapper 会检查模型与 revision，并通过
`sim_run.sh` 生成标准 run 目录。它不启动 PX4、舵机或实机链：

```bash
export UAV_VISION_MODEL_PATH=/absolute/path/to/model.pt
export VSIM04_NAVIGATION_REVISION=<navigation-revision>
bash top_level_scripts/run_vsim04_d50.sh d_single_01
# 只串行运行 loader 与 runner 共同认可的 16 个 expected-target case：
bash top_level_scripts/run_vsim04_d50.sh supported
```

`vsim04_d50_runner.py` 直接消费 `samples` 的位置和四元数。base runner 在 recorder
握手前调用统一 `_set_trajectory_start()` hook，D 覆写该 hook 写入完整首姿态，因此
trial_start 后不会先落一次固定 ZYX-RPY 污染帧。结果的 frames/performance CSV 会带
`design_kind/relative_angle_deg/motion_profile/framing`，但状态始终是 DIAGNOSTIC。

下一步应先实跑默认 `d_single_01`，确认 enter→confirm/selected→leave、真实 CameraInfo
以及四元数姿态轨迹，再扩大到其他 center/quadrant case。多目标仍需 spawner 为第二靶
和 H 发布对应 `target_id` 真值；缺少该接线时保持 NOT_RUN。
