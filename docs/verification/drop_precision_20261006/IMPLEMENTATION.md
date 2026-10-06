# 首批投递曝光几何补丁与交接（2026-10-07）

研究工作树 `/home/xhj/liftrace-worktrees/r2026-high-view-search`。本批没有创建分支/worktree、提交、推送、上板或启动 ROS/SITL。主审 `REVIEW.md` 未覆盖；圆环 NMS 保持 false。

## 实际差距与改动

原控制路径按曝光高度缩放像素，再用当前 yaw 和位置生成目标，没有完整姿态射线；memory 重发布会重复累计稳定帧。`TargetCandidate.map_point` 是历史融合点且 candidate header 未保留源相机 frame，不能作为当前靶心。

新模式从相同 `last_seen`、类别和精修中心的 `detections_mapped` 元数据恢复源光学 frame，用当前 `center_px` 和曝光对应的 CameraInfo 重新求交。CameraInfo/像素校正、完整相机 TF、地面求交均不能回落近似；H projector 与投递复用 `ground_projection.py`，H 检测与默认降落路径不变。地图关联使用这次原始投影，释放证据包装也携带这次投影，而非记忆均值。

targets 和 mapped 无回调顺序保证，故只保留一个待配对批次；到齐重试，超过既有 target_max_age=0.5s 后 watchdog 拒绝。重复或较早图像既不加稳定帧、不重发 offset，也不刷新证据或撤销更新的有效证据。新无效观测仍撤销；未来/零/过期时间拒绝。几何 ID 更换重置稳定计数，不永久锁定同一 ALIGN 的几何 ID。

## 接口与兼容

`DropOffset.msg` 保留原四个像素字段，追加：

| 字段 | 精确模式语义 |
| --- | --- |
| `map_valid` | 全部几何前置条件通过 |
| `map_point` / `map_frame` | **单次槽位补偿后的 FC XY 对准点**；z 为配置的地面平面，不是下降高度命令 |
| `alignment_error_m` | 当前 FC 与上述目标的平面距离 |
| `alignment_tolerance_m` | 此曝光投影下采用的米制容差 |
| `target_id` / `target_first_seen` | 本次几何实例身份；不是新的语义任务权威 |

header.stamp 是图像曝光时间，header.frame_id 是同帧源光学 frame。ROS1 消息 MD5 已变化，两个工作区及跨仓消费者需要同步源码、重新生成消息并编译；默认关闭不意味着旧二进制 MD5 仍兼容。任务身份、ReleaseAuthorization、ServoAction、异步舵机及防重放接口未改变。

开关：视觉 `~exact_drop_projection`、控制 `/uav_vision/drop_exact_projection_enabled`，均默认 false。研究 `uav_high_view/full_strategy.launch` 和有效 Phase D/仿真包装器提供 A/B 接线；新模式要求 external guarded mission/context。正式整机模板由主代理后续适配，本工作树没有另造缺失模板。

控制接收绝对投影，不以当前 yaw 再投影像素，保留每次 XY 移动上限。接收新几何时同时检查源时间和接收时间；旧包忽略且不延寿，新无效包阻断且旧合法包不能复活。下降入口从新鲜稳定观测锁存物理容差；锁后用当前新鲜 FC 位姿与保留的绝对目标复核距离，继续要求原使命许可与飞行状态保护，详见下方 2026-10-07 修复记录。精确模式跳过原 applyDropSlotOffset，避免两次补偿；当前代码 `current_pixel_error` 仅初始化/赋值，没有额外像素释放门槛。

## 安装外参与容差

用户于 2026-10-06 现场确认槽 1 后仓、2 右仓、3 左仓。新 `slot_positions_body` 为 FLU 下投放口相对 FC 的**物理测量向量**：`[[-0.12,0],[0,-0.12],[0,0.12]]`。相机安装 TF 只进入相机射线；槽位只在 aligner 一处使用完整当前机体旋转，`FC_goal.xy = target.xy - (R_body_to_map * slot_body).xy`。硬件研究配置 `drop_geometry_hardware.yaml` 保留这三个 12cm 数值，未修改现场 known_rig 或部署入口。旧直接相加仅保留在兼容路径/显式 legacy 语义；其符号是潜在问题，不能据此认定历史误投原因。仿真明确使用 `zero`，原全零数组保持。

默认 `max_alignment_error_m=0`，不增加固定 5cm 门槛。用曝光射线的双侧一像素差分（含畸变、姿态）得到局部地面 Jacobian，将原像素圆容差沿当前补偿误差方向换算为米；`ReleaseEvidence.aligned/evidence_valid`、DropReady 和控制释放复核使用同一结果。例如无倾角、镜头距地 0.16m、fx=fy=800 时，30px 对应 0.006m。该数值是几何换算示例，不是实测场景参数。倾斜/畸变下属于局部容差换算，不声称精确的大范围像素足迹。正数固定米制阈值仅供明确的实验配置，两端须一致。

CameraInfo 的 `fixed` 契约允许固定标定零时间戳/低频发布；有时间戳的版本仅对其生效时间之后图像使用，零时间戳标定变化从接收时间生效。`per_frame` 契约才要求 0.1s 配对窗口。错误 frame、未标定焦距/尺寸、非有限参数和不支持的畸变模型拒绝。曝光 TF 从不取 latest；仅用于当前误差/槽位补偿的 body TF 可取最新且必须满足 0.1s 新鲜度。

## 变更路径

- `vision_ws/src/uav_vision/scripts/drop_aligner.py`、`target_map_projector.py`。
- `vision_ws/src/uav_vision/src/uav_vision/{ground_projection,drop_geometry_policy}.py`。
- `vision_ws/src/uav_vision/msg/DropOffset.msg`、`config/{drop_aligner,drop_geometry_hardware}.yaml`、`launch/phase_d.launch`。
- `vision_ws/src/uav_vision/test/{test_drop_geometry,test_drop_observation_stamp}.py`。
- `patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp`、`include/patrol_control/{patrol_control,drop_geometry}.h`、`test/{test_exact_drop_control,test_async_servo}.py`、`launch/toudi3_full_competition_sim_new_vision.launch`。
- `patrol_uav_ws-patrol_planner/src/uav_mission/launch/toudi3_visual_delivery_guarded.launch`、`vision_ws/src/uav_high_view/launch/full_strategy.launch`。
- 本报告及共享变更台账；旧链保护为 `legacy_baseline/20261006/drop_geometry_delta/SOURCE.json` 和两个增量原文件，恢复时先恢复现有完整 `posctl_handoff` 快照，再覆盖该增量。没有散落 bak。

## 验证命令与结果

全部 WSL shell 通过 `wsl -e bash -c '...'`；Python 使用已有 rl_drone，未安装依赖。以下是 WSL 内复现步骤，无节点启动：

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
# 实际日志：/tmp/drop_geometry_full_build.log；完整 vision 后 nav，现有 devel，退出 0。
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python -m unittest discover -s vision_ws/src/uav_vision/test -p 'test_*.py'
python -m unittest discover -s patrol_uav_ws-patrol_planner/src/patrol_control/test -p 'test_*.py'
python docs/verification/drop_precision_20261006/projection_preflight.py --self-check --check-build
git diff --check
```

- 视觉完整 Python 回归 **58/58 PASS**（投递定向 20 项，含 15 个生产方法新回归和原 5 项）。
- 控制完整 Python/C++ 生产方法回归 **95/95 PASS**（含 9 个新控制回归）；新增 C++ 测试使用实际生成 ROS 消息、提取生产回调、UBSan，未创建节点或舵机调用。原异步测试夹具补齐已存在的 H POSCTL 字段，不改 H 生产行为。
- 0/roll/pitch/yaw/组合姿态与独立 SciPy 旋转矩阵一致；共享 H projector 与投递一致；曝光 TF 与当前姿态分离、畸变/标定版本、缺失/错时/错 frame/平行/背向/非有限 TF、未来/过期/乱序/重复、不同槽位 yaw、补偿后证据和 ID 更换均覆盖。
- targets(N)→mapped(N)→多次重复 targets(N)，连续 N1..N5 **PASS**；待配对超时与已到达但无效元数据也覆盖。
- 主代理 preflight：seed31/38 各 **32 项 PASS**，16 个冻结场景文件保持，拒绝旧运行参数和缺失/开启圆环 NMS，消息七字段与二进制检查 PASS，`simulation_started=false`。
- 整包构建成功，含既有规划器编译警告；未修改该无关代码。`git diff --check` 通过。

## 剩余边界与跨仓交接

本补丁完成可运行仿真对照的编译/离线前置条件，不是动态投递精度验收。两轮同场景 A/B、真实 CameraInfo/TF 时效、槽位多 yaw 实测及机械带载落点仍待验证；没有人工中心真值的图像差异不能称真实精度提升。完整机体旋转包含倾斜，但不建模箱体自由落体速度、风或机构释放延迟。

向导航权威 `liftrace-controlwork` 交接时同步上述 DropOffset 契约、对应控制最小补丁和 launch 开关，并标注本工作树未提交 diff 的后续正式 revision。几何 target ID 可更新，语义目标/决策/槽位/许可身份仍固定；map_point 新语义是已补偿 FC goal，不能再当原始靶心或叠加槽位。当前未写另一仓、不合并或推送。

## 2026-10-07：锁后目标更新与有界释放承诺修复（已冻结）

修复前源码为 `8c4be8917ba68847b339d907ab794af9182e8a45`。seed31 已关闭运行 `logs/drop_precision_seed31_20261007_002621` 的 88 条 offset 中，锁定来源帧 77.645 的目标为 `(7.393637,-1.052990)`，最后来源帧 83.249 为 `(7.423729,-1.005712)`，相差 **0.05604 m**；后续实际 setpoint 仍保留前者。根因是 `CrossDetectionDone` 的局部静态 `waypoint_temp` 遮蔽回调更新的成员变量。另一个根因是精确释放门持续要求新鲜图像，阻断既有使命层有界下降承诺。该运行仍是修复前 FAIL，未作为补丁动态验收。

本次仅修改控制 cpp、`patrol_control.h`、`drop_geometry.h`、三个控制测试文件（`test_exact_drop_control.py`、`test_async_servo.py`、`test_external_landing_handoff.py`）及本文。后两项只补测试夹具字段；未修改消息、视觉、配置、使命许可权威或共享台账。

- 红十字保留 legacy 静态缓存；精确模式引用成员 `this->waypoint_temp`。标准靶原本已用成员变量。精确模式在本次 alignment tick 后刷新实际 `patrol_cmd`，使锁后新目标直接进入控制输出。
- 只在真正新鲜、稳定的下降入口锁存 `capture_tolerance_m_` 和固定 action/context。入口检查 offset、DropReady 源时间及接收时间、当前 FC 位姿和原物理容差；无需此刻已收到舵机许可，避免消息先后顺序破坏几何锁存。
- 接受同 action 的新鲜合法绝对目标，仍用已有 `max_alignment_move_distance_` 约束相邻观测跳变以及每个输出目标相对当前机体的距离。释放距离使用未裁剪的绝对目标；新帧容差不覆盖入口锁存容差。
- 锁后不再要求连续图像、旧 `alignment_error_m` 或 `uav_drop_ready_`。仍须当前 FC 位姿新鲜且距离在锁存容差内，以及原 `hasFreshMissionReleasePermission()` 的完整身份、epoch/revision、新鲜度和有效期限；固定 action 的原 deadline 也不可被 heartbeat 延长。
- deny/expiry 立即阻断释放但不清几何；新 action、明确 context cancel 或新的无效几何清锁存。无效新包不能被旧包复活。没有刷新旧图像/证据时间戳，没有新增计时器或状态管理器。

验证结果（全部离线，没有 ROS 节点、SITL 或执行机构启动）：

- 控制完整回归 **114/114 PASS**，日志 `/tmp/drop_commitment_full_control_tests.log`；精确控制套件现为 **28 项**（原 9 + 新 19）。使用实际生成 ROS 消息、提取完整生产 `CrossDetectionDone` / `WayPointDetectDone` / `externalMissionTick` 和相关回调/门控，C++ UBSan；运输/发布/恢复支路为测试替身，释放通过表示进入测试提交入口，非实际舵机动作。
- 状态测试覆盖标准靶及红十字锁后 0.05 m 合成新目标更新实际 adjust/patrol 输出；图像超时后合格位姿与新鲜许可可以提交；deny、expiry、错 action、陈旧许可、坏/陈旧 FC 位姿、deadline 到期、无效几何、过大跳变均不能提交。另覆盖初始陈旧/未稳定/未来 DropReady/未到位不能锁存，以及原机体移动限幅。0.05 m 是合成输入差值，不是实拍精度或落点误差。
- 既有 `patrol_control-drop-action-test` **11/11 PASS**，日志 `/tmp/drop_commitment_gtest.log`。
- `cmake --build patrol_uav_ws-patrol_planner/build --target patrol_control -- -j2` 增量 ROS 编译退出 **0**，最终日志 `/tmp/drop_commitment_incremental_build.log`。未启动二进制。全套测试只有既有 H 夹具的有符号比较警告。

生产源码与测试已冻结，未 commit/push。下一步由主代理按其授权提交，并以同 seed31 验证锁后 setpoint 更新与有界承诺释放；本补丁不能据离线 PASS 宣称投递精度或实跑成功。
