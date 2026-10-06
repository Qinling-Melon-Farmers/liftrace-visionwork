# 投递精度两轮评测准备（2026-10-06任务，2026-10-07整理）

本代理负责历史只读收集和评测流水线。用户已授权优化完成后seed31/38各一轮；当前仍等主代理代码审查及编译完成通知，未启动ROS/SITL。主审约束见同目录REVIEW.md，本代理未修改该文件。本代理写入仅限本目录的PLAN.md、分析/预检脚本、薄replay.launch和run_case.py；共享变更记录、提交由主代理统筹，本代理不写共享文件、不提交、不清理日志。

## 基线选择与量化

根H：`/home/xhj/liftrace-worktrees/r2026-high-view-search`。

主配对基线：同一源码`337b583689f46d907546d1d28a8fc52b13f47398`的镜头2m三扫描线seed31/38。两轮均正确三次模拟释放、零实体碰撞；软件Gate均FAIL，不能称整场软件验收通过。

| seed | 原始run（H/logs下） | 红十字ACK机体误差 | bridge | panzer | 物理触地任务秒 | run体积 |
|---|---|---:|---:|---:|---:|---:|
| 31 | snake3_camera2m_snake3_seed31_20261005_095902 | 3.317cm | 11.420cm | 4.815cm | 204.517 | 177MB |
| 38 | snake3_camera2m_snake3_seed38_20261005_101903 | 1.941cm | 7.725cm | 13.982cm | 363.540 | 370MB |

表中误差严格复现旧口径：ACK接收时刻附近truth_pose.csv最近样本的XY距离。工具另外以ACK源时间、两侧真值采样线性插值计算，三类结果分别为seed31的3.474/11.431/4.871cm，seed38的1.861/7.927/13.972cm。新旧比较必须统一口径，不能把这两种口径的差别归为算法收益。主基线旧ReleaseResult没有execution_state/mission_id/decision_seq，也没有RAW_CALL_STARTED，无法还原执行开始时刻；只认可run内execution_id/slot/target_id/class的ACK去重。

最新有效补充：源码`10120dfaf1113787cc08776bc3f1dabdd1571eea`，run `seed38_resume_resume_on_fixed_seed38_20261005_173322`，192MB；panzer/red_cross/bridge ACK最近采样误差6.558/8.654/7.137cm，三投、零接触，持续支撑接触任务204.169s后6.678s保护ABORT。软件FAIL，armed/LANDING，缺ON_GROUND/disarm证据。开启续扫但实际没有触发，且释放事务版本、投递顺序和碰撞停止选项不同，仅作最新补充，不替换主配对基线或计算续扫净收益。

原seed38续扫开关两轮`7a950204`保留：off三投零碰撞；on三投但返航护框碰墙1次，不作完整零碰撞时间基线。相对路径错误启动`snake3_camera2m_snake3_seed31_20261005_095529`不是有效轮次。

## 原始资产索引

- 主基线命令/source/cleanup：`H/logs/snake3_camera2m_20261005_batch/matrix.json`，每个run的`manifest.yaml`、`run.log`、`rosparams.yaml`。
- 主基线汇总：`H/docs/verification/snake3_camera2m_20261005/metrics.json`、`release_truth.json`、`physical_completion.json`、`REPORT.md`、`scenes.json`。
- 主基线中心：该报告目录下`31_snake3_centers/`、`38_snake3_centers/`的`center_summary.json`、`center_samples.csv`、`center_errors.png`、`stages/phase_summary.json`、`phase_statistics.csv`、`center_samples_by_phase.csv`、`PHASE_REPORT.md`。
- 补充轮命令：`H/logs/seed38_resume_20261005_fixed_batch/matrix.json`；报告与中心位于`H/docs/verification/seed38_resume_20261005/fixed_rerun/`，中心子目录`38_resume_on_fixed_centers/`。
- 每个run的原始CSV：`truth_pose.csv`、`lio_pose.csv`、`mavros_pose.csv`、`planner_setpoint.csv`、`mavros_setpoint.csv`；原始消息为`vision_metrics.bag`、已有只读导出`center_export.jsonl`、`key_events.jsonl`、`high_view_full_events.jsonl`。真值为`random_field_truth.yaml`、`red_cross_truth.yaml`；相机为`actual_camera_info.json`。
- 相机images来源是`downward.mp4`与`downward.csv`（frame/image_stamp_ros_sec/receipt_ros_sec），不是bag里的原始Image。另有overview/follow视频及各自CSV。center_errors.png和各run报告中的route_stages.png等是分析图，不是检测原始图。按CSV选择视频帧，不假设MP4时间等于ROS任务秒。
- B只读辅助：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/flight_fov_20260926/`有`frame_*.jpg`、`frames.json`、`camera_info.json`、`motion.json`；`logs/flight_review_20260925/video/`有preview图片、video_frames.csv、records.json。仅供同帧旧新算法差异/负例；没有独立实测靶心与包裹落点真值时不宣称厘米精度。
- B的`tools/bag_replay/bag_replay.py`提供离线TransformTree，旧中心流水线依赖它；不启动B中的launch，也不写B。

## 固定场景和计时边界

使用历史已生成的field.world及YAML，不按同seed重新生成。场地净10×10m，搜索净区约8.45×10m（走廊/隔墙扣除），固定四树中心(2.0,1.8)、(2.7,-1.8)、(5.2,1.5)、(5.7,-1.7)，三线X=1.0/3.7/6.4，端点Y=-3.95/+4.10。镜头目标AGL2.00m、FC目标2.16m，H80cm，80cm门，中部障碍柱及XY膨胀0.25m。逐轮验证实际参数、模型和场景未被其他overlay替换；实际高位镜头中位曾为2.114/2.128m，不能把配置值当实测。

场景来自`docs/verification/snake3_camera2m_20261005/generated/snake3_31/snake3_seed31`和对应38目录。scenes.json显式标记`historical_target_positions_preserved=false`：它们与10月5日同场景历史轮次可比，与更早seed31–40布局不能直接配对。镜头2.6m六轮也不能混为2m基线。

本轮建议只跑同一审查完成版本seed31、38各一次，沿用上述固定场景；两个新轮次均显式续扫关闭，与主基线保持策略一致，仅替换本轮投影/几何/稳定链。主代理若要求续扫开启，两个新轮都保持相同设置并标注策略差异，耗时结论不得只归因于投递精度；不因此扩大组合或追加轮次。释放槽外参按主审要求：仿真显式零，不引入实机12cm偏移。

任务ROS计时从任务接受开始。分别列第三槽ACK、两门、H持续支撑触地、ON_GROUND/disarm、COMPLETE、地面等待与ABORT；触地后等待不加进物理飞行时间，软件FAIL保持FAIL。seed38旧轮触地后额外等待236.467s才失败，不能拿600秒尾段作飞行时间对照。上述触地秒数与更严格“ON_GROUND+disarm+稳定2秒确认”不是同一终点，应分列；新旧必须使用相同端点。两seed单次观测、调度/识别随机性、版本间非精度修复和不同投递顺序均限制因果结论。视频时长和墙钟runtime不是任务秒数。

## 靶心和释放中心的计算

1. 标准靶与红十字真实几何中心取本轮归档random_field_truth.yaml的world_x/world_y，并与冻结field.world的include pose对应。标准靶模型是本地XY居中的1×1m薄板（link只沿Z偏移）；蓝白环以贴图圆心为靶心，不取桥/装甲车图案重心或bbox中心。已查看桥梁贴图，环与模型中心对应；新模型如改变UV/环位置应单独复核。H取field.world中具体landing_h实例pose，起飞H与降落H不能混为一个靶。YAML保留4位小数，约0.1mm舍入远小于本批厘米量级。
2. competition_key_recorder对Gazebo ModelStates位置减去truth_world_offset后写CSV；三轮归档均为[0,0,0.22]，XY保持world一致。precision_eval.py每轮从rosparams.yaml读取该offset，目标world_xy也减相同XY偏移，再比较。不能用检测均值估计真值或拟合真值变换。
3. 视觉中心按原图曝光source stamp、录制TF转到camera_init，统计mapped/targets/selected各流；排除map_invalid、缺TF/缺时间、未来/陈旧、重复重发。按源观测时间分HIGH_SURVEY/REACQUIRE/DELIVERY/LANDING，分别保留最近同类与最近任意类关联，以免把panzer误认pillbox写成定位误差。旧TransformTree使用最近过去TF、动态最大年龄0.5s，不插值，这是离线估算局限。
4. 有新事务消息时，以mission_id/decision_seq/attempt/slot/execution_id关联RAW_CALL_STARTED与成功terminal COMPLETED；NOT_STARTED不算执行，完整重放去重。使用各自header源时间匹配truth_pose.csv两侧样本，最多0.25s间隔，无外推，显示采样间隔和源/接收延迟。同时保留旧ACK接收时间最近采样误差，便于精确对照旧报告。执行开始/ACK是代理观测，不是已测得的盒子脱离时刻。
5. 瞬时视觉中心取该释放之前已发布、源观测距释放<=0.5s的同类最新mapped/targets/selected；targets/selected再匹配target_id。输出center_source、center_refined、中心真值距离、类别关联；缺合格同帧观测时留空，不能用历史记忆均值或最后top_hints补齐。相机姿态/位置对应曝光时刻，不能将后续姿态当同帧姿态。projection工具新旧算法回放须使用相同曝光和内参。倾斜圆面的椭圆拟合中心未必等于真实圆心的像素投影，未经独立几何校正时不能将二者等同。
6. 此处body center是truth_pose.csv所录Gazebo模型原点（旧报告称机体/FC中心）的XY投影。真实投放口若存在外参r_slot，瞬时出口P_exit=P_body+R_body*r_slot；不能把机体目标修正向量直接当物理出口向量。此次仿真槽位显式零时，两者名义XY相同。包裹脱离后的水平速度、姿态、风阻/气流和下落弹道未仿真测量，以上指标不是实物落点或实际得分。红十字/标准靶分别用半边长0.175/0.5m的旋转靶板做名义范围诊断，也不代表环分区得分。

## Overlay与资源准备

只读已核实H的vision_ws/devel、patrol_uav_ws-patrol_planner/devel存在；source顺序如下，rospack解析uav_mission/uav_vision/uav_high_view全部位于H。已有devel存在不能证明新代码/消息已编译，由主代理确认两仓overlay和测试完成。

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && rospack find uav_mission && rospack find uav_vision && rospack find uav_high_view'
```

主代理的标准构建方式（本代理未执行）：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && BUILD_JOBS=2 bash top_level_scripts/build_competition.sh'
```

该脚本先source Noetic，在vision_ws中catkin_make指定/usr/bin/python3与清空白名单，再source视觉devel，在控制工作区catkin_make指定ROS_EDITION=ROS1。仿真roslaunch_rl_drone.sh的运行环境与ROS构建Python角色分别保留；不安装包、不新建conda。

模型已存在：`/home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt`（5.44MB）。F盘物理剩余约39GB是容量依据，WSL ext4显示843GB是虚拟可用容量，不能替代宿主容量；C盘约20GB。启动前再读df与check_sim_processes，不清理任何日志。旧轻量run约177–370MB，修正轮精度bag仅2.4MB；实际新轮体积仍取决于时长。沿用必要topic的bag，不录全topic/Image/点云全场bag，不为评测扩大数据集。

## 新路径入口、参数预检与审查通知后的启动

旧直接复用seed38 replay的命令已撤回，不能原样执行。本目录新增薄`replay.launch`，先include历史记录链，再在所有场景YAML和上层launch之后显式覆盖新路径参数。原始scene文件不改。`run_case.py`只运行一个case；默认/`--dry-run`不创建日志、不启动ROS/SITL。用户要求本轮**全部stop_on_collision=false**，碰撞事实/Gate失败仍保留，不能为历史一致性改为true。主基线碰撞停止选项不同，报告必须注明，零碰撞历史可比较投递误差，遇到新碰撞不能宣称完整时间改善。

当前共享源码可见的参数名为`/drop_aligner/exact_drop_projection`和`/uav_vision/drop_exact_projection_enabled`；worker已在phase_d/full_strategy等入口加入相同名选项。本会话没有直接代理消息工具，尚未伪称收到Beauvoir的最终确认；启动前由主代理/Beauvoir确认命名和容差。预检同时检查源码确实消费这些名字，若worker更名则拒绝。主代理编译/审查完成通知仍是实际启动前提。

| 核对项 | 本轮显式值 |
|---|---|
| 视觉/控制两端exact开关 | true / true |
| drop_aligner body / map / ground_z | vision_body / camera_init / -0.22 |
| target_map_projector、控制drop map/ground_z | camera_init / -0.22 |
| CameraInfo / 光学frame | /downward_camera/camera_info / downward_camera_optical_frame |
| slot_compensation_mode | zero |
| 三槽静态/动态/物理位置 | 三个[0,0]，不引入实机12cm偏移 |
| 圆环quality-first NMS | /circle_detector/circle_quality_ordered_nms=false，启动与运行时均要求显式存在且为false |
| 稳定/像素门槛 | 继承现有stable_frames和30px |
| 两端固定米制容差参数 | 0.0：不是0m门槛，表示曝光Jacobian换算原30px；控制使用消息alignment_tolerance_m |
| drop_offset_timeout | 0.5s，与worker当前exact入口一致 |
| 续扫 / 碰撞停止 | false / false |

quality-first圆环候选已定量恶化合成中心P95，本轮两seed均禁止启用。wrapper最终覆盖为false，预检与在线归档参数核验共用同一检查；true或缺失均失败并收尾，不能依赖源码默认false。此候选不纳入本轮“改善”结论。不新增未经讨论的5cm阈值。新DropOffset的alignment_tolerance_m是实际阈值，评测同时记录error与tolerance。相机SDF确认有与CameraInfo相同的非零畸变，故保留已有rectify_input_pixels=true。真值只用于评测，没有真值控制或选靶节点。

`projection_preflight.py`完整展开ROS launch XML（不启动ROS节点），验证双方显式开关、命名空间、地面/TF/相机/槽位/容差/Guarded Mission、固定航线及高度、当前H源码overlay、无硬件执行器节点，并与337b5836逐字比较每seed八个冻结场景文件（不用哈希）。`--self-check`两seed共16个文件、每seed32项关键参数检查通过，同时实际历史旧rosparams.yaml被拒绝，内存中的NMS参数true和缺失两种误配置也被拒绝。实际开跑前`--check-build`还检查新DropOffset字段、控制/圆环二进制与相关源码mtime；静态预检通过不能替代主代理构建与审查。

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && PYTHONDONTWRITEBYTECODE=1 python -B docs/verification/drop_precision_20261006/projection_preflight.py --self-check'
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && PYTHONDONTWRITEBYTECODE=1 python -B docs/verification/drop_precision_20261006/run_case.py --case 0 --dry-run'
```

主代理通知编译与审查完成、完成源码提交后，将`REVIEWED_HEAD_REPLACE`替换为被审查完整commit。两条实际执行命令必须分开执行，不把它们连成自动批量循环：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && PYTHONDONTWRITEBYTECODE=1 python -B docs/verification/drop_precision_20261006/run_case.py --case 0 --reviewed-head REVIEWED_HEAD_REPLACE --execute-authorized'
```

查看seed31新run、原始Gate、碰撞/误投、首个失败和收尾后，将`PRIOR_RUN_PATH_REPLACE`替换为matrix中seed31的确切绝对路径；须等主代理对case38的明确评审通知再启动，不自动更多重跑：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && PYTHONDONTWRITEBYTECODE=1 python -B docs/verification/drop_precision_20261006/run_case.py --case 1 --reviewed-head REVIEWED_HEAD_REPLACE --previous-run-reviewed PRIOR_RUN_PATH_REPLACE --execute-authorized'
```

runner内部仅调用`env SIM_RUN_AUTHORIZED=1 SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 SIM_STORAGE_GUARD_PATH=/mnt/f timeout --signal=TERM --kill-after=60s 7200s bash top_level_scripts/sim_run.sh drop_precision_seed31/38 bash top_level_scripts/roslaunch_rl_drone.sh <本目录replay.launch> scene_dir:=<冻结场景> field_seed:=31/38 target_model_path:=<同模型> high_agl:=2.16 resume_survey_enabled:=false`；显式UAV_WS/VISION_WS均H，继承原线程设置。不在profile或长期环境设置授权变量。

新批次只写`H/logs/drop_precision_20261006_batch/matrix.json`及`H/logs/drop_precision_seed31/38_<时间>/`。旧matrix不动。当前HEAD必须等于reviewed-head，生产源码必须已由主代理提交，第二轮冻结同HEAD；runner不代提交。授权执行后才创建锁/日志/状态，不接续已有active，不覆盖或自动重试。第一轮结束须检查首因并显式传入已检查的run路径。source变化、基础设施或收尾失败阻断下一轮。

每个新run存`effective_parameters_prelaunch.json`和`projection_preflight.json`；在线只读读取recorder生成的`rosparams.yaml`，再次验证两端开启/零槽位/真实frame/false碰撞停止；不匹配立即TERM当前包装器并统一收尾，不飞完整轮才发现旧路径。离线`projection_evidence.py`读取轻量bag的实际DropOffset字段，统计是否真的出现新路径map_valid、曝光时间和容差，区分“参数启用”与“实际输出”；控制接受/释放事实仍看原事务日志。

`SIM_NO_RECORD=1`只关闭桌面录屏；沿用下视/俯视/跟随视频及精度bag，bag无原始Image和点云，不新增全topic录制。sim_run的flock、host空间、overlay归属、原Gate、超时/信号统一收尾均保留。人工中止用stop_toudi3_sim.sh，复查check_sim_processes零残留。不会由预检启动roscore或Gazebo。

## 只读历史复核与新轮处理命令

本阶段已运行只读验证：三个历史run、九次旧ACK误差全部复现，必要产物零缺失；未重跑大metrics/视频流水线。

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && python -B docs/verification/drop_precision_20261006/precision_eval.py verify'
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && python -B docs/verification/drop_precision_20261006/precision_eval.py inventory --summary'
```

以下是后续新轮实际闭合run产生后的处理步骤，当前没有执行或创建其产物。为遵守本阶段仅写PLAN/工具的边界，现阶段脚本只输出stdout。示例将`RUN_NAME_REPLACE`替换为sim_run实际生成的唯一目录名，`SOURCE_HEAD_REPLACE`替换为该轮manifest的源码；两轮分别存到新的case子目录。命令在WSL内执行；Windows调用时仍包装`wsl -e bash -c '...'`。不要把示例变量照抄为真实结果。

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
TASK_RUN=/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/RUN_NAME_REPLACE
TASK_SCENE=/home/xhj/liftrace-worktrees/r2026-high-view-search/docs/verification/snake3_camera2m_20261005/generated/snake3_31/snake3_seed31
TASK_OUT=/home/xhj/liftrace-worktrees/r2026-high-view-search/docs/verification/drop_precision_20261006/results_seed31
TASK_TOOLS=/home/xhj/liftrace-worktrees/r2026-high-view-search/docs/verification/seed38_resume_20261005/analysis_tools
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONDONTWRITEBYTECODE=1
# 后续获准产物写入后才创建新case目录；不覆盖已有目录。
mkdir "$TASK_OUT"
python -B docs/verification/drop_precision_20261006/precision_eval.py matrix-row --run "$TASK_RUN" --scene "$TASK_SCENE" --seed 31 --source SOURCE_HEAD_REPLACE > "$TASK_OUT/matrix.json"
# 旧轨迹/阶段metrics复用；--skip-video避免向run写合成视频。
python -B "$TASK_TOOLS/analyze.py" --matrix "$TASK_OUT/matrix.json" --out "$TASK_OUT" --skip-video
# 系统Python仅用于读取ROS bag及已生成消息；不会启动ROS节点。
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace-worktrees/r2026-high-view-search/vision_ws/devel/setup.bash
/usr/bin/python3 -B "$TASK_TOOLS/center_compare.py" export --bag "$TASK_RUN/vision_metrics.bag" --out "$TASK_OUT/center_export.jsonl"
# Actual experimental offsets, not just candidate/map centers.
/usr/bin/python3 -B docs/verification/drop_precision_20261006/projection_evidence.py --bag "$TASK_RUN/vision_metrics.bag"
/usr/bin/python3 -B docs/verification/drop_precision_20261006/projection_evidence.py --bag "$TASK_RUN/vision_metrics.bag" --export > "$TASK_OUT/exact_offsets.jsonl"
# 用conda解析/绘图，显式TF/真值坐标约定；若归档offset变了先调整，不能照搬。
/home/xhj/miniconda3/envs/rl_drone/bin/python -B "$TASK_TOOLS/center_compare.py" analyze --data "$TASK_OUT/center_export.jsonl" --run "$TASK_RUN" --world "$TASK_SCENE/field.world" --out "$TASK_OUT/31_precision_centers" --pipeline-root /home/xhj/liftrace-worktrees/r2026-board-vision-tests --world-to-eval-xyz 0 0 -.22 --world-to-eval-yaw 0 --transform-note world_XY_equals_CSV_XY_offset_0_0_minus022 --hint-frame camera_init --scope-label seed31_precision
/home/xhj/miniconda3/envs/rl_drone/bin/python -B "$TASK_TOOLS/center_stages.py" --run "$TASK_RUN" --centers "$TASK_OUT/31_precision_centers" --data "$TASK_OUT/center_export.jsonl" --truth-local-ground-z -.22
/home/xhj/miniconda3/envs/rl_drone/bin/python -B docs/verification/drop_precision_20261006/precision_eval.py analyze --run "$TASK_RUN" --scene "$TASK_SCENE" --center-summary "$TASK_OUT/31_precision_centers/center_summary.json" --exact-offset-data "$TASK_OUT/exact_offsets.jsonl" > "$TASK_OUT/precision.json"
```

seed38步骤相同，只替换scene的31→38、seed参数38、case/center输出目录38。旧centers.py会默认向run写center_export.jsonl，旧physical.py没有--out且固定历史目录，故不直接调用这两入口；上面显式export到新case目录，旧run与历史report均保持只读。旧release_truth.py可用--out但只给接收时刻最近样本，不包含执行开始/插值/新鲜视觉中心；本次由precision_eval.py同时提供两种口径。保留旧metrics的阶段信息，初始高位只看冻结SURVEY源观测快照，不看最终top_hints。

## 评审交付与下一步

提交给主代理的比较表按seed和类别列：旧/新ACK同口径误差、执行开始和ACK插值误差、最新观测中心误差、对准阶段耗时、NOT_STARTED/重试次数、三投正确靶位、碰撞事实、物理触地、软件Gate与终态时间。中心分阶段中位/P95可使用大量去重观测，释放六点仍列逐点，不用六点P95或两个seed宣称总体可靠性提升。

精度改善同时要求没有错类/错靶释放、时序门槛失效或明显任务退化；原始FAIL保留。没有新代码审查/编译通知前本代理仅准备。收到已编译/审查固定hash通知后，本代理作为唯一仿真操作代理先执行seed31；case38等待其独立明确评审通知后执行；同一FAIL先分析，不因自动continuation再启动或追加轮次。主代理负责共享记录、最终版本提交和推广决定。

本轮本代理验证记录：projection_preflight --self-check PASS（两seed/32项参数各自验证/16冻结文件/旧runtime拒绝），runner dry-run通过；precision_eval.py verify PASS（3 baselines / 9 ACK metrics / 0 missing）；rospack解析H正确；check_sim_processes无残留；未启动仿真、未动生产/共享配置或REVIEW.md、未提交。