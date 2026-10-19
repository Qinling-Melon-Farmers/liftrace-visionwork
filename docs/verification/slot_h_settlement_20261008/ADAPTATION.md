# 2026-10-08 三槽补偿与 H 停稳增量适配准备记录

当前状态：已在原稳定分支适配基础上同步F33a521a8的H视觉/运动并行计数与DYNAMIC_FULL；本轮入口离线检查通过。状态为“待动态验收，未部署板端”：5f任务COMPLETE但碰撞GateFAIL，最新H33a仍待末报告。

## 来源与归属

- 本工作树：/home/xhj/liftrace-worktrees/r2026-board-vision-tests（B 板端视觉稳定分支）；适配起点 HEAD 为 397d10d961470c51ccf317e832dc8c72e02cb933。
- 运行补丁权威来源：视觉集成仓 https://github.com/Qinling-Melon-Farmers/liftrace-visionwork.git，F 提交 073c8f19d7b7a2aa6b88bc536e990af59523c4d4，父提交 f79c2a82e727530f42a371dbf30805b310bbaf8f。
- 验收节点小补丁：dbd6e3e2fafb7a9c00854a3bd38efa6a32d516a6。只迁入 navigation_vcl06_assertion.py 和 test_shared_corridor_gate.py；共享门航段索引默认关闭，显式开启才允许合并中继。
- 导航目标仓 https://github.com/sakelier/liftrace-controlwork.git。复制源码标注来源，不能把后续搬运提交当作原实现作者。
- 未操作 F、主根工作树/index、H 或 EV；未创建、删除或改写分支/ref。
- 关联适配说明：
- /home/xhj/liftrace-controlwork-worktrees/high-view-liveness-20260919/docs/verification/slot_h_settlement_20261008/ADAPTATION.md
- /home/xhj/liftrace-controlwork-worktrees/drop-height-board-reference-20261003/docs/verification/slot_h_settlement_20261008/ADAPTATION.md

## 旧链保全与改动范围

编辑 controller 前已对本工作树原控制包与 F 父提交逐文件字节比较，确认一致。复用 073c8f19d7b7a2aa6b88bc536e990af59523c4d4 的 legacy_baseline/20261008/drop_slot_h_handoff/；SOURCE.txt 追加目标 HEAD、父提交和来源引用。CATKIN_IGNORE、FILES.txt、SHA256SUMS 保留，不参与编译，不重复哈希其他文件。

迁入新版 controller、杆臂与运动停稳 helper、DropAlignmentFeedback 消息/aligner、H 参数 helper/生成器、必要消息依赖、launch 接线和相关测试。参数表示 FC→投口 FLU 杆臂：后(-0.12,0)、右(0,-0.12)、左(0,+0.12)m；靶心减去 R(q)r 得到 FC 目标。高位补偿到位和视觉语义证据冻结后下降，释放仍需最终 actual outlet/冻结 FC 的 XY 与运动稳窗。

保留本分支现场 flight_area、motion_optimization、motion_action_timeout 和最低 drop AGL 0.35m。trial_config 只迁入 H helper 调用与补偿语义，settings 只追加 landing_posctl。补齐 competition 生成器依赖 runtime_base.yaml；未引入完整 competition_application 或替换执行机构。对应 2 个完整应用/实体包测试显式跳过，其他用例待运行。保留原 untracked merge_bags.py 和试飞产物，未纳入暂存。

没有未解决 Git 冲突。各分支差异采用文件级最小移植处理，未整仓覆盖，未覆盖共享文档既有条目。具体文件见 PREPARE_STATUS.json。

## 实际生成与接线核对

实际生成器离线调用：03/04/08 各 3 个 FC local Z（0、-0.05、0.09m），共 9 组；另有 3 组自定义 POSCTL 字段消费检查和 3 组 AUTO.LAND 保持检查，均通过。详见 offline_preview/SUMMARY.json。

三树 competition 生成器各检查 4 个 profile × 3 个 FC local Z，共 12 组，通过。已有 field_20261007_validated 几何保持原值；仅缺现场几何的 example/candidates 在内存中填测试夹具，原 YAML 不变。输出见 competition_preview/SUMMARY.json，临时几何不代表可部署的实测路线。

| 检查 | 离线生成/静态核对结果 |
| --- | --- |
| H POSCTL | land_height = ground_z + 0.35m，auto_land_height = ground_z + 0.37m |
| 示例 local Z | FC 静止 local Z=0，FC clearance=0.22m：ground_z=-0.22m、land_height=0.13m、trigger=0.15m |
| 最终 H 稳窗 | XY≤0.05m、Z 误差≤0.02m、水平速度≤0.03m/s、垂直速度≤0.05m/s、至少3新样本且跨度0.5s；odom age/gap 均0.2s |
| motion_feedback | 生成值 /mavros/local_position/odom、twist_frame=child；application 的 controller group remap 到 /navigation/local_odom，按完整姿态转换速度 |
| 补偿语义 | body_flu_lever_arm；controller compensated_alignment=true；三槽表保留12cm |
| 视觉接线 | 正式入口 context=true；require_compensated_alignment 继承该开关并传给 drop_aligner；YAML 缓存值16且 aligner 读取同名参数 |
| 生成配置消费 | application 传 generated_dir/control.yaml 给 waypoint_config，patrol_control launch 加载该文件；controller 读取 land_height、external_landing/auto_land_height、external_landing/posctl/* |
| AUTO.LAND | helper 保持原 ground_z+0.40/0.55m；B/导航板端参考离线检查通过 |

F073 的实际文件清单含 deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_config.py，且包含 landing_control_parameters 调用，并非仅增加 YAML 字段。B 不存在独立 trial_control.py/trial_control.yaml，实际入口链为 start.sh → start_trial.sh → run_trial.py → trial_config.generate → application.launch → patrol_control_px4_sim.launch。原 live preview 会启动 ROS，本轮未执行该路径；本轮直接调用相同真实生成器，以合成静止参考离线生成配置并静态检查接线。

## 验证界限与待办

已完成变化文件 Python AST、YAML 和 launch XML 解析、新增消息/CMake/package 依赖声明核对、git diff --check，HEAD 与 index 保持原状态。未运行 ROS、Gazebo、实机动作、C++提取编译或 catkin 构建。未运行整套控制/视觉单元测试；源 F 的166测试/build及dbd的62离线测试属于来源信息，不能计入本分支成绩。

主代理反馈固定 dbd 的 Gazebo 首个红十字到位但未冻结，正在修 feedback future/odom future 清稳窗时序。当前已承接 Noether/Ptolemy 时序运行补丁（详见下节），尚未承接 Singer/Popper 新增测试；不得将本轮离线 PASS 记为动态 PASS。后续顺序：承接新增测试 → 主通知后执行相关验证 → 主给动态结果与提交通知后再处理提交，继续不 push。

软件测试/仿真不等于实投落点误差或比赛得分。04/08 的真实现场几何仍由现场 profile 提供。


## 后续增量：F 未提交时序运行补丁已同步

以 F 当前 HEAD dbd6e3e2fafb7a9c00854a3bd38efa6a32d516a6 为基点，读取其工作区未提交增量，仅同步以下三个运行文件：

- patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp
- patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/patrol_control.h
- vision_ws/src/uav_vision/scripts/drop_aligner.py

同步前三树对应文件均与 dbd 基点逐字节一致，因而可直接使用 F 对应文件；同步后逐字节确认与读取的 F 快照一致。各分支 CMake、launch 接线与现场 profile 保持本次增量同步前的状态。源补丁保存于 timing_runtime_from_dbd.patch；该来源尚未提交，不能冒充新 commit revision。

控制器区分短暂 motion_time_pending 与真正无效/陈旧/明显未来的运动反馈；暂未到本地评估时刻时保持稳窗、等待，且不发布错误 invalid 反馈。aligner 对短暂未来 feedback/context 做有限等待，达到本地时间后按原时效、上下文与唯一观察戳门控处理；观察缓存仍为16，未来反馈队列最多8，等待参数默认0.1s且约束在(0,0.5]。未降低物理位置/速度稳窗门槛。

主代理已报告 F 整包编译 PASS。本工作树仅完成 Python AST、diff 空白检查及源文件一致性核对，未自行编译或运行 ROS；未同步仍在 Singer/Popper 编写中的新测试。三树仍未暂存、未 commit/push，等待新增测试和主代理动态结果。


## 最终增量：读取原因快照与最终测试同步

本节更新前述准备状态。读取 F 相对 dbd6e3e2 的最新运行源码和最终测试，三树同步后的指定文件与读取快照逐字节一致。源差异保存于 timing_final_runtime_tests_from_dbd.patch；未修改各树原有入口差异。

- freshMotion 改为可选第三参数 const char** rejection = nullptr；一次 motionFeedbackStatus 读取同时返回有效性与拒绝原因。补偿停稳与 H 停稳调用消费该次返回原因，不使用第二次时钟读取判断 pending。
- H tick 先获取本 tick 的 motion_pending 快照，再据此决定是否消费 external_landing_new_mark；odom callback 同样消费已获取 reason 判断 pending。
- drop_aligner.py 与上一轮 F 时序快照相同，已核对保留；context/feedback 有限等待行为不变。
- 同步 test_compensated_drop_geometry.py（19用例）、test_landing_motion_settlement.py（7用例）、test_external_landing_handoff.py（补齐 motionTimePending stub）、test_compensated_drop_alignment.py（28用例）。用例数量来自静态收集检查，不是本树执行通过数量。
- 新控制几何/H 测试 stub 的 freshMotion 同样支持可选 rejection，避免沿用旧两参数声明导致编译失败。

已检查 Python AST、三参数声明/定义/调用接线、H pending 快照、完整指定文件与 F 快照一致性，以及 git diff --check。未运行测试、未提取编译、未构建或启动 ROS。主代理此前运行补丁整包编译 PASS；本次最终 runtime 的 F 整包重建由主代理进行，结果尚未在本请求给出。三树保持未暂存、未 commit/push；等待本地执行通知、最终 F 提交 revision 与动态验收结果。


## 提交收口：95ce182e

最终来源提交：95ce182ed8b912793763327014697def9ba3f71d。本工作树三个运行源码及四个测试文件均与该提交 blob 完全一致，保留最终 freshMotion 可选第三参数 const char** rejection = nullptr、同次 rejection 返回和 H tick pending 快照；不再以未提交快照作为当前来源。精确提交差异保存于 timing_95ce182e_from_dbd.patch；此前快照补丁仅保留过程记录。

主代理已确认 F 在该提交的 controller125、vision28 与完整 build PASS；这些是 F 的来源验证，未计为本树测试成绩。本树已有轻量 preview/静态检查记录，未新增编译或 ROS 执行。三树准备完成，等待主代理 H-only 实跑与动态结果后续报告；未部署、未上板、未暂存、未 commit/push。


## F5f117e18 高位容差分离与本地提交验证

来源5f117e18ddd69c0e266e24f41a03ae3e22ee4511，相对95ce182e共14个文件。controller.cpp/h、H运动测试、POSCTL helper及对应参数测试已同步；七个设置模板只增加 capture_height_tolerance_m=0.10，保留本分支其他配置。导航高位没有 board_trials，其三个模板不引入；competition设置仍接入该参数。源 REPORT 保存在 SOURCE_5f117e18_REPORT.md，原共享文档条目保留并追加本分支记录。

高位 capture 只将高度容差独立为±0.10m；低位 POSCTL 仍为±0.02m、目标AGL0.35m、触发上界0.37m，以及原XY/速度/样本稳窗。最终freshMotion第三参数与时序等待保留。

本树生产生成器与入口定向离线测试结果：39 PASS / 2 跳过，零FAIL/ERROR。完整日志与跳过说明见 generator_entry_tests.txt 和 generator_entry_test_status.json。新增 test_slot_h_entry_wiring.py 实际生成 trial/competition 配置，并核对 controller加载control.yaml、odom remap、feedback发布/订阅话题匹配、物理12cm杆臂表、POSCTL参数、视觉开关及16条缓存；未运行ROS、未进行C++提取或全包重编。生成预览已按最终helper更新。

主反馈 F5f buildPASS、control catkin125PASS。H040634已锁点并提出低位请求，但未观察到POSCTL模式而超时，另查仿真接线；F完整seed38正在进行。**待动态验收，未部署板端**，不记录H动态PASS，不把离线测试当作实投误差或比赛得分。

本分支完成离线检查后按用户授权中文提交、正常推送必要远端；不操作main、不squash、不改写历史、不部署。


## 后续收口：33a521a8 H视觉/运动并行累计

新增独立记录[ADAPTATION_33a521a8.md](ADAPTATION_33a521a8.md)，本节与该记录给出当前状态，前述准备/发布过程保留历史含义。controller与两个H测试精确同步33a；视觉10图与运动0.5s并行计数，freeze仍需两项同时成立，阈值不变。DYNAMIC_FULL.md引用5f已完成的整场；不覆盖本枝共享REPORT。

本轮本枝入口离线检查4 PASS；H运动测试收集10项、nose方法命名修复已同步，但本枝未执行C++用例/编译。F控制catkin135PASS和编译PASS为主反馈。5f full041818三投两门Hland上锁COMPLETE、413.573s，Gate因包络树0.144mm actual_collision仍FAIL；H045333真POSCTL模式交接成功；新H33a正在进行，未写最终H动态PASS。待动态验收，未部署板端。
