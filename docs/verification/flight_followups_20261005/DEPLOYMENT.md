# 部署配置四项小修（2026-10-05，本地待统一提交）

仅修改下表中的现有部署文件、对应定向测试和本说明。未连接 SSH、未部署、未飞行，未写共享联调日志或 AGENTS.md，未提交或推送。主代理负责统一记录、评审与提交。

| 修复 | B：r2026-board-vision-tests | C：r2026-board-frame-fix | H：r2026-high-view-search |
| --- | --- | --- | --- |
| 正赛干净构建显式线程数 | 没有 deployment/competition/build.sh | 默认 FAST_LIO_MATCH_THREADS=3，环境可覆写 0～4，非法值构建前拒绝 | 没有该文件 |
| 专项墙面与起点一致平移 | 修 trial_motion.py | 没有该模块，未新增 | 同 B |
| CLI 只更新 motion_optimization.enabled | 修 run_trial.py | 没有此 CLI 选项，未新增 | 同 B |
| 轻量录包加入 /laserMapping/realtime | 修 trial_bag.py | 直接修共享 hardware_bag.py 默认话题名单，正赛/专项共用 | 同 B |

线程数只传给导航构建；BUILD_JOBS 是编译并发度。显式设 FAST_LIO_MATCH_THREADS=0 可回到 CMake 原架构默认，不是本次板端推荐值。本轮未执行 ARM 构建，板端 cache、编译宏及运行线程仍需后续核对。

墙面使用 runtime.mission.home_xy[wall_axis] 平移，和生成航点采用同一起飞参考。X 起点 +0.2m 时，[1.4,2.1] 生成 [1.6,2.3]；Y 轴墙面仅加 Y 起点偏移。配置原对象不被修改；轴必须为整数 0 或 1。

--motion-optimized 保留原字典，仅设 enabled=True。moving_recovery=false、恢复高度、新鲜度等子配置保留；非字典配置拒绝，不静默替换。

诊断话题不增加原始雷达点云、全场地图或独立 MP4。默认保留单路 /sdf_map/occupancy_inflate、5Hz 压缩图像和 lz4；record_map_clouds / record_inflated_cloud 开关语义保持。

C 的录包实际由共享 hardware_bag.TrialBag 执行。本次按主代理审阅意见直接将 /laserMapping/realtime 加入共享默认名单，正赛及专项共同继承。已撤销先前新增的 run_trial.py 动态绑定与 trial_bag.py 子类层，仅回退本任务引入的内容；保留他人原改动，不修改 hardware_session.py。

## 改动文件

B/H 各自：
- deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_motion.py
- deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py
- deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_bag.py
- deployment/board_trials_4x4/common/uav_board_trials/test/test_deployment_followups.py
- 本说明

C：
- deployment/competition/build.sh
- patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/hardware_bag.py
- deployment/board_trials_4x4/common/uav_board_trials/test/test_deployment_followups.py
- 本说明

## 验证

- B 定向测试：6 项通过、2 项不适用跳过。
- C 定向测试：4 项通过、4 项不适用跳过。
- H 定向测试：6 项通过、2 项不适用跳过。
- B/H 原 test_motion_profiles.py 各 6 项通过，覆盖九组展开、三种航线、投递/高度约束和走廊中继点处理。
- bash -n 与修改 Python 文件语法检查通过。

测试实际调用 generate、CLI main 和 TrialBag.start，不仅检查源码字符串：
- 非零 X/Y 起飞参考下，检查输出墙面和航点，以及传给 optimize_post_route 的墙面。
- 解析 --motion-optimized --check-config 后，将实际 CLI 设置送入生成器，验证子配置保留。
- TrialBag.start 的子进程入口使用替身，捕获 rosbag record、相机节流参数及 bag_topics.json，未启动 ROS 节点。
- 执行 C 实际 build.sh，在临时工作区用 catkin_make 替身捕获完整命令：缺省传 3、覆写 1/4/0、非法 9/bad 在任何构建前退出。未重编译大工程。
- C 验证正赛共享会话及专项兼容导入引用同一个默认 TrialBag 类；实际 CLI 使用共享会话替身，并分别检查专项和 competition_recording 的实际录包命令包含诊断话题，无动态替换或子类。

复跑（所选工作树根目录，既有 rl_drone 环境）：

    source /home/xhj/miniconda3/etc/profile.d/conda.sh
    conda activate rl_drone
    python deployment/board_trials_4x4/common/uav_board_trials/test/test_deployment_followups.py -v

B/H 原回归：

    PYTHONPATH=deployment/board_trials_4x4/common/uav_board_trials/scripts:patrol_uav_ws-patrol_planner/src/uav_mission/src:vision_ws/src/uav_high_view/src python deployment/board_trials_4x4/common/uav_board_trials/test/test_motion_profiles.py -v

## 高位续扫的板端适配点（首次只读记录及后续处理状态）

后续主代理已在 H 以 high_view_stage_limits.py 的 HighViewStageMixin 供 ProbeManager 与 BoardManager 共用，处理下面第 1/2 项：双向恢复本次初始高位参数、规划 ACK 和有界等待。下列缺口是此前只读检查的历史描述，第 1/2 项不再列为 H 待处理；B/C 的同步与验证由主代理负责。本任务未编辑 trial_manager.py 或共享 mixin。第五组 profile 仍暂不开启，策略默认 resume_survey_enabled=False。

B/C/H 的 application.launch 都启动 trial_manager.py；SITL 专项通过 trial_sim_manager.py 实例化相同 BoardManager。BoardManager 直接继承 NavigationMissionManager，不继承 navigation_high_view_probe.py 的 ProbeManager。

运行时另有继承：FullCircleRuntime 继承 HighViewFull；PriorityRevisitRuntime、MemoryOnlyRuntime 等继承 FullCircleRuntime；FullMissionTrialRuntime 直接继承 HighViewFull。第八组可随公共 HighViewFull 接入新状态，但仍受独立板端限高壳影响。

必要适配点：
1. trial_manager.py 的 _publish_action 独立维护 _low_limits_applied：仅在 REVISIT / LOW_COVERAGE 写入低限高一次，没有恢复高限高或本次 planner ACK 等待。需在 RESUME_ASCEND / RESUME_JOIN / SURVEY 恢复高位参数、确认新请求 ACK 后派发；再次回低阶段重新应用低约束。仅修 ProbeManager 不覆盖这里。
2. trial_config.py 只生成 low_stage_parameters；续扫需要明确可恢复的高位参数及规划/控制共同限高（包括 Bridge、虚拟顶棚、goal_adjustment 等已覆盖参数）。恢复值从专项初始配置留存，不硬编码正赛高度；H 扫描上限也要保留。
3. PriorityRevisitRuntime._start_fallback 覆盖公共路径，最终进入 FullCircleRuntime 的记忆任务结束/失败分支，没有走 HighViewFull 的新 _try_resume_survey。第五组以后启用时，需保留可靠候选/冲突/近墙复核优先级，再接公共有界续扫。
4. FullCircleRuntime._all_top 按冻结的 trial_manifest 限定类别，PriorityRevisitRuntime 沿用。需核对筛选和 expected 投递计数，防止续扫找到的缺失类别被清单过滤。
5. FullCircleRuntime 明确关闭提前中断；MemoryOnly/HighSpeedCapture 具有仅记忆/采集终点，不应统一强行启用续扫。

以上只读确认，不修改任何 settings/profile 或上述运行时代码。续扫运行链适配由主代理另行评审与验证。
