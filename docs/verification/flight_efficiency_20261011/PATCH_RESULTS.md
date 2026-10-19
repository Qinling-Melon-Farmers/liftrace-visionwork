# 控制补丁收尾结果（2026-10-11）

门点保护、起飞专用Z领先已完成独立审查、独立提交并同步并推送到六个功能分支；六支控制目标实际编译及链接均通过。**本轮仅离线测试与构建：无仿真、板端连接、部署、实飞或GPU任务。推送结果见文末。**

## 提交与验证

门点保护与起飞专用Z领先两补丁均以导航0928为源头，完成审查、分开记录/提交、同步与构建。下表为完整commit，两个补丁各自可回退。

| 工作树 | 门点commit | 起飞commit | 门点tests | 起飞tests | 控制链接 |
| --- | --- | --- | --- | --- | --- |
| B/integrated | f2f0a2f5dfa961bf49cdda4928edf9cba06ce18a | 74edf2f258cb619aadfceb140fbfdf813bfc612f | 26 PASS / 0 skip | 19 PASS / 0 skip | PASS |
| B/board | dad5c62ebc03d2b60b23ff39f812d07ad71abdbf | df6be10786e8adc1c361e7a4e72c9f6e0a1bcc59 | 33 PASS / 0 skip | 19 PASS / 0 skip | PASS |
| B/high-view | d7142dd25fa3e332282c8b8bb78369980aa82ee4 | d2552f1ab9cd8511f9b1fe72ecda038ff5154dbe | 27 PASS / 6 skip | 11 PASS / 1 skip | PASS |
| N/0928 | 3bf76a82d14eb9c726c464d503d8306898e51a3a | b19010c381a720565a94d3f64b2e36c7733e69d9 | 33 PASS / 0 skip | 18 PASS / 0 skip | PASS |
| N/reference | 367cdcca0b1aaea43a3b0ff18d0b8c51eb3515fb | 99174139f7add846716229681706095f4e03adf0 | 33 PASS / 0 skip | 19 PASS / 0 skip | PASS |
| N/high-view | 7a3797661f5a11dbed4b6d2aee07eaa2fd720113 | 3ac089c5cc924f42dacebc3ebaa7617c87febfbc | 26 PASS / 0 skip | 16 PASS / 3 skip | PASS |

工作树：

- B/integrated：`/home/xhj/liftrace-worktrees/r2026-board-frame-fix`
- B/board：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests`
- B/high-view：`/home/xhj/liftrace-worktrees/r2026-high-view-search`
- N/0928：`/home/xhj/liftrace-controlwork-worktrees/board-trial-20260928`
- N/reference：`/home/xhj/liftrace-controlwork-worktrees/drop-height-board-reference-20261003`
- N/high-view：`/home/xhj/liftrace-controlwork-worktrees/high-view-liveness-20260919`

- B/integrated：feat/r2026-competition-integrated；B/board：feat/board-deployment-flight-20260920；B/high-view：feat/high-view-route-speed-20261004。
- N/0928：liftrace_board_trial_20260928；N/reference：板端参考分支；N/high-view：feat/high-view-liveness-20260919。
- B原分析983330de仍为当前HEAD祖先；未cherry-pick来源大文档覆盖B。所有运行修改均已提交，无本任务运行文件残留diff。模型脚本、model_refresh_20261011/docs、工具新增与试飞资产均未纳入本任务提交；保留其他代理文件。

## 审查后的行为

1. 门点：按原始index保护已知墙平面每次跨越两侧锚点及墙面点，覆盖反向、重复坐标、多墙/斜向跨越；保留H后缀、普通共线合并、真实移除索引及阶段计数。修的是既有合并丢失门前后引导点；没有更改规划器碰撞预测/轨迹截断/地图判据。
2. 起飞：只在Takeoff启用takeoff_z_lead_m，专用Z分支与generic 3D限幅互斥；双向限定到终点的Z步长，保留XY、终点、后续软件限高、恢复例外、ready和模式转换条件。C++缺省0；0/缺失/非法回退原generic。有效候选0.4～0.6m，配置生成非法值提前拒绝，正式入口与08显式0.6m，01～07和09显式0；03未启用。
3. 最终审查补充了当前Z高于终点时的有界下降，避免专用分支跳到较远低目标；目标同步修正overrides在YAML序列化之前赋值。参数日志仅加载时输出，没有逐帧新增日志。领先量是距离，不能写为起飞速度或保证节时。
4. B高位分支保留原Run_point限高保持/重规划代码，数值回归直接对照该支修改前代码；它没有正式生成器或可选speed-profile模块，只测试其真实专项默认入口，未补不存在模块。N高位无专项入口，未复制该入口。各支13份既有YAML字段比较通过，除新增takeoff_z_lead_m外保持一致。

B代码入口行号：motion_optimization.py:67；patrol_control.cpp:66（校验）、1686（最终限幅）、1878（参数加载）；trial_config.py:82/259/331；competition_config.py:36/198/244。完整路径均在patrol_uav_ws-patrol_planner/src中，专项trial_config在deployment/board_trials_4x4/common/uav_board_trials/scripts中。

## tests与构建范围

- 门点：test_motion_optimization.py及存在时test_motion_profiles.py。B高位6项仅因无正式入口跳过；B集成与N高位无专项回归文件，因此只执行已有26项；没有把缺入口报PASS。
- 起飞：test_takeoff_xy_hold.py、test_takeoff_z_lead.py、test_takeoff_z_lead_config.py，以及存在时test_competition_launch_parameters.py。六支各从本支生产C++提取validator/最终输出块，以UBSan验证384场景，其中320对照其他模式旧算法；覆盖关闭/非法参数、XY保持、近终点/超过终点、限高及恢复例外、Takeoff→Run_point切换。
- 实际launch XML仅离线展开，无节点启动；正赛参数及08入口0/.4/.6对照只允许新增起飞参数变化。源头及四个有专项入口的同步目标核对全部九组；B高位测试原默认速度入口，其余有速度档模块的工作树测试08的limited/competition两档。B高位起飞1项、N高位3项因缺入口跳过；各分支既有正式回归数量不同，表中记实际结果。
- B三支：本树patrol工作区执行catkin_make patrol_control，ROS系统Python、-j2 -l2，实际编译并链接。导航三支：/tmp/control_closeout_20261011内独立Catkin工作区，patrol_control/navigation_recovery_msgs/uav_vision均链接各自本树源码，重新生成消息并实际链接控制可执行文件；不是仅语法检查，也不是完整规划工作区验收。
- N/reference首次原整工作区配置失败：plan_manage无法解析navigation_recovery_msgs。随后独立控制构建PASS，未修改CMake或冒称整导航工作区PASS。N/0928隔离构建初始关闭测试时遇到既有无条件catkin_add_gtest；改为测试配置ON后通过，无CMake补丁。
- 运行文件git diff --check通过。导航源头旧控制快照legacy_baseline/20261011/takeoff_z_lead，B高位另有自身快照takeoff_z_lead_high_view，清单与SHA校验通过；原快照行尾空白保留，不为diff检查重写历史包。

原测试、构建及参数比较日志保存在/tmp/control_closeout_20261011：nav_review_tests.log、nav_takeoff_final_tests.log、door_tests_*.log、takeoff_tests_*.log、build_nav0928.log、build_B_*.log、build_isolated_N_*.log、build_N_reference.log（原失败）、parameter_preservation_review.json。各支小型同步记录在docs/verification/control_closeout_20261011；源头报告在door_waypoint_protection_20261011和takeoff_z_lead_20261011。

## 版本与实飞边界

B主仓main仍3d72e3e92625a953fd43e0dcd9a4db974e1f813f（旧默认释放高度0.35m），导航main仍a68925d15293e5510e2b4351c6b3d9bc5aa136ab；main/EV/PX4未改。0928/reference候选正式及08配置保持drop_agl=0.40m及既有0.35～0.45m释放窗口，这两个补丁未改投递参数。此次新增0.6m是Takeoff领先距离，与0.40m释放高度不同。**候选源码不等于已部署版**；本轮未连接板端核实运行revision，不能由旧manifest或当前HEAD认定包内完整运行版本。

门点保护不保证无撞击，第一门实物中心/门框真值仍未闭合，无法将混合高度点云空洞写成free。起飞动态速度、姿态、ready达成时间及节时尚无本补丁实飞数据。下一步经明确运行授权后，以同一场地先核实门前后锚点与真实净空，再单因素对照takeoff_z_lead_m=0与候选（0.4～0.6）；0可直接恢复原generic。不得据这些测试宣布Gate实飞通过。

## 分支推送结果（2026-10-11）

用户已授权必要feature分支远端同步。本次显式推送各功能分支对应ref，无force、无tags或其他分支推送；七个远端分支更新均成功，逐项ls-remote回读HEAD一致。

| 工作树 | 远端 | 分支 | 首次推送HEAD | 回读 |
| --- | --- | --- | --- | --- |
| B/integrated | origin | feat/r2026-competition-integrated | 74edf2f258cb619aadfceb140fbfdf813bfc612f | PASS |
| B/board | origin | feat/board-deployment-flight-20260920 | b19e190b86d69df0b1569ee98fa8e8d203a6164a | PASS |
| B/high-view | origin | feat/high-view-route-speed-20261004 | d2552f1ab9cd8511f9b1fe72ecda038ff5154dbe | PASS |
| N/0928 | origin | liftrace_board_trial_20260928 | b19010c381a720565a94d3f64b2e36c7733e69d9 | PASS |
| N/reference | origin | 板端参考分支 | 99174139f7add846716229681706095f4e03adf0 | PASS |
| N/high-view | origin | feat/high-view-liveness-20260919 | 3ac089c5cc924f42dacebc3ebaa7617c87febfbc | PASS |
| N/high-view | fork | feat/high-view-liveness-20260919 | 3ac089c5cc924f42dacebc3ebaa7617c87febfbc | PASS |

表中B/board为首次推送b19e190b；随后仅本报告与docs/仿真联调变更记录.md追加独立docs提交并推送同一origin分支，本报告所在提交为这次报告更新revision。最终B远端HEAD回执记录于/tmp/control_push_20261011/final_B.tsv；首次七项回执results.tsv及push_*.log在同目录。

导航liveness按既有origin+fork双远端同步；导航0928及reference推送origin。main/EV未改或推送；B模型未提交脚本/docs、其余代理文件及资产均未暂存或纳入提交。本次仅文档与Git远端同步，未重跑构建/测试、未仿真或实飞。
