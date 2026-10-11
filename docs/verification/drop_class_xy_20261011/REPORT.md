# 按绑定类别选择最终投递XY门限

导航权威工作树 board-trial-20260928，既有分支 liftrace_board_trial_20260928。本提交仅控制阈值与配置接线，不含先补偿后下降策略。标准四类 tent/pillbox/bridge/panzer复用 drop_system/settle/xy_tolerance_m，默认0.06；红十字独立 red_cross_xy_tolerance_m，默认0.04。未知类回退 min(0.04,红十字阈值)，绝不继承标准放宽。依据已绑定 compensated_context_.semantic_target_class，live与冻结事务不一致仍拒绝。

双FC/投口误差保持严格小于阈值；高度窗口0.35–0.45与0.40下降目标、许可、取消、freshMotion、槽位和异步执行规则保留。waiting/release_ready已有日志追加xy_tolerance与class，不新增高频日志。新参数启动检查有限且>0；旧标准参数由既有配置验证。

生产方法提取编译（g++ C++14、UBSan）并运行42项补偿回归全部通过，含7项新增类别/参数/严格边界/服务准入；异步Servo24项全部通过。见control_tests.txt。主代理报告B正式launch6项、08 CLI6项PASS，均为配置生成实际参数检查；导航权威工作树缺devel，本地未运行该CLI，不宣称本地CLI或完整catkin构建通过。B完整控制构建由主代理负责；没有启动实机、仿真或GPU。

配置接线由主代理提供并授权并入：uav_mission/config/competition/control_base.yaml、vcl06_horizontal_control.yaml；对应test_competition_launch_parameters.py与test_competition_speed_cli.py。生成器已有整段settle透传，不修改生成器；部署需重新生成control.yaml并编译新控制二进制。

修改前旧包快照 legacy_baseline/20261011/drop_class_xy/patrol_control，56个源文件（排除可再生__pycache__缓存），FILES.txt及SHA256SUMS。本快照按用户本轮显式规则14要求保存在导航来源开发分支，不参与编译。

实飞仍待验证：第二轮旧轨迹合法高度内最小双XY约6.34cm，因此单改6cm不保证该旧轨迹放行；不可把旧Z加5cm称候选模拟。

## 本分支集成

来源导航 b553b74c；按差异集成控制、分类阈值配置与生产方法测试，保留分支其他差异。B工作树实际patrol_control catkin构建通过；正式launch与08 CLI各6项通过。其他副本不得据此称为板端已部署或实飞通过。main和冻结EV分支不纳入本轮候选同步。
