# 高位分支定向同步投递分类门限与串行补偿

高位起点：fc2c61bc，现有分支 feat/high-view-route-speed-20261004。来源：/home/xhj/liftrace-controlwork-worktrees/board-trial-20260928，分支 liftrace_board_trial_20260928，nav仓 origin https://github.com/sakelier/liftrace-controlwork.git；固定 b553b74c3836ebcf0febdf4931ed273f079d0fbc 与 fe890d5b13de384e2cefa2e70989b60abba67c8e，未消费并行工作树的后续未提交内容。

正式补偿投递取消旧 window.update 速度/停稳/样本数门；冻结前仍要求可靠新鲜视觉与原上下文，释放检查当前FC和投口最大XY误差。按冻结任务类别选择标准靶严格小于6cm，红十字4cm，未知类别不放宽。新增 release_min_height，缺省沿用 release_setpoint_height，可支持0.40目标和0.35至0.45释放区间；本树两份YAML仍保持原0.10目标/下限、0.20上限及原px4_max_distance，不覆盖板端高度到高位树。

从fe890d5b定向叠加 compensate_before_descent=true：circle/cross均冻结后在高位补偿，当前双XY到位后锁存下降；冻结后的高位推进不要求不断更新视觉ready或图像。瞬时XY反弹阻止释放而不回升；service下降前不得准入，两处reset清除下降标记，关闭开关保留并行兼容。生产方法包含 require_release_height 参数、compensatedDropCommandHeight及executeDropAction，测试覆盖实际两种下降分支及开关两种模式。

仅改变控制cpp/header、所需测试和实际加载的vcl06_horizontal_control.yaml、vcl06_full_low_corridor_control.yaml。该树没有competition/control_base.yaml，未引入未使用的新入口。高位目标生成/完整投影实验、H稳定窗及配置、导航恢复、原许可/deadline/异步ACK防重复路径保留；不整文件覆盖。来源nav已保留原包保护快照，本高位精简树不再复制旧包。

实际验证（本高位树，rl_drone）：
- compensated_drop_geometry：最终33/33 PASS（按fe890d5b串行行为适配，而非重复记来源测试数）。
- new_vision_config：12/12 PASS。
- exact_drop_control：49/49 PASS。
- async_servo：24/24 PASS。
- 合计118/118 PASS。最初串行提取测试因高位exact替身缺少成员编译失败，补齐后通过，未为通过而删除生产条件。
- catkin_make --pkg patrol_control -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2：6cm版本及最终串行版本均退出0，最终patrol_control实际链接成功。没有整机新建devel或完整整场构建。
- 原始日志：/tmp/high_drop_sync_20261011/{geometry_serial_tests,config_tests,exact_tests,async_tests,build_serial_control}.txt。

未启动ROS节点/仿真、未新增整场验收、未部署；不承诺消除bag2反弹或实投改善。本轮只中文commit，不push，由主代理处理推送。原四项无关untracked资产保留且不纳入提交。
