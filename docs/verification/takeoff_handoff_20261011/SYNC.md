# 起飞完成保持与默认回退定向同步

日期：2026-10-11。目标分支：feat/board-deployment-flight-20260920；同步前 HEAD：0b904bb39b36bf0a8542b9a363556250a9922062。
来源仓库：https://github.com/sakelier/liftrace-controlwork.git
来源分支：liftrace_board_trial_20260928；来源提交：421e6023d69e7026e3151770d8b647ec793004df。
来源说明见该提交 docs/verification/takeoff_handoff_20261011/REPORT.md。

外部 Takeoff 完成后保持原起飞终点和首航点 yaw，设置已有 waiting-for-motion 状态，避免最终限距输出覆盖原始保持目标；等待运动命令授权后，新有效 planner 按原接口接管。
现有正式/08 模板默认 takeoff_z_lead_m=0，显式 0.4/0.6 试验选项保留。仅移植该修复的 runtime、现有配置、测试及本记录，不迁移其他补丁或来源整包。
保留本树 HAS_TRIAL/HAS_FORMAL 条件和原有限场地/比赛速度档位、通用限距。

修改前本树旧控制包保护快照：legacy_baseline/20261011/takeoff_handoff_sync_r2026-board-vision-tests/，含原包、文件清单、SHA256SUMS 和本树 HEAD，不参与编译。仅快照场景计算 SHA256。
定向回归：31 项通过，0 项缺本树入口跳过，合计 31 项。g++ C++14 + UBSan 编译并执行提取自本树生产代码的连续交接、限距/起飞测试；配置/实际 launch 展开为离线检查，不启动 ROS 节点。
测试使用已有 rl_drone 与 Noetic、独立 PYTHONPYCACHEPREFIX；入口 python /tmp/takeoff_sync_20261011.py tests 0。
缺入口跳过原因：无。
git diff --check PASS。来源主审实际 patrol_control 编译 exit 0、30 项回归 PASS；目标整包构建由主审 B 流程负责，本次未重复 catkin_make，CMake 仅注册新回归。
剩余边界：没有动态仿真、上板或实飞；原完成条件不变，默认回退不证明解释全部高度超差。后续现场部署需重新生成配置，再确认终点保持、自动高度/稳态门槛及实际跟踪。

## 主审晚间对准与投后分析引用

本地分析：/home/xhj/liftrace-deliverables/postdrop_slow_20261011/TIMING_AND_ESTIMATE.md
本包 panzer/bridge/red_cross 从 External ALIGN 到正向 Servo ACK 分别为 15.04/19.51/38.52 秒，包括捕获、补偿、下降、条件等待及执行回执，并非纯视觉耗时。
红十字主要等待末段反馈高度约 0.45–0.47 m 超过 0.45 m 上限，最终约 0.449 m 才调用；无证据把等待归为重新视觉确认或仍有速度累计门槛。
首投后日志明确进入 BOUNDARY_REVISIT 前视 0.2 m，离边恢复 CRUISE 1.0 m，接近红十字为 PRECISION 0.4 m，不能据此称投递限幅未恢复。
三次软件 ACK 不证明零误识别或实物落点；没有全量人工真值。原 bag 和大型本地产物保持原位，未复制或修改。
