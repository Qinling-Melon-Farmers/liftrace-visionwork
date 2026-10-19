# 停滞恢复与视觉集成接口（2026-09-19）

用户已授权按两仓设计开展整机工作。导航实现位于本分支；视觉集成仓对应 `docs/planning/reliability_speed_review_20260919/IMPLEMENTATION.md`。不改变旧投递服务或候选消息结构。

- 新增可选 `/planning/progress` (`plan_manage/TrajectoryProgress`)：FSM/server分别发布10Hz心跳及首次状态变化事件。用source、traj_id、traj_start关联两端；server的goal_seq为0，不能当成任务身份。任务/decision对应既有key_events。记录实际odom、投影、原始/最终命令、两端参数进度、保持原因、生效前视、恢复次数、输入年龄。默认关闭，集成实验启用。视觉记录端限额10MiB，满后明确标记截断；不写控制路径磁盘。
- FSM新恢复默认关闭，实验启用。以既有 `/detect/point_class` (`Int8`, Run_point=1)的新鲜状态为实际运动许可；odom与地图均新鲜才累计4s/4cm的物理无进展窗。规划generation变化不重置窗口或次数，同目标最多2次恢复；原decision和deadline不变。主动对准/降落/悬停排除。位移使用XYZ，不以离终点变远判停滞。
- 原目标还未实到时预算耗尽，发布既有`FAILED_ATTEMPT`、reason=`liveness_budget_exhausted`，导航执行桥在完成原有goal/attempt/事件时序检查后映射为终态FAILED，保持并交任务层处理。该原因不等于到达；门不能跳过。一般规划失败仍非终态。迟到或重复轨迹不可清除保持。
- `Bspline`补充原目标`goal_stamp`/`goal_frame`，沿用执行桥已经使用的源stamp身份（ROS会重写Header.seq）。可选`require_goal_identity`在实验启用：新目标使server保持，只有匹配新目标且更新generation的轨迹可解锁。需两端重新编译；默认不强制以兼容其他旧规划器入口。
- 高位线索只用于导航。近墙REVISIT选择内侧可见点并切精细跟随，仍由3D规划器执行；低位新鲜候选和释放许可不放宽。55cm为已膨胀设计包络，不重复加10cm。新增近墙候选准入可拒绝无法在配置区域内执行的APPROACH；不移动场景或掩盖失败。
- 保留全高度障碍柱、独立下降、原始Gate规则。离线测试不证明避障可靠，最终一轮随机高位整场单独报告。

## 失败轨迹复现后的投影修复
seed36 traj20（ROS203.62，26个控制点）在零噪声位置跟随回放下稳定复现：前视弧长不足0.15m却已到参数2.72s，原进度投影只查前1s，导致projection恒0、距终点1.369m。改为连续路径弧长局部窗（至多0.4m，并不大于生效跟随前视）；FSM/server采用同一口径，不能跳到远处回环或按时间强推。原样条冻结到单测，旧实现失败、新实现可收敛到3cm内；4项既有曲线保护也通过。保留有限恢复，不将回放成功等同SITL成功。
