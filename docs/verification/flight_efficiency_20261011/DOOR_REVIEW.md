# 第一门避障响应复核（2026-10-11，_1）

已证实的流程是：整段走廊目标 → traj15预测碰撞并发布replan通知 → server锁定位置保持 → traj16替换并解除保持 → 控制继续跟随向X约8.14偏移、同时下降的曲线 → 用户确认撞第一门 → MANUAL。不能将其简写为“仅换FSM而未通知server”，也不能认定一直执行旧危险曲线。未取得同一内部地图上的精确碰撞查询，根因尚未闭合；本轮止于已有数据、源码及短窗地图头信息。

## 范围、现场事实和版本

- 原包：[flight_debug_0_10111009_1.bag](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/flight_debug_0_10111009_1.bag)，242004580字节。t=0是2026-10-11 09:35:15.362926+08，以下t均相对包起点。
- 用户现场事实：撞的是第一道门，本轮没有布设pillbox；撞击侧、精确撞击时刻、今日门框真实坐标未提供。日志只能给出199.3–199.6秒附近运动突变候选窗口，不能替代现场碰撞时间。
- run_metadata的deployment manifest源revision为51f48fc5970b10b5a8507f59874bd04ee8da7640、日期10月8；不是完整运行binary/revision证明。包中drop_agl=0.35。
- 本轮检视源码快照：B=d4974358dbe06322fcc2d0cc02cee603303e862d，N=e0c06431e08c0b8762ce9b2a919962004d40bd3b。与manifest对应源码对照，危险replan通知、server保持/解除、最终occupancy+EDT检查等相关分支均已存在于51f48fc5970源码；仍不等于板端确实执行了这一整套源码。早先38647c92/.40及后续推送属于候选源码，本包不能当作其部署验证；用户指出的主仓main 3d72e3e9默认.35也应独立识别。
- 不改运行代码/参数，不启动ROS节点、仿真或板端，不使用GPU，不commit，不改共享变更记录。复用原records；只额外读取193–200.5秒膨胀可视化点云header，没有导出点云payload或重演地图。

## 危险重规划与保持的实际时序

| 事件 | bag接收t / 秒 | 消息源时间t / 秒 | 证据 |
|---|---:|---:|---|
| /planning/replan Empty | 195.672950 | 无header | 危险曲线截断通知 |
| current traj in collision. | 195.673103 | 195.670704 | planner预报，非物理碰撞事件 |
| 首条保持pos_cmd | 195.675347 | 195.675067 | traj15，位置锁定、velocity/acceleration均0 |
| server首次interrupted进度 | 195.675364 | 195.675040 | interrupted=true，final_command为锁定点 |
| MAVROS首次保持setpoint | 195.719116 | 195.714556 | 实际输出位置与锁定点相同 |
| initial搜索超时诊断 | 195.922750 | 195.921529 | candidates168320，collision105921，accepted329，timed_out=1 |
| 首条traj16 pos_cmd | 196.042481 | 196.035339 | 已恢复运动命令 |
| recorder收到Bspline16 | 196.045373 | 无header | start_time相对起点196.031865，不是publish时间 |
| server首次traj16 tracking进度 | 196.057774 | 196.035238 | interrupted=false |
| TRAJECTORY_READY | 196.064104 | 196.038699 | 同goal_seq15，planning_attempt2 |
| MAVROS首次跟随新曲线 | 196.070929 | 196.069442 | 解除位置保持 |

不同topic的recorder接收顺序不等于回调顺序。例如replan接收略早于rosout，traj16 pos_cmd接收也早于Bspline接收，不能据此推导“未收到轨迹就执行”。源时间支持warning→保持→新轨迹命令这一响应。首次搜索超时后产生新轨迹，不是executor终态FAILED。

锁定位置=(7.839583,1.977809,0.657097)。按pos_cmd接收样本，保持区间约0.367秒；MAVROS保持约0.352秒。恢复时server odom=(7.904786,1.853533,0.662228)，相对锁定点位移=(+0.065203,−0.124275,+0.005132)m，模长0.140436m。FC水平速度在195.694/196.043/196.094秒分别约0.390/0.314/0.265m/s，没有恢复前已停稳的证据。

新命令=(8.063909,1.293182,0.680719)，距server当时odom约0.5828m，字段target_dist=0.6；这是空间前视命令距离，不是水平跟踪误差。保持时raw_command仍可显示旧曲线采样，但final_command和pos_cmd已锁定，不能只看raw_command判定继续执行旧轨迹。

源码对应：[checkCollisionCallback:538](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp:538)在!safe分支先WARN、publish Empty再REPLAN；[replanCallback:253](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp:253)置interrupted并捕获odom；[命令保持:401](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp:401)锁pos、置vel/acc零；[新轨迹安装:234](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp:234)解除interrupted/tracking_hold。普通重规划可继续旧安全曲线，与本次明确危险通知应区分。这里是位置保持指令，没有生成并检查一条物理制动轨迹。控制层实际跟随由bag的MAVROS位置设定点证实，参考[外部planner转发:1391](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp:1391)。

预报时FC中心Y约1.98，距配置第一墙Y=−0.012559还有约1.99m，距疑似撞击窗口约3.8秒。短保持且未停稳是已观测状态，不能仅凭这一相关性认定它造成撞门。

## 目标合并、横偏与高度

原输入走廊点[(6.5,2.1),(7.7,2.1),(7.7,1.0),(7.7,−0.7),(7.7,−1.1),(7.7,−2.5),(7.7,−3.45)]，均AGL0.9；墙axis=Y、相对平面[0,−1.4]。初始化参考FC平移=(−0.008351,−0.012559,−0.035943)，ground_z=−0.255943；本包map→camera_init静态变换为identity。目标生成只加参考XY，不按起飞yaw旋转。

既有共线合并移除了中间四点，走廊7点+H尾段2点变为5点，保留入口/转弯/H尾段。seq13/14/15分别在184.626/190.742/194.862秒发出，实际goal依次为(6.491649,2.087441,0.644057)、(7.691649,2.087441,0.644057)、(7.691649,−3.462559,0.644057)。seq15 requested与effective目标一致：直接穿整条走廊是既有优化行为，不是虚构任务越点。

[N optimize_post_route:67](/home/xhj/liftrace-controlwork-worktrees/board-trial-20260928/patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/motion_optimization.py:67)将整段碰撞约束交给3D planner；合并循环只判共线/同高度/无反向，wall_coordinates在该循环未用于保护门前后锚点。保留端点AGL0.9不等于约束整条曲线恒高。

| 门附近量 | traj15 | traj16 |
|---|---:|---:|
| 配置第一墙附近重建曲线X | 8.144974 | 8.144121 |
| 对应AGL | 0.473811 | 0.423245 |

以上是曲线离配置墙Y最近的采样点，不是实际穿门点。t=199.5附近planner/mission设定X=8.144672，nav FC样本X=8.135628：曲线相对goal列X=7.691649偏+0.453023m，FC相对同时段设定X差−0.009045m。直接MAVROS FC t=199.496为X8.139538，X差−0.005135m。约45cm主要是规划曲线偏移；X方向控制确实跟随，不能记成45cm控制跟踪误差。此时FC AGL约0.514、设定AGL约0.423，二者也不能混为同一高度；Y仍约0.38，距前视设定Y约0.0004有差，横向X的结论不代表全XYZ误差均小。

t=199.5时manager=POST_DELIVERY_ROUTE/RETURN_HOME、index2/size5；bridge planner_accepted=true、trajectory_ready=true、terminal=false、faulted=false；FC为OFFBOARD、armed。MANUAL首次记录200.257850，disarmed202.257800；随后估计X/Z大跳不应计作真实运动距离。

## 地图时序能支持什么

可视化膨胀点云接收：195.407686→196.066416秒；全部header stamp=0。新Bspline的recorder接收比后一个可视化包早约21ms，这不能证明使用旧内部地图。[visCallback:850](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp:850)独立发布可视化；[publishMapInflate:1320](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp:1320)未设置有效source stamp。

只取FSM source=0的/planning/progress，header−map_age得到上游地图source时间：

| progress接收t | map_age / 秒 | 推得上游source t |
|---|---:|---:|
| 195.513949 | 0.199921 | 195.313626 |
| 195.673039 | 0.133161 | 195.537790 |
| 196.065578 | 0.036939 | 196.023071 |

说明预报/新轨迹附近仍有新鲜度更新迹象，不能按可视化约0.66秒间隔断言内部地图停更。[FSM地图stamp入口:104](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp:104)监测上游消息时间，不记录内部occupancy/ESDF更新完成时刻；server source=1的map_age=0不能当地图即时新鲜证明。

[cloudCallback:945](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp:945)更新膨胀占用并置ESDF待更新标记（1165行）；[updateESDFCallback:886](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp:886)另行计算ESDF。周期[checkTrajCollision:102](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/planner_manager.cpp:102)按轨迹投影后的未来曲线、0.02秒采样，仅看EDT<clearance，未与最终检查统一使用occupancy；也不是实机机体/跟踪偏差/制动扫掠体的证明。

最终[is_clear:266](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/src/planner_manager.cpp:266)已经对整条曲线检查有限值、参考高度、inflated occupancy与EDT；优化失败时初始曲线也用同样检查。不能提出“补上缺失的最终碰撞检查”，也不能在板端revision未知时宣称该检查在运行中失效。

包未记录/sdf_map/cloud或/freedom/static_pointcloud原始地图输入、完整内部occupancy/ESDF快照或版本、首次违规采样点/距离。本轮无法回答“195.673究竟哪个体素触发、196.045同体素为何放行”。

已有点云切片仅作定位线索：早期192.403秒在混合Y/Z投影中出现候选缺口X约8.10–8.60，随后同带出现占用；高低切片不一致，不能把缺点区当observed free或真实门洞。详见已有[第一包报告](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/analysis_flight_10111009_1/COLLISION_REPORT_1.md)。没有现场门框真值，不能宣称旧打点错或撞了某一侧。

## 可执行的小修方向与验证判据（本轮未实施）

1. **保护门前后锚点。** 在既有wall_axis/wall_coordinates入口上，让跨墙的最近前后点不可被共线合并。本配置第一墙应保护输入Y=1.0/−0.7，第二墙保护−1.1/−2.5；这四点正是本次被移除的点。最小验证：本route展开后明确保留四锚点，seq目标不再一次跨两墙。它能缩短规划段、保留逐门进度，但端点仍不保证中间曲线净空/恒高。门中心与前后净距须现场复核，不能从点云缺口硬写固定数值。
2. **统一周期与最终安全判据。** 抽取共用“finite+inflated occupancy+EDT+高度”查询，先使周期检查也覆盖occupancy；输出首个违规点、occupancy、EDT、clearance、traj/projection及地图版本。最小验证：固定小地图上的occupied但EDT暂未更新样本，两处均拒绝。现有记录支持检查分支不一致这个源码事实，不支持认定本次发生了ESDF滞后漏判；该假设仍待同图数据。
3. **危险保持解除有明确条件。** 区分普通重规划与unsafe hold；后者收到新曲线时，按最新实测状态复核连接段及制动/跟踪包络，并记录解除原因，避免仅按形状/代际/高度验收后清除interrupted。最小验证：unsafe通知后仍有速度的状态输入，解除前能说明最新曲线与当前状态、地图版本的关系；普通重规划不增加无必要停顿。本包证实未停稳即恢复，尚不能把它列为碰撞根因。若选择先停稳再过门，速度阈值/等待上限须按实机能力确定，不能拿元数据braking_accel当已测制动性能。

优先级建议：先做第1项及第2项诊断与判据统一；第3项需要制动能力和连接段验证再定参数。需要保护的不只是0.9端点：本次曲线下降到AGL约0.42，是否可过门取决于现场机体包络及门框，不宜未经核实强加新的高度阈值。

## 最小补录与边界

只需门前触发的短窗/环形缓存，不需完整地图长时重演：记录unsafe触发和新轨迹验收各自对应的**内部occupancy与ESDF快照或可回放的局部子图、版本及更新完成时刻**；保存首次违规点、EDT/occupancy、投影t、clearance及实际膨胀/分辨率/高度参数；记录hold进入/解除理由、当前odom源时间与速度及新轨迹id/goal_seq。若只补上游原始cloud和header，仍可能不能还原内部回调顺序/ESDF版本，应明确保留这一限制。部署时另记完整两仓revision、实际包路径及二进制对应构建版本。

门框平面/边缘、厚度/净空及机体包络由现场测量闭合。现有日志能证明规划横偏与控制追随，不能证明真实门中心偏差或某侧碰撞。

## 本轮数据产物

- [door_review_timing.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/analysis_flight_10111009_1/door_obstacle_review/door_review_timing.json)：约21KB，事件、保持/速度、目标、地图新鲜度与边界；不复制全量cloud/odom。
- [door_short_window.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/analysis_flight_10111009_1/door_obstacle_review/door_short_window.csv)：195.5–199.6秒83行、50ms网格的最近历史样本；保留每topic实际接收t，不当精确同步。
- [map_receipts_versions.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/analysis_flight_10111009_1/door_obstacle_review/map_receipts_versions.json)及[只读提取脚本](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/analysis_flight_10111009_1/door_obstacle_review/read_map_receipts_versions.py)：短窗可视化header、源码分支行号、原包stat不变检查。
- 复用analysis_flight_10111009_1/collision_1_records.json；原记录及既有报告保留。未补录、未实跑，以上验证判据是下一步计划。
