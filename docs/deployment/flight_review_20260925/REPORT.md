# 9月25日视觉中断实飞复盘

本轮只读取新增的 `flight_debug_2026-09-25-18-20-20_0.bag`，未运行飞行或仿真，未修改飞行代码。bag覆盖18:20:20.193—18:22:09.29，共109.10秒；任务命令初始时间早于录包约6.64秒。下文相对时间均以录包开始为零点，时钟为北京时间。

## 结论

**仅红十字形成了具备类别和有效地图位置的投递候选，并触发接近、对准和投递调用。装甲车被识别到，但没有形成panzer候选，更没有进入投递队列。**红十字调用执行端失败，任务锁定槽位后停止后续搜索/投递，转入返回与降落流程。并非两个投递候选已就绪、仅被舵机兜底一起取消。

本轮搜索目标为local Z=1.0m，不是2.6m高位搜索；前飞实速中位约0.85m/s，峰值约0.98m/s。没有成功投递ACK。`RETURN_HOME`指令目标是 `(4,0,1)`，不是最初的 `(0,0,1)`；名称不能当成回到起飞点的证据。

## 视觉与任务时间线

| bag相对秒 | 北京时间 | 事件 |
|---:|---|---|
| 8.797 | 18:20:28.990 | MAVROS记录解锁，ALTCTL |
| 11.798 | 18:20:31.991 | 进入OFFBOARD |
| 19.624 | 18:20:39.817 | 首点完成，SEARCH目标转为(4,0,1) |
| 21.854 | 18:20:42.047 | 原始检测话题首次出现red_cross |
| 22.378 | 18:20:42.571 | 红十字首次有效地图投影 |
| 22.381 | 18:20:42.574 | 红十字候选ID=0进入DETECTED |
| 22.669 | 18:20:42.862 | 红十字连续3次观测，CONFIRMED并被selected_target发布；置信度0.939、地图质量0.912 |
| 22.725 | 18:20:42.918 | high_weight_search_interrupt，转APPROACH，预留槽1；目标约(2.108,-0.092,-0.22) |
| 26.618 | 18:20:46.811 | 接近完成，旧控制接受视觉对准 |
| 39.199 | 18:20:59.392 | strict_alignment_context_valid，进入RELEASE阶段 |
| 41.736 | 18:21:01.929 | 槽1投递失败：raw_actuator_unavailable |
| 41.758 | 18:21:01.951 | candidate_release_state_uncertain，转RETURN_HOME |
| 42.054 | 18:21:02.247 | 同一红十字/槽1第二次raw_actuator_unavailable，仍无成功ACK |
| 48.56—48.73 | 18:21:08.75—08.92 | 两条panzer进入精修/投影链，均circle_association_missing、map_valid=false |
| 49.149 | 18:21:09.342 | 返回目标完成，发LAND |
| 49.207 | 18:21:09.400 | 视觉align_mode切landing，仅H允许进入融合输出 |
| 100.799 | 18:22:00.992 | 飞控由OFFBOARD转POSCTL |
| 105.692 | 18:22:05.885 | 记录landed_state_and_settle_confirmed |
| 106.800 | 18:22:06.993 | MAVROS记录未解锁 |

红十字从首次有效投影到确认约0.291秒，到中断约0.347秒；从原始检测首次出现到中断约0.871秒。这是录包接收时序，不是同一帧的端到端延迟。中断时飞机约x=2.232m、local Z=0.990m，已接近/略越过首个融合靶心的x坐标，随后减速回到目标附近。

## 装甲车为什么没有形成候选

全包原始panzer检测349条；首次对应图像时间是bag+48.331秒，第一条原始话题接收在+48.578秒。两条经过融合/精修的记录为+48.561、+48.727秒，跨话题接收顺序有十几毫秒差异，不能倒推处理因果；其图像时间与原始检测一致。

这两条都因 `circle_association_missing` 不能获得有效地图位置。`targets`里始终只有red_cross ID=0与circle ID=1；泛化圆环虽然曾进入CONFIRMED，但不能当作装甲车类别已确认。`selected_target`全包只有红十字，任务指令中也只有红十字APPROACH。

![装甲车首帧原始相机图](panzer_first.jpg)

上图与首次panzer图像时间戳精确一致，未经标注或旋转。目标及圆环在左边明显出画；第一条检测框x_offset=0，类别置信度0.659。这支持圆环不完整导致关联失败的解释，但bag未记录每个圆环筛选候选及评分，不能把裁切认定为唯一原因。

装甲车首次检测已经晚于返航约6.8秒。随后+49.207秒切换landing模式，融合器只允许landing_pad；原始YOLO仍继续报panzer，到+102.075秒。因此349条原始检测不等于349次有效投影机会，也不等于装甲车被记忆后放弃。可从[融合器模式过滤](../../../vision_ws/src/uav_vision/src/uav_vision/detection_fusion.py)和[圆环关联精修](../../../vision_ws/src/uav_vision/scripts/target_refiner.py)核对该机制。

## 舵机失败为什么结束后续任务

两条 `release_result` 都是success=false、slot=1、target=red_cross，原因 `raw_actuator_unavailable`。本地[guarded_servo_proxy](../../../patrol_uav_ws-patrol_planner/src/uav_mission/scripts/guarded_servo_proxy.py)在等待底层服务或RPC调用发生异常时产生该原因；不是“视觉没对准”的拒绝。它与现场忘记启动舵机程序的描述一致，但bag本身不能区分服务未启动、名称接线错误与其他服务异常，也不能证明只缺舵机供电。

[任务核心](../../../patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py)在已进入RELEASE而失败时将槽位标为QUARANTINED，并以candidate_release_state_uncertain结束后续投递；本包状态正好由[RESERVED,FREE,FREE]变成[QUARANTINED,FREE,FREE]，committed_slots始终为0。它保守地避免在释放状态不确定时复用槽位，没有把失败算作成功。两条失败回执是同槽两次调用，不能算两投。

因此“舵机失败导致结束当前任务”成立；“装甲车也已形成候选，只因舵机没投”不成立。若执行器正常，本包也不足以证明装甲车之后一定能完成几何确认和投递。

## 飞行高度和速度

原始FC local Z在地面约-0.091m，而非精确0。下表离地高度仅按已知机架落地FC离地0.22m估算：AGL≈local Z+0.311m；不是独立测距真值。bag无启动参数快照，视觉投影地面记录-0.22m与这次地面零点之间约9cm差异应在下轮核对，不能据此直接宣布外参错误。

速度由odom位置按0.5秒中央差分计算，并与旋转到世界系的twist交叉检查；twist属于base_link，不能直接把原始vz当世界竖直速度。差分峰值会受定位修正影响，采用中位/P95描述稳定航段。

| 阶段 | local Z范围m | 估算FC离地范围m | 水平速度中位 / P95 m/s |
|---|---:|---:|---:|
| 前飞搜索 | 0.933—0.990 | 1.24—1.30 | 0.855 / 0.958 |
| 红十字接近 | 0.993—1.119 | 1.30—1.43 | 0.202 / 0.731（含开始制动） |
| 对准至释放失败 | 0.226—1.119 | 0.54—1.43 | 0.033 / 0.116 |
| 返回测试终点 | 0.179—1.048 | 0.49—1.36 | 0.320 / 0.383 |
| LAND指令后、POSCTL前 | 0.796—1.357 | 1.11—1.67 | 0.079 / 0.192 |

首次红十字确认时local Z≈0.987m、水平速度约0.91m/s；服务失败时local Z≈0.232m。搜索前爬升峰值约0.25m/s；红十字对准末段下降峰值约0.41m/s。装甲车首次精修拒绝时local Z≈1.007m、水平速度约0.25m/s，不能把圆环失败简单归因于高速掠过。

![高度、速度及视觉时序](height_speed_vision.png)

LAND后约51.65秒飞控仍在OFFBOARD，随后POSCTL才快速下降落地。故末尾COMPLETE只能证明程序观察到了落地与稳定，不能证明全自主降落成功；是否人为接管需结合飞手现场说明。本轮没有landing_pad检测进入已记录的视觉检测流。

## 后续建议与数据边界

### 视频回放与标准靶视场恢复（9月26日补充）

[带章节播放器](index.html) · [视觉叠加MP4](../../../logs/flight_review_20260925/video/camera_annotated.mp4) · [原始下视MP4](../../../logs/flight_review_20260925/video/camera_raw.mp4)。共109.13秒、15fps、1倍速，3113张录包相机图像按时间重采样为1637帧；原始版1280×720、叠加版1280×1080，H.264，两份均完整解码检查通过。复用原录像脚本的检测绘制函数；检测框按图像时间戳匹配、容差30ms，状态按录包接收时间显示。没有重新推理或发布ROS话题。离线像素证据可在其图像上显示，但当时的在线结果存在处理延迟，不能把视频框出现时间当成任务已知时间。

“装甲车缺圆环时适度升高以扩大视场”是合理的主动观测策略，但当前精修节点没有飞行控制权，也没有将circle_association_missing转为爬升指令的闭环。标准靶几何不合格时无法形成可用候选，因此不能期待既有候选接近流程自动修复这个前置缺口。本次已进入执行器失败返回流程，随后landing模式只保留H，也不会再为装甲车调整高度。

后续可设计为：允许搜索的阶段保留可信类别粗线索→先减速并将目标移入画面中心→若圆环仍因视场不足无法关联，则在高度/障碍约束内有界升高→重新取得同帧类别和圆环证据→形成确认坐标后再低位精对准。升高扩大视场但降低目标像素尺寸；偏心、遮挡、光照、关联门槛及时间不同步也可能导致缺环，不能用一个固定更高高度保证解决。该建议本轮只记录，未修改飞行策略。

1. 下次起飞前确认底层舵机服务实际存在、名称与代理一致；“真实投递”和“mock专项”必须明确选定，不能用未启动执行器代替模拟。
2. 标准靶需要一次独立的圆环关联验证：先保证靶心和足够完整圆环位于画面内，再看panzer的map_valid、CONFIRMED和selected_target，不能只看YOLO框。
3. 核对现场巡航高度/速度、返回点、地面基准与降落专项配置。当前bag的SEARCH z=1.0、终点x=4.0和接近1m/s实速说明不能直接套用本机4×4旧默认参数。
4. 缺底层服务与“已调用但结果不明”可考虑后续区分，但本次只诊断，未放宽释放失败保护。

本地源码参考为板端分支18e433b；新bag未附9月25日板端源码revision及参数快照，因此源码解释以日志实际原因码和状态转移交叉核对，不能保证所有本地默认值就是现场配置。原始bag与全量提取JSON留本机；仓库只收报告、脚本、摘要和图像。[指标摘要](summary.json)、[ROS离线提取](analyze_bag.py)、[统计及绘图](summarize.py)。

## 多画面回放与后段圆环解释（2026-09-26）

用户指出的后段画面确实同时存在panzer与圆环。进一步按源图像时间匹配：bag+50.186006秒同一帧，圆环几何置信度0.8622（接收50.272534秒），panzer置信度0.9678（接收50.456589秒）。此时49.206654秒已经切换landing模式，投递类别被融合入口过滤。因此前述“左侧裁切”仅解释最早两条拒绝记录，不能解释后段所有检测；本帧不能说没有圆环，也不能据此认定在线已形成装甲车地图候选。是否在search模式能通过后续关联仍需单独验证。

旧视频只显示当帧匹配地图结果，未持续显示已有坐标。新版加入class、ID、XYZ、frame、CURRENT/HISTORY和记录年龄：例如红十字末次有效融合位置约(2.104,-0.074,-0.220)，坐标系camera_init。HISTORY仅是历史有效坐标，不是当前投递许可；circle#1也不是panzer。bag没有有效panzer坐标，因此不补造坐标。

[新版多画面播放器](index.html) · [多画面视频](../../../logs/flight_review_20260925/replay_suite/dashboard.mp4) · [轨迹动画](../../../logs/flight_review_20260925/replay_suite/trajectory.mp4) · [完整输出目录播放器](../../../logs/flight_review_20260925/replay_suite/index.html) · [可分享工具说明](../../../tools/bag_replay/README.md)。四种视频均为10fps、109.100秒、1倍速；多画面为1920×1080。全部经过FFmpeg完整解码和时长验证，6项自动测试通过，包括缺失可选话题的导出。工具仅离线读包，不启动仿真、不发布控制话题、不重跑模型；本轮没有修改飞行代码。
## 试飞组板载分支对照（2026-09-26）

试飞组今后长期维护并同步实际板载代码的来源：[liftrace-controlwork/板载代码](https://github.com/sakelier/liftrace-controlwork/tree/板载代码)。本次只读fetch至本地导航参考仓，核对revision `fa62126213995780015d58773511de1e4569cfb8`；未向试飞组分支写入或覆盖。用户确认9月25日bag使用“最简投递验证测试”。

入口为 `patrol_uav_ws-patrol_planner/src/uav_mission/launch/minimal_delivery_test.launch`，专用参数为同包 `config/minimal_delivery_test.yaml`。实际为今年Mission Manager（start_mode=full）+ Planner Bridge + RKNN/几何/投影/记忆/对准 + ReleaseGuard，底层复用external_mission_mode的patrol_control。不能将复用旧执行器等同于旧任务链，也不能将full理解成高位搜索已启用。

| 配置/源码 | 与bag对应事实 |
|---|---|
| 手动搜索航点(0,0,1)、(4,0,1)，camera_init | 低空前飞时中断；没有先完成高位航线再重访 |
| approach_altitude=1.0、return_altitude=1.0 | 与任务指令Z=1吻合；说明文档的0.6已落后于YAML |
| home_xy=[4,0] | 失败后前往4米终点，不是回到起飞原点 |
| cruise_lead_m=1.0、precision_lead_m=0.4，规划max_vel=1.2 | 与搜索约0.85–0.96m/s、返回约0.32m/s实速相容；前视距离本身不是速度设定 |
| require_alignment_context=true、require_evidence_context=true | 正在使用新对准/释放证据链 |
| raw_servo_service=/legacy/Servo_raw | 服务等待或调用失败产生raw_actuator_unavailable；不能仅凭此区分未启动、命名不一致与通信异常 |
| mission_core的candidate_release_state_uncertain | 释放阶段失败后隔离槽位并返回；不是装甲车候选被投递优先级抛弃 |
| detection_fusion landing仅接受landing_pad | 解释后段panzer与圆环同帧却没有装甲车有效地图坐标 |
| camera_quat_xyzw=0,1,0,0；pixel_to_body_matrix=[-1,0,0,1] | 配置已针对机头朝图像左侧修正；不应继续将更早known_rig值当成本次运行配置 |

### revision时间边界

bag为18:20:20至约18:22:09；HEAD fa621262提交于18:24:51，即飞行结束之后。相对父提交d96c55b（15:25:23），它将地图尺寸14×8改为16×4，并新增(6,0,1)航点，同时新增详细录包入口。故HEAD不是可证明的飞行时源码快照；父提交也不能排除现场未提交修改。报告可确认共同逻辑与bag时序相互吻合，但不据此断言当时已经使用新增6米航点或最新地图尺寸。

### 后续协同约定

以试飞组板载分支作为实际板端代码同步参考；本地板端部署分支保留工具、分析和待交付配置，研究分支仍是仿真研究，三者不自动互相覆盖。每轮日志最好同时留下git revision、工作区diff、实际启动命令及参数导出，尤其记录舵机入口、外参、地面Z和专用测试配置。当前说明文档仍有高度表与YAML不一致、record_debug默认false却称自动录制的问题；应由试飞组后续同步订正文档，本次不改其运行代码。
## 本机高位、整机与试飞组板载三方比较（2026-09-26）

比较版本：试飞组板载fa621262；本机高位 `feat/high-view-search-research` 04c6ec8；本机整机 `feat/r2026-competition-integrated` 86e382d。原整机目录已是归档软链接，不是活动Git工作树，因此以仍保存的同名Git分支为比较依据，不把归档目录冒充当前工作树。主仓main及另一个main-integration分支不等同于这里的competition-integrated。

| 项目 | 板载最简测试 | 本机整机 | 本机高位研究 |
|---|---|---|---|
| 任务入口 | navigation_mission_manager，手动低空航点，full投递闭环 | 常规SearchPolicy覆盖搜索任务 | 另有navigation_high_view_full与high_view_full策略层 |
| 高位先搜后重访 | 该分支未包含high_view模块，不能仅把高度改成2.6就获得该策略 | 无该研究模块 | 有线索记忆、排序重访、延期问题目标、优先补搜跳过片区等 |
| mission_core.py | 与整机完全相同 | 共同任务基线 | 新增共享接近准入及中断类别覆盖入口 |
| 手动航点读取 | manager显式读取search/manual_waypoints | 所比较manager无该读取分支 | 所比较常规manager也无该读取分支；不能原样替换板端manager |
| 分段跟随 | 有FollowingSpeed，根据任务切前视 | 所比较manager没有此实现 | 有FollowingSpeed及走廊分段与投后膨胀切换 |
| 投后恢复高度 | patrol_control新增标准靶/红十字恢复设定点参数及检查 | 相关位置仍用1.2/1.15固定值 | 所检查相关位置同样未含板载参数化，需要保留板端成果 |
| 近墙处理 | 未含高位研究新增围栏逻辑 | 未含研究围栏逻辑 | 控制设定点及释放许可加入近墙围栏；尚不能据此宣称板端已验收 |
| 视觉五份核心文件 | 与右两列逐文件相同 | 同左 | 同左 |
| guarded_servo_proxy.py | 与右两列完全相同 | 同左 | 同左 |

这里“视觉五份”精确指：scripts/target_refiner.py、src/uav_vision/detection_fusion.py、scripts/drop_aligner.py、scripts/target_memory.py、scripts/target_map_projector.py。不是宣称相机驱动、全部检测器、配置或整套视觉仓完全一致。现场外参、CameraInfo及模式控制不同，仍会产生不同运行结果。

由此，昨天红十字中断与释放失败逻辑不是一套独立旧视觉链：共同核心与整机一致，低空专项入口不同；后段装甲车被landing过滤在三版共有融合源码中都有依据。高位策略实机尚未由该包验证。反向同步时优先保留板载手动航点、相机方向和恢复高度参数化，再逐项评估高位策略与研究边界策略；不得以研究版覆盖板端正在使用的源码。此次仅比较并记录，未修改或运行控制代码。