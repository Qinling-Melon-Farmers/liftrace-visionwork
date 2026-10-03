# 远端“板载代码”与八组专项对比（2026-09-27）

> 后续处理：本报告记录48541a7对495e9df的改参前差异。用户已要求对齐负载/建图并改为0.25m，当前结果见[板端参考分支说明](../board_reference_20260927/README.md)，本报告历史参数不回写。

## 结论与版本

本轮重新 fetch 导航组远端，`板载代码` 最新仍是 **48541a7（2026-09-27 14:13:59 +08:00，feat:添加高位搜索测试）**，没有比八组模块此前参考的 revision 更新。八组来源文件中的 `board_reference` 正是同一提交。因此不存在“今天又新增了一批板端代码而八组尚未拉取”的情况，但**两套运行行为、建图参数和消息版本并不相同，也没有完全互相同步**。

比较对象是远端源码，不是当前香橙派上正在运行的二进制。本轮没有连接硬件，没有启动仿真，也没有改动飞行代码、参数或试飞组分支。

| 对象 | 分支 / revision | 本次角色 |
| --- | --- | --- |
| 导航组现场代码 | `sakelier/liftrace-controlwork` / `板载代码` / `48541a7a8d91a2993711a10b556f7028504c3e1c` | 最新远端板端入口及源码 |
| 我们的八组专项 | `Qinling-Melon-Farmers/liftrace-visionwork` / `feat/board-deployment-flight-20260920` / `495e9df1ba8178c4edda6a15bf77457dd63aebed` | 本次被比较的代码状态；随后文档提交不改变该代码 |
| 八组集成的视觉研究来源 | `c9df53fd3d4ac942b4734148df45954f83ab92cd` | 高位粗线索、确认间隔、重访和边界修复 |
| 八组集成的导航研究来源 | `5962dd1edf47cbded61bd7d44e38766b1dcc2689` | 最新障碍柱等导航成果 |

来源清单：[八组 sources.json](../modular_trials_20260927/sources.json)。近期远端提交依次为 `fa62126` 投递日志、`4e831ce` 返航交接、`e94d0a7` 最简投递返航后改为 POSCTL、`48541a7` 新增高位搜索入口。

## 最需要先处理的具体问题

### 1. 远端两个入口的默认高度参数会触发启动异常

`minimal_delivery_test.yaml` 与 `high_view_priority_search.yaml` 都是：

```yaml
external_landing:
  hold_for_external_auto_land: true
  capture_height: 0.10
  auto_land_height: 0.15
land_height: 0.05
```

同 revision 的控制器启动条件要求 `capture_height > auto_land_height >= land_height`，而且即使 `hold_for_external_auto_land=true`，该不等式也没有豁免。**0.10 ≤ 0.15 会进入 `invalid external_landing parameters` 异常。** 这项在9月26日报告中已经指出，最新高位配置仍复制了它。远端走廊专项是 `0.30 > 0.05 >= 0.03`，没有这一处冲突。

这是默认配置与源码的确定性矛盾，不是本轮观察到的实飞故障。若现场运行正常，应查当轮实际参数和二进制，不据此推翻源码检查。我们的八组按同一地面参考生成 capture/landing/recovery 高度，没有复制上述0.10/0.15组合。未来处理远端时，应使委托外部降落时的保持高度及验证规则一致，而不是删除所有高度校验。

依据：[最简配置](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/config/minimal_delivery_test.yaml#L29)、[高位配置](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/config/high_view_priority_search.yaml#L31)、[控制器参数校验](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp#L1550)。

### 2. 远端高位配置的注释与实际航点不一致

注释写“低位接入→原地升高”“2.38m local Z”，实际起飞点是 `(0,0,2.4)`，12个搜索点全为 `z=2.4`；没有低位接入点。路线 X=0–7.2m、Y=±1.5m，也不是我们的4×4专项环线。按其默认 `ground_z=-0.22`，2.4m local Z 对应约 **2.62m FC AGL**。

我们的高位组则从低位起飞，低位到 staging，再由规划器升高到2.6m AGL。应按实际参数说明远端行为，不能凭文件名/注释认为它已采用研究版完整时序。依据：[实际起飞点](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/config/high_view_priority_search.yaml#L65)、[全部搜索点](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/config/high_view_priority_search.yaml#L138)。

### 3. 八组仍有未对齐的板端建图/定位参数

我们继承了静态TF、机架外参及投递接口，但不能笼统说“所有现场调参都已相同”：

| 项目 | 远端最简 / 高位 / 走廊 | 八组公共板端入口 |
| --- | --- | --- |
| FAST-LIO `feature_extract_enable` | true | false |
| FAST-LIO `cube_side_length` | 20m | 1000m |
| 点过滤 / 迭代 / surf和map滤波 | 3 / 3 / 0.15m | 相同 |
| FreeDOM `sensor/max_range` 与 raycast range | 最简/高位6m；走廊5m | 15m |
| FreeDOM sensor/raycast Z裁剪 | 最简−1至2m；高位−1至3.2m；走廊−1至1.5m | −3至5m |
| FreeDOM `voxel_depth` | 2 | 0 |
| 空闲计数 / 恢复计数 | 6 / 20 | 相同 |

八组来源是当前研究/旧专项组合，不是最新现场参数的完整镜像。以上差异会影响点云筛选、地图规模、清空细节和CPU负载，**不能仅凭笔记本SITL证明板端资源与地图行为相同，也不能直接断言它们已经造成启动失败**。

`voxel_depth=0` 是研究版细化空闲判据的选择，不能为了继承而直接改回2；较宽Z裁剪和15m范围是否适合4×4板端，应结合现场点云数量、延迟及地下噪点核对。下一次适配应优先形成明确的板端定位/建图档案，保留新导航逻辑，并分别验证这些差异，而不是整目录覆盖。

依据：远端各测试YAML；本地 [localization.launch](../../../deployment/board_trials_4x4/common/uav_board_trials/launch/localization.launch)、[competition_freedom.yaml](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_freedom.yaml)。

## 八组分别对应什么

| 八组模块 | 测试目的与结束方式 | 远端对应情况 | 现有同链SITL结果 |
| --- | --- | --- | --- |
| 01 视觉中断 | 低位直飞，一投，恢复后原地AUTO.LAND | 最接近 `minimal_delivery_test`，但远端按任务/路线完成返航后POSCTL，不保证仅一投或原地落 | PASS，1投 |
| 02 高位完整环线重访 | 完整走圈，冻结1–3个高权重目标，逐个重访，最后目标附近落 | 新高位入口不是此逻辑，没有整圈冻结/顺序重访阶段 | PASS，3投 |
| 03 H降落 | 低位到H附近，定点升高，CV对齐，自动降H | 两个投递入口关闭H；走廊入口末点直接AUTO.LAND；没有等价独立专项 | PASS，H对齐后落地 |
| 04 走廊＋H | 实测导航点＋在线避障，末端升高识别H | 远端走廊可对照导航部分，但无H视觉末端；默认航点/高度也不同 | INCOMPLETE，第二门边55cm包络接触 |
| 05 低位连续多投 | 中断→投递→恢复搜索，默认两投，可设1–3投 | 普通manager具备多次调度/恢复，但没有相同计数后原地落的模块封装 | PASS，2投 |
| 06 高位提前中断重访 | TOP3支持成立即中断高位，低位复核和重访 | 远端新高位只在高位航线上使用普通候选中断，不是TOP3记忆早退 | PASS，3投，发生提前中断 |
| 07 只记忆不投递 | 完整高位环线，记忆后下降落地，零APPROACH/释放 | 无等价入口 | PASS，记忆2个、零投递 |
| 08 三投接走廊/H | 研究版高位/重访/补搜/三投→走廊→H | 远端分开的高位与走廊入口没有串成这套完整任务 | INCOMPLETE，3投后第一门边包络接触 |

完整结果与录像沿用[八组报告](../../verification/board_modules_20260927/REPORT.md)，不是本次新跑。01–03在近墙控制补丁前运行，04–08在补丁后；不能把前三组说成该补丁的动态验证。现场走廊已有成功记录，按用户要求不继续纠缠这两组走廊SITL；仍保留其未完成H的事实。

04/08交付配置的实测航点与H位置仍留空，需要现场填写；仿真夹具航点不会自动带上实机。八组的RKNN实时负载、真舵机释放、实物落点以及全链实飞尚未被上述笔记本仿真代替验收。

## 视觉与高位策略的区别

远端高位入口只替换 `minimal_delivery_test.launch` 的YAML，继续调用普通 `navigation_mission_manager.py`。其搜索路线来自 `search/manual_waypoints`，发现合格候选后进入普通APPROACH/ALIGN/RELEASE/RECOVERY，再恢复路线。它仍有普通候选队列，但**没有接入研究版独立粗线索目录、TOP3提前退出、低位全队列路径排序和冲突复核阶段**。

我们的板端 `trial_manager.py` 从同一manager外壳派生，调用 `HighViewFull` 等纯策略；硬件适配器明确拒绝模拟时钟。不是直接把一个仿真自动起飞脚本拿上板。

| 视觉环节 | 远端板载默认链 | 八组当前链 |
| --- | --- | --- |
| RKNN粗检测 | conf≥0.50、NMS IoU0.45、640输入 | 相同；检测端按r2026排除tank，保留原模型类别编号 |
| 高位类别框投影 | 旧投影只接受精修中心及有效关联 | 高位四组另开 `navigation_hints`：单次类别conf≥0.60且投影有效即可留作导航线索，不必同帧圆环 |
| 低位CONFIRMED | target_memory默认3次连续有效观测，漏帧会中断累计 | 搜索模式仍要3次，允许相邻有效源图间隔≤1s；重复/倒序帧不算新证据，模式切换清确认状态 |
| 高位提前结束 | 没有研究版TOP3记忆早退；正常候选可直接打断搜索去投递 | 仅06/08启用当前支持条件；panzer需精修支持，其余可由一致粗观测支持；02/07必须完整圈 |
| 对准证据稳定帧 | `drop_aligner` 默认3 | 八组覆盖为5；conf0.60、30px门槛仍保留，最终释放还需事务/高度/槽位等条件 |
| 投递类别约束 | memory/aligner/guard由入口传r2026，但原检测器没有新增检测端类别过滤 | 从检测端到任务/释放统一r2026 |
| 低空重访顺序 | 未启用研究版全队列重访 | 最多3目标按地图路径代价枚举顺序，段间仍由在线规划器避障；局部降落模块不添加虚构返起点边 |

提前中断的粗支持间距为0.1–1.0s、坐标一致性0.5m；单帧≥0.60“能存线索”不等于单帧就能提前退出。高位候选、低位确认与投递许可是不同门槛。06仍保持专项范围的局部复核和结束约束，08才是完整任务含必要补搜。

远端 `min_streak=2` 并不能绕过其上游target_memory的3次CONFIRMED条件，因为候选入口还要检查 state==CONFIRMED。不要把该2解释为高位只要两次YOLO或只需两个目标。

依据：[远端高位launch](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/launch/high_view_priority_search.launch)、[远端manager](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py#L240)、[候选门槛](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py#L326)；本地 [trial_config.py](../../../deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_config.py)、[trial_runtime.py](../../../deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py)。

## 保留的硬件设计与有意不同的参数

| 项目 | 对比结果 |
| --- | --- |
| 坐标系 | 两边默认单位静态 `map→camera_init`，并用同一双向 `navigation_frame_adapter.py` 转换数据；不是只改frame标签 |
| 相机/IMU安装 | 八组与远端最简/高位相同：body→IMU +0.05m、IMU→camera −0.21m、q=[0,1,0,0]、像素矩阵[-1,0,0,1] |
| 远端走廊相机TF | 仍写旧q=[−0.7071,0.7071,0,0]；其入口没启动视觉，因此不能用它作为新视觉专项的外参依据 |
| 槽位补偿 | 两边相同三行[-0.12,0]、[0,−0.12]、[0,+0.12]，标准与红十字表一致；仍是固定坐标偏移，并非已验证随yaw旋转的槽口刚性外参 |
| 地面参考 | 远端固定ground_z=−0.22；八组未解锁静置采样FC local Z后减0.22，统一生成各级Z与视觉地面；不要求每次手填 |
| 实投接口 | 继承 `patrol_control/Servo` 与 `/legacy/Servo_raw`；远端默认接真服务，八组默认独立mock，01/02/05/06/08另有显式实投入口 |
| 飞行启动 | 八组preview与flight分开，READY检查相机/定位/视觉输出/控制设定点；flight READY不自动解锁或调用任务开始 |
| 结束方式 | 远端最简/高位回home后POSCTL人工下降；远端走廊末点AUTO.LAND但不识别H；八组按各专项原地AUTO.LAND或H视觉降落 |
| 高度 | 远端低位/接近local Z1.0≈AGL1.22、高位local Z2.4≈AGL2.62；八组低位AGL1.4、高位2.6、投递0.60，03低位1.0、H捕获1.8 |
| 速度 | 远端最简/高位规划上限0.60m/s、1.00m/s²，走廊0.80/0.80；八组统一0.50/0.35；前视距离是另一参数，不能当速度 |
| 前视距离 | 远端投递/高位巡航0.8m、精确段0.4m、走廊档0.25m；八组巡航0.5m、精确/走廊档0.25m |
| 录像 | 远端可选yolo_debug和bag；八组自动生成相机原片、视觉叠加、任务事件等，保留模拟/实际释放的区分 |

远端控制器有 `hold_for_external_auto_land` 专用保持分支，本地旧控制实现没有同名开关；八组用自己的LAND上下文和 `trial_auto_land.py` 收尾，03/04/08走H链。**不能把远端minimal的YAML/launch单独换到本地控制器上，然后期待它保持同样的POSCTL交接。** 这不是要求八组改回人工降落，而是需要保持整套入口与控制代码配套。

本次新实测FOV（FC 2m、地面3.6×1.8m）尚未写入这两套CameraInfo；八组仍消费既有CameraInfo，之前覆盖复算是文档分析。静态TF一致也不代表外参/内参已重新实测标定。

## 障碍柱、膨胀与停滞修复

| 项目 | 远端板载 | 八组专项 |
| --- | --- | --- |
| 正常点云水平膨胀 | 最简/走廊0.05m，高位0.10m | 全部0.275m |
| 上/下膨胀 | 共享档案0.10/0.10m | 0.20/0.10m |
| 栅格分辨率 | 0.10m | 0.10m；0.275向上量化成3格≈0.30m |
| 虚拟顶棚 | 最简/走廊−0.1关闭；高位重新设为2.88m local Z | 所有模块及高低位/走廊阶段切换均−0.1关闭；控制指令限高仍保留 |
| 障碍柱 | 最简/走廊关闭；高位使用旧竖向支持柱 | 02/06/07/08开启修正后的中部柱；其余专项关闭 |
| 柱生成细节 | 满足支持的点XY膨胀后，从统一floor_z向顶层填充；没有树冠中部和有限凸包修复 | 按分量中部40%–60%轮廓向上；仅小分量可补内部，最大跨度1.6m、离观测轮廓补洞距离≤0.35m；墙体/大连通结构不填全局凸包 |
| 局部重建 | 旧局部Z窗清理 | 启柱时局部XY范围全高重建，避免遗留合成占据 |
| 轨迹/FSM | 有旧跟踪保持与前视，但没有当前进度反馈闭环 | 连续弧长投影、轨迹目标身份、hold反馈与有限次重规划、初始规划超时等当前修复 |
| 近墙投递 | 没有当前统一边界对准末端约束 | 任务/重访许可＋控制最终设定点/释放边界约束配套 |

最新远端高位没有今天有限填充的障碍柱实现。这解释了为什么“参考板端可飞”不等于应该把八组地图代码回退成远端版本；同样，八组0.275m膨胀比现场0.05/0.10m明显保守，不能保证现场同一窄门仍具有相同可达性。规划用点云栅格，仿真碰撞用55cm包络，两者不是自动相加的一套几何。

关闭虚拟顶棚和关闭障碍柱是两件事；八组允许前者关闭、后者在高位继续启用。树冠中部策略允许经过下部外缘上方，真实三维膨胀仍保留，不宣称等于全树最宽投影禁越。

依据：[远端SDF实现](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp#L929)、[远端竖向支持](https://github.com/sakelier/liftrace-controlwork/blob/48541a7a8d91a2993711a10b556f7028504c3e1c/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/include/plan_env/vertical_obstacle_support.h)；本地 [公共规划档案](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/horizontal_planner.yaml)。

## 不能混用的接口和还需保留的现场成果

1. 本地 `plan_manage/Bspline.msg` 比远端多 `goal_stamp`、`goal_frame`；本地新增TrajectoryProgress，并由FSM/轨迹服务器成对使用。ROS消息定义不同，不能只替换traj_server可执行文件或沿用旧devel；部署八组应编译配套导航与视觉workspace。
2. 远端普通manager显式读 `search/manual_waypoints`；本地普通manager没有这个现场扩展。八组通过 `trial/waypoints`＋`CoverageRoute` 实现专项路线，所以八组本身并非丢了航点功能，但直接把远端minimal/high的YAML拿来运行本地普通manager，会忽略该手工路线字段。
3. 远端 `drop_min_confidence/drop_max_offset_px` 两个launch参数化接口在本地 `phase_d_board.launch` 不存在；默认数值0.60/30px没有丢，但现场若曾通过这两个arg调参，需要专门继承接口，不能原样传给本地launch。
4. 远端可选bag记录参数缺 `-o`：`args="$(arg record_dir)/yolo_debug ..."` 会把预期前缀当作位置参数/话题，不能按注释保证输出到record_dir。默认record_debug=false所以默认不触发；确认它时依据的是本机Noetic rosbag选项解析，未启动录制。这应由现场分支补齐，不影响八组独立录像入口。

## 建议的处置顺序

- **先明确运行入口。** 要复现现场最简飞行就保持试飞组完整版本及当轮参数；要验证高位记忆/重访就使用视觉仓八组中的02/06/07，不能仅凭远端文件名 `high_view_priority_search` 替代它们。
- **远端优先修确定性问题。** 默认降落高度参数矛盾、低位入场注释与航点不符、可选bag输出前缀；高位顶棚是否保留应显式决定，不能与“全组关闭顶棚”的约定混为一谈。
- **本地优先补板端适配差异。** 明确FAST-LIO/FreeDOM板端资源配置，保留小范围实飞验证成果；旧manual_waypoints与外部降落hold如需兼容，做明确适配，不覆盖现有高位策略。
- **随后按模块上板。** 先01单投/03 H、05恢复多投，再07只记忆、02完整环线、06提前中断；08最后。保留0.5m/s、默认mock和显式实投入口。现场验证记录实际HEAD、展开参数和输出状态即可，不需要重复人工填写已知外参。

本次仅完成源码/参数比较、文档归档，没有对任一飞行实现做自动合并，也没有新增仿真或实飞结论。
