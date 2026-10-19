# 九组试飞操作与验收顺序（2026-10-05）

本页替代早期手册中自动请求OFFBOARD、独立MP4录像、硬件跨目录链接和默认高位2.6m的现场操作描述。以下是最新Git配置，不代表本轮已经部署到机载电脑。仍沿用0928工程目录，不新建机载目录。

## 两套编号先对应

现场1–6的顺序与源码目录01–09不同，尤其“第五组”是目录06高位优先，“第六组”是目录09高速采集。

| 源码目录 / trial | 现场简称 | 做什么 | 使用现场配置时的高度/速度 | 正常结束 |
|---|---|---|---|---|
| 01_visual_interrupt / visual_interrupt | 1 / single | 低空直线搜索，视觉中断，接近、确认、对准，一次投递 | FC低位1.4m、投递0.6m；0.5m/s、0.35m/s² | 在当前结束位置降至0.3m悬停，飞手落地 |
| 02_high_view_revisit / high_view | 4 / revisit | 完整高位巡航，冻结有效位置，低位逐点复核投递；不凭齐类提前结束整圈 | FC高位2.0m、低位1.4m、投递0.6m；0.5m/s | 完成限定目标清单，在结束位置0.3m悬停；不是第五组返起飞点流程 |
| 03_h_landing / landing | h / landing | 低位接近已知H，升高观察，OpenCV H识别、投影、对准和降落 | FC转场1.0m、观察1.8m，最高2.0m；0.5m/s | 真正视觉降落/AUTO.LAND，不是0.3m悬停；不投递 |
| 04_corridor_landing / corridor_landing | 显式目录入口 | 按实测门前/门后引导点在线避障，接H链 | 按实测航点；优化走廊目标0.9m、共同上限1.0m；0.5m/s基线 | H视觉降落；走廊与H未填则拒绝启动 |
| 05_low_multi / low_multi | 2 / multi | 低空检测中断、投递、恢复搜索，默认两投 | FC低位1.4m、投递0.6m；0.5m/s | 指定投数完成后在结束位置0.3m悬停 |
| 06_high_priority / high_priority | 5 / priority | 高位搜索，满足支持条件可提前中断；低位复核、三投 | FC高位2.0m、低位1.4m、投递0.6m；0.5m/s | 返回起飞点，再下降至0.3m悬停，飞手落地 |
| 07_memory_only / memory_only | 3 / memory | 完整高位航线，只观察并记忆，不APPROACH、不投递 | FC高位2.0m、低位1.4m；0.5m/s | 规划下降、结束位置0.3m悬停；不把记忆数当作投递成功 |
| 08_full_mission / full_mission | 显式目录入口 | 高位搜索→低位重访/投递→走廊→H | FC高位2.0m、低位1.4m、投递0.6m；实测走廊与H；0.5m/s基线 | 整场H视觉降落；走廊/H未填则拒绝启动 |
| 09_high_speed_capture / high_speed_capture | 6 / capture | 沿第五组场内巡航路线采集五类/光照/H负例；不重访、不投递 | FC高位2.0m；速度可0.5/1.0/1.2m/s，默认1.2，加速度1.0m/s² | 返起飞点0.3m悬停；完成航线不等于识别指标通过 |

表中0.3m来自现场test_area.yaml；不传现场配置时模块原始默认仍为4×4范围、高位2.6m，普通结束链可进入自动降落。**不要省略--site-config后把原始模板当现场档案。** H相关三个模块必须使用各自专用现场YAML，不能套带terminal_hover_agl的普通test_area.yaml。

所有高度均为FC参考点离地高度，local Z由自动地面基准换算；相机下偏约0.16m，FC 2.0m对应水平静态光心约1.84m，不能按2m光心计算覆盖。研究六轮单独采用光心2.60m/FC2.76m，不覆盖现场最高2.0m。

## 共用配置和最新补丁

- 现场普通范围：X[-0.35,6.0]、Y[-1.5,1.5]；搜索X[0.6,6.0]；环线X[0.8,5.5]、Y[-1.1,1.1]。新蛇形根据本组外接范围生成，不带入10×10比赛尺寸。
- 静态TF、已知相机/槽位补偿保留；虚拟顶棚关闭。普通现场组障碍柱关闭，full_mission专用现场档案仍开启；三维膨胀0.25/0.20/0.10m，不恢复0.275m。
- 五分类RKNN/NPU；投递圆环复核0.75，搜索/H仍保留各自门槛。H强化含反光适应及结构兜底，进入H阶段暂停YOLO，H接管后不恢复旧LAND。
- 新记忆修复：异类观测不能延续旧标签confirmed；新类别连续证据可以推翻长历史；物理ID合并不复活旧标签。高位粗线索用于导航，低位仍重新确认、对准、取得许可。
- 新运动修复：规划与执行共同限高；正式MAVROS速度按完整姿态从child转任务系；投后提前交接至少3个新样本；过期位姿先降档、恢复后按当前阶段重选；限高等待同时检查deadline、地图/frame与规划回执。
- 直线偏好独立，默认权重2.0；是软代价，绕障、转弯与样条仍可成弧线，不保证处处严格直线。
- 位姿异常保护沿当前Git实现；不因为实机偶发跳变就删除保护。本轮没有改现场PX4参数或部署。
- 板端bag-only：默认压缩相机5Hz、CameraInfo、TF、视觉/任务/许可/控制/位姿及小范围膨胀点云/sdf_map/occupancy_inflate。默认不录完整FreeDOM/原占据地图，不运行独立MP4编码；视频回本机生成。

## 新优化怎样开启

各组原有飞行参数默认保留。新运动衔接用--motion-optimized显式开启；高位类02/06/07/08/09还可选--survey-pattern rectangle、snake2或snake3。只记忆/采集/H不会因此启用投递或投后恢复。

~~~bash
# 仅配置检查，不启动ROS或硬件
bash deployment/board_trials_4x4/06_high_priority/start.sh preview   --site-config deployment/site_20260928/test_area.yaml   --motion-optimized --survey-pattern rectangle --check-config

# 模拟投递入口；现场准备和授权完成后才运行
bash deployment/board_trials_4x4/06_high_priority/start.sh flight   --site-config deployment/site_20260928/test_area.yaml   --motion-optimized --survey-pattern rectangle

# 同组真实投递入口；必须机构已准备、真实服务可用
bash deployment/board_trials_4x4/06_high_priority/start_real.sh   --site-config deployment/site_20260928/test_area.yaml   --motion-optimized --survey-pattern rectangle
~~~

旧site/start_test.sh 5 flight会选择真实投递，且普通1–5快捷入口不接收额外优化参数。需要新参数时使用上面的显式目录入口；不能在旧快捷命令后直接拼选项。

近墙访问由距离选择慢档，仍保留净空和到点条件。走廊合并必须显式提供corridor_geometry的wall_axis、wall_coordinates、entry_waypoints；没有实测几何时保留逐点路线，不声称已经不停顿。H扫描点、转角、必要降高点不合并。仅关闭运动衔接不会清零全域直线偏好，需关闭时设置planner_line_preference_weight: 0。

## 需要几个终端、按什么顺序

从全部停止开始：**6个常驻终端加1个观察终端**；无投递的03/04/07/09不启动舵机，减少为5+1。可以使用tmux窗口保持会话，但这里没有新实现自动一键飞行。

每个终端先用实际SSH地址登录，再进入同一工程。不要同时source另一套旧r64或hardware_ws：

~~~bash
ssh orangepi@实际IP
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
~~~

按顺序启动；这些命令供下一次现场使用，本次没有在实机执行：

1. 主节点：roscore；已有主节点时不重复启动。
2. 飞控：roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600。先确认connected=true，串口按本机实际设备。
3. 雷达：roslaunch uav_mission mid360_driver2.launch user_config_path:="$PWD/deployment/site_20260928/MID360_config.json"。Wi-Fi地址与雷达网段是两回事；不另开FAST-LIO。
4. 相机：bash deployment/board_trials_4x4/start_camera.sh /dev/video0；设备号按实际相机。
5. 仅实投：地面未解锁、机构允许复位后，执行下面舵机命令；启动会初始化/复位槽位。
6. 应用：先--check-config，再preview观察；退出preview后用同一组flight或start_real.sh。
7. 观察：分别看mavros/state、mission_status、probe_status和末端状态，不能只看最后一个“LAND”字样。

~~~bash
# 终端5，真实投递组才需要
sudo bash patrol_uav_ws-patrol_planner/src/actuator_pwm/init_pwm.sh
roslaunch actuator_pwm launch_all.launch

# 观察终端，只查询服务，不释放载荷
rosservice type /legacy/Servo_raw
rosnode info /servo_controller1
# 期望patrol_control/Servo及/legacy/Servo_raw

# 每个echo持续运行，Ctrl+C后换下一项，或各用一个tmux窗
rostopic echo /mavros/state
rostopic echo /navigation/mission_status
rostopic echo /uav_high_view/probe_status
rostopic echo /board_trials/terminal_hover_status
~~~

READY后由飞手**人工解锁并拨到OFFBOARD**；只解锁不会由本应用替飞手请求OFFBOARD。起飞高度和稳定条件满足后自动进入任务，无需再发start_mission。退出OFFBOARD后取消自动流程，不自动夺回控制。所谓auto_start_after_arm仍需OFFBOARD，不能按这个旧参数名字推断“只解锁就飞”。

末端0.3m悬停等待飞手落地；H组真正AUTO.LAND。异常ABORT不保证执行正常返航/降落收尾，应依据实际状态接管，不重复调用启动服务。落地上锁后等待bag正常关闭，再退出应用及硬件节点；空中不要关闭MAVROS。

## 各组的现场配置入口

~~~bash
# 单投、多投、仅记忆、整圈重访、优先中断：配test_area.yaml
bash deployment/board_trials_4x4/07_memory_only/start.sh flight   --site-config deployment/site_20260928/test_area.yaml

# H：专用YAML，无投递，不带terminal_hover_agl
bash deployment/board_trials_4x4/03_h_landing/start.sh flight   --site-config deployment/site_20260928/h_landing_test_area.yaml

# 走廊/H：先实测填写corridor_waypoints、landing_xy及可选corridor_geometry
bash deployment/board_trials_4x4/04_corridor_landing/start.sh preview   --site-config deployment/site_20260928/corridor_landing_test_area.yaml --check-config

# 整场：同样先补现场几何；默认mock，实投另用start_real.sh
bash deployment/board_trials_4x4/08_full_mission/start.sh preview   --site-config deployment/site_20260928/full_mission_test_area.yaml --check-config

# 高速采集：先0.5m/s对照，再1.0/1.2；正常/较暗各记录
bash deployment/board_trials_4x4/09_high_speed_capture/start.sh flight   --site-config deployment/site_20260928/test_area.yaml   --capture-speed 1.0 --capture-lighting normal --motion-optimized
~~~

## 哪些已经做过，哪些必须重验

用户现场反馈：原现场1单投、2多投、3记忆已成功；4整圈重访曾有完整逻辑验收，但因靶位超限未投递，并暴露过ABORT；5优先中断后来有成功投递轮。走廊避障有历史实飞通过。这些证明历史链路，不替代最新补丁同版验收。

**本次全新、尚无同版实飞验收的重点：**

- 长历史panzer/pillbox纠正：已实际bag复跑4202帧并跨层测试，未新飞行。
- 四项运动约束及后续None/pending/ACK修复：离线测试、编译已通过，最新六轮整机仿真固定87258798，结果见高位研究分支docs/verification/latest_six_20261005/REPORT.md；不拿前一批2b0678b9替代。
- --motion-optimized下的移动恢复、动态近墙降速、走廊共线中继衔接，以及独立全域直线权重；旧实飞多为0.5m/s逐点链。
- 矩形/双线/三线参数化在现场狭小范围的效果；配置展开通过不等于都飞过。
- 第09组第五组场内路线、默认1.2m/s及新版直线偏好的组合；历史高速直线采集不是同一配置。
- H反光强化、模式切换/接管取消等虽有离线bag与部署检查，仍需最新版本完成H全过程；08最新完整三投→走廊→H同次飞行尚未验收。
- 实际EV/LIO延迟与高度重置的改善必须由真实bag/ULog证明，仿真不能替代板端实时性验收。

## 离最终整机闭环最近的试飞顺序

无需把历史已成功的九组重新全跑一遍。优先完成缺失的连接段：

1. 静态就绪/人工OFFBOARD和接管检查，核对新定位链与轻量录包；必要时01做一次短链回归。
2. **06高位优先**：先按0.5m/s、原矩形验证新记忆纠正、三投、投后恢复与返起飞点；同一位置让粗判与低位真类有竞争，确认不会带旧标签释放。再显式启用运动衔接做对照。
3. **03 H降落**：标准无反光H与现场较差光照，完成识别、对齐、真正落地和接管；不能只看检测框。
4. **04走廊接H**：使用实测80cm开口、门前/后引导点，验证0.9m目标/1.0m共同上限及进入H后切换。历史纯走廊成功不能替代本轮接口交接。
5. **08全任务**：保持前面已经通过的速度/高度，完成同一次三投、走廊和H落地；这是距离最终验收最直接的一轮。启用新快速补搜前，先解决本次seed38双扫描线在LOW_COVERAGE绕树时的机体包络接触，完成针对性仿真回归；不能拿四轮三投落地结果替代失败路径验收。
6. 第09高速采集可穿插于第2步前后，逐级0.5→1.0→1.2比较识别覆盖、模糊/光照、实际速度和EV/LIO年龄。数据支持后再提高08速度，不同时改变航线、阈值和定位参数。

验收同时记录原软件终态、物理落地、实际投递类别/位置、碰撞、接管、耗时。落地静止后数秒才终态异常按用户口径可记飞行完成，但仍保留软件缺陷；空中碰撞或错靶不能合并算作完整成功。

## 本批动态验证的现场影响

同版最新六轮单独归档，矩形seed31/38均完成正确三投、走廊与H落地；先保持矩形作为下一轮对照。双扫描线seed38补搜发生真实树体接触，碰撞前LIO/飞控估计正常，机体中心对已发布样条存在约18cm空间偏离，需核查弯道跟踪、前视和机体扫掠净空。不要通过缩小碰撞包络、膨胀或忽略接触将其判为通过。现有记录缺少该时刻完整占据地图，不能排除建图/规划净空贡献。

走廊共同约束为FC中心AGL1.0m，但部分通过轮真实高度出现约5–7cm超调，旧Gate仍按1.2m区域门槛评价；共同约束接通不等于物理限高严格验收。进入实际限高通道前还需验证跟踪余量。新补丁已同步Git，不等同上板或同版实飞。完整结果以研究分支报告为准。

三线seed38已完成正确三投和H落地上锁，但上锁后的OFFBOARD反馈触发旧控制flight_controller_control_lost，使状态退出LAND；Bridge持续control_state_not_landing，软件完成确认未闭合。后续03/04/08必须同时验收落地后终态与人工接管取消，不能只验H检测框；本批按用户口径另记物理飞行完成。
