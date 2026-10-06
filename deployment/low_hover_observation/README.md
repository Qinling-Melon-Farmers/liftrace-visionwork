# 低空电机观察试飞（独立于九组任务，2026-10-06）

本入口只做自动低空动作和诊断采集，不验收识别/投递/整机算法。代码本轮仅本地准备和离线验证，未上板、未起飞。飞手手动解锁并**重新拨入OFFBOARD**后动作；只解锁不会开始。结束保持悬停，飞手切手动降落，脚本不会自动LAND、解锁、切模式或调用舵机。

## 三种动作

| profile | 动作 | 高度 / 速度 | 结束 |
|---|---|---|---|
| `hover` | 起飞至指定高度，稳定后观察10秒 | FC中心估计离地0.60m；上升0.15m/s | 原地悬停，等待手动降落 |
| `forward` | 先悬停观察，再沿初始机头方向前移0.5、1.0、1.5m；每点观察5秒 | 同高度；水平0.20m/s | 最后一点悬停，无自动返航 |
| `square` | 先悬停观察，再飞1m×1m四边形；每点观察5秒 | 同高度；水平0.20m/s；机头指向不主动改变 | 回起点上方悬停 |

参数统一在[profiles.yaml](profiles.yaml)。四边形在初始机头的前方与左侧展开，整个1m×1m区域及起飞点需要现场净空；这是无避障的专用观察轨迹，没有启动Fast-Planner。停顿是为了看电机与姿态，不沿用竞赛不停点提速。

**高度不是从静置位置再爬升0.60m。** 默认继承FC静置离地0.22m：采集初始FC local Z，地面基准=初始Z−0.22，目标Z=地面基准+0.60。因此名义爬升0.38m。相机外参不参与。起落架/机架改变后先测量并改`fc_ground_clearance`；输出是估计FC高度，不是独立测距真值。现场用尺/外部视频记录实际离地高度，不能用同一条FC曲线反证它本身准确。

## 启动顺序

每个板端终端先执行（0928原目录，不新建部署工程）：

```bash
cd ~/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
```

1. ROS master：没有运行时才执行`roscore`。
2. MAVROS：沿用现场已验证串口；此前配置是：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

3. 雷达驱动：

```bash
roslaunch uav_mission mid360_driver2.launch \
  user_config_path:="$PWD/deployment/site_20260928/MID360_config.json"
```

4. 仅定位和EV输入：

```bash
bash deployment/low_hover_observation/start.sh localization hover
```

启动前要求飞控连接且未解锁，拒绝重复LIO/EV和已有任务/相机/控制发布者。此launch复用已部署FAST-LIO配置、板端负载参数和`lio_external_pose.py`，不启用FreeDOM、相机、YOLO、导航任务、舵机或setpoint适配器。保持现有三线程构建，不调整EKF参数或300ms EV输入时效阈值。飞行中不要关闭定位终端。

5. 地面确认飞控SD卡ULog录制已准备，按[补录说明](px4_logging/README.md)补充必要字段，然后启动本轮动作及独立诊断录制进程：

```bash
# 可以先离线预览，不连接ROS、不发控制
bash deployment/low_hover_observation/start.sh preview hover

# 每次只选一个；一轮落地上锁后再重新初始化下一轮
bash deployment/low_hover_observation/start.sh flight hover
# bash deployment/low_hover_observation/start.sh flight forward
# bash deployment/low_hover_observation/start.sh flight square
```

默认需5个终端/窗格（ROS、MAVROS、雷达、定位、动作+录制）；已有前三项时只增开后两项。脚本不启动相机、压缩图、JPEG relay、MP4或常规九组bag入口，也不启动PWM。先停止旧的视觉/任务应用和相机录制进程，程序会检查冲突，不会擅自杀掉它们。预览模式不需要ROS；如本机用conda，可设置`BOARD_PYTHON=/home/xhj/miniconda3/envs/rl_drone/bin/python`。

定位/XML入口已做静态展开确认，仅有FAST-LIO和EV桥两个节点；17项动作回归、29项录制/profile离线检查及20项分析工具测试通过；GUI冒烟和三份ULog无显示批处理通过。这些结果不等于完成真实飞行或闭环SITL。

地面FC与LIO同参考检查沿用静态单位关系：新鲜消息、时间配对、位置差≤0.20m、航向差≤5°持续2秒，FC静置参考采样至少1.5秒；不是自动估一个变换把差异藏起来。通过后预发保持目标，看到`READY_FOR_MANUAL_ARM_AND_OFFBOARD`再由现场飞手解锁、拨入OFFBOARD。若启动时飞控已是OFFBOARD，需要先切出再切入；不会因沿用旧模式而自动起飞。READY后断流会锁定参考失效，需在地面退出并重新初始化，不能恢复几条消息就沿用旧参考起飞。

## 飞行中与结束

- 悬停/前移/四边形均采用30Hz位置目标，发布路径没有舵机服务等待或磁盘写入。前视最多0.15m，不按时间盲目前移。
- `FINISHED_HOVER`表示观察路线结束，继续悬停；飞手切出OFFBOARD后不再发布飞行目标，也不会自行恢复任务。手动降落上锁后动作程序关闭本轮录制。LIO和MAVROS由各自终端继续运行。
- `HOLD_FOR_PILOT`表示停止继续路线，等待接管。例如定位/EV变旧、估计位姿明显跳变、范围超出或观察期限耗尽。它只是受限观察的接管机制，**不是完整FC reset补偿方案**，不能据此保证任意错误定位下物理位置不动。
- 空中按动作终端Ctrl+C只请求保持等待接管，不立即结束设定点发布；切手动并落地上锁后才正常退出。强杀进程、断电或ROS整体退出不在此保证内。
- 0.6m很低，地效、载荷偏置和安装状态会影响电机命令；记录实际硬件/载荷/电源条件，不能仅根据某路控制值较高判定电机损坏。

## 录制产物和分析

`logs/low_hover_<profile>_<time>/`保存配置、地面参考、规划观察点、结束原因和`recording/`诊断bag；不录图像或点云。必要数据包括两路位姿、EV输入、LIO实时耗时、IMU、遥控模式、目标位置/姿态、电池和可用ESC遥测。标准MAVROS不一定提供EKF所有拒绝与reset字段，因此**飞控SD卡ULog仍是主要依据**；飞后取回，不在空中经MAVLink下载。

配套工具：[ULog查看器](../../tools/flight_logs/ulg_motor_viewer/README.md)、[飞控日志取回](../../tools/flight_logs/README.md)。三份新增历史日志分析见[诊断](../../docs/deployment/low_hover_diagnosis_20261006/README.md)。记录ROS时间、接收墙钟、单调时间及模式/解锁事件，板端日期有偏差时用事件对齐，不只按文件名匹配。

录制器失败会停止新观察动作、提示接管；不会自动停止LIO或更改飞控模式。诊断记录不能恢复不存在的ESC电流、RPM或测距数据，缺项在分析中明确列出。
