# 4×5 红十字单次投递独立测试入口

本目录为新增 ROS Python 包 red_cross_4x5_test，直接引用原工程中的定位、地图、视觉、规划桥和控制器。没有修改原工程源码，没有重新编译。已获用户授权新增独立适配节点。

## 参数与行为

| 项目 | 当前值 |
|---|---|
| 任务坐标系 | camera_init；沿用原 4×4 的 map 单位静态变换 |
| 地面 Z | 固定 -0.25 m |
| 巡航目标 Z | 固定 0.25 m，即 FC 离地 0.50 m，恢复原 4×4 巡航离地高度 |
| 输出目标 Z 上限 | 0.25 m；不构成实际高度不超调保证 |
| 释放目标 Z | 0.10 m，即 FC 离地 0.35 m；沿用已有控制器释放目标默认值 |
| 释放高度门限 | 控制器 Z≤0.15；许可层 0.06≤Z≤0.15 |
| 投后回升交接 | Z≥0.20；后续航点 Z=0.25 |
| 规划速度／加速度 | 0.25 m/s／0.35 m/s²，采用原参考案例的低速值 |
| 水平障碍膨胀 | 0.30 m；与最近 4×4 的 0.10 m 不同，按原参考案例保守配置 |
| 地图 | 6×12×1.5 m，分辨率 0.05 m |
| 路线 | 原图七航点，末点 (1.4,4.4,0.25) |
| 投递目标 | 仅 red_cross；候选中心限制在 X[-1.4,1.4]、Y[0.6,4.4] |
| 释放次数 | 每个进程任务最多一次机会，每个运行目录最多一次物理服务调用 |
| 舵机映射 | 任务逻辑槽 1 → 物理 req=2（右舱 Pin7/pwmchip5） |

单次任务从 P1 开始，红十字目标可中断当前航段；投递恢复后回到未完成航段，而不是重走路线。未发现目标直接完成路线；投递失败、丢标超时或释放响应不确定后关闭本轮投递机会，继续路线。释放响应不确定保留失败/隔离记录，不伪造投递成功。

规划失败、定位异常、超出总时限、飞手切换模式属于飞行异常，不按普通投递失败强行继续；航点两次移动失败会中止，不跳点冒充路线完成。末点完成后再次确认末点位置，再等待新鲜里程计、OFFBOARD、已解锁和稳定保持 1 秒，最多请求 AUTO.LAND 三次。不发送视觉 LAND 命令给旧控制器，不依赖 H 标志。确认 AUTO.LAND、ON_GROUND 和解除武装后才标记 COMPLETE；不主动解锁或解除武装。

## 新增适配层

- scripts/single_drop_core.py：继承原 MissionCore/覆盖路线，新增单目标一次机会与原路线续航规则。内部为单红十字 profile；对既有视觉证据接口继续使用 r2026 协议标识。
- scripts/test_mission_manager.py：继承原 ROS 任务壳，保留原消息身份/时效校验与人工启动服务，增加末点非视觉降落及真实落地确认。
- scripts/right_servo_once.py：保留 guarded_servo_proxy 与释放许可；把逻辑槽 1 映射为物理右舱 2，物理调用前持久写入运行目录收据，失败也不重试。同一进程不能再次开始任务。
- scripts/bounded_frame_adapter.py：复用原双向坐标变换，在转换前将任务系输出 Z 限制为≤0.25，覆盖旧控制器较高的投后回升目标。

真实舵机节点复用原 actuator_pwm/launch_all.launch，源码已确认服务 remap 至 /legacy/Servo_raw。**该原节点启动会复位三只舵机；本测试只有释放动作限定右舱。** 不会自动执行 PWM 初始化、sudo 或修改系统设置。

## 使用

在板端进入本目录：

```bash
cd /home/orangepi/navigation_frame_fix_20260912/obstacle_4x5_red_cross_design/runtime
bash start_test.sh check
```

check 仅解析 Python/launch/YAML、展开两种节点图并检查参数，不启动节点，不写 ROS 参数，不调用设备服务。

已有 MAVROS 与 Livox 驱动，且原应用节点停止后，可由操作员启动预览：

```bash
bash start_test.sh preview start_camera:=true video_devices:=/dev/video0
```

已有相机节点时不传 start_camera:=true。预览启动定位、地图和完整视觉链，但没有控制器、任务执行节点或舵机节点，不发布飞控设定点。视觉可以显示其他类别，任务执行时只接纳红十字。

以下为后续现场授权后使用的命令，本次未执行：

```bash
HARDWARE_TEST_AUTHORIZED=1 bash start_test.sh flight start_camera:=true video_devices:=/dev/video0
```

flight 启动真实控制与舵机服务，舵机会复位；需先按原机械流程完成全部 PWM 通道初始化。脚本发现原应用冲突时退出，不停止用户进程。每次 flight 新建 runs/session_* 并保存 ROS 日志和单次释放收据，不自动录 bag。

启动全部节点、遥控器解锁后，先调用下列服务登记任务，再用遥控器切入 OFFBOARD；控制器自动起飞，在起点 Z=0.25 稳定后自动开始路线：

```bash
rosservice call /navigation/start_mission "{}"
```

服务返回 success: True 和 start_queued_waiting_for_armed_OFFBOARD_takeoff_stable_Z_0.25 表示登记成功，不表示已经开始路线。节点不主动解锁或切入 OFFBOARD。实际开始路线仍要求控制器报告起飞完成，起点 XY 误差≤0.15 m、Z 误差≤0.08 m、速度≤0.12 m/s、新鲜定位和 OFFBOARD/已解锁，连续满足 1 秒后调用原任务启动检查。

登记有效期为 120 秒，重复调用不延长；观察到解锁后再上锁、进入 OFFBOARD 后再退出、或调用 /navigation/abort_mission，都会取消待执行任务。取消登记不负责停止控制器起飞或降落。原任务启动检查失败时不自动重试，原因见 /test_4x5/status；每个进程只允许成功开始一次任务。

观察 /test_4x5/status 和 /navigation/mission_status。若无需开始任务，不能为了看路径调用启动服务。

## 使用边界

- 固定 ground_z 和 map/camera_init 单位变换是本次配置约定，不替代现场零点与朝向一致性检查。
- 0.25 是任务系目标 Z，地面 -0.25 下 AGL 为 0.50。输出限幅只能约束目标，不能保证实测高度无超调。
- 释放目标 Z=0.10、槽位偏移为零是当前配置值，尚未经本机带载精度验收；右舱安装偏移及实际投放效果需现场核对。
- 4×5 是布场范围，非硬电子围栏；规划绕障和目标接近有跟踪误差。
- 新建运行目录意味着新的释放机会；不能在同一次带载试验中通过重新启动 flight 绕过已消费收据。
- 当前只做静态与离线回归，没有启动 ROS 节点、相机、舵机、仿真或飞行，不能称已实机验收。

## 文件

test.launch 为统一入口；route.yaml 为路线和任务时限；map.yaml 为地图与规划参数；control.yaml 为控制/下降/恢复参数；bridge_guard.yaml 为执行桥及释放许可参数。修改时须一起保持高度与路线末点一致。
