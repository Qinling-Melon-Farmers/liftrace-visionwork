# 11 临时低位单次释放试验

独立于正常投递许可、靶标检测与任务链，复用 `deployment/low_hover_observation` 的 runner 和 `Observation` 状态机。无需真实靶标。2026-10-16 本地开发与离线测试交付；未联网、未部署、未启动 ROS、未调用真实舵机。

## 工作台入口契约

`folder=11_low_release_test`、`channel=module`、`low_release_test:true`、`release_slot=1|2|3`。

```bash
# 纯离线：打印配置/路线；无ROS连接、控制发布、服务调用或PWM输出。
bash deployment/board_trials_4x4/11_low_release_test/start.sh preview --slot 1
bash deployment/board_trials_4x4/11_low_release_test/start.sh flight --slot 1 --real-release --check-config

# 运行命令留给现场操作者；本次开发未执行。
# 默认只有释放动作是mock，flight仍会发布真实飞行控制目标。
bash deployment/board_trials_4x4/11_low_release_test/start.sh flight --slot 1
# 显式确认使用现场已运行的真实raw舵机服务：
bash deployment/board_trials_4x4/11_low_release_test/start.sh flight --slot 1 --real-release
```

`--slot` 默认 1=后仓，2=右仓，3=左仓。`--real-release` 必须逐次显式传入；启动输出与状态明确标记 REAL/MOCK。服务类型检查不能区分同类型的mock与实物后端，真实开关表示操作者已确认当前 `/legacy/Servo_raw` 是现场5b真实链；程序不把bool ACK宣称为机械投放成功。

还支持 `--config PATH`、`--output PATH`。任何模式含 `--check-config` 均只离线校验；`preview --real-release` 也不触碰服务。配置检查可打印文本/JSON，但不产生控制/机构输出。WSL离线预览可设 `BOARD_PYTHON=/home/xhj/miniconda3/envs/rl_drone/bin/python`。

现场复用已运行的 ROS/MAVROS/MID360/低空定位；定位仍用原 `low_hover_observation/start.sh localization hover` 入口。本模块不会启动这些设备，也不会启动、重启、初始化或停止5b舵机驱动。真实模式仅只读等待/检查现有服务，服务缺失或类型不符在飞行目标发布前退出。

## 动作与参数

所有高度、路线、速度、话题和服务名在 [config.yaml](config.yaml)，不硬编码动作尺寸到源码。

1. 地面未解锁时初始化，沿用原FC/EV有效性、地面参考和录制就绪检查；预发保持目标。
2. 看到本轮 `READY_FOR_MANUAL_ARM_AND_OFFBOARD` 后，由飞手手动解锁、重新切入 OFFBOARD。无需 `/start_mission` 或第二次“开始任务”。仅解锁或继承启动前的OFFBOARD不会自动起飞。
3. 自动升到 FC AGL **0.60m**；按原观察 **0.08m / 0.12m/s** 到位条件连续确认 **2秒**。
4. 沿切入OFFBOARD时锁定的初始机头方向前移 **1m**，复用原forward的yaw坐标变换；不是固定FC local X轴。实到前点并完成同样2秒确认后，连续悬停 **5秒**；离开到位窗口会重新计时。
5. 之后才下降，目标 FC AGL **0.45m**。下降阶段只要当前高度消息新鲜、飞控已解锁且在OFFBOARD、原观察保护未取消，AGL严格 **<0.50m** 就允许本轮一次释放。**不等到0.45m，不要求低位XYZ到位/低速/停稳，不设0.35m下限。**
6. 无论ACK、失败或超时，都不重发；正常继续到前移点的0.45m目标并保持，飞手接管。无自动返航或降落。原定位失效/跳变/超时/停止保护仍优先进入 `HOLD_FOR_PILOT`；切出OFFBOARD后停止目标发布，任务不自动恢复。

默认上升 **0.15m/s**、水平 **0.20m/s**、下降 **0.10m/s**；位置目标30Hz。消息时间戳年龄和本地接收年龄均需≤0.30秒，沿用FC、EV和速度有效性检查；低位速度只要求有效，不设释放速度门槛。状态接收时效沿用2.5秒。

高度仍是原观察的FC中心AGL估计：`FC local Z - 初始FC local Z + fc_ground_clearance`，默认静置FC离地0.22m，因此名义爬升0.38m。它不是独立测距仪真值，机架变动时须按实测更新配置。

## 真实链、异步与本轮防重复

真实服务类型沿用 `patrol_control/Servo`，请求 `req=1|2|3`，只发一次。临时阶段/高度检查在专用继承类中完成，绝不修改或发布正常投递许可，不改 `/Servo` 代理、正常检测链、舵机脉宽或驱动。

5b已有 `actuator_pwm/pwm_node1` 继续持有其原有 `/run/lock/liftrace-servo.lock`；临时客户端不争抢此硬件锁。临时入口自身另用自动释放的文件锁防止多开，不是每槽永久锁。

调用、服务类型复核和尝试记录写盘均在异步线程；最后提交前再次读最新遥测/停止事件。raw服务无请求ID，故本轮调用一经安排就消耗机会；negative ACK、异常、取消或默认3秒超时均不重试。迟到ACK不能覆盖UNKNOWN，也不影响30Hz设定点循环。已发送的服务调用无法撤销，结果不确定需由现场确认。

每轮独立目录 `logs/low_release_test_<时间>/`；真实模式仅在本轮目录创建 `release_attempted.json`，不跨新run锁槽。同run的已有尝试记录禁止再次发送，已有输出目录也不复用；重新开始新轮须新run，不自动恢复旧轮。

复用原诊断bag/配置/reference/ending记录，增加 `release_result.json`。状态继续发 `/low_hover_observation/status` 以复用诊断录制；增加 `release_mode`、`release_slot`、`release_state`、`release_detail`、`release_gate_open`。`PWM_ACKNOWLEDGED` 仅表示驱动返回电气成功；`MOCK_COMPLETE` 没有调用舵机服务。

## 文件与离线验证

本目录：`start.sh`（入口）、`entry.py`（复用runner/服务客户端）、`release_core.py`（继承状态机/单次事务）、`config.yaml`、`test_release.py`、本README。

必要共用改动仅两处：`low_hover_observation/observation_core.py` 增加可覆盖的路线/分段速度/停留方法；`low_hover_observation/flight.py` 接受控制器工厂、准备钩子和独立节点名，输出专项状态。原观察默认行为保持，由原回归检查。

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python -m unittest discover -s deployment/board_trials_4x4/11_low_release_test -p test_release.py -v
python -m unittest discover -s deployment/low_hover_observation -p 'test_*.py' -v
bash -n deployment/board_trials_4x4/11_low_release_test/start.sh
git diff --check
```

2026-10-16结果：**28项专项＋54项原低空观察/录制回归通过**，shell语法及diff空白检查通过。专项覆盖完整跟随状态过程、yaw变换、人工OFFBOARD自动触发、到位/悬停计时、下降途中释放、严格0.50边界、无低位距离/速度/下限门槛、失效/接管取消、同轮去重/新轮复用、全部槽位、超时/迟到ACK、只读服务类型接线和离线入口无ROS。测试中的服务为内存替身，没有启动ROS；结果不替代板端飞行、实际机构或载荷验证。

初次交付未提交Git、未写共享changelog。2026-10-16审查通过后，按用户新增授权追加导航仓联调记录并独立中文提交，暂不push；不改工作台，板端只读检查与同步仍由主代理负责。
