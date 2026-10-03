# 舵机服务反馈修复（2026-10-01）

## 链路与原因
任务释放许可 → /board_trials/Servo代理 → /legacy/Servo_raw → actuator_pwm。
代理会依据raw res返回ReleaseResult；False/异常不会提交成功。20:07任务三次成功提交与现场未释放不一致，根源候选位于raw反馈以下，而不是首次手动终端没有source消息环境。
板端源码确认忽略PWM返回值，req1..3无条件res=true。析构又unexport通道，重启普通用户节点可能无法导出/写入；这些是明确缺陷，但没有当时sysfs错误日志，不能证明唯一现场原因。

## 实施
- 采用现场5Plus映射：槽1 chip2/febf0000、槽2 chip3/febf0010、槽3 chip4/febf0020。保留脉宽及1秒动作时间。
- 每次disable、duty、enable、等待、disable检查返回值，写入后读取核对；失败返回False并打印路径错误。
- 初始化先禁用/清零，再设置周期与极性，依次复位。全部完成才advertise原服务。
- 构造要求init_pwm.sh已经初始化可写通道；不再隐式导出。析构只disable，不unexport破坏下次启动条件。
- 使用WallDuration，真实硬件动作等待不依赖ROS仿真时钟。
- 保持服务名、槽号和代理协议，不自动重试未知投递结果。
- True只表示PWM写入和读回成功，无法证明舱门动作或包裹离机；机械确认仍需传感器或现场检查。

## 验证、部署
本机C++假PWM故障注入：正常脉冲及初始化通过；4个脉冲步骤与8个初始化步骤失败均不回报成功，前置失败不使能。
板端hardware_ws catkin_make -j2编译通过，仅更新源码和可执行文件，未启动/重启节点、未调用服务。
正在运行的旧进程不会自动更新。下一次地面安全准备时，先停止旧舵机服务，执行原init_pwm.sh，再launch_all.launch；启动会依次复位三个槽位，必须由现场确认机构允许动作。调用终端仍需source硬件工作区。
原本机及现场包快照、文件SHA256清单保存在legacy_baseline/20261001_servo_feedback/。
修复适用于该部署hardware_ws实际使用的actuator_pwm参考包，不覆盖试飞组远端板载代码或其他机型的旧接线。

## 待现场验证
实际复位与三槽释放、电源供电、拔除/权限失败时服务拒绝（地面无载情况下安排）。本轮不声称已完成物理投递验收。

## 地面许可链实测
用户明确授权真实机构测试，飞机静止未解锁。先读取飞控armed=false，发现旧pwm_controller与servo_controller1两个驱动同时存在，停止两者后启动新驱动，依次复位三槽。
使用工程guarded_servo_proxy，隔离许可/result/公开服务话题，raw仍为/legacy/Servo_raw；注入明确标注为地面模拟的ReleasePermission，不伪造飞控解锁或修改飞行话题。
- 无许可：permission_missing，False。
- 槽1/2/3有效许可：均raw_actuator_ack，True。
- 每槽再次调用：payload_slot_already_used，False。
- 测试结束驱动与代理均停止，三个PWM enable均0，通道仍导出；读回脉宽分别1700000/2100000/2100000ns。
结果已拉回logs/ground_servo_check_20261001/results.json及final_pwm_state.txt。
此测试验证许可代理到真实PWM链，不覆盖视觉证据生成和仲裁器生成许可的过程；物理舱门动作待现场反馈。未解锁/起飞。再次试飞前须重新准备载荷和启动单一舵机服务。
