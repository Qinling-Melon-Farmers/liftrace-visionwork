# 运行链和接口

Mission Manager 决定搜索、接近、恢复、返航和降落。Planner Bridge 是唯一 `/fastplanner/goal` 发布者。Fast-Planner 输出曲线，由轨迹服务器生成位置指令，再经 patrol_control 发布 MAVROS setpoint。外部任务模式不启动旧任务管理器或旧视觉桥。

视觉以 `uav_vision` 的候选、目标记忆、对准偏差、释放证据为输出。候选身份和时间戳用于关联当前任务；已结束目标的规划事件不会终止新目标。走廊到达必须相对原始门前/门后航点判定。

雷达自身 IMU 与点云进入 LIO；LIO→MAVROS `vision_pose/pose`→PX4 约束水平位置和航向，飞控 IMU/气压计负责高度融合。FreeDOM 使用融合机体位姿与 IMU 安装外参变换点云，与控制和相机共用坐标。此转换只处理消息类型和安装外参，不增加任务握手协议。

机械组对接 `patrol_control/srv/Servo.srv`，当前请求值对应三个槽位，响应表示执行确认。`/Servo` 经过现有释放许可代理调用 `/legacy/Servo_raw`；仿真使用 mock 服务。本分支不包含 actuator_pwm 实现，不运行舵机、PWM 或真实相机驱动。

`legacy` 服务名属于现有通信接口，不能仅因名称含旧字样而删除。与比赛路径无关的参考副本从本分支移除；仍参与编译的 patrol_control 是当前执行器。
