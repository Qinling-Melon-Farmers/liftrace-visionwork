# 当前运行链与计算图

R61只在既有标准靶对准分支修复外部丢标回退，保留当前事务目标；没有新增飞行节点/协议。安装TF更新为FC→IMU +0.05m、IMU→光心 −0.21m，落地FC零点下ground_z=−0.22。当前图仍来自R60实跑，不能当作R61已飞行的注册快照。

Mission Manager负责搜索、目标事务、返程阶段与降落；Planner Bridge是唯一`/fastplanner/goal`发布者；Fast-Planner→轨迹服务器→patrol_control→MAVROS。R60用任务已有航点完成边界设置现有参数，消费者缓存读取，没有新增飞行节点、消息类型或二次任务权威。

视觉链包含检测、融合/精修、投影、目标记忆、投递对齐；低空H笔画补充位于原landing_detector内部，使用原detections/CameraInfo/TF链。目标时间/身份关联与释放许可保留，不能把超时或失去目标时的“未投”改成成功。

`/Servo`经过guarded_servo_proxy调用`/legacy/Servo_raw`；仿真使用mock，机械组提供实际执行器。本精简分支只保留服务定义及调用链，不含机械PWM实现。服务不画成ROS话题。

LIO→MAVROS vision_pose约束水平/航向，飞控IMU/气压计测高；点云经对应安装TF进入FreeDOM。Gazebo接触和真值仅在评测/记录链，硬件入口不启动这些节点。

[rqt三版Nodes only图](verification/r60_full_matrix/topology/index.html)：来自R60先导PASS的真实ROS Master注册快照，通过rqt_graph后端生成。完整34节点，核心30节点，仅飞行筛选23节点。椭圆为节点，连线文字为话题，箭头为发布→订阅；边数量不等于话题数量，多发布/订阅会产生多条边。

仅飞行图移除Gazebo/布设/评测/记录/mock与仿真启动辅助；它仍是该次仿真的实际节点筛选。板端应用静态展开22节点，外部MAVROS/设备驱动另供输入，target_detector由target_detector_rknn替代。该图不能证明板端链已经实时验收。

最新十seed2/10完整通过，根因以[失败报告](verification/r60_full_matrix/FAILURE_ANALYSIS.md)为准；Gate中提前结束后的多个false检查不等于多个通信协议故障。
