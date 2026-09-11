# 当前运行链与计算图

2026-09-12搜索研究接口约定：在研究分支增加默认关闭的`plan_manage/CheckLocalSegments`只读服务，复用当前Fast-Planner的膨胀占据栅格与最近点云更新窗；限制批量、长度、体素访问量与墙钟预算。服务仅报告局部折线是否碰到已知膨胀占据，不运行轨迹优化，不更新目标/FSM，不承诺动态可达或已知自由。独立研究建议节点消费观测状态和原任务SEARCH/RESUME目标，输出局部候选及拒绝原因；不发布NavigationDecision或规划目标，不更改任务游标。图像观测历史和规划地图的身份/时效分别校验，联合定位重置仍待联调。

[R64三版rqt_graph Nodes only图](topology/r64/README.md)来自固定seed11完整PASS运行的ROS Master注册快照。全图36节点，核心22节点，仅飞行23节点。椭圆为节点，连线文字为话题，箭头为发布→订阅，服务不画成话题。2026-09-09离线更新，无新仿真。

Mission Manager负责搜索、目标事务、投后路线和降落；Planner Bridge是唯一`/fastplanner/goal`发布者；Fast-Planner→轨迹服务器→patrol_control→MAVROS。视觉包括检测、融合/精修、投影、目标记忆和对准。R63恢复保持与R64 PX4重置修复没有新增第二套任务权威。

雷达/IMU→FAST_LIO；`/cloud_registered_body`→FreeDOM，结合TF生成`/freedom/static_pointcloud`→规划器/任务层。LIO→MAVROS外部位姿约束水平/航向；飞控IMU、气压计测高。相机FC下16cm、雷达IMU下21cm，刚性随完整机体姿态，投影使用安装TF。

视觉生成身份/时间相关对准证据；`release_permission_arbiter`发布许可，`guarded_servo_proxy`提供`/Servo`并调用`/legacy/Servo_raw`。仿真raw端为mock，机械组实物执行器另供，本年度精简分支不包含PWM实现。

全图含Gazebo、布设/碰撞/Gate评测及两路录像；真值和观察节点不进入实机飞行入口。仅飞行图移除它们，不虚构本轮未注册的设备。板端使用`target_detector_rknn`替代SITL的`target_detector`；MAVROS、相机、雷达和机械服务按设备配套。此图不证明板端实时链已验收。

当前R64：seed11完整PASS，十seed原始7/10完整PASS，5/7/8有旧布设压墙，生成器已修但新矩阵未重跑；seed3近地落地仍失败。[矩阵报告](verification/r64_matrix/REPORT.md)。随机门实验已完成五seed，2/5整场PASS、RL未采样；辅助相机仍未启用，[本轮仅作计划与离线比较](planning/r64_time_camera/PLAN.md)。
