# 实机入口与待验收工作

R60仅有笔记本SITL先导完整PASS，十seed2/10，不能作为稳定比赛放飞验收。本轮未启动实机驱动、解锁、起飞或舵机动作。硬件入口未自动切换到R60低空候选，仍默认加载vcl06_horizontal_field_runtime.yaml与vcl06_horizontal_control.yaml；这与当前SITL实验默认不同。

`competition_hardware.launch`静态展开22个应用节点，不包含Gazebo、碰撞/真值评测、mock、自动解锁或PX4参数写入。MAVROS、真实Livox雷达/IMU驱动、相机SDK与CameraInfo由设备侧提供，机械组实现`/legacy/Servo_raw`。

用户确认雷达正装在机顶、IMU到相机下方21cm；`imu_to_camera_z=-0.21`表达该关系。`body_to_imu_xyz`默认z=0.12412来自仿真机架，必须按真实飞控位置原点测量。`mid360_hardware.yaml`保留实机雷达内部外参，不能用Gazebo ray frame外参覆盖。

板端主路径为OrangePi5Plus/RK3588 RKNN/NPU。FP32 RKNN离线曾有约16fps记录，但真实相机、TF/CameraInfo、连续运行与整机负载下延迟仍需独立验收。笔记本PyTorch结果不代表板端。

下一阶段先处理本轮确认的软件问题：窄门横向总误差、H转向/落地净空、外部投递丢标误退起飞点、被占据搜索端点与候选可达性调度；修复后须另行实跑，不能把未验证修复直接写成新PASS。

此外还需现场坐标/高度零点复核、真实护圈遮挡与低空定位、机械带载投递/落点、遥控接管/急停及分级试飞。实际飞行动作仍需用户明确确认；当前只交付代码与诊断材料。

[全部结果](verification/r60_full_matrix/REPORT.md) · [失败与后续最小方向](verification/r60_full_matrix/FAILURE_ANALYSIS.md) · [本轮硬件静态节点名单](verification/r60_full_matrix/hardware_nodes.txt)。
