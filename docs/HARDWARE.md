# 实机入口与待验收工作

当前R64机载包用于部署联调准备：新机架/地图seed11已完整37/37 SITL PASS；十seed原始7/10完整PASS，5/7/8存在旧布设压墙且生成器已修。合法seed3近地降落仍失败，板端/实机完整验收尚未回传。R60/R61/R62结果按历史范围阅读，不能继续称当前尚无完整仿真成功，也不能称已稳定实机放飞。用户已安排队员准备实机，后续闭环清单见[就绪度与代码规模评估](engineering/READINESS_CONTRIBUTION_LOC_20260909.md)。

`competition_hardware.launch`为应用入口，RKNN视觉、LIO/FreeDOM、Planner、任务与控制共用已有链。它不加载Gazebo、真值/接触评测、mock、自动解锁辅助或PX4仿真参数写入。MAVROS、真实MID360驱动及机械`/legacy/Servo_raw`由设备侧配套；相机包已包含`camera_calibrated_1280x720.launch`和标定。

实测相机FC下16cm、IMU下21cm，`body_to_imu_xyz=0 0 0.05`，`imu_to_camera_z=-0.21`；相机落地高6cm，若落地FC为local0则ground_z=-0.22。`mid360_hardware.yaml`内雷达自身测量外参保留，不用Gazebo ray外参替换。

**硬件runtime/control仍为此前版本**，搜索由launch设1.40local、外部指令上限2.30local；R61 SITL分别为1.18local/2.08local以维持1.40/2.30AGL。不能只更新一个yaml就直接上机。部署时须按现场坐标、高度零点和选定流程整体配套，并确认启动条件。现场坐标按用户要求到比赛再修改。

板端主路径为RK3588 RKNN/NPU，已有独立视觉性能记录不代表本次整机并发LIO/Planner的实时稳定性。需用实际相机、曝光/反光、CameraInfo/TF同步和板端负载重新验收。实机尺寸50×50×37cm，SITL保守包络55×55×40cm；通用动力学未标定，模型支架不是实机CAD。

待完成：未知80cm门开口实际点云规划、赛前调整缓存与一次启动、窄门定位/跟踪、合法随机场景回归、近地降落与近墙捕获净空、设备链/板端并发、机械带载与落点、RC接管/急停和分级试飞。失去蓝环回旧(0,0)的漏洞、投后恢复保持及PX4历史重置重复应用已修；固件补丁仍须匹配目标飞控验证。本地actuator.zip仅PWM定值测试，不提供当前raw Servo服务，不能当作真实执行器已接通。

[机载包说明](../deployment/README_ONBOARD.md) · [规则校正](competition/RULES_20260906.md) · [验证范围](VALIDATION.md)。
