# 实机入口与待验收工作

当前R61机载包可用于部署联调准备，不能宣称已具备稳定比赛放飞验收。R60历史十seed仅2/10完整通过；新几何、回退修复与新路线未完成全场飞行。本轮无实机驱动、解锁、起飞或舵机动作。

`competition_hardware.launch`为应用入口，RKNN视觉、LIO/FreeDOM、Planner、任务与控制共用已有链。它不加载Gazebo、真值/接触评测、mock、自动解锁辅助或PX4仿真参数写入。MAVROS、真实MID360驱动及机械`/legacy/Servo_raw`由设备侧配套；相机包已包含`camera_calibrated_1280x720.launch`和标定。

实测相机FC下16cm、IMU下21cm，`body_to_imu_xyz=0 0 0.05`，`imu_to_camera_z=-0.21`；相机落地高6cm，若落地FC为local0则ground_z=-0.22。`mid360_hardware.yaml`内雷达自身测量外参保留，不用Gazebo ray外参替换。

**硬件runtime/control仍为此前版本**，搜索由launch设1.40local、外部指令上限2.30local；R61 SITL分别为1.18local/2.08local以维持1.40/2.30AGL。不能只更新一个yaml就直接上机。部署时须按现场坐标、高度零点和选定流程整体配套，并确认启动条件。现场坐标按用户要求到比赛再修改。

板端主路径为RK3588 RKNN/NPU，已有独立视觉性能记录不代表本次整机并发LIO/Planner的实时稳定性。需用实际相机、曝光/反光、CameraInfo/TF同步和板端负载重新验收。实机尺寸50×50×37cm，SITL保守包络55×55×40cm；通用动力学未标定，模型支架不是实机CAD。

待完成：未知80cm门开口感知/航点更新，赛前调整后的缓存与一次启动；窄门定位/跟踪误差；新几何全场回归；设备链/板端并发稳定性；机械带载与落点；RC接管/急停和分级试飞。失去蓝环回旧(0,0)的源码漏洞已修且回归通过，不能继续列为未修复，也不能据此认为所有失败都已解决。

[机载包说明](../deployment/README_ONBOARD.md) · [规则校正](competition/RULES_20260906.md) · [验证范围](VALIDATION.md)。
