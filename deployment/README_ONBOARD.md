# 机载联调源码包

本包包含当前视觉、camera_sdk、导航/LIO/地图/规划/控制/任务代码、消息、参数和文档，附已选用的`runtime_models/merged_standard_fp32.rknn`。不含机械组PWM实现；保留`Servo`定义和`/legacy/Servo_raw`对接约定。`BUNDLE_MANIFEST.json`记录源码版本和权重来源。

这是部署联调准备包。R61新机架/地图没有全场飞行验收，硬件默认runtime/control仍为此前配置；不能把解包完成等同可直接比赛。请先读`docs/HARDWARE.md`、`docs/CAMERA_AND_FLIGHT.md`与`docs/competition/RULES_20260906.md`。

在已有Ubuntu20.04/ROS Noetic、编译依赖及Livox SDK的机载环境解包后，于包根目录编译：

```bash
BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash --extend
export UAV_VISION_RKNN_MODEL_PATH="$PWD/runtime_models/merged_standard_fp32.rknn"
```

代码包不携带笔记本x86的build/devel，不替板端安装驱动或Python环境。RK3588需已有匹配的NPU驱动、RKNN Lite2/librknnrt；ROS Python使用板端可导入的RKNN环境，不在系统Python临时pip安装。MAVROS和飞控连接、MID360设备驱动/网络与机械服务由设备侧配套。导航树保留已有Livox驱动源码，实际MID360驱动版本/消息类型须按板端已部署链核对。

相机入口：`camera_sdk/camera_calibrated_1280x720.launch`，原始1280×720不旋转图像；UVC设备列表和话题可覆盖。应用入口：`uav_mission/competition_hardware.launch`，传`model_path:=$UAV_VISION_RKNN_MODEL_PATH`。这两个入口在本次打包时均未启动。准备实际设备运行时遵守实机动作授权要求；应用入口不是完整设备上电脚本。

外参最终为相机FC下16cm、IMU下21cm，落地镜头离支撑面6cm。`ground_z=-0.22`只适用于落地FC为local0；现场不同零点须重标。硬件默认旧搜索1.40是local值，不是新SITL的1.40m AGL；不能只覆盖runtime文件而忽略launch内搜索/高度上限。现场应整体选配并复核路线/坐标/高度、开启条件、RC接管，再做分级飞行。包中保留共享包内的测试及仿真launch便于构建追溯，但硬件入口不加载Gazebo/真值/接触评测/mock/自动解锁辅助节点。

当前不提供自动启飞的systemd开机服务，也不包含机械舵机实现。随机门开口感知、赛前调整缓存/单次启动联调、带载投递及整机实时稳定性仍需完成。
