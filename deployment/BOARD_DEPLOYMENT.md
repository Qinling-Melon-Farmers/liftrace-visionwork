# 板端部署与试飞分支

分支：`feat/board-deployment-flight-20260920`。以当前板端专项成果为基础，专门保存部署入口、现场修复、试飞配置及结果摘要；不合入main、不替换正赛基线。

## 当前可维护入口

| 入口 | 目标 | 结束方式 |
|---|---|---|
| [视觉中断](board_trials_4x4/01_visual_interrupt/README.md) | 1.4m直飞，中断、对齐、一次模拟投递 | 原地降落 |
| [高位记忆重访](board_trials_4x4/02_high_view_revisit/README.md) | 2.6m完整一圈，重访已记忆的1–3目标 | 最后目标处降落 |
| [H降落](board_trials_4x4/03_h_landing/README.md) | 前方约2m H，接近、定点升高、对齐 | H上降落 |
| [走廊＋H](board_trials_4x4/04_corridor_landing/README.md) | 按实测点自主避障，末端H对齐 | H上降落；航点/H留空时拒绝运行 |

四套统一默认：`alignment_mode=legacy_static`、`virtual_ceiling_enabled=false`、巡航上限0.5m/s。单位静态TF采用现场旧板端口径；已知相机/IMU外参和双向数值适配保留。不要同时启动实测对齐与单位静态TF。静态TF不证明两套估计器绝无误差。

共同修复：自动地面基准；float/double高度比较一致；READY同时要求视觉与控制设定点；只有经历已解锁IN_AIR后落地上锁才自动收尾；高位转低位也不重新开启顶棚；模拟投递服务独立隔离。目标高度、控制Z限幅与水平障碍柱仍有效。

板端现目录：`/home/orangepi/liftrace_board_trials_20260920`，当前SSH地址`10.231.47.193`（网络变更后重新确认）。[完整操作说明](board_trials_4x4/README.md)含构建、模型、初始化和任务启动。

## 设备与相机

MAVROS和MID360 driver2按现场既有配置先启动；视觉测试还必须有相机图像与CameraInfo。相机未启动时可在工程根目录另开终端运行：

```bash
bash deployment/board_trials_4x4/start_camera.sh /dev/video0
```

如果现场相机进程已在发布同名话题，复用该进程，不重复启动。四套测试入口不自动解锁或调用任务开始服务。应用READY只表示输入/控制输出就绪；起飞、控制就绪和航线启动是后续步骤，不能把它们混为一件事。

仓库不含RKNN权重、build/devel、原始bag/视频。模型使用板端已有`runtime_models/merged_standard_fp32.rknn`，元数据在`vision_ws/src/uav_vision/config`。不能只复制某个settings.yaml到旧包而忽略匹配源码与消息版本。

## 现场旧工程留档

[旧板端4×4参考镜像](onboard_obstacle_reference_20260920/README.md)保存现场原始源码/配置、4×4说明、其他现场设计笔记及四轮茶几实跑参数快照。该目录有CATKIN_IGNORE，在活动工作区之外，供协同差异分析；不是新四套的运行overlay。

## 本轮实际飞行

[2026-09-20日志复盘](../docs/deployment/board_flight_20260920/REPORT.md)：首轮日志与录像已取回，0.4→1.2m约6.13s；任务全程IDLE、无模拟投递。第二轮任务已启动，第一点通过，第二个固定终点被膨胀地图占据，12秒后ABORT；详见[地图占据专项分析](../docs/deployment/board_flight_20260920/MAP_ABORT_ANALYSIS.md)。两轮均不能标作专项通过。
