# H识别、观测时间与人工模式接管同步（2026-10-03）

来源：视觉 `feat/board-deployment-flight-20260920` 的 bb8877b7、8d4cb2df、3721e7cc。正赛共享 hardware_session 同步手动OFFBOARD门控，保留无专项组依赖的正赛入口；随仓专项薄入口仍复用该会话实现。

- 投递偏移使用真实图像观测时间查询测高，未更改像素/米转换接口、相机外参、槽位或释放许可。
- H采用81像素局部阈值窗口及7像素闭运算；PT/RKNN在 `align_mode=landing` 暂停类别推理并丢弃跨阶段在途结果。旧版仅切换下游H处理，没有停止YOLO计算；走廊导航阶段保持原行为。
- 外部H降落收到人工模式变化、断连或过期状态后取消事务；重复LAND/旧轨迹不能恢复已取消的降落。保留H几何、十帧稳定及位姿跳变保护。
- 旧控制改动前快照继承 `legacy_baseline/20261003_h_rc_handoff/`（来源c916cdd1，两个被修改控制文件与本分支修改前一致）。

本次仅同步本地与远端功能分支，飞机已断电，未部署、未启动ROS/SITL；源码同步不等于新版已实飞验收。原视觉板端分支已有实际C++构建、H离线回放及标准H/负样本测试通过。当前分支验证结果另见联调记录。

板端报告与原始回放记录：[H专项复盘](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/3721e7ccb12a85041fb4be7010c97ec1f6e7ff5d/docs/deployment/board_redeploy_20261001/H_LANDING_REPLAY_20261003.md)。
