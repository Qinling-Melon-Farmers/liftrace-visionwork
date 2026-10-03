# 现场冻结位姿保护回收（2026-10-03）

本次按用户明确要求连接 `orangepi@192.168.3.126`（Orange Pi 5），读取现场可用版本，再回收公共保护；未用取消跳变检查的开发头覆盖板端。

## 公共代码结论

现场 `HighViewProbe.update_pose` 保留同frame、有限值、时间不回退检查，同时：

```text
dt = stamp - previous_stamp
若 dt > 0 且三维距离 > 3.0 * dt + 0.25 m：拒绝本次位姿
```

现已恢复到视觉板端、视觉高位研究和导航 `feat/high-view-liveness-20260919` 的同名实现。三个分支仍保留此前“上报最初ABORT原因”的独立修复；没有整体撤回下降净空、轨迹交接、记忆冲突及五分类成果。

这是位姿一致性保护，不是把实机瞬态问题判定为已经解决，也不是提速配置。拒绝时保留上一次有效位姿与时间；板端调用层原有异常收尾保持。零时间差等边界沿用现场实现，不在本轮另行放宽或改造。

## 验证

- 高位probe/full等69项、mission core49项、release commitment5项，共123项通过。
- 真实跳变样本：约40ms内三维位移0.402m，超过约0.37m门槛，抛出`probe pose discontinuity`，不覆盖有效状态；随后正常样本可更新。
- 同时验证三维斜向位移及边界相等时可接受、时间倒退/非法frame/非有限数仍拒绝、原始ABORT原因不被后续tick覆盖。
- 三个工作区的公共实现/测试按文本比对相同；测试在本地运行，无新SITL或实飞。

## 现场舵机与录包范围

同轮对现场舵机补回既有PWM返回检查，**保持本台的4/5/0通道及原脉宽**。在地面未解锁状态，通过隔离的合成严格视觉证据驱动实际arbiter→许可代理→raw舵机服务，两轮均为三槽PWM成功，错误/重复请求拒绝。现场确认三槽均正常作动；随后说明回位是舵机后的填充物顶回，不再追查此现象。源码只在启动时发送初始脉宽，释放1秒后关闭PWM，未在释放后再写初始脉宽；保持原时序不改。本轮确认机构动作正常，未据此声称实际带载离机已验证。

现场 `mission_core.py`、`trial_bag.py`、`test_area.yaml` 与本地板端对应文件相同。轻量录包 `record_map_clouds=false` 默认剔除FreeDOM全图与占据图，同时 `record_inflated_cloud=true` 保留 `/sdf_map/occupancy_inflate`；不恢复原始雷达和独立相机视频编码。具体现场记录见视觉板端分支 `docs/deployment/board_redeploy_20261001/FIELD_FREEZE_20261003.md`。

## 分支处置

公共保护/回归/本文推送视觉 `feat/high-view-search-research`、`feat/board-deployment-flight-20260920`，及导航origin/fork的 `feat/high-view-liveness-20260919`；现场特定舵机参考包/入口则留在视觉板端和导航 `板端参考分支`。不把5的PWM映射覆盖5 Plus参考。

`main`、旧 `feat/r2026-main-integration`、`feat/r2026-competition-integrated`和根目录main保持原位；它们仍需完整冻结组合的同版任务验收，不能凭本次地面舵机验收宣布整机通过。用户未提交资产和旧VCL06工作树保留。工作台保持关闭；完整独立正赛部署与速度实现均未执行。
