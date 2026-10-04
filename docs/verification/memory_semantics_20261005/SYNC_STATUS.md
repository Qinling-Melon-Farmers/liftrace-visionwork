# 类别记忆修复同步（2026-10-05）

| 仓库/分支 | 记忆修复提交 |
|---|---|
| 视觉 feat/high-view-route-speed-20261004 | 4a8324b1 |
| 视觉 feat/high-view-search-research | 3271a79f |
| 视觉 feat/r2026-competition-integrated | d1b49918 |
| 视觉 feat/board-deployment-flight-20260920 | 823c1ea7 |
| 导航 feat/high-view-liveness-20260919 | c5ba7a76 |
| 导航 板端参考分支 | 15a4d739 |

表中为代码修复提交，不是后续报告提交。研究、整机候选、专项、导航liveness（origin和fork）及板端参考分支均已推送并核对远端。导航旧视觉副本一并补齐当前target_memory依赖的target_selection_policy，按视觉仓来源版本同步，未另行演进检测模型或接口。

验证：来源视觉Python43项；整机候选、板端专项和导航liveness各10项生产记忆及13项冲突链回归通过；导航真实ROS消息下生产target_memory及本地依赖导入通过。原bag回放4202帧，关键窗口旧版72帧错误可用panzer、新版0帧。

没有部署现场机载电脑，没有改动试飞组“板载代码”、main或EV连续性研究分支。八轮旧头SITL独立归档，不算本补丁的闭环验证。

八轮图表/视频与固定四树后续研究场景仅同步高位研究分支，未作为新的整机候选默认。
