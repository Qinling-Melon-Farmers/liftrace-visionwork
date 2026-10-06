# POSCTL控制补丁分支同步（2026-10-06）

本表记录控制/导航交接补丁及部署说明，不声称六个分支的整仓源码相同；不同任务/投递/视觉研究能力保留。main未合并或推送。

| 位置/分支 | 控制提交 | 部署文档提交（本次诊断更新前） | 验证 |
|---|---|---|---|
| 试飞source：feat/board-deployment-flight-20260920 | fd1019ec | 6ddaa23a | 本地实际构建/回归；板端ARM+43项+6配置展开 |
| 导航来源：板端参考分支 | d35d0304 | e819a6f4 | 同源控制/桥接，旧包快照先行7fe30950 |
| frame：feat/r2026-competition-integrated | c182dafb | d2b89b7d | 161 PASS |
| 高位搜索：feat/high-view-route-speed-20261004 | f9bda612 | 5df16453 | 161 PASS |
| EV：feat/ev-continuity-20261004 | e522c48e | 9fb9170e | H交接全通过；189 PASS、2 skip、2既有失败 |
| 导航liveness：feat/high-view-liveness-20260919 | e8874792 | ca7f0832 | 153 PASS；origin+fork均已push |

四个其他feature分支均先提交旧包快照，再同步局部补丁；EV按自身旧结构移植，没有整文件覆盖。其余分支对齐H交接身份/超时/新鲜度，保留各自高速、motion、投递和同步中的视觉改动。

EV两个既有失败为 `test_recovery_hold.py` 的轨迹进度C++测试fixture缺符号，及 `test_release_commitment.py` 的旧源码契约断言；已在修改前HEAD复现。两个跳过是EV旧链没有来源分支的异步ALIGN能力。本次没有引入EV缺失的release_transactions投递实现，没有把这些结果记作整仓验收PASS；该研究分支未部署到飞机。

所有上述运行及部署文档提交均已推送对应feature远端；独立视觉候选f0ff8999尚未在本轮板端部署。用户原有未跟踪产物/脚本保留。新ULog诊断是文档追加，不改变已通过检查的运行代码，不额外启动测试飞行。
