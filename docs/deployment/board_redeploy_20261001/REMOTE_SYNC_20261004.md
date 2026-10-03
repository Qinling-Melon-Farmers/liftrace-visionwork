# 2026-10-04远端补推与H链同步

本次真正漏推的是视觉板端三笔本地提交：`bb8877b7`（投递偏移真实观测时间）、`8d4cb2df`（反光H修复及landing阶段暂停YOLO）、`3721e7cc`（人工OFFBOARD启动、H接管取消保护）。已补推，并将公共实现同步到现有研究/整机分支。

| 仓库/分支 | 本轮代码提交 | 范围 |
|---|---|---|
| 视觉 `feat/board-deployment-flight-20260920` | `3721e7cc` | 三笔完整板端补丁；此后提交仅追加本同步记录 |
| 视觉 `feat/high-view-search-research` | `b9778b05` | 视觉及控制公共补丁，不新增专项测试组 |
| 视觉 `feat/r2026-competition-integrated` | `4bce9f2e` | 同上；独立hardware_session同步人工OFFBOARD启动，保留正赛入口 |
| 导航 `板端参考分支` | `8c131b5f` | 九组板端入口、H视觉/控制、对准观测时间及文档 |
| 导航 `feat/high-view-liveness-20260919` | `3cf127cc` | 按已有接口移植视觉镜像修复；完整控制/硬件入口继续配套视觉集成仓 |

视觉推送地址为 `Qinling-Melon-Farmers/liftrace-visionwork`；导航参考推送地址为 `sakelier/liftrace-controlwork`；导航liveness同时推送至sakelier仓及Qinling-Melon-Farmers同名fork。以上均直接读取服务器 `git ls-remote` 核实，未仅依据本地tracking分支判断。

## 追踪信息过期的纠正

最初缓存显示导航参考停在61e5438、liveness停在5890a02，但服务器在本轮开始前实际已有a27852d、33f60739。原因是本地 `remote.origin.fetch` 仅包含main及旧VCL06，普通fetch不会更新这两条分支；先前据缓存称它们漏推不准确。已追加当前liveness、板端参考两个fetch映射，并为参考工作树设置origin上游，未更改或删除旧分支。

## YOLO与H的实际切换边界

此前整机已切换下游H检测、记忆和对准，YOLO节点仍计算并输出。新PT/RKNN门控在 `align_mode=landing` 暂停类别推理、丢弃跨阶段在途结果，退出该模式后恢复；走廊导航阶段保持原行为。不是杀进程，也不是删除类别模型。

## 验证与部署状态

研究37项、正赛62项、导航参考51项离线行为测试通过；公共视觉/C++控制文件与已完成构建及H离线回放的3721e7cc逐一比较一致。正赛额外验证独立启动器和配置，导航镜像另有21项离线测试及视觉包实际构建、安装通过，结果见其随提交说明。

飞机已断电，本次没有SSH、上板、ROS运行或SITL。远端已更新不代表飞机运行的文件已更新；H/人工模式/对准时间补丁仍待后续部署和同版实飞验收。未合并或推送main，未修改试飞组维护的“板载代码”。本地未提交的 `tools/bag_replay/merge_bags.py` 保留原样，未混入本次补推；bag、视频、模型也未加入Git。
