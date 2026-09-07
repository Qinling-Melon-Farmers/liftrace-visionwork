# 2026 竞赛集成路线图

本文件是当前任务优先级来源。最新 R56 已按用户要求在 WSL 项目内完成同轮重跑并收尾，不自动续跑。

1. **V-CL-06 当前阻断：门前地图可达性。** 三投、三恢复完成；首门前第 4 航段 goal 19 被一个持续静态点膨胀覆盖，90.04 s 超时。优先核对观测支持/地图清空与门洞安全高度窗口；不是本轮发生远距离无进展轨迹。见 [根因](docs/verification/r56/REPORT.md)。
2. **V-CL-06 H 与完整 Gate。** H capture 已 .75→1.60 m，生产几何离线 3/3；R56 未到 H，仍待完整记录。R55 三投、11 段、三门零碰撞但 H 等待时 I/O 中断，仍为 INCOMPLETE。
3. **V-SIM 交付。** 本机统一 WSL 项目 logs；R56 native 重跑越过启动、原图有记录且缓冲溢出 0。原 E 盘尝试作为同轮历史保留。main 仍待完整 PASS，最终 --no-ff 合并、保留分支并打 tag。
4. **V-DEPLOY 与量化。** 固定版本多 seed/启动与记录负载波动、板端 RKNN ROS 10 min、实际内外参/测高、扰动与接管、机械 ACK/落点。槽位补偿暂不做。

R55 的 5 cm 地图、有界搜索和 horizon 进度修复已经实施；不能继续写成“尚未应用”，也不能说已经解决所有占据波动。R41 的 429.875 s 历史功能 PASS 包含旧墙钟判据离线修正，不等同于当前配置通过。

[详细报告](docs/verification/r56/REPORT.md) · [参数全文与阈值](docs/verification/r56/PARAMETERS.md) · [实跑记录](docs/VALIDATION.md)

当前推进：接触代理射线遮挡、FreeDOM .40→.10 m 空闲尺度、视场和输入队列修正已经构建及离线验证；按用户新授权进行整场验证。此前失败 Gate 保留，失败轮次不发布全量 Release。见 [根因修复](docs/verification/r56_root_fix/REPORT.md)。
