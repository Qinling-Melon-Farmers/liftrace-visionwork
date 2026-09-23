# 走廊任务只读诊断记录

现有 rosout 能证明首个航点未完成及超时，但不能还原规划桥每次里程计拒绝原因。首点重合不是已证实的根因；尤其要核对任务 `camera_init` 与 MAVROS odom 的 `header.frame_id`，静态 TF 连通不意味着桥接代码会自动转换消息。

在工程根目录、已有 ROS 系统运行时执行：

```bash
bash top_level_scripts/record_corridor_diagnostics.sh 180
```

建议在用户自行开始任务前启动记录，等待 `logs/corridor_diag_*/recorder.log` 中出现订阅记录。脚本不启动硬件、仿真或任务，不设置参数、不调用飞行/解锁服务。默认 180 秒后结束，最长允许 3600 秒；Ctrl-C 可提前结束，只收尾自己的记录进程，不停止用户节点。第二个参数可指定归档父目录。

输出目录每次唯一，包含：

- `navigation*.bag`：LIO 与 MAVROS 位姿/里程计、目标、规划生命周期、B 样条、控制设定点、任务决策/结果、桥接状态、飞控模式/落地状态、TF 和 rosout。保留 bag 接收时间与原始 header 时间戳，可计算消息年龄、断流、坐标系差异以及目标距离。
- `start/`、`end/`：节点/话题连接及明确白名单中的导航参数，缺失参数也保留错误文字。快照为顺序读取，不是原子快照。
- `topics.txt`、`metadata.txt`、`recorder.log`、`bag_info.txt`：采集范围、时段、记录器告警及实际收到的话题统计。

不记录点云、图像；内存缓冲 64 MB，bag 每 128 MB 分片，保留所有分片。总量随时长增长，不是 128 MB 总容量限制。未发布话题不会有消息，需结合 `bag_info.txt` 与连接信息判断；记录器缓冲溢出等告警见 `recorder.log`。异常留下 `.bag.active` 时脚本会提示采集未完整关闭。bag、大日志留在本地 logs，不加入 git。

分析时优先检查：首个目标 Z 与起飞 Z、bridge `last_reason`、odom frame/新鲜度、规划 ACCEPTED/READY/FINISHED 与任务 seq 对应关系、到达距离/速度/停留时间、实际发给 MAVROS 的设定点。超时后的定位变化不能直接当作此前悬停的原因。

验证：Bash 语法及参数校验通过；ROS 录制命令选项与本机 Noetic 一致；隔离的假 ROS 命令测试覆盖定时结束和 TERM 中断、bag 关闭及末尾快照。未启动真实记录器、ROS 节点或飞行任务，实际话题采集需用户运行后验证。
