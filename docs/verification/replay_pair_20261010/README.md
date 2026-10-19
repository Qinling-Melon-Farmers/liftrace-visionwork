# 10月8日两轮实飞bag回放

2026-10-10使用现有 `tools/bag_replay/run.sh` 离线处理两轮原bag，不运行检测模型、不启动ROS节点或仿真、不修改原始bag。

| 轮次 | 完整四视图播放器 | 对准与补偿重点播放器 | 记录时长 |
| --- | --- | --- | --- |
| 22:36 | [完整回放](../../../试飞产物/board_full_mission_20261008_223632/analysis/index.html) | [重点时序](../../../试飞产物/board_full_mission_20261008_223632/analysis/focus_player.html) | 154.47秒 |
| 23:13 | [完整回放](../../../试飞产物/board_full_mission_20261008_231313/analysis/index.html) | [重点时序](../../../试飞产物/board_full_mission_20261008_231313/analysis/focus_player.html) | 154.79秒 |

各轮analysis目录包括原片、视觉叠加、俯视轨迹、多画面dashboard四个MP4，以及REPORT.md、events.csv、motion.csv、目标坐标、帧时间映射和focus时序。图像实际约5Hz，视频以10fps展示，重复帧不代表新增视觉观测。俯视图采用camera_init，使用navigation位姿/里程计/任务设定点，避免将MAVROS坐标与任务坐标直接混合。

重点播放器支持按任务/对准/冻结/释放记录跳转。冻结点来自同轮application.log，bag话题按接收时间显示，二者没有额外进行时钟修正。22:36轮记录了1号槽raw调用及成功ACK；23:13轮记录了取消、未执行结果。软件ACK不能证明当时无供电的舵机实际动作。

两个bag都没有 `/uav_vision/drop_alignment_feedback`，因此不伪造实时FC/投口补偿误差或完整控制反馈；也没有载荷落点真值。粉色设定点、最后有效目标和历史许可均需按各自图例理解。回放还原旧版飞行，不是10月10日新部署补丁的动态验收。

原始产物不进入Git，需在本机上述试飞产物目录打开播放器。

验证：八个视频全片FFmpeg解码通过，ffprobe时长与各轮记录一致；两个播放器相对链接和重点跳转检查通过，结果在各轮 `artifact_validation.json`。原bag大小、mtime、ctime未变。主代理抽看23:13轮dashboard预览并核对时序摘要。复现命令保存在各轮 `REPLAY_COMMAND.txt`；每轮生成和解码约171/181秒。
