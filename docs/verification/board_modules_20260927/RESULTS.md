# 八组板端同链仿真结果

| 专项 | 状态 | 投递 | 任务ROS秒 | 结束原因 |
| --- | --- | --- | --- | --- |
| visual_interrupt | PASS | 1/1 | 27.677 | landed_after_flight |
| high_view | PASS | 3/3 | 111.61500000000001 | landed_after_flight |
| landing | PASS | 0/0 | 34.22 | landed_after_flight |
| corridor_landing | INCOMPLETE | 0/0 | 28.308 | obstacle_contact |
| low_multi | PASS | 2/2 | 45.056 | landed_after_flight |
| high_priority | PASS | 3/3 | 103.43799999999999 | landed_after_flight |
| memory_only | PASS | 0/0 | 50.656 | landed_after_flight |
| full_mission | INCOMPLETE | 3/3 | 95.64699999999999 | obstacle_contact |

任务计时从手动启动服务被接受开始，不包含前置自动起飞。视频和图表覆盖准备、起飞及结束；估计高度不等同于 Gazebo 真值测量。

三路录像逐组检查可解码，完整入口：[index.html](index.html)。仿真使用笔记本 PyTorch 和模拟执行器，不能替代板端 RKNN/机构实投验收。
