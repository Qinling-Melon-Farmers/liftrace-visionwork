# 板端同链仿真结果

| 专项 | 状态 | 投递 | 任务ROS秒 | 结束原因 |
| --- | --- | --- | --- | --- |
| visual_interrupt | PASS | 1/1 | 29.479999999999997 | landed_after_flight |
| high_view | INCOMPLETE | 3/3 | 119.389 | mission_failed |
| landing | INCOMPLETE | 0/0 | 30.237000000000002 | landed_after_flight |
| low_multi | PASS | 2/2 | 43.482 | landed_after_flight |
| high_priority | PASS | 3/3 | 101.3 | landed_after_flight |
| memory_only | PASS | 0/0 | 50.81100000000001 | landed_after_flight |

任务计时从手动启动服务被接受开始，不包含前置自动起飞。视频和图表覆盖准备、起飞及结束；估计高度不等同于 Gazebo 真值测量。

三路录像逐组检查可解码，完整入口：[index.html](index.html)。仿真使用笔记本 PyTorch 和模拟执行器，不能替代板端 RKNN/机构实投验收。
