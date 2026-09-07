# 当前验收结果

R60先导seed11：**完整PASS37/37**，269.748 ROS s，三投/三恢复、9航点、入口和两处0.80m通口、H对准、ON_GROUND/disarm，零碰撞/越界/超高。

随后同一飞行源码十seed：**2/10完整PASS，8/10三投完成，6/10走完投后路线**，实际10种不同布设；无补跑替换、无中途调参。六次实际碰撞、两次任务事务失败。所有组收尾通过、0bag；不合main。

[完整报告](verification/r60_full_matrix/REPORT.md) · [失败分组与后续方向](verification/r60_full_matrix/FAILURE_ANALYSIS.md) · [原始批次状态](verification/r60_full_matrix/matrix_status.json) · [三版rqt图](verification/r60_full_matrix/topology/index.html)。本轮整机Catkin构建与239项任务/执行/Gate回归通过，但不能替代动态鲁棒性验证。

| 历史记录 | 保留结论 |
|---|---|
| R56 cc899f2 | 历史场景完整37/37 PASS，main仍为该验收及模型说明 |
| R57 7035415 | toudi4样式0.85m完整PASS；严格0.80m后续失败 |
| R58 59ed04b | 三投后走廊外墙接触FAIL |
| R59 e633a9b | 8航点/两通口/H对准及触垫，原始FAIL，最终落地解除武装未确认 |
| R60 9cfb3e5 | 先导完整PASS，十seed仅2/10完整通过 |

仿真三投是视觉、任务与mock执行确认，不是机械带载实投。笔记本SITL、板端离线RKNN、板端实时链、实机飞行是不同验收范围。
