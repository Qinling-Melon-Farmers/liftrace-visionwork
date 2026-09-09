# 当前验收结果

R64最新：seed11完整37/37 PASS，未实机验收；必须同步PX4补丁。R64新包替代旧R62运行源码，十seed原始7/10完整PASS，5/7/8有墙重叠布设，修复后的新布设未重跑矩阵。见[本轮报告](verification/r64_seed11/REPORT.md)和[固件依赖](../deployment/px4_patches/README.md)。下文较早状态按历史记录阅读。

2026-09-09最新：R63完整seed11：三投、三恢复、9个投后航点和两门通过；AUTO.LAND后碰墙，完整Gate FAIL。恢复修复已验证，最后降落仍阻塞。[报告](verification/r63_recovery/REPORT.md)。本轮只跑一次，不追加矩阵，旧包未重新打包。

以下为此前记录，按各自版本阅读。

R62已完成seed11建图/自动起飞/搜索/首投/恢复搜索运行门槛；完整Gate未执行到终点，不称整场PASS。[报告](verification/r62_operational/REPORT.md)。244项任务/Gate回归、1项生产起飞限幅回归、4项SDF/模型契约回归通过，实际构建通过。

以下为冻结的R60飞行记录，不能作为新机架/新场景验收。

R60先导seed11：**完整PASS37/37**，269.748 ROS s，三投/三恢复、9航点、入口和两处0.80m通口、H对准、ON_GROUND/disarm，零碰撞/越界/超高。

随后同一飞行源码十seed：**2/10完整PASS，8/10三投完成，6/10走完投后路线**，实际10种不同布设；无补跑替换、无中途调参。六次实际碰撞、两次任务事务失败。所有组收尾通过、0bag；不合main。

[完整报告](verification/r60_full_matrix/REPORT.md) · [失败分组与后续方向](verification/r60_full_matrix/FAILURE_ANALYSIS.md) · [原始批次状态](verification/r60_full_matrix/matrix_status.json) · [三版rqt图](verification/r60_full_matrix/topology/index.html)。R60当时整机Catkin构建与239项任务/执行/Gate回归通过，但不能替代动态鲁棒性验证。

| 历史记录 | 保留结论 |
|---|---|
| R56 cc899f2 | 历史场景完整37/37 PASS，main仍为该验收及模型说明 |
| R57 7035415 | toudi4样式0.85m完整PASS；严格0.80m后续失败 |
| R58 59ed04b | 三投后走廊外墙接触FAIL |
| R59 e633a9b | 8航点/两通口/H对准及触垫，原始FAIL，最终落地解除武装未确认 |
| R60 9cfb3e5 | 先导完整PASS，十seed仅2/10完整通过 |

仿真三投是视觉、任务与mock执行确认，不是机械带载实投。笔记本SITL、板端离线RKNN、板端实时链、实机飞行是不同验收范围。

完整集中分析与合法性标记见[11轮报告](verification/r64_matrix/REPORT.md)，新随机化工具范围见[说明](verification/r64_randomization/README.md)。旧包及旧验收按生成时的源码阅读，最新交付索引在deliverables/README.md。
