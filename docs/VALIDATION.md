# R59单轮最终结果

走廊8/8航点、两通口完成，最高实际高度.6701m；低空H对准、AUTO.LAND和触垫已观察。原始Gate FAIL22/31，终点平面H克隆未列为地面导致提前收尾；最终ON_GROUND/disarm未确认。分类已修正、45项回归通过，未重跑。详见[原始结果与分析](verification/r59_corridor_landing/REPORT.md)。下文为此前验收历史。

# 当前验收状态：R58停止，十seed暂停

最后实跑59ed04b完成三投与三恢复，但179.495 ROS s护圈接触外侧长墙，FAIL；收尾零残留。R58共有6次启动尝试，5次进入飞行，1次在起飞前因地面标记接触类别结束，十seed执行0组。没有新实机飞行验收。

R56完整PASS与R57/0.85m的37/37 PASS继续保留；R57严格0.80m及R58没有完整PASS。详见[收口与改动对照](verification/r58_closeout/REPORT.md)、[最后Gate](verification/r58_closeout/last_run/gate_status.json)。新增低空0.65m方案尚未加载或飞行验证。

## R56历史验收

# 联合全量验证

**历史 R56 完整 PASS：37/37，三投/三恢复 3/3、投后 11/11、三门 3/3、H 对齐、AUTO.LAND、ON_GROUND 与 disarm；碰撞/越界/超高 0，收尾零残留。** [完整报告](verification/r56_final/REPORT.md) · [原始 Gate](verification/r56_final/gate_status.json) · [Nodes only 图](verification/r56_final/topology/index.html)。

| 记录 | 真实结论 |
|---|---|
| R41 | 历史完整功能 PASS，429.875 ROS s；包含旧 Gate 墙钟字段离线误判修正，不是当前场景验收 |
| R42/R43 | 三投与恢复有通过记录，走廊未完整成功；不得冒充 8/8 或 9/9 走廊统计 |
| R44—R50 | 逐步处理前视/端点、护圈余量、雷达自遮挡、LIO/飞控高度与 EKF 初始化，过程性失败/中止保留 |
| R51/R52 | 定位与短连接/稀疏拟合修复；三投或首投成功后走廊/搜索仍失败 |
| R53/R54 | 三投成功但门前超时；R54 原地图冻结分析定位量化、无进展 horizon 和搜索阻塞 |
| R55 | 三投、11 航段、三门零碰撞；H 段 I/O 中断，保持 INCOMPLETE |
| R56 第一次 | E 盘 run 启动超时，未进入任务，FAIL |
| R56 原生目录重跑 | 三投成功、首门前第 4 段超时，FAIL；记录无缓冲溢出 |
| R56 根因修复后 | 完整 PASS 37/37，182.924 ROS s；实际运行源 cc899f2 |

本轮相机 bag 4046 帧，导出逐帧回放；记录缓冲溢出 0；PX4 8 项关键参数读回成功。新口径注册接口检查 11/11，来源校验由包装器完成；不是放宽运行 Gate。

失败记录只保留报告与必要小产物，不新增全量 Release。完整成功记录在 WSL 项目 logs 内生成和归档。仿真、板端离线推理、板端实时链和实机是不同验收范围。
