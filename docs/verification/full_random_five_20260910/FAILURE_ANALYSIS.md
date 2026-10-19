# 失败定位与证据边界

全批结论与图表入口见[REPORT](REPORT.md)。以下只分析已有五轮，未修复飞行代码或启动额外轮次。

## seed31：LAND近地状态估计突变

- 368.951s LAND；375.371s AUTO.LAND；376.560s首次Wall_9接触。没有目标事务或规划失败，9个投后航点和两门已完成。
- 375.604s真实local z0.034、邻近估计0.036尚接近；375.806s真实0.021、估计−0.175开始明显分离。376.523s真实z0.455、邻近估计−0.419，XY也向东北分离。
- 终端图中落地前90°→0°实际机头转向发生于AUTO.LAND之前；PX4自动降落交接窗口的**航向目标**只变化0.067°，自动段保持不变。不能把图上的前置转向误认成旧飞控重置漏洞复现。
- ULog自动段未取得ground_contact/maybe_landed/landed确认。接近支撑高度不等于已经稳定接地；碰墙后的回升/姿态样本也不能作为正常降落状态。
- 当前能确认“近地估计/实际运动分离后回升和侧向冲墙”，尚未隔离IMU接地冲击、气压高度融合、LIO水平/航向输入及接地模型各自贡献。后续应围绕此时间窗口区分传感器与动力学原因，不能先用强制空中停机掩盖。

[末段四图](seed_31/terminal_diagnostics.png) · [时序](seed_31/timeline.json) · [ULog摘要及位置对照](seed_31/diagnostic_details.json)。

## seed33：感知图中的第二门后目标占据

- 225.203s第三次mock ACK；393.978s第7投后点完成；393.986s请求第8点(2.3,8.35,0.23)。
- 对应planner goal_seq24从394.162至484.201s有110次new_trajectory_attempt_failed，未得到可执行新轨迹；483.996s任务90秒期限触发safety_motion_timed_out，之后ABORT确认。
- 日志明确为“No collision-free goal within requested waypoint neighborhood”和“kinodynamic goal occupied or outside map”。394.357s实际接收的地图快照支持目标附近存在静态/膨胀占据；图中的inflated_map也包含高度边界等规划占据，不能全部解释为物理实体墙。
- 布设几何合法；实验使命没有按真实右门位置直接给穿门航点。尚需区分低空映射/离散膨胀、目标调整半径0.1m及虚拟高度边界对该目标的作用。局部截面无法证明整张感知图存在可行路径。
- 截止全试验600秒仍没有第二门/终点H/解除武装；观察后段的ABORT状态约139s，不是正常完成后的空闲，更不是成功回收。无实际碰撞。

[停滞区间](seed_33/planning_stall.png) · [目标高度附近地图与XZ截面](seed_33/local_map_failure_2.png) · [日志摘录](seed_33/planner_log_excerpt.txt) · [统计](map_diagnostics.json)。膨胀图原header stamp=0，使用接收时刻定位快照，不由此推算精确源年龄。

## seed34：重复不可达搜索请求耗尽任务预算

| 请求 | 时间范围ROS s | 结果 | 新轨迹失败次数 |
|---|---|---|---:|
| (4.3,0,1.18) SEARCH | 约42.6–132.6 | 90s deadline | 95 |
| 同点 RESUME | 约132.6–222.7 | 90s deadline | 93 |
| (4.3,0.7,1.18) SEARCH | 约222.7–312.7 | 90s deadline | 92 |
| 同点 RESUME | 约312.7–402.7 | 90s deadline | 96 |

四段合计约360s，占600s任务窗口的60%。首段日志从西侧起点搜索至东端目标，输出kinodynamic search fail/Can't find path；与seed33明确的终点占据报错不同，不能混为一个已证明根因。局部目标快照没有覆盖从西端到东端的整条通路，可能涉及随机树箱遮挡、感知可达区域和动力学搜索，尚未定位到单个阻挡体。

跳过后能继续飞行，485.578s投panzer、520.955s投red_cross；595.300s才开始bridge接近，622.627s证据截止恰逢任务总时限，随后RETURN_HOME/ABORT。不能说第三目标单独用了120s，也不能说视觉花了整整600s才识别靶标。本轮14次mock ACK总体统计中的两次属于此组。

后续应使覆盖游标推进与实际可达性、失败消耗及局部观测相结合；是否能更快跳过或先观测/调整目标需要独立验证，不能只缩短timeout后宣称问题已修好。提前返航开关与本次规划停滞原因分开处理，本批一直关闭。

[四次等待的航迹与时间图](seed_34/planning_stall.png) · [完整高度/阶段图](seed_34.png) · [首次失败目标地图](seed_34/local_map_failure_1.png) · [原始搜索失败摘录](seed_34/planner_log_excerpt.txt)。
