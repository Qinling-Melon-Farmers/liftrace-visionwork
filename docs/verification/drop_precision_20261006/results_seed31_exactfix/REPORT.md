# seed31 修正版首次投递及恢复阻塞诊断（2026-10-07）

**INTERRUPTED_DIAGNOSTIC / 原 Gate FAIL；不是完成 seed31，也不是完整对照验收。** 固定源码 `71dc1f540ae63b415d41e2ae69aa3150368c9476`。本轮只完成红十字槽1一次 mock 释放，未释放 bridge/panzer，未返航/完成降落。

原始运行 `logs/drop_precision_seed31_20261007_011730`；原批次 `logs/drop_precision_20261006_exactfix_batch/matrix.json` 保留 INFRA_STOP。第一次诊断的报告、matrix、原始日志均未覆盖。运行采用原冻结 10×10/四树/三线/镜头2m场景，两端 exact=true、NMS=false、zero slot、stop_on_collision=false，32项运行参数复核 PASS。

## 已实际释放及中心误差

RAW_CALL_STARTED 与成功 COMPLETED / raw_actuator_ack 均为 ROS **87.931** 秒，同 mission/decision11/attempt1/slot1/execution1；raw mock ACK 无可分辨开始到完成时长，不代表机械零延迟。

| 指标 | 数值 |
| --- | --- |
| 真实红十字中心，world/CSV XY | (7.4413, -0.9705) m |
| 开始/ACK源时间插值机体中心 | (7.445771, -0.954456) m |
| 源时间插值中心误差 | **1.666 cm** |
| 旧ACK接收时间最近样本口径 | **1.549 cm**，样本ROS87.977，年龄46 ms |
| 历史seed31红十字同最近样本口径 | 3.317 cm |
| 旋转后名义红十字板内 | 是；局部XY=(-0.016651, 0.000376) m，半边长0.175 m |

插值左右样本间隔101 ms，低于既定250 ms上限；未外推。以上是仿真机体/model-origin中心及mock事务时刻误差，**不是快递真实脱离或撞地落点**。单个样本看起来改善，但本轮只有1次释放，不能接受为完整31/38精度比较，也不能把省略后续任务的时间当提速。

## exact目标实际进入输出

闭合1.2MiB精度bag有 **97条** exact offset，全部通过 map_valid/frame/ground/source年龄检查，目标8。最后源时间87.138、接收87.179，map_point=(7.417401974,-1.019864291,-0.22)。

MAVROS setpoint在ROS87.190已更新为(7.417401791,-1.019864321,0.10)，释放前87.897仍是该XY；与最后exact目标XY差 **1.86e-7 m**，证实新绝对目标更新进入实际控制输出，没有沿用先前红十字静态目标。

ACK距最后图像源时间0.793秒，超出0.5秒图像新鲜度；87.8秒bag的release evidence为stale_observation，许可active/context active仍成立。释放依赖已审的下降入口物理容差与有界action承诺、当下位姿和新鲜许可，**不声称ACK时图像新鲜，也未刷新旧时间戳**。最后低高度图像自身容差1.406cm不等同下降入口锁存容差。81.090秒入口日志前最新记录offset的容差为5.848cm，仅作为采集邻近值；内部锁存值未直接记录，不能将邻近CSV样本当同一回调的严格判据。

## 首次释放后恢复阻塞与停止

87.931释放已提交1槽；88.1秒bag中alignment context inactive、许可active=false，但align mode仍drop_cross。到103秒机体仍低，任务首个APPROACH未恢复完成；已有日志反复CrossDetectionDone且没有后续恢复事件。

主代理结合生产代码定位：context inactive走clearUavVision→clearExactDropCommitment将count_aligning清零，已executing/completed的动作恢复嵌套于count_aligning>=1，因此pollDropAction完成后不能进入原恢复分支。该原因由主代理/B实施最小修复，本评测代理未改生产或飞行状态。

主代理明确授权停止后，在 **ROS118.003** 保留最后progress/key events/局部pose/setpoint/runlog样本，并仅向当前batch所属`bash sim_run.sh`包装器PID10798发送TERM。不是因碰撞触发停止，不等动作deadline或600秒；未启动更多仿真。

原记录碰撞 **3个episode**，事实全部保留，stop_on_collision一直false；详见原gazebo_contact_status.json。不能宣称零碰撞或自主降落完成。

## 收尾失败与后续恢复（两者均保留）

包装器首次收尾 **FAIL**：rosmaster10953、rosout10963残留。runner退出并保留matrix **INFRA_STOP**，原cleanup失败没有改为PASS。

随后执行规定`bash top_level_scripts/stop_toudi3_sim.sh`，返回Local toudi3 SITL fully stopped；独立`bash top_level_scripts/check_sim_processes.sh`确认 **零残留**。主代理也独立确认两PID消失。补充`cleanup_recovery.json`和原run的`cleanup_recovery.log`；仅在原run.log末追加明确标注Supplementary cleanup recovery的记录，未覆盖原FAIL。

## 交付与下一步

本目录新增stop_request.json、precision.json、exact_path_summary.json、exact_offsets.jsonl、diagnostic_summary.json、release_recovery_contexts.json、cleanup_recovery.json；原CSV、bag、三路视频全部保留，无重分析视频。这里和原日志以外无写入，没有修改tracked文件或提交。

后续使用全新 `--batch-name drop_precision_20261006_recoveryfix_batch` 保留当前INFRA_STOP，待主代理新编译/审查/pinned hash及明确case0授权；仍仅跑seed31，case38须独立审查授权。当前未创建新batch或启动。
