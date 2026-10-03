# 新五分类模型六组专项验收

2026-09-28。4组PASS、2组INCOMPLETE；全部保留录像。

[完整模型/日志诊断报告](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/high-view-search-research/docs/verification/panzer_model_20260928/REPORT.md)；[本地多画面与图表](index.html)；[逐组数值](RESULTS.md)。

两个INCOMPLETE的首个未满足环节：

1. **high_view：** 先完成一圈、三类记忆、逐点复核并三投。117.560s发LAND，118.600s本地降落交接；131.030s `probe pose discontinuity` 触发 `board_pose_exception:ValueError`、ABORT。没有记录障碍接触。最终真值Z约0.223m而估计Z约0.785m，后段位置也偏离交接点；应分开查落地估计变化、飞控落地状态与高位runtime尾段的位姿检查。不能因三投就把整轮改为PASS，也不能直接归咎于检测器。
2. **landing：** H几何检测有输出，完成前往H、扫描爬升与下降，落地真值XY约(1.970,-0.177)，距H中心约0.18m；结束时任务仍是LAND而不是COMPLETE。当前Gate在判落地后3秒停止，尚未收到任务终态；保留INCOMPLETE，不改验收门槛掩盖交接问题。需后续核查终态反馈与观察窗口，不能直接写成“H识别失败”。

## 2026-09-28 后续：取消panzer特判

八组当前生成配置统一`interrupt_refined_classes: []`。06/08可凭两帧一致的panzer粗线索参与TOP3提前结束；02/07仍完成各自完整环线。低空复核与释放许可不变。本次仅离线验证，之前六组结果不代表取消后的动态验收。完整[策略、逐类退化和增强明细](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/high-view-search-research/docs/planning/panzer_five_class_20260928/VALIDATION_AND_AUGMENTATION.md)。
