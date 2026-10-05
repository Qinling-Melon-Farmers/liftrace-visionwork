# 首次启动诊断：异步舵机命令身份误用
源16a344f2，seed38续扫关闭。原run logs/seed38_resume_resume_off_seed38_20261005_155057。52.146 ROS秒主动中断高位并下降；75.546秒红十字复核成功进入投递，首槽未调用raw，也没有成功投递。

99.023秒保护包络接触红十字靶面，原Gate actual_collision FAIL。采样峰值穿透约0.0000167m，力约0.342N，观测持续0.242秒；这是投递中持续下降后的接触，不是整场落地后的收尾。不能按已完赛处理，亦不能凭轻接触宣称重大实体撞毁。

源码复现明确：Bridge的MissionCommand顶层header.seq发布前虽然赋值决策号，但rospy.serialize_message会自动改为话题传输序号；控制器将它与AlignmentTargetContext.decision_seq比较，使有效上下文被拒绝。原始bag没有录MissionCommand/AlignmentTargetContext，不能声称本包直接读到了两者数值差；因果定位结合未提交服务日志、生产序列化复现及修复后对照验证。

采用消息内已有goal.header.seq作为稳定决策号，不删除匹配、不改slot/许可门槛。正式两轮增加命令/上下文/许可记录，二者仍同头唯一续扫开关差异。该轮单列失败诊断，不覆盖也不计为有效续扫收益对照。仿真已正常收尾零残留。

[多画面视频](../../../../logs/seed38_resume_resume_off_seed38_20261005_155057/presentation_review.mp4)
