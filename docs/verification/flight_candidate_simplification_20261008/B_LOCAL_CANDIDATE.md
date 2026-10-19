# 2026-10-08 板端测试分支本地候选

整机权威来源：视觉仓 `feat/r2026-competition-integrated` / `87fb2726cd3a2078b5ee32a2d448bdaae7f9ee70`；[具体取舍与验证](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/87fb2726cd3a2078b5ee32a2d448bdaae7f9ee70/docs/verification/flight_candidate_simplification_20261008/REPORT.md)。恢复来自研究62afbef8，并回流速度frame修复4a3f5244；导航权威feature已选择性接入95020d15/66506d15。

本地已接入去高位重复精密等待、正确旋转槽位补偿、最终4cm/0.05m/s/0.30秒释放保护和短程点云净空检查；H保留原10帧确认，低位0.35/0.37m、0.08/0.10m/s、0.15秒短确认允许缓降，未来状态有限暂缓。运动/续扫默认开，恢复仅独立候选开启；不复制实验投影/EV预测，不撤销ABORT。

本分支保留现场08 drop=.35/capture=1.2/budget60与03/04既有motion值，历史field_20261007_validated.yaml原样保留；本轮新默认不冒充昨日第八组已飞版本。补齐独立competition入口缺失依赖，不覆盖原执行机构资产；旧控制快照源4ae68a3c，保存在legacy_baseline/20261008/simplified_drop_h_recovery。

本分支视觉与导航完整Catkin及成功日志增量build PASS、控制142/142零错误；H10/交接25/异步24/限高HOLD5/桥接16/参数9 PASS，硬件生成15PASS/1skip（本地缺旧PWM实体包），不把skip写成板端硬件通过。导航两feature仅源码及定向验证，未独立整包构建。

工具已提交4ae68a3c，随后f6404e75仅修2个旧JS测试夹具。新Windows包deliverables/liftrace_flight_workbench_windows_20261008_motion_resume_4ae68a3c.zip为已验证不可变源码包。解压新目录运行start_windows.bat，访问新窗口打印URL；旧端口占用自动找新端口，旧服务未被停止。正赛卡分别选择运动优化与续扫，先配置检查，再在实际生成后确认本次生成值；修改路径/开关使旧确认失效。历史validated YAML续扫仍false，使用它需显式选择开启。

本轮只本地与feature推送，未连接/部署板端，不启动ROS/SITL/真实执行机构。原用户未跟踪资产保留，main未动。下一轮先定向恢复，再同场测冻结到下降/到低位后到释放或POSCTL请求及最后未通过条件。
