# 五组收尾与交付

- 运行范围：31–35共五组，冻结HEAD为7eb9446，10:11:59开始，12:22:38最后一组结束（UTC+8）。没有额外先导或补跑。
- 原始结果：31 FAIL，32 PASS，33 FAIL，34 FAIL，35 PASS；批次COMPLETE表示五组结束，**不表示五组都PASS**。
- 五轮run.log均有统一包装器收尾PASS；最后check_sim_processes.sh复查无ROS/Gazebo/PX4/RViz残留。
- 0机载视频、0俯视视频、0桌面录屏、0全场bag；相机在线图像仍供视觉使用。CSV、事件、参数、Gate、接触和ULog留在原run目录。
- 实际靶标布局五种不同，五组目标几何检查通过；门组合只有LL/LR/RR，RL未抽中。
- 五组使命runtime一致，关键EKF实读一致且成功，详见[实读检查](parameter_readback_validation.json)。无途中改参/算法修复。
- 报告共20张PNG，另有HTML浏览页；完整局部点云和大日志留原run，只提交地图摘要及渲染图。原始路径与体积见[归档索引](archive_manifest.json)。
- 本次保留实验feature分支；失败场景与实验中线配置没有替换main的原验收基线。未更新机载压缩包或执行实机动作。

本批运行授权已经用完；查看或重画报告不构成新的仿真启动授权。
