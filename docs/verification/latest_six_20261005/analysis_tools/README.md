# 六轮分析复用入口

这些脚本只读取已闭合的run/bag，不启动ROS节点或仿真。SITL固定源码头及单轮授权仍由上一级run_case.py控制。

顺序：analyze.py --skip-video → centers.py → compare.py → physical.py → release_truth.py → report.py → analyze.py（视频）→ report.py → finalize.py（本批结论）。analyze.py复用已完成指标；修改分析算法后使用--refresh强制重算。分析/绘图使用现有rl_drone环境；中心导出子进程使用ROS系统Python，先source Noetic与本工作树vision_ws/devel/setup.bash。centers.py的pipeline-root仍指向本机板端试飞工作树的tools/bag_replay，异机复用需改成对应的已部署toolkit目录；matrix/scenes中的run路径也需映射到实际产物位置。

physical.py保留原始Gate，以H支撑接触和真值静止附加判定；release_truth.py将模拟释放时机体中心与最近靶标比较，不等同包裹弹道和实物得分。报告脚本不会修改gate_status.json。
