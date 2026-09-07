# 复现入口

以下命令用于feat/r2026-competition-integrated整机分支；来源导航/视觉开发checkout请先切换到该整机分支再复现。

只有获得新的明确启动授权才运行仿真。本次先导/十seed已全部结束，不自动续跑。

单轮使用README的run_competition_sim.sh，`field_seed:=N`设置靶标布设。完整先导必须先PASS，十seed脚本才接受该run；飞行期间冻结HEAD，各轮复核无残留。

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 UAV_VISION_MODEL_PATH=/absolute/path/best.pt python top_level_scripts/run_seed_matrix.py --project-root "$PWD" --output-dir "$PWD/logs/_artifacts/new_matrix" --pilot-run /absolute/path/to/full_PASS_run --scene-prefix r2026_new_matrix --seeds 1 2 3 4 5 6 7 8 9 10
```

这条命令是未来复现方法，不构成新的运行授权。分析所需NumPy/PyYAML/Matplotlib/SciPy/pyulog位于已有conda环境，ROS图工具使用系统ROS Python。

结束后无需启动ROS即可从已记录快照调用rqt_graph后端绘图：

```bash
source /opt/ros/noetic/setup.bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 top_level_scripts/render_recorded_rqt_graph.py /path/to/run/ros_system_state.json /path/to/output --project-root "$PWD"
```

完整、核心、仅飞行三版输出为SVG/DOT和HTML切换页。节点/话题来自输入快照，不伪造未运行的硬件驱动。
