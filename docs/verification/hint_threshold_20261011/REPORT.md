# 2026-10-11：视觉高位粗HINT门槛定向同步

仅将既有 trial_config.py 两端；本树无独立正赛 YAML，不补造文件的 coarse_min_confidence 从0.60改为0.75。各自高度、航路、控制参数及其他值保持原样。

来源仓库：https://github.com/Qinling-Melon-Farmers/liftrace-visionwork.git；来源分支：feat/board-deployment-flight-20260920；来源提交：d4974358dbe06322fcc2d0cc02cee603303e862d。来源工作树：/home/xhj/liftrace-worktrees/r2026-board-vision-tests；选择依据见该提交的 docs/verification/hint_threshold_20261011/REPORT.md。回归根据来源 test_hint_threshold_config.py 按本树现有入口收缩，未复制模型统计或 recount.py。

修改的既有配置：

- deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_config.py

## 本树验证

定向 CPU 配置回归 **2/2 PASS**，无跳过；9组专项生成投影发布端与任务接收端0.75。低空 target_memory 四项门槛保持本树原值。逐字段替换检查确认上述生产文件除指定门槛无其他文本变化；git diff --check 通过。

~~~bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
PYTHONDONTWRITEBYTECODE=1 PYTHONPYCACHEPREFIX=/tmp/hint_config_clean_cache_20261011 python -m unittest discover -s docs/verification/hint_threshold_20261011 -p test_hint_config.py -v
git diff --check
~~~

本轮仅配置生成验证，没有整包构建、ROS/GPU、仿真、板端部署或实飞。需重新生成实际运行参数才生效；提高粗HINT门槛可能丢失弱线索，且精修旁路仍可提供错类，不保证消除地堡误识或提高任务成功率。
