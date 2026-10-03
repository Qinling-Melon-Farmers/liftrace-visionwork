# 障碍柱离线示意

illustrate.py编译并调用生产C++辅助类，对合成树采样，输出原始点、物理膨胀和禁越柱三视图。不是现场点云，不启动ROS或仿真。

    source /home/xhj/miniconda3/etc/profile.d/conda.sh
    conda activate rl_drone
    python tools/obstacle_columns/illustrate.py --out logs/column_example --tree-height 1.8 --canopy-base .55 --radius .45 --band .4 .6 --inflation .25 .20 .10

依赖已有g++、NumPy、Matplotlib、SciPy。输入米。参数只作用于示意，不修改飞行配置。[实际配置与验证](../../docs/planning/obstacle_board_alignment_20260926/REPORT.md)。
