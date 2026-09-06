# 仿真日志存储与相机记录

R55 的 VHDX 位于 F:\WSL\Ubuntu20.04\ext4.vhdx。WSL 内部曾显示 818 GB 可用，但宿主 F 盘只剩约 17 MB，随后读取出现 I/O 错误；因此包装器不能以 ext4 的虚拟空闲量作为宿主余量。

整机包装器现在接受 SIM_LOGS_DIR 指定 run 目录的根位置；WSL 调用者必须用 SIM_STORAGE_GUARD_PATH 指定 VHDX 宿主盘挂载点。默认 SIM_MIN_FREE_GIB=20，分别检查记录目录和宿主盘。该数值是启动检查，不代表无限录制保证。

本机示例（只有获得当前仿真授权后才执行）：

```bash
SIM_STORAGE_GUARD_PATH=/mnt/f SIM_LOGS_DIR=/mnt/e/LiftraceRuns \
SIM_RUN_AUTHORIZED=1 SIM_NO_RECORD=1 UAV_VISION_MODEL_PATH=/absolute/path/best.pt \
top_level_scripts/run_competition_sim.sh
```

R55 的实时 ROS 图证明原始相机为 /downward_camera/image_raw，原记录器订阅的 compressed 子话题没有发布者。记录器改用 record_camera_topic 参数，默认真实原始图像话题，继续使用 rosbag LZ4 压缩；没有增加图像桥接节点。另外补录原始检测和 planner goal，以便复核 H 检测与规划延迟。

本次修正只做离线 XML、shell 和容量分支检查，未额外飞行。R55 原始画面缺失如实保留为数据缺口。
