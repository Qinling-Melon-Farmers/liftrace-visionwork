# 08_full_mission

整任务：高位搜索、低位重访三投、实测走廊航点自主避障、H 视觉降落。corridor_waypoints 和 landing_xy 默认留空，未填写会拒绝启动。不能把仿真航点直接用于现场。

共同参数、坐标系、阈值和启动步骤见 [八组说明](../MODULES.md)。

先运行 `bash start.sh preview --model /实际路径/model.rknn`；检查通过后使用 `bash start.sh flight --model /实际路径/model.rknn`，默认模拟投递。任务仍需现场手动启动。已知外参来自共用 known_rig.yaml，地面静置自动建立高度参考。

每轮保留下视原始/视觉叠加视频、候选/坐标/许可事件和轨迹，结束状态按实际提交槽位数与落地状态判断。
