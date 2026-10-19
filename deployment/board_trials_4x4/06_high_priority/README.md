# 06_high_priority

高位提前中断：三类高权重目标满足支持条件后退出高位环线，下降并逐个重访；panzer 仍需精修支持。已冻结目标全部投完后在最后目标附近降落。

共同参数、坐标系、阈值和启动步骤见 [八组说明](../MODULES.md)。

先运行 `bash start.sh preview --model /实际路径/model.rknn`；检查通过后使用 `bash start.sh flight --model /实际路径/model.rknn`，默认模拟投递。任务仍需现场手动启动。已知外参来自共用 known_rig.yaml，地面静置自动建立高度参考。

每轮保留下视原始/视觉叠加视频、候选/坐标/许可事件和轨迹，结束状态按实际提交槽位数与落地状态判断。
