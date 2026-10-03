# 05_low_multi

低空连续多投：默认完成两次独立模拟投递，每投后恢复搜索，最后原地降落。delivery_count 可设为 1–3。请依次布设不同类别靶，避免同一靶重复计数。

共同参数、坐标系、阈值和启动步骤见 [八组说明](../MODULES.md)。

先运行 `bash start.sh preview --model /实际路径/model.rknn`；检查通过后使用 `bash start.sh flight --model /实际路径/model.rknn`，默认模拟投递。任务仍需现场手动启动。已知外参来自共用 known_rig.yaml，地面静置自动建立高度参考。

每轮保留下视原始/视觉叠加视频、候选/坐标/许可事件和轨迹，结束状态按实际提交槽位数与落地状态判断。
