# 模型资产

源码仓库不在本目录重复存放大模型权重。面向控制组的交付 ZIP 会在此加入已经验证的候选：

- `merged_standard_best.pt`：笔记本开发/仿真使用的 PyTorch 检测模型；
- `merged_standard_fp32.rknn`：OrangePi RK3588 使用的 FP32 RKNN 检测模型；
- `merged_standard_6cls_metadata.yaml`：固定六分类顺序和输入契约。

运行 launch 通过参数接收模型路径。禁止在 YAML 或 Python 源码中恢复与某台机器绑定的
绝对路径。
