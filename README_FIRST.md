# 2026 RoboCup 无人机工程：接收后先读

这是一份完整源码与模型交接包，包含导航控制、视觉感知、舵机实体驱动、MID360/LIO接线、任务入口、工具和工作台。不是可以跨硬件直接运行的预编译镜像。

1. 先读 **docs/closeout_20261019/NEXT_TEAM_README.md**，其中有安装、构建、模型、入口、场地测量和排错步骤。
2. 结项范围与历史状态在 **docs/closeout_20261019/README.md**，本次构建/测试及未通过项在 **VALIDATION.md**；不要把归档结束理解成所有研究补丁实飞通过。
3. 工程根目录runtime_models带最终v8的PT、ONNX、FP16 RKNN与metadata；源码环境脚本和工作台默认引用同一对。Linux使用tar解压以保留可执行位/软链接；不要把x86生成的build/devel复制到ARM板。
4. Windows只用工作台时，解压distributions内工作台ZIP，按其README_FIRST.md安装Python依赖，双击start_windows.cmd。工作台本身不需要WSL；板端仍需本工程及ROS/RKNN环境。
5. 完整训练集另交 **flight_5cls_v8_20261011_models_dataset_20261019_portable.zip**，2583图像与标签，不靠本机旧软链接。历史资料保留原作者与分支来源；工程历史包不包含个人材料。
6. 对应源码commit见ARCHIVE_MANIFEST.json。默认正赛模板尚未确认现场，必须实测起飞原点、朝向、场地/门/H、相机和投口外参。03/04/08/10/11配置有用途差异，不要只看文件名猜测默认模式。
7. 原始项目bag已按本人要求清理，历史报告中的bag路径可能失效。分析报告和已生成回放仍在本地外置资料目录；完整源码包不带大视频及原始bag。
8. 各工具索引在 **docs/closeout_20261019/TOOLS.md**。

本轮未重新启动实机、机构或SITL。恢复飞行前由现场人员按指南确认设备标定、依赖版本、配置和执行机构；配置检查成功不等于实机验收。
