# 补验分析复用方式

固定测试配置与运行源提交见上级PLAN.md、BATCH.json；先看STARTUP_DIAGNOSTIC.md区分无效启动。所有脚本只读取已有run，不启动仿真或硬件。

在本工作树根目录，使用已有rl_drone环境依次运行本目录的analyze.py、centers.py、physical.py、release_truth.py、geometry.py、behavior.py、report.py。analyze.py复用既有合成器，生成配对时间戳的下视/跟随/俯视汇报视频；centers.py复用原中心分析器。ROS消息导出使用已有Noetic环境和overlay，不安装系统Python依赖。

视频、bag和逐帧CSV留在logs，不入Git；报告中的本地相对视频链接要求对应run目录仍存在。只传报告Git仓不足以携带视频。原始Gate与用户约定的物理落地验收分别保存，不能互相覆盖。
