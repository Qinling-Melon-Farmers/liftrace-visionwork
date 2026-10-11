# 起飞专用Z领先同步（2026-10-11）

分支：B/integrated。来源导航0928 b19010c381a720565a94d3f64b2e36c7733e69d9，采用明确文件范围同步，独立于门点补丁。

Takeoff专用takeoff_z_lead_m有效范围0.4～0.6m；0/缺省/非法回退本支旧3D限幅，配置生成器对非法输入提前拒绝。正式及08候选显式0.6m；通用模板0。专用分支与generic互斥，双向步长有界且不越终点，原XY保持、软件限高、恢复例外及ready/模式判据保留。领先距离不是速度，不保证缩短起飞时间。只在load_params输出参数日志。

本支正式入口及08两档实际launch展开均通过；01～07和09全部显式0，03未启用。

测试：19项PASS，0项因不存在入口跳过；384场景从本支生产C++抽取并经UBSan，其他模式与修改前代码对照。所有现有配置其他字段比较通过（五支13份YAML）。

本树catkin_make patrol_control实际编译并链接PASS（-j2 -l2，ROS系统Python）；日志build_B_integrated.log。

日志：/tmp/control_closeout_20261011/takeoff_tests_B_integrated.log，以及同目录build日志。运行修改git diff --check通过，冻结快照旧空白保持原样。修改前四支control与导航快照一致；高位B使用自身快照。未同步来源大文档，未纳入模型脚本/docs。

本轮无仿真、板端连接、部署或实飞；main/EV未修改。0回退已验证；动态起飞速度、ready达成时间、真实过门与跟踪误差需要后续明确授权实飞验证。
