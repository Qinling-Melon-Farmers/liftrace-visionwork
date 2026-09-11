# 启动诊断与修正

`logs/local_entry_seed32_baseline_20260912_015423`保留为STARTUP_DIAGNOSTIC_STOP：仿真时钟未推进、没有起飞，完整Gate未生成，不计入基线/候选成绩。

1. 发现20260911诊断轮遗留FAST-LIO进程（旧ROS日志ID a93a2e84…）。原脚本只检查ROS/Gazebo/PX4主进程；强制停止新栈后，FAST-LIO和局部建议节点也可能成为孤儿。已按实际Linux comm名称补充起跑检查及TERM→KILL/零残留复查，实际清理验证通过。
2. Gazebo日志停在模型数据库请求。机架SDF引用的`r2026_iris_dynamics`和`r2026_mid360_mounted`仅在`vision_ws/src/uav_vision_eval/models`存在；直接研究入口没有继承run_competition_sim的资源路径。现由公共sim_run补齐该目录和打包模型根，保留原PX4模型目录优先级。所有场地URI和机架嵌套模型的本地解析已检查，未改SDF本身。

Gazebo模型数据库在取模型列表时包含同步等待，缺少本地资源路径可能把启动带入网络等待；代码参见[Gazebo 11 ModelDatabase](https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11/gazebo/common/ModelDatabase.cc)。此次未修改网络配置或安装依赖。

仅启动/收尾脚本修正，局部执行源码仍为4b47c34的实现。后续正式基线与候选均使用修正后的同一提交。原始诊断见[startup_diagnostic.json](startup_diagnostic.json)。

后续补充：在第二轮正式运行期间发现启动诊断还遗留一个只读`competition_key`记录器（PID7931、旧日志ID d37adb58…）。其诊断输出一直停在零时钟、文件未继续增长，累计CPU约0.1%；核对身份后仅清理该进程，当前仿真未被停止。四轮飞行仍冻结31374fd；全部完成后再把该实际15字符comm名补入两份预检/停止列表。各正式轮均复查全部带ROS日志参数的进程，最终零残留。此记录器不能据此认定为航线或降落失败原因。
