# 先高位补偿再下降：待实飞候选

本独立提交接在6/4cm阈值提交 b553b74c 后；不修改视觉或其他控制状态机。控制 cpp/header、补偿与异步测试夹具、两份control YAML。参数 drop_system/compensate_before_descent，控制默认true且正式/试飞base均显式true；false保留原同时补偿下降路径。0.40目标/0.35–0.45窗口/分类6与4cm门限保持。

冻结仍由原视觉capture确认，固定靶心和补偿FC XY；冻结后不需要新视觉持续ready。圆环与红十字原下降分支调用 compensatedDropCommandHeight：先发固定FC XY与capture高度；同一新鲜运动反馈的FC和投口两项XY均严格低于绑定类别门限、上下文/控制有效，立即单向进入下降。下降入口复用 compensatedDropSettled(... require_release_height=false)，只略过释放高度约束，不增加速度上限或时长稳窗。该入口不打印release_ready，单独记录一次descent_started；冻结日志stage可区分compensate_xy/enter_descend。

下降开始后XY反弹/反馈陈旧不把高度目标重新改回capture；最终释放仍检查当前双XY、高度、新鲜性与有效许可。默认串行模式下 executeDropAction 额外拒绝尚未开始下降的服务准入（返回Pending）；即使初始高度位于释放窗口也不能跳过该阶段。取消/身份不符由原externalMissionTick guard保持当前位置；仅阶段标记保持单向，不代表释放权限。原resetDropState与resetDetectionState清除阶段标记，新动作必须重新冻结/补偿。

验证：生产方法提取g++ C++14+UBSan，49项补偿回归全部PASS（较第一提交新增7项串行序列用例，覆盖两类别的完整下降/反弹/许可/服务序列、上下文/期限/取消/陈旧反馈、实际投口、新动作双reset、默认与关闭开关、初始在高度窗也不能提前调用）；异步Servo24项PASS。旧放行专测显式关闭串行开关来隔离原阈值/许可/执行规则，7项串行用例使用真实默认开关。见control_tests.txt。

第一提交主代理已实测B catkin_make PASS（/tmp/build_drop_threshold_B.log）、正式launch6与08 CLI6 PASS。本第二提交尚待主代理按明确commit范围同步B并完整编译；nav缺devel，不宣称本地CLI或catkin构建通过。未启动仿真、硬件、GPU、未push；本候选还没有新的实飞轨迹，不能宣称正常释放/提速已经验证。

修改前完整旧控制源快照 legacy_baseline/20261011/compensate_before_descent/patrol_control，56源文件，附FILES.txt/SHA256SUMS，缓存排除，不参与编译。部署必须同步cpp/header及两份control YAML，重新生成control.yaml并编译；配置生成器未修改。

## 集成验证更新

来源导航fe890d5b。主代理在B完成patrol_control实际catkin构建、49项补偿生产方法回归，以及正式launch6项/08 CLI6项（显式检查串行开关true），全部通过。同步保留各分支无关代码差异；修正同步时测试期望字典的重复键。未部署板端，不宣称串行策略已实飞成功。
