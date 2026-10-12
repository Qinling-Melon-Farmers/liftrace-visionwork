# H 第4种下降候选：OFFBOARD 保持靶心直至触地

## 来源与范围

基于试飞组 `liftrace_board_trial_20260928` 的 `f85a00fb` 接入。保留已有高/低位对准、下降模式1/2/3和当前默认选择，新增显式模式4，不自动替换现场模式。

模式4借鉴旧工程的“固定H中心后继续下降”，不采用旧版固定local Z=-1或低高度强制上锁。流程为：原H检测和对准→冻结XY→OFFBOARD水平位置闭环、独立向下速度→飞控确认触地→普通解锁服务请求上锁→观察真实未解锁状态→任务完成。

当前为本地候选；未部署、未实飞。本轮不启动新的Gazebo。试飞组同批提交中的投递实验不因本次H同步而推广到其他分支。

## 视觉—控制—任务接口约定

- `landing_descent_mode: 4` 必须与 `landing_handoff_mode: OFFBOARD` 成对使用。
- 控制器新增 `/navigation/landing_setpoint_touchdown`，类型 `mavros_msgs/PositionTarget`，位于任务坐标系；仅启用水平位置、竖直速度和航向。位置Z仅作为TF转换锚点，不作为高度控制目标。
- 现有 `navigation_frame_adapter.py` 沿用图像以外的控制坐标转换规则及公共时间戳水位，转换为MAVROS本地坐标后发布raw设定点。拒绝非重力对齐TF及旧指令，普通位置通道与新通道不同时生效。
- 控制器沿用带任务编号、决策编号和命令时间戳的降落状态事件；模式4的 `REQUESTED` 表示开始直接下降，`OBSERVED` 仅表示本次触地及实际上锁均已确认，不能把“当前仍是OFFBOARD”当完成。
- Bridge必须同时收到同一动作的完成事件、新鲜ON_GROUND和未解锁状态，才允许结束任务。保留空中人工接管和取消处理。

## 触地与下降边界

下降保持冻结的H中心，不追低空残缺图形。输入陈旧时停止推进下降；实测高度达到根据地面参考、静置FC高度及小量下探余量计算的最低值时停止下压。低高度本身不触发上锁。

触地确认需要本次LAND之后见过在空中状态，再收到不同源时间戳的新鲜ON_GROUND，持续满足确认时间；普通上锁请求异步执行，不能堵塞持续设定点发布。未确认触地、断连、人工接管或动作取消时不得发起新的上锁调用。服务成功回执不能替代飞控实际未解锁状态。

本机PX4 v1.15.4的 `MulticopterLandDetector.cpp` 在爬升率控制有效时要求向下速度请求达到 `1.1×LNDMC_Z_VEL_MAX`；只有位置目标可能不满足这条条件。模式4因此使用明确的竖直速度请求，初始候选0.30m/s对应本机默认阈值0.25m/s。**板端固件版本及落地检测参数尚未核验，不能宣称任意旧固件必然识别触地。**不自动修改PX4参数，不通过强制上锁掩盖不兼容。

## 验证记录

- 0928来源控制器实际Catkin编译及链接通过；控制器工作集60项离线检查通过，新增Catkin/nose入口12项通过。
- 配置、接口、入口相关工作集72项离线检查通过；任务Bridge单独24项检查通过。上述集合存在交集，不相加声称独立用例总数。
- 测试包含冻结XY、正确raw掩码、最低高度锁存、陈旧及重复地面消息、先地面后上锁/先上锁后地面的回调顺序、普通上锁回执与实际上锁的区分、取消及接管、旧模式隔离。
- 各同步分支另行执行实际源码的H控制、配置生成、TF/raw转换及Bridge回归；分支差异保留情况见本次变更记录。
- 未运行Gazebo、未上板、未实飞；新的竖直速度控制与机体触地检测仍是下一轮必须验收的项目。
## 显式选择及回退

小配置补丁位于 `deployment/competition/candidates/h_offboard_mode4.override.yaml`，叠加到已经填写好的现场配置，不复制、不替换场地坐标。03/04/08及独立正赛入口支持 `--landing-config`。例如先离线检查：

```bash
bash deployment/board_trials_4x4/03_h_landing/start.sh preview --landing-config deployment/competition/candidates/h_offboard_mode4.override.yaml --check-config
```

需要直接写入所选现场YAML时，核心字段如下（不等于本轮已上板启用）：

```yaml
landing_descent_mode: 4       # 新候选：固定H点水平位置，竖直速度下降至触地
landing_handoff_mode: OFFBOARD # 不请求AUTO.LAND/POSCTL
landing_offboard:
  descent_speed: 0.30        # m/s；模式4独立参数，不使用模式2/3的Z轨迹速度
  touchdown_overtravel: 0.04 # m；最低反馈高度=地面Z+静置FC高度-此余量
  ground_confirm_duration: 0.3 # 至少两个不同时间戳；1Hz反馈实际至少约1秒
  state_max_age: 2.5         # s；姿态/里程计仍使用原来更严格的新鲜度要求
  disarm_retry: 1.0          # s；只在触地已确认时请求普通上锁
```

新模式不修改靶心检测，不取消水平位置闭环。姿态/定位异常、触地无法确认或到达最低高度后仍无触地消息时，不以强制上锁掩盖问题。回退时去掉此补丁，继续使用各分支原有模式；不要只把4改成1而留下OFFBOARD，配置校验会拒绝不匹配组合。

源码同步不等于上板或模式启用。0928保留试飞组实验投递逻辑；其他分支仅同步H能力，保持各自投递控制及POSCTL默认参数。main与EV冻结分支未修改。
