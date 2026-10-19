# H 末段 POSCTL 交接接口约定（2026-10-06 已确认）

旧导航桥接把空中 POSCTL 当成人工提前接管并取消 LAND；只修改控制器不能形成完整闭环。现有 `/control_state` 是没有时间戳和事务身份的 Int8，不能证明 POSCTL 是程序本轮请求的结果。用户已明确批准“同意，完成最小接口并部署检查”。

## 最小改动

- 控制器发布可配置的 `std_msgs/String` JSON 状态，默认话题 `/patrol_control/external_landing_handoff`，无需新增 ROS 消息类型。控制器和桥接通过参数读取同一话题。
- JSON 字段为 `mission_id`、`decision_seq`、`command_stamp_ns`、`event_stamp_ns`、`mode`、`stage`；时间使用纳秒整数的十进制字符串，Python 按整数解析，不经过浮点换算。`stage` 取 REQUESTED / OBSERVED / CANCELLED。MAVROS `mode_sent` 仅表示请求已发送，实际 MAVROS 模式观察才发布 OBSERVED。
- 沿用现有 `MissionCommand` 消息，不改变消息 MD5。仅无目标类别的 LAND 命令以 `target_class` 传递 `mission_id`；其他命令仍保留目标类别语义。`goal.header.seq` 与 `goal.header.stamp` 分别传递 decision 序号和线上的命令时间；默认 AUTO.LAND 继续兼容旧的无身份 LAND，POSCTL 要求完整身份。
- 桥接仅接受匹配当前 LAND 身份、命令后产生且新鲜的状态。提前手动切 POSCTL、旧轮状态、错误序号及其他模式仍取消 LAND；模式与状态跨话题乱序最多等待一个有界短窗口，超时取消，不自动恢复。
- 跨话题等待上限0.5s，模式请求到达上限2.5s。LAND的飞控/ON_GROUND遥测新鲜度复用既有 `/external_landing/state_max_age_sec=2.5`，适配约1Hz更新；里程计和交接OBSERVED状态证据仍要求0.5s，不放宽定位新鲜度。
- `landing_handoff_status_topic` 可在H现场配置中指定；生成器写 `/external_landing/handoff_status_topic`，控制器、桥接和bag读取一致的名称。默认状态只增加轻量String记录，不新增相机编码或原始雷达记录。
- POSCTL 交接后的 OFFBOARD 回切取消旧事务。服务 ACK 和实际模式到达都不等于落地；落地判断仍要求现有新鲜 ON_GROUND、位置稳定和落地区域，专项结束仍需解除武装。
- 保留 H 检测、笔画兜底、10 新帧 / 8cm / 0.5s 门槛及高度：1.0m 接近、1.2m 识别、0.40m 下降目标、0.55m 交接门槛。POSCTL 后由飞手操作油门完成下降。

## 归属与验证

控制器与桥接补丁先记录至导航来源仓的板端参考 feature 分支，再同步当前视觉集成 feature 分支；两个仓的联调记录互相引用本约定及来源 revision，不修改 main。

测试覆盖正常请求、状态乱序、提前人工 POSCTL、旧事务、过期状态、回切 OFFBOARD、请求失败，以及真实落地判据。随后板端增量编译与配置展开；本轮只部署和检查，不启动节点或飞行。

本约定已按仓库 AGENTS.md 第18条的跨组接口要求取得用户确认；部署和实飞验证结果在同目录报告与两仓联调记录中维护。本轮没有获得新一轮飞行授权，部署检查完成后保持应用停止。
