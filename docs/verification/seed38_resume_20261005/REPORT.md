> **修正轮已完成：** [三投、零碰撞、H触地后保护中止](fixed_rerun/REPORT.md) · [视频](fixed_rerun/index.html)。以下保留7a950204原A/B，不混入新源码结果。

# seed38 高位续扫对照

本报告只读取本批原始日志；SITL/mock ACK 不代表实物投递落点或板端验收。

**收益可衡量：否。**
未触发、回高/接回失败、任务失败或证据不足时，续扫收益保持 `null`，不能解释成收益为零。

不可衡量原因：resume_off:NO_COMPLETE_MISSION_TIME, resume_off:RUN_NOT_PASS, resume_on:NO_COMPLETE_MISSION_TIME, resume_on:RUN_NOT_PASS, resume_on:ZERO_COLLISION_NOT_VERIFIED。

| 轮次 | 原 Gate | 续扫状态 | 请求/回高派发/实际续扫 | 软件完成秒 | 物理完成秒 | 三槽 ACK | 真值正确模拟释放 | 碰撞 |
|---|---|---|---|---|---|---|---|---|
| resume_off | FAIL | DISABLED | False/False/False | N/A | 390.115 | 3 | 3 | 0 |
| resume_on | FAIL | FOUND_MISSING | True/True/True | N/A | N/A | 3 | 3 | 1 |

完整任务原始差值（off−on）：N/A s。
可归因于续扫的差值：N/A s。

物理完成端点原始差值（off−on）：N/A s。
物理端点可比：False；阻断原因：resume_on:FAIL_NOT_EXPLAINED_BY_POST_GROUND_OPERATOR_STOP, resume_on:PHYSICAL_COMPLETION_NOT_VERIFIED, resume_on:ZERO_COLLISION_NOT_VERIFIED。

第三槽 ACK 任务秒：off 304.982，on 161.098；阶段差值 off−on 143.884 s。这是投递里程碑，不作为整轮飞行节省或 Gate 结论。

## 计量口径

- 内嵌历史事件按完整内容去重，保留首次发布；阶段时长来自状态首次发布，含录制采样延迟。
- `SURVEY_RESUME_ONCE` 是请求；`resume_ascent` 是实际派发；`SURVEY_RESUMED` 才是接回后续扫。`resume_completed` 在失败时也会置位。
- 限高 ACK 参数载荷未直接录制。回高派发只提供按该源码准入检查推断的证据；规划 STARTED 回执另行统计。请求至派发时间不是纯 ACK 延迟。
- 以释放回执源时间匹配 truth_pose.csv 最近样本；最大样本年龄 0.250 s。类别匹配且机体 XY 位于真值靶板名义旋转方形内才算正确模拟释放：普通靶 1m、红十字 0.35m。
- CSV 真值已减 recorder offset；目标采用同一 XY 变换。判别跳过/证伪位置时，仅依据日志内明确坐标，不猜测位置。
- 原 Gate、物理支撑接触、执行 ACK 与任务记账各自保留。物理完成要求持续接触、ON_GROUND、解除武装及至少2秒连续真值稳定；短暂碰地不当作稳定落地。
- MAVROS state/extended_state 日志只记录状态变化；持有的地面/解除武装状态不套用0.5秒 odom 新鲜度门槛。
- 阶段和五类 CSV 飞行指标在已验证物理完成处截尾；之后地面等待及人工停止另列，不加进飞行时长。软件完成仍只认可 COMPLETE，原 FAIL 不改写。
- 续扫结论只依据续扫专用事件；后续人工 abort 不覆盖此前 FOUND_MISSING/RESUMED_ENDED。

- NOT_STARTED 的不同 execution_id 仅表示拒绝回执，不计重复执行或占槽。执行次数只计 RAW_CALL_STARTED/COMPLETED；完整相同回执重播另计。
- 本地重试须同时关联严格 action 身份的 Bridge preflight 拒绝、下一条 REVISIT 与该槽 FREE 状态；后续成功投递另按 decision/attempt 记账，原失败回执仍保留。

## 同门接触与轨迹快速比较

以下只比较 on 接触时真值 y 对应的两轮第三段返航横截面。采样不同步，机体中心到墙面的距离不是防护包络净距；不能据此归因续扫策略。

记录状态：RECORDED。
on 接触时刻：228.978 ROS s。

| 轮次 | 真值 ROS秒 | 真值 x/y/z | 真值 yaw度 | 中心在墙东侧距离 m | 规划 x/y/z | 规划 yaw度 |
|---|---|---|---|---|---|---|
| resume_off | 373.663 | [9.062128847874183, -1.8946816944935139, 0.577550468642777] | 0.149 | 0.362 | [8.99283809226041, -2.001242110952972, 0.6761876201440877] | 0.097 |
| resume_on | 229.009 | [8.974632206014098, -1.896635486215356, 0.7598335228974507] | -0.124 | 0.275 | [8.931783047504425, -2.014975209852721, 0.6681436777459271] | 0.110 |
## resume_off

运行目录：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_off_seed38_20261005_161407`。
首次失败：`null`。

原 Gate：`{"errors": ["manager_failed", "manager_aborted"], "failed_checks": ["contract_errors_zero", "land_success", "manager_complete", "mission_ros_within_limit", "return_before_land"], "reason": "manager_failed", "software_complete": false, "software_terminal_ros_s": 500.807, "status": "FAIL"}`。
结果分类：`PHYSICAL_LANDED_OPERATOR_STOP_SOFTWARE_INCOMPLETE`。

### 物理完成与人工停止

- 首次支撑接触：395.304 ROS s。
- 持续支撑开始：396.505 ROS s。
- 物理完成确认：401.748 ROS s。
- ON_GROUND：399.147 ROS s。
- 解除武装：401.748 ROS s。
- 人工停止请求：500.754 ROS s；ABORT 派发：500.769 ROS s。
- 已确认地面等待：99.006 s。
- 飞行指标终点：401.748 ROS s（verified physical completion）。
- 稳定地面尾段：`{"end_ros_s": 500.754, "max_gap_s": 0.14100000000001955, "samples": 962, "state_contradictions": [], "verified": true, "xyz_range_m": [9.593757628323374e-07, 6.455753451817259e-07, 5.716330937766134e-07]}`。

### 接触 Gate 与完成状态

实际碰撞次数：0；接触 Gate 停止：False。
防护包络擦碰保留为 Gate 碰撞；根据采样深度、力和接触对描述，不表述为严重坠毁，也不把 Gate 停止前的返航称为完成。

| 首次/末次 ROS秒 | 时长 s | 样本数 | 最大深度 mm | 峰值力 N | 仅记录 guard | 接触对 |
|---|---|---|---|---|---|---|

### 阶段时长（ROS 秒，截于物理完成或软件终点）

| 阶段 | 时长 |
|---|---|
| SURVEY | 45.036 |
| DESCEND | 2.745 |
| REVISIT | 34.410 |
| REACQUIRE | 0.790 |
| DELIVERY | 19.475 |
| LOW_COVERAGE | 205.016 |
| TAIL | 82.632 |

### 回高与接回

| 动作 | decision_seq | 请求→派发秒 | 派发→规划 STARTED 秒 | 派发→到达秒 |
|---|---|---|---|---|

### 三槽回执与真值

| 槽 | 执行 ID | 类别 | ACK ROS秒 | ACK任务秒 | 最近真值类别 | 距离 m | 样本年龄 s | 正确模拟释放 |
|---|---|---|---|---|---|---|---|---|
| 1 | 1 | red_cross | 91.076 | 79.443 | red_cross | 0.067 | 0.029 | True |
| 2 | 2 | bridge | 112.475 | 100.842 | bridge | 0.121 | 0.011 | True |
| 3 | 3 | panzer | 316.615 | 304.982 | panzer | 0.038 | 0.034 | True |

记账核对：`{"bridge_commit_count": 3, "counts_agree": true, "gate_release_commit_count": 3, "high_view_slots": 3, "proxy_completed": 3}`。

### 本地 preflight 重试（未消耗槽位）

| 原 decision/attempt/槽 | NOT_STARTED IDs | 未耗槽确认 | REVISIT decision | 后续成功 decision/attempt/exec | 拒绝至后续 ACK秒 |
|---|---|---|---|---|---|
未记录 NOT_STARTED 本地重试。

执行/拒绝 ID/回执重播数：3/0/0。

### permission_stale 时间诊断

未记录 bag 摘要；不猜测许可过期原因。

### 跳过、否定与类别修正

- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 65.705, "xy": [4.568942458487072, -3.4376807722632643]}, "first_publication_ros_s": 65.705, "truth_association": {"board_local_xy": [0.11863637203848895, -0.09942426364894591], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.15478944722619134, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.5775879151047083, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 65.737, "xy": [4.568979722330556, -3.4374634218879305]}, "first_publication_ros_s": 65.741, "truth_association": {"board_local_xy": [0.11843374152970856, -0.09933725412877528], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.15457826881735576, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.5776106028336083, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 65.768, "xy": [4.569024905172533, -3.4372469977632623]}, "first_publication_ros_s": 65.768, "truth_association": {"board_local_xy": [0.11823386165640318, -0.09924276112800287], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.15436441195591463, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.5776254337470363, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 65.785, "xy": [4.5690802521376455, -3.4370310562341664]}, "first_publication_ros_s": 65.793, "truth_association": {"board_local_xy": [0.11803682554010211, -0.09913849800217017], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.15414646920935554, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.577630365879635, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"reason": "low_view_class_disproved", "stage": "TARGET_DEFERRED", "target": "panzer", "time": 65.8, "visits": 1}, "first_publication_ros_s": 65.898, "truth_association": null}`

输入问题：无。

## resume_on

运行目录：`/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_on_seed38_20261005_165940`。
首次失败：`{"data": {"attempt": 1, "command": 1, "decision_seq": 19, "event_seq": 91, "evidence_source": "guarded_servo_proxy:3:NOT_STARTED", "executor_id": "vcl06-planner-bridge-5ca054ba56eb4da3b1713401f157233c", "has_target": true, "header": {"frame_id": "camera_init", "seq": 91, "stamp": {"stamp_ns": 139554000000}}, "mission_id": "vcl06-random-12-043000000-1", "payload_committed": false, "payload_slot": 3, "reason": "release_preflight_rejected", "retryable": true, "schema_version": 1, "stage": 3, "status": 4, "target_class": "panzer", "target_first_seen": {"stamp_ns": 119815000000}, "target_id": 7, "terminal": true}, "kind": "result", "ros_sec": 139.554}`。

原 Gate：`{"errors": ["actual_collision"], "failed_checks": ["contact_ready_zero", "contract_errors_zero", "final_landed_on_ground", "final_vehicle_disarmed", "land_success", "landing_align_mode_seen", "landing_h_mark_valid", "manager_complete", "manager_post_delivery_route_matches", "mission_ros_within_limit", "post_delivery_return_goals", "post_delivery_return_sequence", "return_after_deliveries", "return_before_land", "return_home_success", "zero_collisions"], "reason": "actual_collision", "software_complete": false, "software_terminal_ros_s": null, "status": "FAIL"}`。
结果分类：`CONTACT_GATE_STOPPED_BEFORE_LANDING`。

### 物理完成与人工停止

- 首次支撑接触：N/A ROS s。
- 持续支撑开始：N/A ROS s。
- 物理完成确认：N/A ROS s。
- ON_GROUND：N/A ROS s。
- 解除武装：N/A ROS s。
- 人工停止请求：N/A ROS s；ABORT 派发：N/A ROS s。
- 已确认地面等待：N/A s。
- 飞行指标终点：229.615 ROS s（software terminal or censored observation）。
- 稳定地面尾段：`{"end_ros_s": 229.615, "max_gap_s": null, "samples": 0, "state_contradictions": [], "verified": false, "xyz_range_m": null}`。

### 接触 Gate 与完成状态

实际碰撞次数：1；接触 Gate 停止：True。
防护包络擦碰保留为 Gate 碰撞；根据采样深度、力和接触对描述，不表述为严重坠毁，也不把 Gate 停止前的返航称为完成。

| 首次/末次 ROS秒 | 时长 s | 样本数 | 最大深度 mm | 峰值力 N | 仅记录 guard | 接触对 |
|---|---|---|---|---|---|---|
| 228.978/229.049 | 0.071 | 5 | 0.039 | 0.150 | True | [['iris_mid360::iris::base_link::competition_guard_collision', 'toudi2::Wall_22_north::Wall_22_north_collision']] |

### 阶段时长（ROS 秒，截于物理完成或软件终点）

| 阶段 | 时长 |
|---|---|
| SURVEY | 47.246 |
| DESCEND | 6.788 |
| REVISIT | 44.587 |
| REACQUIRE | 2.461 |
| DELIVERY | 25.644 |
| RESUME_ASCEND | 3.190 |
| RESUME_JOIN | 6.738 |
| LOW_COVERAGE | 27.003 |
| TAIL | 53.905 |

### 回高与接回

| 动作 | decision_seq | 请求→派发秒 | 派发→规划 STARTED 秒 | 派发→到达秒 |
|---|---|---|---|---|
| resume_ascent | 13 | 0.000 | 0.339 | 3.202 |
| resume_rejoin | 14 | N/A | 0.331 | 6.729 |
| remaining_survey | 15 | N/A | 0.119 | 4.097 |
| remaining_survey | 16 | N/A | 0.132 | N/A |

### 三槽回执与真值

| 槽 | 执行 ID | 类别 | ACK ROS秒 | ACK任务秒 | 最近真值类别 | 距离 m | 样本年龄 s | 正确模拟释放 |
|---|---|---|---|---|---|---|---|---|
| 1 | 1 | red_cross | 81.088 | 69.045 | red_cross | 0.049 | 0.033 | True |
| 2 | 2 | bridge | 100.702 | 88.659 | bridge | 0.073 | 0.039 | True |
| 3 | 5 | panzer | 173.141 | 161.098 | panzer | 0.107 | 0.050 | True |

记账核对：`{"bridge_commit_count": 3, "counts_agree": true, "gate_release_commit_count": 3, "high_view_slots": 3, "proxy_completed": 3}`。

### 本地 preflight 重试（未消耗槽位）

| 原 decision/attempt/槽 | NOT_STARTED IDs | 未耗槽确认 | REVISIT decision | 后续成功 decision/attempt/exec | 拒绝至后续 ACK秒 |
|---|---|---|---|---|---|
| 19/1/3 | [3, 4] | True | 20 | 25/2/5 | 33.587 |

执行/拒绝 ID/回执重播数：3/2/0。

### permission_stale 时间诊断

- `{"candidate_age_s": -0.0010000000000047748, "interpretation": "LIKELY_TINY_FUTURE_CLOCK_SKEW", "latest_permit_by_bag_receipt": {"bag_time": 139.548, "decision": 19, "permitted": true, "reason": "permission_granted_from_commitment", "slot": 3, "source_time": 139.55, "topic": "/mission/release_permission", "valid_until": 139.8}, "long_expiry_confirmed": false, "node_internal_trace_available": false, "note": "permission_stale covers negative age as well as expiry. Bag receipt and source stamps do not prove the exact permission object/current ROS time used by the proxy. No threshold change in this A/B.", "prior_recorded_permits": [{"bag_time": 139.503, "decision": 19, "permitted": true, "reason": "permission_granted_from_commitment", "slot": 3, "source_time": 139.5, "topic": "/mission/release_permission", "valid_until": 139.75}, {"bag_time": 139.548, "decision": 19, "permitted": true, "reason": "permission_granted_from_commitment", "slot": 3, "source_time": 139.55, "topic": "/mission/release_permission", "valid_until": 139.8}], "refusal": {"bag_time": 139.552, "decision": 19, "execution_id": 3, "reason": "permission_stale", "slot": 3, "source_time": 139.549, "state": 1, "topic": "/mission/release_result"}}`
记录支持约1 ms未来时间戳/跨节点时钟回调差异这一推断；并未确认长期过期。缺少代理内部许可对象与当时 now 的 trace，不能给出确定原因。本对照未改阈值。

### 跳过、否定与类别修正

- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 59.209, "xy": [4.58196295810302, -3.3893747838258084]}, "first_publication_ros_s": 59.209, "truth_association": {"board_local_xy": [0.07470889188527063, -0.0754789422571347], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.10620023187818814, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.5785501489907814, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"class_name": "pillbox", "previous_class": "panzer", "stage": "LOW_VIEW_LABEL_RESOLVED", "time": 59.209, "xy": [4.582172542512438, -3.3893868795546256]}, "first_publication_ros_s": 59.209, "truth_association": {"board_local_xy": [0.0747696154630095, -0.07527798311095435], "label_matches": true, "nearest_class": "pillbox", "nearest_distance_m": 0.10610028340084411, "nearest_instance": "random_pillbox", "next_nearest_distance_m": 2.578345965485098, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`
- `{"event": {"reason": "low_view_class_disproved", "stage": "TARGET_DEFERRED", "target": "panzer", "time": 59.301, "visits": 1}, "first_publication_ros_s": 59.321, "truth_association": null}`
- `{"event": {"reason": "reacquisition_timeout", "stage": "TARGET_DEFERRED", "target": "panzer", "time": 148.701, "visits": 2}, "first_publication_ros_s": 148.707, "truth_association": null}`
- `{"event": {"alternative_xy": null, "reason": "reacquisition_timeout", "stage": "UNCONFIRMED_LOCATION_RETIRED", "target": "panzer", "time": 148.701, "xy": [7.021993970403927, -4.218025433980852]}, "first_publication_ros_s": 148.707, "truth_association": {"board_local_xy": [0.008129889534132177, -0.09029342267812784], "label_matches": true, "nearest_class": "panzer", "nearest_distance_m": 0.09065868564438954, "nearest_instance": "random_panzer", "next_nearest_distance_m": 2.555059297341895, "nominal_half_side_m": 0.5, "within_nominal_board": true}}`

输入问题：无。
