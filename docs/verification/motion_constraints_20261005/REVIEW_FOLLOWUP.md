# 复核补丁：恢复选档与限高等待（2026-10-05）

针对746efc8b复核，保留四项改进并补齐调用流程，不改速度/高度/时间门槛。

## 已修正

- 动态边界选速支持缓存为None；真实timer顺序测试覆盖SURVEY、REVISIT、DELIVERY、LOCAL_WALL_VERIFY的“过期→重复缓存→3条新帧→重新选档”，不再因None下标导致ABORT。
- pending超时先于位姿新鲜度检查，已终态则清除pending；pending每次派发前走完整_readiness，地图过期暂停、非暂态frame错误沿既有abort处置。新动作尚未送Bridge时不依赖Bridge计时。
- 限高namespace首次解析后缓存；限高ACK改为request/ack单条结构化参数，每次request含唯一ID、cap、frame、stamp。规划FSM仅在匹配本次请求且已采用相应约束后写一次ACK。请求检查10Hz、getCached，不再100Hz非缓存namespace查询和常态30次/秒写回。等待过久的请求刷新ID以满足原0.5s新鲜条件；后续同值限高仍有独立确认。
- 正式MAVROS child速度有效；原始FAST-LIO /Odometry目前未填twist，header仅说明表达约定，不等于有效速度。通用Bridge raw入口默认execution/odom_velocity_available=false，不使用moving recovery，沿原位置稳定恢复路径。只有确实提供速度的入口显式启用该字段。
- 新增真正非零三次系数的内部越高反例，以及从生产C++FSM提取的ACK测试：100次定时调用只ACK一次、同cap新ID再次ACK、尚未采用的新cap不ACK、错误frame不ACK。

## 验证与边界

修复工作树402项任务测试：398通过、4项整机生成器专属跳过；独立连续高度测试通过。五包增量编译通过；Bridge launch离线展开确认header默认关闭速度恢复、child正式入口开启。控制实现未再次修改，此前同源27项及导航工作区27项已通过。
八轮仿真依旧冻结2b0678b9，不能算本补丁的动态验收。下一轮应验证过期恢复继续飞、限高请求产生新轨迹；本次不自动新增仿真或启动硬件。

如果约束已生效后估计高度再次超限，规划会拒绝超限起点并保持，可能走既有动作超时/失败；本补丁没有实现自动单调下降回入，不承诺任意超限都能自主恢复。不得恢复“沿原XY裁低Z”以绕过此限制。

request/ack是任务与规划共同接口，所有包含本补丁的工作区必须同步两端；旧planner_applied_*心跳不再被本版本读取。
