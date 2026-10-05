# 工程投递许可与执行接口

依赖保持 `uav_mission -> patrol_control`；控制包不反向依赖任务包。旧 `Servo.srv` 和 `/legacy/Servo_raw` 不变，正式 external 控制路径使用 `ServoAction`，不得在服务失败时退回裸硬件调用。

## 2026-10-06：请求绑定许可版本

许可节点每次发布生成同一对 `permission_epoch`（节点实例UUID）和递增 `permission_revision`，同时发送完整 `uav_mission/ReleasePermission` 与供控制器消费的 `patrol_control/ReleaseAuthorization`。后者只是同次许可副本，不独立做许可判断。旧 Bool 仅保留旧控制接口；正式 external 控制不再用它启动舵机。

控制器检查许可新鲜度、有效期、完整任务身份与当前 ALIGN 一致，将许可版本和动作身份冻结进异步 `ServoAction` 请求。`request_id` 是控制 worker 编号；它不等于许可版本、任务 decision_seq 或硬件 execution_id。不要使用由 ROS 传输改写的 Header.seq 做许可版本。

代理若只有同实例较旧许可或尚未收到许可，在原有 `permission_refresh_wait`（默认0.25秒、墙钟）内等待；等待释放互锁，允许许可与撤销回调更新。等到对应或更新版本后，仍须检查任务/槽位/目标、几何、有效期和原有互锁。当前或更新的明确否决立即拒绝；不同节点实例不能借用旧请求。等待超时不执行硬件，不把所有 no_release_commitment 列为免冷却。

新的完整许可与控制副本必须成套生成。`ReleasePermission.msg`、`ReleaseAuthorization.msg`、`ServoAction.srv`、许可节点、代理和控制二进制必须一起重新构建部署；仅覆盖Python脚本不兼容旧消息MD5。`authorization_topic` 和控制侧 `uav_vision/release_authorization_topic` 需要指向同一话题。

## 执行事实与异步互锁

- 请求固定 mission_id、decision_seq、attempt、payload_slot、target_id、target_first_seen、target_class、align_mode。
- worker 等待真实舵机服务期间控制定时器持续运行，不先报虚假成功。
- UNKNOWN=0、NOT_STARTED=1、RAW_CALL_STARTED=2、COMPLETED=3。只有匹配请求编号/槽位且终态 NOT_STARTED 能证明本次未开始；成功须 terminal COMPLETED 且 res=true。
- 原始执行一旦进入即锁住动作和槽位；未知结果不得自动再次执行。成功只提交一次。
- 回执以固定动作身份归属，不受后来图像时间改变影响。对准结束立即撤销未来许可，但许可节点保留历史授权以接受迟到的执行事实。
- 已排队RPC的终态不可因先收到对准结束而丢弃。接管/停机仍取消未来任务；不确定硬件调用保持隔离。

本轮仅本地离线验证与编译；该文档不代表已部署或完成实飞验收。
