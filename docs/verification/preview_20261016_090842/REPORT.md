# 2026-10-16 09:08 preview 初始化超时

## 结论

本轮08 preview正确加载site_waypoint_map_20261016.yaml、正赛速度档及运动优化。初始化90秒内FC与LIO航向不一致，未取得静止参考而退出；尚未生成本轮运行地面参考、启动地图和任务。非航点错误，非桌面高度越界，也没有飞行或舵机动作。

## 直接记录

- 命令见workbench_trial.log。状态为INITIALIZING，最后RuntimeError: No stationary disarmed camera_init reference。
- 实际启动阈值：同帧位置差≤0.20m、航向差≤5°，持续2秒；源位姿年龄≤0.30秒、配对时差≤0.10秒；另要求静置位置跨度≤2.5cm、航向跨度≤0.04rad及任务航向接近0。
- FC/LIO位置差约5.5～6.3cm，稳定后位置跨度毫米级；航向差从约144°至最后打印82°，整段未满足5°。
- bag最后FC yaw约−78.11°，LIO外部位姿yaw约0.108°。当前alignment_mode=legacy_static，map与camera_init采用零旋转静态TF，不能把此巨大差值当成已对齐。
- LIO874条诊断：输出年龄P95=41.36ms、最大56.37ms；lidar_queue最大0；本轮没有显示LIO积压。外部位姿到bag年龄P95=39.82ms，持续约10Hz。
- 飞控为何缓慢调整航向，仅bag不能确定磁场干扰/估计器融合设置等原因；不能直接归因于桌子，需对应飞控估计器日志或状态进一步区分。

## 与昨晚的初始化预算差别

19:24实飞bag元数据含initialization_timeout=300秒；本次登记场地配置未显式填写，使用入口默认90秒。工作台module08手动地图模板包含300秒。这是本轮较早超时的配置差别，但不能证明多等一定能消除航向偏差；本轮90秒末仍约78°。未改动该参数。

## 收尾提示不是首因

BAG_CLOSED: INCOMPLETE仅缺/uav_vision/detections；检测应用未启动。recorder_exit_code=0、errors为空，原bag已完整取回并可遍历。FrameAdapter的publish() to a closed topic发生于收到SIGINT之后，是收尾回调竞态，不是此前初始化失败原因。

## 桌面检查边界

现有AGL基准为静止FC local Z减0.22m。若在桌面成功初始化，基准相对支撑桌面，不等于真实地板；搬回实际起飞点后重新初始化，不复用桌面运行参考。仅检查场地文件可使用preview --check-config，无需依赖现场定位；实际preview需要上述定位一致性。

本轮只读取与分析，没有修改融合、时间门槛、导航参数或重启板端节点。
