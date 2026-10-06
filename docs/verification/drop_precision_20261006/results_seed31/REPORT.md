# seed31 首次 exact 集成诊断（2026-10-07）

结论：**DIAGNOSTIC_FAIL，不是完成 seed31，也不是精度改善验收。** 首槽红十字无 RAW_CALL_STARTED/成功释放，首次动作 deadline 后自动结束；没有追加重跑或 case38。

- 冻结源码：`8c4be8917ba68847b339d907ab794af9182e8a45`。
- 原始运行：`logs/drop_precision_seed31_20261007_002621`；原批次：`logs/drop_precision_20261006_batch/matrix.json`。原 matrix、Gate、日志、CSV、bag、三路视频保留。
- 原 Gate：FAIL / actual_collision，退出码 1；首次动作失败另见下表。碰撞一直 `stop_on_collision=false`，碰撞不是人为停止理由。
- 收尾：包装器自动归档和清理 PASS；独立 `check_sim_processes.sh` 复查零残留；runner 记录运行期间源码未改变。准备 TERM 时批次已 inactive，保护检查拒绝继续，没有发送 TERM，也没有误停止其他进程。

| ROS 时间（秒） | 已记录事实 |
| --- | --- |
| 76.276 | patrol_control_alignment_accepted，开始首槽对准 |
| 77.691 | strict_alignment_context_valid |
| 76.481–83.303 | 88 条实际 exact offset，全部 map_valid/frames/ground/source age 有效，均目标 8 |
| 83.249 | 最后一条 exact offset 的**源**时间；接收时间 83.303 |
| 84 / 172 / 184 / 195.09 | 原 bag 的 release evidence context 为 stale_observation，context_active=true、permission_active=true；184 时最后 offset 接收年龄 100.697 秒 |
| 195.119 | release_result_deadline_reached；释放回执 NOT_STARTED / alignment_context_revoked |
| 195.123 | release_proven_not_started；末 MAVROS state 为 armed=true / OFFBOARD，未证明完成降落 |

两端 exact 开关、vision_body/camera_init、ground_z=-0.22、CameraInfo/光学 frame、zero slot、NMS=false 均有预检和实际 rosparams 的 32 项 PASS。这证明实际视觉新路径输出；**不能等同于控制端接受或实际释放**。

最后 exact map_point=(7.423729,-1.005712,-0.22)，target_id=8。真实红十字中心=(7.4413,-0.9705)，本条地图中心误差 3.94 cm；消息 alignment_error=3.34 cm，alignment_tolerance=1.13 cm（既有 30 px 的曝光 Jacobian 换算）。该条仍超容差。88 条短片段中心误差中位 9.07 cm、P95 9.59 cm，仅描述 76–83 秒相关观测，不能作为新投递精度或与历史释放精度混比。

首槽长期无新 exact offset，生产控制同时持续收到 legacy alignment target 并下降；已有日志例如 129.715 秒 permission_active=true / fresh=false / should_drop=0。这是当前可复现的集成阻塞表象，停止更新的具体原因及修复由 A 负责；不据此推断是哪一个检查首先造成下降。

碰撞事实保留：4 个接触 episode，开始于 99.523、99.574、99.637、102.848 秒，全部 guard 与 random_red_cross 碰撞；原详情见 gazebo_contact_status.json。对准期 CSV 机体原点最低 z≈0.025 m（该 frame 地平面 -0.22 m）；这不是快递落点，也不是返航 H 自主降落完成。

## 与历史的可比范围

复用原固定 10×10、四树、三条扫描线、镜头离地 2 m（FC 2.16 m）的 seed31 冻结场景；不恢复续扫。历史 seed31/38 主基线各 3 次正确 mock 释放、0 碰撞，但原软件 Gate 都 FAIL，物理触地与软件结束需分别比较。

| 基线 | 红十字 / bridge / panzer，旧 ACK 接收时刻最近真值样本误差（cm） | 首次支持触地任务时间（秒） |
| --- | --- | --- |
| seed31 | 3.317 / 11.420 / 4.815 | 204.517 |
| seed38 | 1.941 / 7.725 / 13.982 | 363.540 |

本次释放为 0，释放时刻误差应缺失，不能填 0；也不能将约 195 秒的诊断失败和旧完整任务触地时间比较。ACK 时刻插值与最近样本是不同口径，前者历史 seed31 为 3.474 / 11.431 / 4.871 cm。本次短观测窗口没有因果支持证明精度改善。仿真机体中心及 mock 释放 ACK 不是实物落点，未仿真快递脱离或测量撞地。

## 性能与素材

启动至自动收尾墙钟 1074.30 秒；末 live_progress 为 ROS195.001，最后失败/采样约195.7。含启动与收尾的平均 RTF 约0.182；101→144 快照段约0.183。宿主内存采样 31.63 GiB 总计、8.04 GiB 可用；WSL 可用约9.4 GiB，交换未使用。后续监测切换本地 Windows cwd + login:false，读取明显加快；本轮仿真负载/线程/设置未调整。

轻量 bag 1,986,023 bytes，不含原始 Image/点云。原 downward / overview / follow 视频共约33 MiB，全部保留；未生成重分析或合成视频。

分析产物：`precision.json`（0 个完成事务）、`exact_path_summary.json`、`exact_offsets.jsonl`、`blocked_context_samples.json`（10 个时间快照，含 offset/context/许可/局部位姿/末飞控状态）、`diagnostic_summary.json`。

`live_blocked_sample.json` 的采样跨越首次终止，其 195.369 秒 drop_ready=false / align_disabled 是**终止后**样本，不能误标为终止前 stale_observation；终止前 stale 证据以 closed-bag 快照和主代理/A live sample 为准。原轻量 bag 未直接记录 drop_ready。

闭合 bag 重提取（不会启动 ROS）：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace-worktrees/r2026-high-view-search/vision_ws/devel/setup.bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
/usr/bin/python3 -B docs/verification/drop_precision_20261006/projection_evidence.py --bag logs/drop_precision_seed31_20261007_002621/vision_metrics.bag
/home/xhj/miniconda3/envs/rl_drone/bin/python -B docs/verification/drop_precision_20261006/precision_eval.py analyze --run logs/drop_precision_seed31_20261007_002621 --scene docs/verification/snake3_camera2m_20261005/generated/snake3_31/snake3_seed31 --exact-offset-data docs/verification/drop_precision_20261006/results_seed31/exact_offsets.jsonl
```

Windows 使用本地 cwd、login:false 的 PowerShell，再包装 `wsl -e bash -c '...'`。所有历史评测与原始素材只读；无源代码提交。

## 修复后入口（待主代理编译、提交及新授权）

`run_case.py` 增加 `--batch-name drop_precision_20261006_exactfix_batch`。仅允许当前任务前缀与最长48字符的小写字母/数字/下划线/横线后缀，拒绝目录、路径穿越和其他任务前缀。dry-run 两 case PASS，5 个非法名称拒绝；尚未创建新 batch。原批次不覆盖。

case0 使用新 batch + 新 pinned hash。case1 使用同 batch/同 hash，并显式传入**修复后 seed31**已审 run。每次开跑仍分别等待主代理授权；当前不启动。PLAN 末尾给出确切命令。源码修改与共享联调记录提交由主代理负责。
