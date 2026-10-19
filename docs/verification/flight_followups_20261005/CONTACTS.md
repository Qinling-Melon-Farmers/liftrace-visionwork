# 接触监测与碰撞后继续观测（2026-10-05）

本轮只修改接触监测器、接触策略、Gate 的可选碰撞终止行为及对应单元测试。未启动 ROS/Gazebo/实机，未修改历史场景配置、共享变更日志或 AGENTS，未提交或推送；主代理负责文档接线、共享变更记录与最终提交。

## 问题与修复

- 原障碍接触事件只保存第一帧 `details`，后续更深接触和更大力没有写入。现在每帧累计，按规范化 pair 合并；同帧重复或反向 pair 不重复计样。
- 障碍与 ground/H 支撑分别记录在 `events` / `support_events`，使用同一个峰值累计方法。支撑的 depth 峰值与力峰值独立更新，避免“力没有变大，因此 depth 未更新”。同时碰到支撑和墙体仍计障碍碰撞。
- 保持原计数规则：任意障碍接触从无到有增加一个 `actual_collision_count`，连续帧及连续期间换 pair 不另增 episode；全部障碍接触消失后再接触才增加下一次。每个 episode 保存期间出现过的所有 pair。
- 支撑事件不再只保留最后 64 条，保留本轮全部支撑 episode。没有新增 depth/force 阈值，也没有用轻碰判定替代碰撞事实。

## 接触数据字段

`gazebo_contact_status.json` 与发布的状态 JSON 使用相同数据：

| 位置 | 字段 | 含义 |
| --- | --- | --- |
| episode | `ros_stamp`, `last_ros_stamp`, `duration_sec` | 首次及最后一次正接触采样时间；持续时间为二者之差 |
| episode | `ended_ros_stamp` | 第一次无该类接触的帧时间，仅结束后出现 |
| episode | `sample_count`, `pairs` | 正接触帧数、整个 episode 出现过的规范化 pair |
| episode | `peak_sampled_depth_m`, `peak_sampled_force_n` | 本 episode 的采样 depth/力峰值，独立取最大值 |
| `details[]` | `max_depth_m`, `total_force_norm_n` | 该 pair 的独立 depth/力峰值，不必来自同一帧 |
| `details[]` | `peak_depth_ros_stamp`, `peak_force_ros_stamp` | 该 pair 两种峰值各自的采样时间；无 depth 时前者为 null |
| `details[]` | `first_ros_stamp`, `last_ros_stamp`, `duration_sec`, `sample_count` | 该 pair 的采样起止跨度及帧数，同帧重复 pair 只计一次 |
| `details[]` | `info`, `positions`, `normals` | 该 pair 最深采样的几何信息；保持原有最多 8 个位置/法向量限制 |

这些是传感器收到的采样峰值和采样时间跨度；不声称连续物理接触精确持续了多久。单帧 episode 的 `duration_sec=0`，可以结合 `ended_ros_stamp` 看下一次无接触采样；pair 中途消失又出现但整个 episode 未结束时，pair 的持续时间仍是首末采样跨度。

## Gate 配置接线

生产节点 `navigation_vcl06_assertion.py` 新增私有 bool 参数 `~stop_on_collision`，**默认 true**。现有节点完整参数名是 `/navigation_vcl06_assertion/stop_on_collision`；本轮没有修改任何 launch 或历史 YAML 的默认值。

后续研究场景可在独立 `gate_geometry_config` YAML 顶层加入：

```yaml
stop_on_collision: false
```

已有 `navigation_horizontal_search_vcl06.launch` 将该文件加载到 `/navigation_vcl06_assertion` 命名空间；高位 `full_strategy.launch` 已透传 `gate_geometry_config`。也可由主代理在研究专用 wrapper 中显式写入：

```xml
<param name="/navigation_vcl06_assertion/stop_on_collision"
       type="bool" value="false" />
```

参数在 Gate 节点构造时读取，须在节点启动前配置。现在只有私有参数，没有新增通用 `stop_on_collision:=...` launch 参数；主代理如增加转发，应保持默认 true。上述是配置说明，本轮没有执行这些启动或参数设置。

行为：

- true：保留碰撞立即终止的旧行为，包括原 `observe_full_trial=true` 场景。
- false：只有碰撞这一项失败时先写 `gate_status.json` 原始 **FAIL**，继续收集任务和 H 落地事实，不调用 shutdown；碰撞计数增加时更新原始 FAIL 报告。Gate 不发布任何飞行命令。
- 落地且解除武装、任务检查完成或现有超时到达后，仍以 **FAIL / exit code 1** 终止。即使 `observe_full_trial=false`，record-only 也保留 ROS 任务预算；墙钟 watchdog 保持原行为。
- 其他错误和 `observe_full_trial` 的旧处理逻辑保留；false 不会抹掉碰撞错误或放宽其他 Gate 阈值。监测器的 `status=READY` 仍只表示传感器在流式工作，不表示无碰撞。
- reducer 的 `actual_collision` 错误与 `metrics.actual_collision_count` 锁存；后续收到较小计数甚至零，也不能把碰撞报告改成 PASS。`zero_collisions` / `contact_ready_zero` 保持 false。报告附带实际 `stop_on_collision` 值，便于区分观测策略。

碰撞后的继续观测不是 Gate 验收通过，也不是实机动作授权。历史 run 的原始 FAIL 和统计不重写。

## 验证

在已有 conda `rl_drone` 中执行 62 项单元测试，全部通过；调用生产 `_on_contacts` / `_publish` / `_check_terminal` 方法，ROS 边界使用 mock，没有启动 ROS 节点。

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONPATH="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/src:$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/test:/opt/ros/noetic/lib/python3/dist-packages"
python -m unittest test_support_contacts test_navigation_vcl06_assertion test_h_contact_record_only test_staggered_corridor_gate
```

Windows 侧通过 UTF-8 临时脚本和 `wsl -e bash -c 'bash /tmp/codex_contacts_test.sh'` 调用。覆盖首次轻后重、独立力峰值、重复/反向 pair 多帧合并、缺失 depth 后恢复、全部支撑 episode 留存、H/地面与墙同时接触、JSON 持久化、默认立即终止、record-only 原始 FAIL/计数锁存、任务完成与超时收尾。

专用 `test_h_contact_record_only.py` 分别在 `observe_full_trial=false/true` 下验证：H 接近期间有碰撞，Gate 暂不终止；随后调用生产 reducer 接收 H 标记、landing 对准、ON_GROUND 和 disarm，最终落地检查成立，但落盘 JSON 始终 FAIL、碰撞计数 1、退出码 1。另测 full-trial 已有其他错误时，碰撞仍立即落盘为原始 FAIL，不因保留旧延后终止行为而漏记。

## 本代理改动文件

- `patrol_uav_ws-patrol_planner/src/uav_mission/scripts/gazebo_contact_monitor.py`
- `patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/contact_policy.py`
- `patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_vcl06_assertion.py`
- `patrol_uav_ws-patrol_planner/src/uav_mission/test/test_support_contacts.py`
- `patrol_uav_ws-patrol_planner/src/uav_mission/test/test_navigation_vcl06_assertion.py`
- `patrol_uav_ws-patrol_planner/src/uav_mission/test/test_h_contact_record_only.py`
- `docs/verification/flight_followups_20261005/CONTACTS.md`

遗留/下一步：主代理合并本报告、按需要给研究专用场景接线，并可在其统一 CMake 修改中注册 `test_support_contacts.py` 与 `test_h_contact_record_only.py`（本轮不改其他代理负责的构建文件）。动态 ROS/Gazebo 结果尚未验证，须另获当轮明确仿真授权。
