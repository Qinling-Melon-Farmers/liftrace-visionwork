# H mode4：OFFBOARD 直接接地候选

`h_offboard_mode4.override.yaml` 是小覆盖文件，通过 `--landing-config` 叠加到原现场 YAML，不复制整场参数。03/04/08、field.example 及原路线候选保持 mode1/POSCTL 默认；历史 field YAML 不变。mode4 与 `landing_handoff_mode: OFFBOARD` 必须同时选择；low/high 对准都可用，原 mode1/2/3、AUTO.LAND 与 POSCTL 仍保留。

新模式在合法 H 对准后冻结 XY，保持 OFFBOARD，以 XY 位置＋负 VZ 请求下降。关闭 Z 位置控制；消息 `position.z` 仅使用当前测量高度作为 TF 锚点。近地保护终点为 `ground_z + fc_ground_clearance - touchdown_overtravel`，默认越程0.04m，必须小于机架离地间隙。新鲜真实 ON_GROUND 连续0.3s 后回到普通位置通道并请求普通 disarm；不得伪报 POSCTL、不得 force disarm。模式请求或到达名义高度都不能替代真实落地。

controller 参数位于 `/external_landing`：`offboard_ground_z` 来自测量地面；`offboard_fc_clearance` 来自 known_rig；`offboard_descent_speed=0.30`、`offboard_touchdown_overtravel=0.04`、`offboard_ground_confirm_duration=0.3`、`offboard_state_max_age=2.5`、`offboard_disarm_retry=1.0`。`land_height=ground+0.30` 仅为 staging，`auto_land_height=ground+0.37` 在此模式不用于交接，也不注入 POSCTL 稳定门槛。原 `landing_posctl` 配置保留用于显式选择 fallback。

给 controller agent `01a12ffb-1d55-7882-8668-d3fdc52f4ae4` 的接线契约：controller 读取 `external_landing/offboard_setpoint_topic`（生成 YAML 的 `offboard_setpoint_topic`），默认 `/navigation/landing_setpoint_touchdown`；adapter 读取 `~landing_touchdown_setpoint_topic`，生成器写同名 `/navigation_frame_adapter/landing_touchdown_setpoint_topic`。覆盖入口为 `landing_offboard.setpoint_topic`，两端统一配置。消息为 mission frame 的 `mavros_msgs/PositionTarget`，`FRAME_LOCAL_NED` 标识下的数据仍按 MAVROS 的 ENU 输入约定；mask=2524，启用 PX/PY/VZ/yaw，忽略 PZ/VX/VY/全部加速度/yaw rate，`velocity.z=-speed`。adapter 严格使用源时间的重力对齐 TF 转换 XY、锚点 Z、yaw 与 VZ，沿用源新鲜度和普通/降落/投递通道共用时间戳锁；controller 陈旧或确认落地后，必须用更晚时间戳恢复普通位置通道。

`landing_descent_speed_mps` 是 mode2/3 的已有 profile 参数，默认仍0.20，mode4不使用它。本候选只设置实际 VZ 所用的独立 `landing_offboard.descent_speed=0.30`。实际固件必须具备兼容的触地检测条件：已知 PX4 v1.15.4 的 `LNDMC_Z_VEL_MAX=0.25` 需要下降请求至少0.275m/s；位置控制慢降可能不能进入落地判定。配置允许其他正速度，现场必须结合固件参数检查，不据离线生成声称触地检测已通过；本改动不修改固件参数。两套轻量bag均记录配置的touchdown输入与MAVROS raw输出，可复核VZ和mask。

以下仅离线检查与生成，参考 Z=0.09 是测试参考，不是现场测量值；正式模板仍须填好实测门/H/墙面并确认场地。不要为较低 local ground 放宽既有投递高度约束。

```bash
PATCH=deployment/competition/candidates/h_offboard_mode4.override.yaml
bash deployment/board_trials_4x4/03_h_landing/start.sh preview --site-config deployment/site_20260928/h_landing_test_area.yaml --landing-config "$PATCH" --check-config --generate-config /tmp/h_mode4_03 --reference-fc 0 0 0.09
# 04/08：保留各自实测 site-config，额外加同一个 --landing-config；不是替换 site-config。
bash deployment/competition/start.sh preview --site-config deployment/competition/field.example.yaml --landing-config "$PATCH" --check-config
# 对已确认实测场地，额外使用 --output-dir /tmp/h_mode4_field --fc-reference 0 0 0.09 离线生成。
```

工作台命令预览/配置检查 API 可显式传 `landing_config: "deployment/competition/candidates/h_offboard_mode4.override.yaml"`，仅支持模块03/04/08与独立正赛，和 CLI 使用同一校验链；默认 UI/入口不自动选择候选。本轮只接线与离线测试，未部署、未飞行、未完成动态接地验收。
