# 整场搜索限高修复（2026-10-07）

来源：18:23:57 启动的 module08，实飞日志 `试飞产物/board_full_mission_20261007_182357/`。18:27:05 首次报控制指令超高，18:27:15 起持续报超高保持；任务仍显示 SEARCH，三槽未释放。原配置 high_agl=max_agl=2.0。首次超限的精确量及定位变化来源尚未确认，不能将其写成已证实的 FC reset。

## 修改

- `deployment/site_20260928/full_mission_test_area.yaml`：搜索 FC AGL 2.0m 保留，上限改 2.3m。现场实际使用的 `logs/flight_workbench/site_overlays/08_full_mission_9051d1d7e283b1ccffcf.yaml` 同步该值。原场地坐标保留。
- 控制端采用与 Fast-Planner `ReferenceHeight` 相同的 1e-9m 数值容差；此值仅处理浮点边界，不是飞行余量。
- 超限拒绝整条当前指令，锁定首次保持点；不会每帧用漂移后的位姿重新选择保持目标，不沿超高轨迹的 XY 单独裁低 Z。
- 合格新轨迹经过原有时效/距离检查及最终距离限幅后，可解除该保持。切换对准、降落或既有恢复流程时，清除旧限高保持状态。
- 节流日志记录 planner/rejected/current/limit Z 及保持坐标，区分轨迹入口和最终指令检查。

## 验证

- 5项限高连续测试通过；40项相关回归通过，本地控制包编译成功。
- 使用本轮实际地面参考生成配置：本地 Z 上限 2.0352338086m，即 FC AGL 2.3m；Bridge 与控制相同。走廊阶段仍为 FC AGL 1.0m，走廊完成后恢复 2.3m。
- 未运行新仿真或新实飞。

## 部署状态与边界

现场上限配置及控制源码已写入192.168.43.59的0928目录。恢复网络、确认连接正常且未解锁后，板端 patrol_control 增量编译成功（ARM aarch64），已在实际 devel/lib/patrol_control/patrol_control 核实新增限高诊断字符串。编译日志：logs/height_margin_20261007/control_build.log。随后按用户要求停止所有工程运行节点；三路PWM enable均为0，未自动重启任务。新启动任务将加载补丁。板端原包和原二进制保存在 `legacy_baseline/20261007_height_hold_field/`；本地快照在 `legacy_baseline/20261007_height_hold/`。

重新启动08任务会重新读取现场配置；无需为此次配置变更重启舵机、相机或雷达设备节点。控制补丁必须完成板端编译，并由新启动的任务加载后才生效。

本补丁不实现超限后自主下降，不修复 FC reset 坐标连续性；当前起点确实在共同限高外、规划器无法给出合法新轨迹时仍可能保持。不能把本次修复描述为任何超高状态都能自动恢复。