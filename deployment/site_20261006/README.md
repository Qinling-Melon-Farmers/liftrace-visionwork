# 10月6日现场入口

复用 `site_20260928` 的实测 YAML；本目录不复制、覆盖或改写旧现场参数。默认模式是 preview；preview 会连接 ROS，只有 `--print-command` 和 `--check-config` 是离线检查。flight 必须由飞手手动解锁并重新拨入 OFFBOARD，继承已有就绪、位姿保护及接管逻辑。默认投递为 mock，只有显式 `--real-release` 才选择许可链实投。

默认设置见 `defaults.json`：motion 开启、高位路线 rectangle、高速采集先用 0.5m/s。低空/H组不传路线参数。高位02/06/07/08/09可选 `rect`（rectangle）、`snake2`、`snake3`；路线从对应现场 `survey_xy` 范围生成，普通现场为 X[0.8,5.5]、Y[-1.1,1.1]，保持 FC 高位/上限2m。无实测走廊/H时04/08仍拒绝启动；走廊和H使用各自专用旧 YAML。full_mission旧配置的障碍柱开启状态照原档保留。

九个模块的 `settings.yaml` 由主代理分别合并 `motion_optimization.enabled: true`，保留原字典子项。因此旧 `site_20260928/start_test.sh` 也继承 motion 默认开启；九个 settings 不设置 `survey_pattern`，旧入口继续使用现场原始航点。新 wrapper 才显式传入 rectangle/snake2/snake3。H/走廊、记忆、采集组由原生成器禁用 moving recovery，且不开启额外投递。

旧 site 的投递组 `flight` 原本选择真实投递，仍保持该语义；新 wrapper 的 `flight` 默认 mock。不要把 motion 默认开启理解为改变了投递模式或解锁/OFFBOARD方式。

现场编号：1=单投01，2=多投05，3=记忆07，4=重访02，5=优先06，6=采集09，h=H03。也接受九组 trial 名字及完整目录名字；04用 `corridor_landing`，08用 `full_mission`。

```bash
cd ~/liftrace_board_trials_20260928
# 只打印命令，不加载ROS；默认 rectangle + motion，不选真实舵机
bash deployment/site_20261006/start_test.sh 5 flight --print-command
bash deployment/site_20261006/start_test.sh 5 preview --check-config
bash deployment/site_20261006/start_test.sh 5 preview --survey-pattern snake2 --check-config
bash deployment/site_20261006/start_test.sh 5 preview --survey-pattern snake3 --check-config
```

授权现场飞行时，使用同一入口显式选择 `flight`；本轮只做配置/离线验证，没有执行飞行。记忆/采集/H不会启用投递。默认不启用续扫；若以后需独立验证，05现场优先组（目录06）及08支持显式 `--resume-survey on`。双线/三线可部署选用，但不等于同版实飞通过；矩形作为默认对照。

不要在旧 site YAML 加 `survey_pattern` 或 `motion_optimization`：旧生成器的现场参数白名单不接受这两项。本目录通过现有CLI传入选项，从而保留现场子参数。不要从整机分支覆盖专项 `trial_motion.py`、板端OMP绑定、相机/槽位配置或舵机包。

2026-10-06补充：[H专项FC 1.2m与板上待实飞更新清单](../../docs/deployment/board_refresh_20261006/H_1P2_AND_PENDING_FLIGHTS.md)。03识别高度已部署为1.2m，接近仍为1.0m，其他专项高度不变。
