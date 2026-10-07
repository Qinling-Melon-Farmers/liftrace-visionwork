# 独立入口 CLI 契约

入口：deployment/competition/start.sh，只使用自己的 workspace overlay，不运行或借用 board_trials。

命令形式：start.sh preview|flight --site-config <yaml> [--motion-optimization on|off] [--obstacle-columns on|off] [--motion-action-timeout 秒] [--check-config]。

| 参数 | 契约 |
| --- | --- |
| preview / flight | 预览 / 真实受保护投递；实际运行均需已确认场地。入口不自动解锁或设置飞行模式。 |
| --site-config 文件 | 必填完整现场配置；不自动选择 namedsite 或测试配置。 |
| --motion-optimization on或off | 显式覆盖 motion_optimization.enabled；省略继承 YAML，未配置即关闭。开启仍须填写已有 corridor_speed_schedule。 |
| --obstacle-columns on或off | 显式覆盖 obstacle_columns_enabled；关闭只影响附加禁越柱，保留实体避障。 |
| --motion-action-timeout 秒 | 显式运动动作预算，有限数字、0<秒数<=600；省略继承 YAML 或原正式默认90s。目标预算仍为原120s，可显式在 YAML 配置。 |
| --check-config | 只检查，无 ROS 节点；preview 可检查未填完整的正式模板，flight 仍要求场地确认和门/H。 |
| --output-dir 目录 / --fc-reference X Y Z | 两者须一起提供且配合 --check-config；离线生成 runtime/control/overrides，参考单位 m，不代替现场监督器实测。 |
| --model 文件 / --metadata 文件 | 显式模型与匹配元数据；不从测试目录借用模型或 setup。 |

工作台将自身已有的实投确认落实为 flight，控制实际入口单实例，不能并行启动08或其他任务。无需新增 --real-release、--motion-optimized 或 --resume-survey；这些不是本入口接口。

ROS launch 参数对应 mode、site_config、model_path；额外显式覆盖为 motion_optimization:=on/off、obstacle_columns:=on/off、motion_action_timeout:=秒数，省略默认空串，不覆盖 YAML。

field.example.yaml 保留比赛参数、中文注释、空门/H和site_confirmed:false。两个 candidates 保留全部原值、仍是待验收候选。field_20261007_validated.yaml 仅是成功有限测试场地参考，必须显式选择，绝不是比赛默认。

真实入口已检查：bash deployment/competition/start.sh --help，以及 preview --site-config deployment/competition/field.example.yaml --check-config，均不启动 ROS。
