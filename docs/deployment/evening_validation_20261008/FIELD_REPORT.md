# 2026-10-08 两套完整工程现场部署

## 结果

板端 `orangepi@192.168.3.126`：0928已更新，独立正赛工程已从空目录构建通过。两套均通过配置接线、84项任务测试及真实ROS传输的软件三槽调用（执行器为mock）。没有启动飞行、真实舵机、PWM、相机或雷达。

正赛不是裁剪版0928，也不借用0928的模型或devel：两套分别包含导航/任务/控制、视觉、MID360驱动、FAST-LIO、FreeDOM、Fast-Planner、相机包、执行机构包、五分类RKNN及metadata。0928额外提供模块化试飞入口。共有任务机制相同；场地、航线、速度档及部分预算通过各自YAML配置。

| 项目 | 板端目录 |
| --- | --- |
| 今晚试飞工程 T | `/home/orangepi/liftrace_board_trials_20260928` |
| 独立正赛工程 C | `/home/orangepi/liftrace_competition_20261008` |
| 更新前源码备份 | `/home/orangepi/board_deploy_backups/20261008_endpoint/source_before.tar.gz` |
| 构建与软件检查记录 | `/home/orangepi/board_deploy_backups/20261008_endpoint/` |
| 本机完整交付记录 | `/home/xhj/liftrace-deliverables/field_deployment_20261008/` |

本文 T/C 为上表根目录缩写，命令中的相对路径均从对应根执行。

## 本轮修复与已经部署的更新

- 保留现有靶心检测；12cm按FC到投口的物理杆臂处理，完整姿态旋转后从冻结靶心减去。固定基点计算，不叠加旧补偿；水平补偿与下降同时进行。
- 最终FC/投口均需到位且低速稳定再释放；保留原许可和事务防重复机制。冻结后不追低位残缺图像。
- 删除新增投递下降点云准入；补偿终点仅复用既有边界检查。不可达时绑定原动作取消，确认未执行后回收槽位，不把裁剪后的错误位置当作已到位。
- H沿用形态检测和冻结目标下降，降低交接高度并检查低位稳定；POSCTL完成末段，不启用AUTO.LAND。
- 运动优化、高位续扫默认开启，低空遍历兜底保留；实验性超高/虚拟柱/膨胀区脱困恢复默认关闭，EV预测旁路不接入主链。
- 干净ARM构建发现Fast-Planner包漏声明既有CMake依赖：补齐5包的cv_bridge及plan_manage的quadrotor_msgs依赖。未修改规划算法。
- 正赛环境固定使用当前工程模型及本机回环ROS通信，防止继承其他终端的旧路径/网络设置。正赛打包脚本必须携带权重和metadata，缺资产即报错；模型仍不进Git。

## 今晚08：保留昨晚成功场地，使用正赛速度

入口：`T/deployment/board_trials_4x4/08_full_mission/start.sh`。

配置按顺序理解：

1. 基础逻辑/新投递/H参数：`T/deployment/board_trials_4x4/08_full_mission/settings.yaml`。
2. 成功轮场地与航点：`T/deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml`。
3. 正赛速度覆盖：`T/deployment/board_trials_4x4/08_full_mission/competition_speed.yaml`，必须选择 `--speed-profile competition`。

```bash
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
bash deployment/board_trials_4x4/08_full_mission/start.sh preview   --site-config deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml   --speed-profile competition --check-config
```

以上只做检查。本轮没有执行flight或start_real.sh。原 `deployment/site_20260928/full_mission_field_20261007_evening.yaml` 原样保留；它的部分门前后Y不同于221730成功轮快照，不要混用。

成功轮高位矩形：(1.1,-1.5)→(5,-1.5)→(5,1.8)→(1.1,1.8)→起点。走廊X=7.7，入口(6.5,2.1)，H=(7.7,-3.45)；门点保留原快照。运动优化会合并共线中继点，规划仍检查整段障碍。

## 独立正赛入口与模板

入口：`C/deployment/competition/start.sh`；环境：`C/deployment/competition/environment.sh`。

| 可选路线 | C下的YAML相对路径 | 高位FC / 镜头AGL |
| --- | --- | --- |
| 基础矩形 | `deployment/competition/field.example.yaml` | 2.60 / 2.44m |
| 内收矩形 | `deployment/competition/candidates/rectangle_motion.yaml` | 2.76 / 2.60m |
| 两扫描线 | `deployment/competition/candidates/snake_motion.yaml` | 2.76 / 2.60m |
| 三扫描线 | `deployment/competition/candidates/snake3_motion.yaml` | 2.16 / 2.00m |

四份模板均FC软件上限3.2m。**正式模板仍site_confirmed=false，门/H坐标待现场填写，当前不能直接flight。** 这是保留真实配置边界，不是缺代码。历史field_20261007_validated.yaml只作旧场地参考，不自动作为今晚新版配置。

```bash
cd /home/orangepi/liftrace_competition_20261008
source deployment/competition/environment.sh
bash deployment/competition/start.sh preview   --site-config deployment/competition/field.example.yaml --check-config
```

## 有效参数

| 参数 | 独立正赛默认 | 今晚08正赛速度 |
| --- | --- | --- |
| 规划速度 / 加速度上限 | 1.2m/s / 1.0m/s² | 相同 |
| 搜索 / 精密前视 | 1.0 / 0.4m | 相同 |
| 近边界前视 | 0.2m | 相同 |
| 走廊快 / 慢前视 | 0.6 / 0.4m | 相同 |
| 低位复核FC AGL | 1.4m | 相同 |
| 高位FC / 软件上限 | 2.6 / 3.2m（基础矩形） | 2.0 / 2.3m |
| 走廊巡航 / 上限 | 0.9 / 1.0m | 相同 |
| 投递FC AGL | 0.35m | 相同 |
| H观察FC AGL | 0.9m | 1.2m |
| H下降目标 / POSCTL请求上限 | 0.35 / 0.37m | 相同 |
| 动作运动 / 单目标预算 | 90 / 120s | 60 / 60s |
| 总期限 | 600s | 相同 |
| 运动优化 / 高位续扫 | 开 / 开 | 相同 |
| 障碍虚拟柱 / 实验脱困恢复 | 开 / 关 | 相同 |

速度上限不等于实际飞行速度，前视是距离。正式低空覆盖线间距0.7m，08为1.2m；局部下降搜索半径/候选数正式1.5m/25、08为1.0m/9，其余详见本机PARAMETERS.md。共有机制相同不表示不同场地的所有参数必须相同。

投递最终检查：FC及投口XY误差≤4cm、高度误差≤5cm、水平速度≤0.05m/s、|垂直速度|≤0.10m/s、至少3个不同反馈跨0.30s，另需有效原许可。AGL许可窗口0.27～0.47m；它不是单独的放行条件。

H保留高位10帧/8cm确认；低位XY≤5cm、水平速度≤0.08m/s、|垂直速度|≤0.10m/s、至少3个不同反馈跨0.15s，满足高度条件后请求POSCTL。低位冻结H不追残图；POSCTL不再消费H视觉目标，实际落点仍需本轮实飞验证。

## 模型、雷达与处理优化

两套根内分别使用：

- `runtime_models/flight_5cls_20260928_fp16.rknn`
- `vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml`
- `deployment/site_20260928/MID360_config.json`（独立工程中为复制的自身设备配置）
- `vision_ws/src/camera_sdk/param/`（实测相机参数）

五类顺序bridge、panzer、pillbox、tent、red_cross；两套模型各自NPU加载和合成图推理通过，输出[1,9,8400]。该检查不代表现场识别率验收。

两套ARM产物均FAST_LIO_MATCH_THREADS=3、MP_EN、MP_PROC_NUM=3、fopenmp；定位launch为OMP_PLACES={4},{5},{6},{7}、PROC_BIND=close、WAIT_POLICY=PASSIVE。独立OpenMP测试3个worker分别位于CPU4/5/6；没有把它写成真实雷达运行负载测试。特征提取、点过滤3、迭代3、降采样0.15m、20m地图盒、6m量程、1Hz地图发布等优化保持。

MID360使用雷达专网主机192.168.1.100/传感器192.168.1.175，不应改成Wi-Fi地址。最初部署检查eth0为DOWN，未启动驱动，本报告不宣称雷达或整机READY。

## 工作台与后续验收

WSL工作台：http://127.0.0.1:8771/ 。双工程设置见同目录 `WORKBENCH_TWO_PROJECTS.md`。根目录与环境必须成套切换；只选正赛卡片不会改变全局root。今晚08先选0928、成功轮快照和competition速度；独立正赛选C及competition/environment.sh。

本轮仅软件调用，剩余实飞重点：正赛速度下的实际跟踪；三槽补偿完成后才释放及落点；H低位稳定/POSCTL末段偏移；现场门口可通行性。实验导航恢复未作为今晚默认启用。模板需现场几何确认，工程部署完成不等于新参数已实飞验收。

清理：只删除核实有本机完整备份的22项旧日志，释放1,602,236,416字节（约1.49GiB）。最新成功轮、未完整备份日志、回滚文件和2025工程保留。完整清单在本机cleanup.json。
