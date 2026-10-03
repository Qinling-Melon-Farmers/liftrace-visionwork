# 竞赛整机候选更新与操作（2026-10-03）

本次在现有 `feat/r2026-competition-integrated` 合入板端 `713556e` 与视觉研究 `9dce2a0`；导航共享功能对应 `feat/high-view-liveness-20260919@01a6da4`。复用原空闲工作树 `/home/xhj/liftrace-worktrees/r2026-board-frame-fix`，未新建工作树。两次正常 merge 分别为 `00e9185`、`428259d`，保留来源历史。

**整机代码链已接齐，属于待整场验收候选；不是已经完成正赛实飞验收。** 本轮不部署、不启动仿真、不开实机节点、不操作舵机。现场0928工程保持原样。main、r2026-main-integration和根目录liftrace未被覆盖。

## 与0928目录的关系

0928已具备完整导航、视觉、执行与舵机链。其第8组 `FullMissionTrialRuntime` 继承 `HighViewFull`，可以高位搜索、低位重访三投、过走廊、H降落，绝非只有演示片段。第5组则是提前集齐中断、投递后回起飞点收尾，不能代替正赛走廊/H。

本次把成熟实现合回长期未更新的整机分支，并整理独立运行入口：

- `competition_supervisor.py`：共用现场初始化、录包、人工解锁后的启动时序。
- `navigation_competition_manager.py`：仅实机适配壳，调用原 `FullManager/HighViewFull`；没有第二套选靶/投递状态机。
- `hardware_session/hardware_startup/hardware_bag`：由专项与整机共用，原专项脚本保留兼容入口。
- `actuator_pwm`：10月3日实际成功验收的实体源码，位于主工作区，三槽PWM映射4/5/0；不依赖另一个目录的hardware_ws。其他硬件型号不可照搬引脚映射。
- `deployment/competition/package.sh`：导出只含正式源码的包，剔除九组专项、研究报告和回放资产的运行依赖；模型是独立交付资产。

完整任务为：静置定位一致→新建地图→视觉/控制READY→人工解锁→OFFBOARD/低空稳定→低位入场→高位巡航及线索→低位重访/确认→逐槽许可投递及恢复→实测走廊→H识别对齐→AUTO.LAND与终态。保留未达三投时的既有任务期限/降级收尾，不承诺任意场景一定三投。

## 保留与明确的配置

| 项目 | 本次候选 |
|---|---|
| 位姿跳变保护 | 保留 `distance > 3×dt + 0.25m` 的现行检查；不自动解除ABORT |
| TF/定位 | 默认现场单位静态TF；仍要求FC/LIO同坐标、静置一致后才建图，不能用静态TF掩盖航向差 |
| 负载 | 特征提取开启、地图20m、det_range 6m，PCD落盘关闭；地图盒问题研究尚未实施 |
| 地图 | 虚拟顶棚关闭；三维膨胀0.25/0.20/0.10m；10cm栅格水平向上取整到0.30m |
| 障碍柱 | 保留已修正的中部轮廓算法；独立现场档案开关，不继承室内临时关闭状态 |
| 视觉 | 五分类RKNN及对应metadata；高位粗0.60、连续支持/冲突规则、低位新鲜确认和严格释放链继承原实现 |
| 外参/槽位 | 使用已知安装外参与每槽独立偏移；高度从静置飞控参考自动生成 |
| 默认正赛模板 | 低位1.4m、高位2.6m，规划速度1.2m/s/加速度1m/s²；原研究前视1.0/0.4/0.15m。是既有快速基线，不是新提速 |
| 走廊速度 | 未给门墙平面时沿用已有保守走廊档；填写 `corridor_speed_schedule` 可使用既有门区/开阔区调度 |
| 到点方式 | 保留原有减速、到位/驻留；连续过点和爬升提速仅在计划中 |
| 录制 | 5Hz压缩图、关键任务/视觉/位姿/投递话题和局部膨胀点云；不录全场FreeDOM/raw雷达，不做相机MP4编码 |

`field.example.yaml` 是旋转场地的配置模板，不是已经实测的航线。巡航路线和低空覆盖边界可调整；门洞、走廊航点、H坐标故意留空，必须实测填写并标记 `site_confirmed: true` 才允许运行。实际FOV、遮挡和目标完整入镜仍需现场验收，几何模板不代表全场识别保证。

## 构建、交付与配置

源码来自视觉仓 `feat/r2026-competition-integrated`。使用新独立目录；保留0928专项工程用于复盘。执行：

```bash
cd <competition_root>
BUILD_JOBS=2 bash deployment/competition/build.sh
source deployment/competition/environment.sh
cp deployment/competition/field.example.yaml deployment/competition/field.yaml
# 编辑field.yaml：实测边界、入场点、巡航点、走廊点、H、门墙平面、柱开关及site_confirmed。
bash deployment/competition/start.sh flight --site-config deployment/competition/field.yaml --check-config
```

`build.sh`只构建正式硬件包及依赖，不编译专项/评测节点。需要本机ROS Noetic、OpenCV4.2、Livox SDK2、相机SDK和RKNN运行时；不复制笔记本build/devel。环境脚本检查uav_mission、uav_vision、uav_high_view、camera_sdk、actuator_pwm必须来自当前根目录。`patrol_control/Servo`由本工程构建生成，终端都source同一环境。

模型放在 `runtime_models/flight_5cls_20260928_fp16.rknn`，或用 `--model /绝对路径` / `UAV_VISION_RKNN_MODEL_PATH`；metadata默认对应五分类版本，不能把旧六类权重改名替换。已准备的模型资产见原五分类模型交付目录。

只导出源码时，在干净提交后执行：

```bash
bash deployment/competition/package.sh /绝对路径/liftrace_competition_source.tar
```

源码包不含model/bag/build/devel，也不含专项源码；需另带匹配模型、现场MID360 JSON和测量后的field.yaml。实体舵机包已包含，不创建外部symlink。

## 现场启动顺序：五个前台终端

每个终端先进入 `<competition_root>` 并 `source deployment/competition/environment.sh`。下面命令供后续现场使用，本次未执行。

1. **MAVROS/ROS master**：沿用现场已确认的飞控串口与波特率。
   ```bash
   roslaunch mavros px4.launch fcu_url:="${FCU_URL:?填现场飞控串口URL}"
   ```
2. **MID360驱动**：现场JSON的主机/雷达IP必须与实际网口一致。
   ```bash
   roslaunch uav_mission mid360_driver2.launch user_config_path:="${MID360_CONFIG:?填本机JSON绝对路径}"
   ```
3. **下视相机**：保留已标定1280×720及原始图像方向。
   ```bash
   roslaunch camera_sdk camera_calibrated_1280x720.launch
   ```
4. **舵机raw服务**：init配置PWM，launch会初始化/复位机构；只在现场允许机构动作时执行。不要同时再开一套servo_controller1。
   ```bash
   sudo bash patrol_uav_ws-patrol_planner/src/actuator_pwm/init_pwm.sh
   roslaunch actuator_pwm launch_all.launch
   ```
   查询应为 `rosservice type /legacy/Servo_raw` → `patrol_control/Servo`。整机投递走 `/Servo`许可代理再到raw服务，启动命令没有新增第二套舵机调用链。
5. **整机监督器/录包**：先地面预览；结束预览并确认落地未解锁后再进入flight。
   ```bash
   bash deployment/competition/start.sh preview --site-config deployment/competition/field.yaml
   # 结束preview后，现场授权下一轮：
   bash deployment/competition/start.sh flight --site-config deployment/competition/field.yaml
   ```

第一个roslaunch可启动本机ROS master，无需第六个长期终端。preview不启动执行控制和释放代理；flight得到READY后只等待**人工解锁**，沿用现场OFFBOARD→低空稳定→启动任务时序，程序不自动解锁。READY以前不解锁。

查询任务可另开临时终端 `rostopic echo /navigation/mission_status`。ABORT仍是终止，人工接管后落地复位；本轮未接入调研中的自动恢复方案。监督器退出会收尾所属应用/地图/定位/录包，设备终端保留；一次只启动一套任务。不要在飞行中用重启监督器当作恢复。

## 验收与边界

本轮验证明细见 [REPORT.md](REPORT.md)。尚需测量正赛实际门洞/H/边界、核对真实镜头覆盖，取得授权后进行同版完整任务实跑，并完成硬件模型/图像/性能确认。高度跳变原因已研究，但EKF内部原因与ULog验证、20m地图盒候选修订、ABORT有界恢复仍是后续独立工作。

main不合入此次未整场实跑候选；`r2026-main-integration`保留为主干集成工作区。板端试飞分支和导航参考分支只补推另一对话已完成的713556e研究报告，不把本轮新入口直接覆盖现场冻结部署。
