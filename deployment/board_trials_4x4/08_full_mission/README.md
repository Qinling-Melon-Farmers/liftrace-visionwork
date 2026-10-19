# 2026-10-16 当前现场（下方10月15日及更早段落为历史）

最新实测搜索区X[-1.34,6.56]、Y[-4.85,4.85]；走廊内表面X6.76/8.26，中线7.51，入口前(6.10,4.20)，横墙Y1.05/-1.91，H(7.51,-3.85)。六个走廊点AGL0.9，H捕获1.1。搜索默认原航点逐点不变；低位后移1米仅为另存的shiftback可选文件，默认不启用；低位X6.2距近壁仅0.36m，FOV覆盖不代表净空可达。

- 08默认：`08_full_mission/site_waypoint_map_20261016.yaml`。
- 10牛耕默认：`10_low_coverage_full/site_lawnmower_20261016.yaml`，显式保留原六点。
- 10回字：`10_low_coverage_full/site_nested_rectangles_20261016.yaml`，显式保留原外框与内框，原11点不变；CLI使用`--coverage-pattern nested_rectangles`自动选择。

以上路径相对`deployment/board_trials_4x4/`。工作台须使用同步后的三文件选择；旧10月15日文件仅保留为历史，显式传旧文件仍会覆盖新默认，勿继续使用旧手动覆盖。正赛1.2m/s、1.0m/s²和全部动作参数不变。见`docs/deployment/field_geometry_20261016/README.md`。本次纯配置验证，不启动ROS或飞机。

---

> 2026-10-15最新：08高位、10牛耕/回字形默认 competition 档，规划速度1.2m/s、加速度1.0m/s²，巡航前视1.0m、其他导航前视0.6m。显式 --speed-profile limited 仍覆盖默认。两份登记现场文件 H捕获均为FC1.1m，转场保持1.0m；下方旧0.9m捕获/默认低速表述仅为历史。无实飞验收。

# 2026-10-15 当前真实场地图默认入口

默认现场文件为 [site_waypoint_map_20261015.yaml](site_waypoint_map_20261015.yaml)，替代旧10/07有限场地默认选择。无需手填或勾选实测航点；无显式 --site-config 时自动加载，文件缺失报错，不回退旧小场地。显式 --site-config 优先；工作台若继续显式传旧路径会覆盖此默认，须同步切换到本文件。

FC高位2.6m、max_agl 3.2m、rectangle闭合四边形 (1,-3.5)→(5.5,-3.5)→(5.5,3.5)→(1,3.5)→(1,-3.5)。低位1.4m，投递继承08的0.25m，H捕获按最新指令1.1m。搜索范围保留原map search_center_bounds X[-0.1,7.05]/Y[-4.35,4.35]；10另用coverage_bounds，未混用。走廊、H和其他几何复制10已登记用户确认地图。靶区不包含走廊，完整任务校验搜索区位于靶区内、飞行区仍包含所有航点。

纯离线检查（不启动ROS）：

```bash
python deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py full_mission preview --root "$PWD" --check-config
```

本次仅离线配置验证，未上板或实飞。以下10/07说明为历史参考，不代表当前默认配置。

# 2026-10-08 昨日现场几何与显式正赛速度

工作台第08组固定选择 [site_20261007_221730.yaml](site_20261007_221730.yaml)，来源为
`试飞产物/board_full_mission_20261007_221730/run_metadata.json` 的 `settings`，
不是旧 namedsite。它保留昨日现场搜索四边形、七个走廊输入航点、门墙平面、H(7.7,-3.45)
和 FC 离地高位2m；**昨日现场不是10×10正赛场地**。正赛速度版是同一现场几何的新配置，
未实飞验收。速度与加速度是规划上限，没有此新版本的实测速度。
速度版制动模型与F正式策略相同：速度1.2m/s、减速度假设0.6m/s²，
不把规划加速度上限1.0m/s²当作制动能力。试飞生成参数显式关闭导航恢复四端及地图恢复层。

| 配置量 | 有限空间版 | 显式 `--speed-profile competition` |
|---|---:|---:|
| 规划 max_vel（m/s） | 0.5 | 1.2 |
| 规划 max_acc（m/s²） | 0.35 | 1.0 |
| 搜索/接近/边界前视（m） | 0.5/0.4/0.2 | 1.0/0.4/0.2 |
| 初始化 controller距离/轨迹前视/规划启动阈值（m） | 0.25/0.25/0.75 | 0.4/0.4/1.2 |
| 走廊空旷/门前或近H前视（m） | 0.6/0.4 | 0.6/0.4 |
| 投递 FC AGL（m） | 0.35 | 0.35 |

两档的运动优化与高位续扫均开启；有限空间版保留原规划上限，不是旧二进制复现。
前视/距离阈值单位是m，不能当作实测m/s。04和08有实测墙面时生成0.6/0.4切换；
没有墙面坐标时保留航点，走廊按0.4m回退，不猜墙面。运动优化按原逻辑合并共线中继点，
两档使用相同几何输入与生成路线，速度选择不改搜索图案或航点。

速度仅从 [competition_speed.yaml](competition_speed.yaml) 覆盖规划、前视、投递高度与两个开关。
H观察/转场高度取本组 `settings.yaml`，不继承历史 validated 中的旧H门槛；
今天的 POSCTL 参数为目标/触发0.35/0.37m、水平/垂直速度阈值0.08/0.10m/s、稳定0.15s。

离线检查通过真实CLI，下面命令**不启动任何ROS节点**：

```bash
bash deployment/board_trials_4x4/08_full_mission/start.sh preview \
  --site-config deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml \
  --speed-profile competition --check-config
```

需要预览具体生成YAML时，在同一命令附加
`--generate-config /tmp/08-config-preview --reference-fc 0 0 0.22`。
该参考只用于离线预览；现场入口仍测量静止FC参考。普通 `preview` 不带
`--check-config` 会启动应用链，不能拿来执行纯配置检查。工作台下拉选择会真实传入
`--speed-profile competition`，配置检查显示有效规划、前视、走廊与投递值。

---

# 08_full_mission

整任务：高位搜索、低位重访三投、实测走廊航点自主避障、H 视觉降落。corridor_waypoints 和 landing_xy 由上述默认登记现场文件提供。不能把仿真航点直接用于现场。

共同参数、坐标系、阈值和启动步骤见 [八组说明](../MODULES.md)。

先运行 `bash start.sh preview --model /实际路径/model.rknn`；检查通过后使用 `bash start.sh flight --model /实际路径/model.rknn`，默认模拟投递。任务仍需现场手动启动。已知外参来自共用 known_rig.yaml，地面静置自动建立高度参考。

每轮保留下视原始/视觉叠加视频、候选/坐标/许可事件和轨迹，结束状态按实际提交槽位数与落地状态判断。

### 2026-10-06 H成功结果推广（本地，待上板）

现场H成功由用户确认。03/04/08硬件使用形态检测及视觉对准下降，末段POSCTL交接飞手完成降落；不再按本页早期AUTO.LAND描述启动。仿真生成入口显式AUTO.LAND。各组原观察高度保持：03为FC离地1.2m，04/08为1.8m。04/08可在工作台填实测走廊/H/墙面坐标，须先配置检查再预览。参见[推广与九组参数报告](../../../docs/deployment/h_promotion_20261006/REPORT.md)。

### 2026-10-06 后续部署状态

上述H交接推广现已部署192.168.3.126原0928工程并通过配置检查，未启动新试飞。工作台已增加搜索范围自动航线与FOV图，04走廊点及08走廊/H仍需实测。详见[本次部署报告](../../../docs/deployment/survey_workbench_20261006/REPORT.md)。

可选 OFFBOARD mode4 候选：

用 `--landing-config deployment/competition/candidates/h_offboard_mode4.override.yaml` 叠加原实测 `--site-config`，仅改变H末段：冻结XY＋负VZ、关闭Z位置控制，新鲜真实落地后普通disarm。默认 mode1/POSCTL、投递/路线配置和原low/high对准保持。[参数与离线调用](../../competition/candidates/README.md)；本轮未部署、未动态验收。

### 普通航点到达时间开关

`settings.yaml` 的 `ordinary_waypoint_dwell_enabled` 默认 `false`：固定普通航点在原位置容差内即可推进，不额外等待；设为 `true` 后要求位置连续达标 0.3 秒。两种设置都不检查到达速度，且不改变起飞、动态重访、投递 APPROACH、视觉对准、恢复及 H 降落交接条件。新设置在下次启动生成配置后生效。

可在原第08组启动命令后追加 `--ordinary-waypoint-dwell on` 或 `--ordinary-waypoint-dwell off` 覆盖 YAML；有限空间和正赛速度档均支持。此开关只取消到达计时，不取消位置、定位新鲜度、规划身份或动作超时检查。
