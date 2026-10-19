# 2026 RoboCup 无人机竞赛工程结项技术交接明细

日期：2026-10-19。用途：交接工程组成、历史运行事实及遗留问题。最终源码来源和本轮验证见SOURCES.json、VALIDATION.md；本次未重新部署或实飞。

## 1. 结项范围与事实口径

工程结项是源码、文档、模型、数据和恢复入口的交接，不代表全部实验通过。应分别记录“已实现”“离线检查通过”“已部署”“实际运行过”“整场验收通过”。一个场地或固定 seed 的成功不能替代其他布局、最新参数、其他二进制或真实载荷落点的验收。

本次只读范围：

- B：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests`，试飞、现场运行、工作台及最近资料来源。
- F：`/home/xhj/liftrace-worktrees/r2026-board-frame-fix`，独立整机/正赛入口与历史集成验证来源。
- D：`/home/xhj/liftrace-deliverables`，外置交付包、训练导出、部署记录。
- 应新增要求，只读核对 `/home/xhj/liftrace/vision_ws/test_data/` 中 v8 数据集及训练产物。

下列B/F标记表示资料来源；仓库相对入口已整合到结项源码。04现场YAML与11板端入口已从导航0928补齐。历史绝对路径用于定位保留资料，不是新机器运行依赖。

板端路径是已有归档中的实际路径，本次没有 SSH 连接或实时读取设备；最新已归档运行状态不能冒充 10 月 19 日设备当前状态。

## 2. 有效文档和操作入口

| 用途 | 有效仓库相对入口 | 来源与阅读边界 |
| --- | --- | --- |
| 项目与任务 | `README.md`、`VISION_2026_ROADMAP.md` | B/F；只在 ROADMAP 维护优先级，但顶部仍可能止于10月15日，应继续读最新现场报告 |
| 环境、编译、接口 | `docs/ENVIRONMENT.md`、`docs/INTERFACES.md`、`docs/CAMERA_AND_FLIGHT.md` | B/F；含历史R64描述，按日期和版本阅读 |
| 部署总览 | `deployment/BOARD_DEPLOYMENT.md`、`deployment/README_ONBOARD.md` | B/F；历史机载包描述不等于目前0928现场文件集合 |
| 正赛独立入口 | `deployment/competition/README.md`、`deployment/competition/start.sh` | B/F；显式选择现场YAML，未填写/未确认模板不能作为可飞配置 |
| 正赛集成说明 | `docs/deployment/competition_integration_20261003/README.md` | F；历史来源和独立运行依赖 |
| 两套现场工程部署 | `docs/deployment/evening_validation_20261008/FIELD_REPORT.md` | B/F；两套工程独立构建，不能因0928更新推定独立正赛更新 |
| 10月10日板端更新 | `docs/deployment/board_update_20261010/README.md` | B；只更新0928，回收现场YOLO执行优化，保留性能结论边界 |
| 10月15日板端更新 | `docs/deployment/board_update_20261015/README.md` | B；保留现场未提交配置、记录实际LIO三线程与大核亲和性 |
| 最新场地/路线 | `docs/deployment/field_geometry_20261016/README.md` | B；正文历史点表与文末后续变更须结合最新实际YAML/运行归档阅读 |
| 最新工作台发行 | `docs/deployment/workbench_distribution_20261016/README.md` | B；Windows启动、10路线选项和11临时单投边界 |
| 地图编辑 | `tools/flight_workbench/WAYPOINT_EDITOR.md`、`docs/deployment/workbench_20261013/README.md` | B；在线板端源文件与离线参考副本分开 |
| 回放 | `tools/bag_replay/README.md`、`tools/flight_workbench/REPLAY_INTEGRATION.md` | B；后者10月10日Windows仅查看说明已被10月13日原生生成能力扩展，不应当作最新唯一能力描述 |
| 模型导出/回滚工具 | `tools/model_release/README.md`、`tools/model_release/README_candidate_zh.md` | B；实际版本以第3节记录为准 |
| 规则 | `docs/competition/RULES_20260906.md` | B/F；正式五静态类，tank/移动靶取消 |
| 仿真固件依赖 | `deployment/px4_patches/README.md` | B/F；仅恢复相应SITL结果时需要，不能直接当现场固件已经匹配 |
| 历史验收 | `docs/VALIDATION.md`、`docs/verification/r64_seed11/REPORT.md`、`docs/verification/r64_matrix/REPORT.md` | B/F；历史固定场景与矩阵结果，不替代10月实机结论 |

`deployment/MODEL_CONTENTS.md` 是有效历史文档，但存在“五类”段落仍列出tank/六类、FP32等混杂旧说明。交接不能复制其错误类别顺序或精度描述。实际五类契约见以下同源metadata及当轮run_metadata。

## 3. 最新训练数据与实际运行模型（单独打包优先项）

### 3.1 最后训练数据集 v8

已存在且实际训练配方引用：

`/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v8_5cls_new_video_20261011`

必须交付完整 `images/`、`labels/`、`data.yaml`、`manifest.json`、`balance_manifest.json`、`review/`。本次只读计数为train 2313、val 203、test 67，共2583张，标签数量逐划分相同；不是2583个独立采集场景。

**关键打包要求：大量成员是软链接，必须解引用，不能只打包链接本身。** 本次检查无失效软链接。train图片1966个、标签1890个是软链接；val图片/标签各203个、test各67个均为软链接。链接实际依赖：

- `/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v6_5cls_flight_h_20260928/`
- `/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v7_5cls_pillbox_20261011/`
- `/home/xhj/liftrace-deliverables/new_video_20261011/review_1fps/frames/`

负责大包的worker应生成自包含的数据副本/归档，并在异地解包后核对图像和标签可读；原data.yaml的`path`是本机绝对路径，随包给出重定位说明，不改原始训练配方或源数据。源视频另存：`/home/xhj/liftrace/vision_ws/test_data/new_real_target.mp4`。

训练语义：0 bridge、1 panzer、2 pillbox、3 tent、4 red_cross；H不是YOLO正类。框为中心黑圈和内部图案，不含整块蓝白靶板。新视频76个保留源帧及其派生仅进train。历史val/test存在同源相关性；test无pillbox/tent真值；H负例为模拟图/纹理，无真实H负例。805帧是反复审阅的留出困难回归集，不能称未触碰盲测。

### 3.2 最终PT、导出与训练恢复文件

| 内容 | 已确认本机准确路径 |
| --- | --- |
| 最终交付PT | `/home/xhj/liftrace-deliverables/new_video_20261011/final/best.pt` |
| 训练原始best.pt | `/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_new_video_20261011/candidate/weights/best.pt` |
| 完整训练目录 | `/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_new_video_20261011/candidate/` |
| 训练起始权重来源 | `/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_pillbox_20261011/candidate/weights/best.pt` |
| 最终报告与配方 | `/home/xhj/liftrace-deliverables/new_video_20261011/final/TRAINING_REPORT.txt`、同目录`recipe.yaml`、`source_statistics.json`、`test_summary.json`、`handoff.json`、`results.csv`、`training_history.json` |
| 完整模型导出目录 | `/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/` |
| RKNN | `/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/flight_5cls_new_video_v8_20261011_fp16.rknn` |
| 同源metadata | `/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/metadata.yaml` |
| 同源PT/ONNX | 上述导出目录下`flight_5cls_new_video_v8_20261011.pt`、`flight_5cls_new_video_v8_20261011.onnx` |
| 导出与本地验证记录 | 同目录`export.json`、`validation.json`、`rknn_simulator.json`、`formal_image_validation.json`、`formal_handoff_validation.json`、`formal_simulator_validation.json`、`PACKAGE_CONTENTS.json` |
| 替换与回滚说明 | `/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/README_替换与回滚.md` |

模型ID为`flight_5cls_new_video_v8_20261011`，YOLO11n，五类，640×640。RKNN实际FP16非INT8；metadata的输入float32不是内部FP32精度声明。图像RGB/NHWC、输入归一化一次，letterbox边框114，输出[1,9,8400]，无objectness/内置NMS，模型和metadata必须成对。

训练完成记录`PT_TRAINING_COMPLETE.txt`明确是临时交接，不是最终评估；结项应使用`final/TRAINING_REPORT.txt`及最终JSON，不能因export.json内保留旧handoff pending段落而忽略最终附件。

### 3.3 实际已部署并运行的是 v8，不是旧0928

10月16日两轮原始运行归档顶层字段直接确认：

- 板端模型：`/home/orangepi/liftrace_board_trials_20260928/runtime_models/flight_5cls_new_video_v8_20261011/flight_5cls_new_video_v8_20261011_fp16.rknn`
- 板端metadata：`/home/orangepi/liftrace_board_trials_20260928/vision_ws/src/uav_vision/config/flight_5cls_new_video_v8_20261011_metadata.yaml`
- 原始依据：`B/试飞产物/board_low_coverage_full_20261016_162041/run_metadata.json`及`B/试飞产物/board_full_mission_20261016_162935/run_metadata.json`的顶层`model_path`、`metadata_path`、`model_metadata.model_file`。
- 交叉报告：`docs/verification/competition_video_20261015/REPORT.md`、`docs/verification/flight_1015_1501_20261015/REPORT.md`、`docs/verification/flight_1015_1656_lio_20261015/REPORT.md`；15:02轮启动日志确认加载。

**不要使用run_metadata内嵌的旧`deployment.model=runtime_models/flight_5cls_20260928_fp16.rknn`判断当轮模型。** 这是10月8日历史部署清单，和顶层实际选择值不同。export目录metadata仍保留`candidate:true`、`hardware_validated:false`，它描述10月11日导出时状态，不能据此否认后来现场运行；也不能因运行过就补写“完整NPU性能/稳定性/精度验收通过”。本次未实时读取板端文件，未做本地与板端逐字节一致性比较。

旧0928回滚模型本机仍在B和F各自`runtime_models/flight_5cls_20260928_fp16.rknn`，各7,545,013字节；对应各自`vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml`。10月8日T/C两根NPU单帧检查见`D/field_deployment_20261008/model_check.json`。独立正赛C当时使用旧0928，后续0928试飞v8运行不能证明C也已换v8。

最终模型数据包为外置交付目录 models_dataset/flight_5cls_v8_20261011_models_dataset_20261019_portable.zip；内部保留版本说明，不能将旧回滚对冒充v8。

## 4. 板端部署与恢复所需配置

历史板端工程根：T=`/home/orangepi/liftrace_board_trials_20260928`，C=`/home/orangepi/liftrace_competition_20261008`。两套各有自身源码、build/devel、模型与环境，不能混用。源码合入main不等于更新T/C。

- 10月8日完整独立构建和软件mock三槽检查：`D/field_deployment_20261008/FIELD_REPORT.md`、`board_checks/`、`packages.json`、`deployed_heads.json`、`model_check.json`。检查没有启动真实相机/雷达/飞行/PWM。
- 10月10日只更新T：`D/board_update_20261010/`，修改前恢复包`predeploy/board_sources_configs_control_artifacts.tar.gz`及`deployment.patch`；保留`vision_return/`现场执行优化来源。
- 10月15日更新T记录HEAD=23100bcb；板端备份`/home/orangepi/deployment_backups/0928_20261015_low_coverage/`。没有更换控制/LIO二进制。
- 10月16日场地配置部署记录HEAD=29ca35d0，来源导航239c63bd；备份`/home/orangepi/deployment_backups/0928_20261016_field/`。记录保留08现场下降速度1.5m/s、释放上界0.45m等未提交设置。文末初始化预算来源f78ca923、模块11来源fec20b54分别记录，不能把某一HEAD说成最后所有文件的统一revision。
- IP随现场网络变化：历史192.168.3.126、10.189.79.232，10月16日发行默认192.168.43.99。这些是历史连接配置，不是本次设备在线判断。

后续恢复必须保留：

1. 最终合并源码与annotated tag、来源提交说明；T/C的实际部署快照/未提交配置diff、每轮生成的runtime/control/overrides和ground_reference、run_metadata、manifest，不仅是Git HEAD。
2. T环境`deployment/site_20260928/environment.sh`、C环境`deployment/competition/environment.sh`，不能只改工作台卡片而不改工程root/环境。
3. 实际所选现场YAML和CLI覆盖。现场部署文档记录的场地入口（不是声称B源码树全部已具备）：`deployment/board_trials_4x4/04_corridor_landing/site_waypoint_map_20261016.yaml`、`08_full_mission/site_waypoint_map_20261016.yaml`、`10_low_coverage_full/site_lawnmower_20261016.yaml`、`site_nested_rectangles_20261016.yaml`及两套`*_shiftback_1m_20261016.yaml`。后移默认关闭；后续左端点Y4.10→3.60更新不能回写旧轮日志。本次存在性检查发现，B/F均没有04的上述10月16日YAML，应从板端备份/导航对应版本或D场地交付副本补齐后核对，不可直接承诺main已有该入口。
4. RKNN和实际metadata成对；`vision_ws/src/uav_vision/config/target_detector_rknn.yaml`及相关阈值/阶段配置；相机`vision_ws/src/camera_sdk/param/`实测标定、安装TF、三槽FC→投口杆臂参数。
5. `deployment/site_20260928/MID360_config.json`、LIO/FreeDOM/Planner实际YAML、Livox SDK2/driver2、板端RKNN runtime和相机SDK等外部环境。雷达专网192.168.1.100/175不可误改成Wi-Fi地址。
6. 当前实际控制二进制及构建说明、执行机构原始实现与服务/独占锁约定。`/legacy/Servo_raw`真实PWM节点需要机械现场配套，旧“机载包不含机械实现”与后来完整现场包范围不同，不能互相套用。
7. 如果要复现旧SITL，另保留对应PX4补丁/revision、AstraDroneOpen底座与项目world/model资产；本交接不启动它们。

FC AGL与local Z不同，镜头约低于FC0.16m；实际ground_reference应按当轮测得，不复用旧local Z。现场生成结果和反馈比旧README默认值更能代表实际运行。

## 5. Windows工作台发行与replay入口

最新已核实发行目录：`/home/xhj/liftrace-deliverables/workbench_distribution_20261016/`。

- `flight_workbench_20261016_routes_release.zip`
- `flight_workbench_20261016_latest.zip`
- `README.md`、`PACKAGE_CHECK.json`

PACKAGE_CHECK记录routes_release包181成员、707773字节、完整性PASS，来源当前运行文件而非旧ZIP；latest包实际含`flight_workbench/start_windows.cmd`、bat/ps1、回放源码与requirements。latest只是发行文件名，后续打包应保留带日期明确包名。本次未新启动工作台或重做浏览器测试。

完整解压后Windows双击`start_windows.cmd`，需要Windows原生Python3.10+及requirements，不内置Python，也不是独立exe；默认8771，端口冲突以终端实际URL为准。Linux/WSL入口`start_workbench.sh`。源码在B的`tools/flight_workbench/`，旧完整现场fixed目录`D/workbench_review_20261015/fixed/flight_workbench/`保留用于来源恢复。

路线/场地外置包：`D/field_geometry_20261016/field_yamls_20261016.zip`；该目录另有历史`flight_workbench_20261016_field.zip`、默认/后移路线YAML、coverage计算脚本与图表。不要把较旧field包或10月15日包当最新routes_release。

工作台模块11：入口T下`deployment/board_trials_4x4/11_low_release_test/start.sh`与`config.yaml`，来源fec20b54，已部署且有板端28项状态机/配置检查；结项源码已从导航0928补齐该模块start.sh，不把Windows包视作板端源码包；**尚未实飞验证**。flight即使mock也会发送飞行目标；preview不等于仅离线检查，纯配置检查必须保留`--check-config`。本手册不执行任何动作。

回放入口：

- 已生成结果直接打开各轮`replay/index.html`或`analysis/index.html`，需保留同目录视频、summary/validation和资源。
- 工作台`/replay`页可从主页或`/logs`进入；默认只扫描指定资料根，外置目录需配置`replay.data_roots`。Windows应使用本机可读路径，不能直接照搬WSL绝对路径。
- 独立离线工具`tools/bag_replay/run.sh`及`bag_replay.py`读取ROS1 bag；Windows发行还含`offline_replay/`和`bag_replay/`。无需运行ROS节点、飞控或检测模型。
- 10月13日已扩展Windows原生生成代码，需rosbags/OpenCV/SciPy/FFmpeg；该轮Windows缺rosbags，未完成新bag生成验收。Windows启动/UI通过不能写成所有新bag原生生成都验证通过。
- 回放按已录消息重建，不是重新推理，不证明修复后的飞行表现；定位轨迹不是外部真值，ACK也不是物理松脱时间。

## 6. 现场报告、原包与大文件准确落点

大文件通常不在Git，以下绝对路径是恢复资料的关键。B下“试飞产物”不在D内，不能只备份deliverables就宣称备齐。

| 内容 | 准确路径/有效报告入口 |
| --- | --- |
| 最近两轮总报告 | B相对`docs/verification/flight_1016_1620_1629/REPORT.md` |
| 10月16日低位原包、参数和回放 | `/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/board_low_coverage_full_20261016_162041/`，报告`analysis/REPORT.md`，回放`replay/index.html` |
| 10月16日高位原active、副本恢复和回放 | `/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/board_full_mission_20261016_162935/`，报告`analysis/REPORT.md`，回放`replay/index.html` |
| 10月15日15:02轮 | B相对`docs/verification/flight_1015_1501_20261015/REPORT.md`；外置`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/flight_20261015_1501_analysis/`及同级`1015-1501-投递H2D2-H降落vhD2.bag` |
| 10月15日16:56 LIO | B相对`docs/verification/flight_1015_1656_lio_20261015/REPORT.md`；外置`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/flight_20261015_1656_lio_analysis/` |
| 10月15日晚投递 | B相对`docs/verification/drop_20261015_evening/REPORT.md`；外置B下`试飞产物/1015-1924_drop_analysis/`、`试飞产物/flight_20261015_1933_analysis/`和同级两份原bag |
| 10月12日双轮复盘 | `/home/xhj/liftrace-deliverables/flight_review_20261012/REPORT.md`，含`full/source_reindexed.bag`及分析/重建限制 |
| 10月16日preview | `/home/xhj/liftrace-deliverables/preview_20261016_090842/REPORT.md`，原bag、supervisor_result/result/lio_summary等同目录 |
| 10月8日完整工程包 | `/home/xhj/liftrace-deliverables/field_deployment_20261008/liftrace_competition_complete_20261008.tar`、`liftrace_0928_update_20261008_field.tar`、各自manifest |
| 板端更新Git恢复包 | `/home/xhj/liftrace-deliverables/board_update_20261015/board_0928_final_20261015.bundle`、`board_0928_merged_20261015.bundle`、`field_settings.patch` |
| 训练已有交接包 | `/home/xhj/liftrace-deliverables/new_video_20261011/new_video_v8_training_handoff_with_newbag_20261011.zip`、`new_video_v8_training_handoff_20261011.zip`；本次未假定已包含完整解引用训练集 |
| 模型已有候选包 | `/home/xhj/liftrace-deliverables/new_video_20261011/flight_5cls_new_video_v8_20261011_candidate.zip`及同名`.tar.gz`；文件名candidate不否认后来的现场部署 |

以下为清理前的记录情况（2026-10-19已按用户要求删除原始bag）：10月16日低位主bag完整，但原始点云分包仍未齐；高位取回104251392字节active文件，对副本恢复出133.606秒/56035条消息，尾部缺失，没有完整结束状态。不得将恢复副本标成全程原始真值。

## 7. 已验证、已部署、实验候选的区别

| 项目 | 可交接的结论 | 不能扩展的结论 |
| --- | --- | --- |
| R64固定seed11 | 历史SITL37/37，422.712秒，0碰撞 | 不等于随机布局和实机鲁棒 |
| R64十seed | 原始7/10完整PASS，旧5/7/8存在布设压墙问题 | 不用后修布局替换原失败，不宣称修后矩阵已跑 |
| 全随机五seed | 历史2/5整场PASS、4/5三投 | 不等于未知门与落地已全部通过 |
| 10月7/8用户现场确认 | 特定有限场地08完整链路及实际H/POSCTL交接方式通过 | 软件JSON可能仍为INCOMPLETE/接管取消；不等于无人干预自主落地得分验收 |
| 10月8日T/C完整部署 | 独立ARM构建、配置和mock软件调用；旧0928NPU单合成帧通过 | 不代表传感器READY或比赛新参数实飞 |
| v8训练/导出 | 最终训练和本地PT/ONNX/Toolkit检查有记录，后续确实上板运行 | 不代表五类全真值指标、RKNN一致性/整机10min时延通过 |
| YOLO执行优化 | 历史短时独立链约13.19→19.67FPS，另约21.78FPS记录 | 前一批/tmp原数据已丢，不等于本轮独立重验或全机并发性能 |
| 低位10与场地新路线 | 入口/配置及部分现场飞行已存在，默认地图更新已部署 | 16:20仅首投软件ACK，后续接管，非整场通过 |
| 三类导航恢复 | 真实节点加合成运动模型定向验证有PASS，默认关闭候选 | 非完整PX4/Gazebo和真实执行链鲁棒验收 |
| H OFFBOARD模式4 | 编译和离线状态机/接口检查通过 | 新触地检测/竖直速度控制未上板实飞验收，POSCTL默认不应被草稿自动改写 |
| 工作台发行 | Windows启动/UI、地图命令、ZIP内容检查有记录 | 不代表板端所有模块自动安装或新bag原生生成全通过 |

## 8. 最近未收尾事项

1. **10月16日CAPTURE同id改类等待。** 原panzer线索后来更正pillbox，捕获仍等旧类，接管前等待约39秒，不足以认定永久死锁。导航ed1289be和B定向修复通过离线/生产方法回放；实际消息重放约t208.026s触发改类退出。**10月16日已部署到0928（板端6f7cc4ea，192项离线通过）；尚无修复后实飞记录**。修复是取消未ALIGN动作并有限重试，不改已冻结/已开始执行事务。另保留导航契约23通过、1项既存可执行位失败。
2. **高位当前位置被柱化/膨胀占用。** 16:29 start_occ=1、goal_occ=0，换目标后仍不能出发；原始支撑究竟实体/误点/残留未定。没有证据证明末段一次规划成功后自动恢复。三类恢复仍默认off，不能因为代码存在就宣布解决。
3. **低位端点可达性与路线后改。** 16:20原Y4.10端点占用，约34秒后耗尽预算；后续内收到3.60已上板，不等于更改后整场通过。后移1m是默认关闭选项，其理论100%覆盖不等于检出率/避障保证。
4. **LIO全机负载/外源中断。** 10月15日16:56自报输出年龄最高691ms、队列5帧、EV缺口4.90秒、跳变触发ABORT；后续已核实真实三线程和大核绑定，不能保证永不积压。10月16日飞行段LIO较好，也不能反推前一轮问题已根治。EV预测保持冻结。
5. **v8高置信错类和数据独立性。** 地堡→装甲车可通过圆环关联；训练困难集中仍有tent→pillbox@0.801。需真实五类独立真值与真实H/黑圈负例；训练报告的历史近满AP不能当比赛泛化精度。最终数据/模型包已制作，详见结项README。
6. **实际投递落点和机构时序。** 10月15日晚调用到ACK约1秒，恢复通知可早于ACK约0.5秒；ACK时飞机已回升。缺松脱传感、同步外部落点/ULog，软件ACK不是实物落点达标。
7. **高度和H末段。** 10月15日轮曾实际用投递0.15m；后续用户要求0.25m，不能把历史0.35/0.40目标统一涂改。H POSCTL为飞手末段，模式4仍候选；板端固件/触地参数未核验，不称自主触地上锁已验收。
8. **数据缺口及运行收尾。** 最新高位尾部缺失、低位点云分包不全；10月15日旧preview主管退出后孤儿节点的历史说明不能当当前已清零。本次未检查/启停设备，恢复运行前由现场维护者确认实际进程。
9. **独立正赛与main最终恢复闭包。** 最新现场T更新不自动进入C；正式门/H模板仍需实际确认。最终来源/验证/合并tag见结项README；没有以本次源码归档替代实机复验。

## 9. 归档说明

本文保留来源盘点时的B/F差别。结项main已经补入导航0928来源的04最新现场YAML与11临时入口，不再存在草稿盘点中的这两个缺文件问题。最新提交、验证和模型包详见本目录README及VALIDATION。本文历史绝对路径指向保留的资料，不代表现场设备实时状态。

原始bag清理说明：表内bag路径仅保留历史出处，2026-10-19已清理121份项目bag/active及重复链接。图表、报告和已生成视频保留；不再承诺按原bag重新提取未保留话题。
