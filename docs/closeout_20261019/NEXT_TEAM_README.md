# 下一届上手指南：2026无人机竞赛完整工程

> 当前分发的是机载运行工程包，具体清单与直接上手步骤以[ONBOARD_QUICKSTART.md](ONBOARD_QUICKSTART.md)为准。下面涉及历史开发、训练数据和Git恢复的部分是扩展资料索引，不要求随飞机运行包携带；历史报告可在完整源码仓中查阅。

交接日期：2026-10-19。本文面向第一次接手的成员，按“收到什么→搭环境→构建→核对模型和实物→理解运行入口→排错”组织。它随完整源码及结项分发包发布。文中仓库相对路径均以最终工程根目录为基准。

## 1. 你接手的是什么

这是一套ROS Noetic无人机自主搜索、识别、投递和H降落交接工程，包含导航、控制、视觉、执行机构源码、运行配置及全部小工具。视觉实时板端主路径是OrangePi 5 Plus/RK3588上的RKNN/NPU，笔记本用于开发、训练、分析和仿真。Git源码、权重数据包、硬件软件环境三者合起来才是完整交接，不是复制一个Git目录即可运行。

2026-10-19最新事实：main已补入导航来源最新04/11入口和CAPTURE改类退出补丁；试飞与独立正赛环境脚本默认选择v8模型及配套metadata。CAPTURE修复已有板端部署记录，尚无修复后整场实飞验收。竞赛结束不代表全部研究通过，旧失败和人工接管记录仍应保留。

先读这些入口：

1. `AGENTS.md`、`docs/closeout_20261019/README.md`：操作边界和结项范围。
2. `docs/closeout_20261019/SOURCES.json`：导航/视觉来源；导航取自`liftrace_board_trial_20260928@ed1289be`，不是导航main。
3. `docs/closeout_20261019/TOOLS.md`、`docs/closeout_20261019/TECHNICAL_HANDOVER.md`：资料和工具索引。历史运行事实保留原日期；原始bag已清理，以最终清理记录为准。
4. `VISION_2026_ROADMAP.md`：任务优先级唯一来源；较早“当前”段落不能覆盖后续日期报告。
5. `docs/competition/RULES_20260906.md`：当年规则；下一届必须重新核对当届规则，不能直接沿用得分和尺寸。

当前完整源码根为`/home/xhj/liftrace`，Git权威仓为`Qinling-Melon-Farmers/liftrace-visionwork`。工作树是开发/历史现场资料位置，不应成为新机器运行的隐式依赖。

## 2. 收件清单与不能恢复的资料

完整接收应有：源码与Git历史/结项tag、v8权重及metadata、可移植训练数据、导航控制和舵机实体源码、全部工具源码、现场配置与测量、环境/驱动版本记录、分析报告和已经生成的回放。不是只收Windows工作台ZIP或只有best.pt。

本机外置交付总目录：

`/home/xhj/liftrace-deliverables/competition_closeout_20261019/`

模型数据最终分发包：

`models_dataset/flight_5cls_v8_20261011_models_dataset_20261019_portable.zip`

完整绝对路径为`/home/xhj/liftrace-deliverables/competition_closeout_20261019/models_dataset/flight_5cls_v8_20261011_models_dataset_20261019_portable.zip`。同目录`README_交付入口.md`和`ZIP_VALIDATION.json`记录最终选择：5317成员、1227639762字节、已解引用软链接、2313/203/67图像与标签配对。不要分发不带`_portable`的初版检查包；`package/`只是小文件工作副本，不含完整图像标签，不能替代ZIP。

包内`dataset`、`models`、`evidence`、`training`分别是可移植数据、模型导出、报告和训练恢复资源；具体相对文件名以随包中文README与VERSION_SELECTION为准。原data.yaml/recipe保留旧绝对路径用于追溯，新机使用配套可移植配置；本指南不提供自动训练命令。

**原始bag已经按用户授权删除121份。** 记录在外置总目录`bag_deletion_manifest.json`，状态completed，deleted_count=121。报告、参数、图表和已生成回放保留；旧文档提到的bag路径如今可能不存在。现有视频/CSV/JSON不足以重新构造全部ROS消息、原始点云/图像及精确时间，不能从回放反向恢复raw bag，不能承诺重跑原包提取或补算未保留的话题。需要新原始数据必须重新采集，或由持有独立副本的人另行提供。

本机历史分析和回放主要位于`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/`，外置复盘亦在`/home/xhj/liftrace-deliverables/`各日期目录。交给下一届时应连同实际资源一起传递；报告相对链接存在并不证明原bag存在。

## 3. 源码地图：问题该去哪里找

| 目录 | 职责 |
| --- | --- |
| `vision_ws/src/uav_vision/` | 检测、几何精修、投影、目标记忆、对准、兼容话题 |
| `vision_ws/src/uav_high_view/` | 高位观测/线索支持，和任务侧高低位策略配套 |
| `vision_ws/src/uav_vision_eval/` | 仿真真值、评分、评测；不是实机控制输入 |
| `vision_ws/src/camera_sdk/` | 相机输入与标定；`param/`保存相机参数 |
| `patrol_uav_ws-patrol_planner/src/uav_mission/` | Mission Manager、Planner Bridge、任务/事务许可、配置生成和supervisor |
| `patrol_uav_ws-patrol_planner/src/patrol_control/` | 控制执行、目标跟随、投递与H下降/模式交接 |
| `patrol_uav_ws-patrol_planner/src/Fast-Planner/` | 在线地图、搜索、优化、轨迹及执行支持 |
| `patrol_uav_ws-patrol_planner/src/FAST_LIO/`、`src/FreeDOM/` | 雷达惯性定位与静态点云处理 |
| `patrol_uav_ws-patrol_planner/src/tool/livox_ros_driver2/` | MID360 ROS1驱动；本地适配见`INTEGRATION.md` |
| `patrol_uav_ws-patrol_planner/src/actuator_pwm/` | 当前OrangePi 5 Plus舵机实体实现，不只是服务占位 |
| `deployment/competition/` | 无需测试组的独立正赛入口、环境与模板 |
| `deployment/board_trials_4x4/` | 各专项试飞入口、现场YAML及共用runner；名字4x4不表示全部场地仍为4×4 |
| `deployment/low_hover_observation/` | 低空观察与模块11复用的状态机/runner |
| `top_level_scripts/` | 构建、仿真包装、包生成等工程脚本 |
| `runtime_models/` | 外置权重安装落点；Git克隆本身不保证已带权重 |
| `tools/` | 工作台、离线回放、ULog、模型导出、阈值/障碍柱/舵机工具 |
| `docs/` | 接口、环境、现场报告、验证及结项记录 |
| `legacy_baseline/`和旧工作区 | 历史保护资产，只用于对照，不当作当前生产配置 |

主链：MID360/IMU→FAST_LIO→FreeDOM→Fast-Planner→任务Bridge/控制器→MAVROS/PX4；下视相机→RKNN检测/几何/投影/记忆/对准→任务许可→舵机代理→raw执行机构。目标检测、视觉对准、任务释放许可和机构ACK是四种不同状态。

## 4. 两种计算机，三种Python用途

| 环境 | 用途与已知基线 |
| --- | --- |
| Windows 11宿主 | 编辑器、浏览器、原生工作台；不是ROS/ARM运行环境 |
| WSL2 Ubuntu20.04 x86_64 | ROS Noetic开发、GCC9/CMake/Catkin、离线分析/训练；Gazebo11和PX4为独立仿真依赖 |
| OrangePi 5 Plus RK3588 aarch64 | 实机ROS、MID360/相机、RKNN/NPU、导航/控制和舵机；必须按真实板型/内核匹配驱动 |

宿主PowerShell命令使用`login:false`执行约定；进入WSL统一`wsl -e bash -c '...'`，不要用宿主bash代替。带中文路径或内容的操作先写UTF-8脚本/文件再在WSL消费，避免Windows→WSL参数编码损坏。

非ROS Python分析/训练/转换使用已有conda `rl_drone`。原交接环境为Python3.9、PyTorch2.5.1+cu121、Ultralytics8.4.33、OpenCV4.13、NumPy2.0.2，版本来源见训练导出`export.json`及AGENTS环境表；不是ARM系统Python的安装清单。系统`/usr/bin/python3`用于ROS和板端既有RKNN环境，禁止把conda的cv2/NumPy与系统cv_bridge混载，也不在系统Python临时pip装训练包。

原生Windows工作台要求Python3.10+，包不内置Python，不是独立exe。依赖以`tools/flight_workbench/requirements.txt`为准：PyYAML/pexpect、Windows Paramiko，以及离线回放的NumPy/Matplotlib/rosbags/SciPy/OpenCV；FFmpeg/ffprobe另外提供。安装应在接收方受管理环境中完成，不把WSL conda目录或Linux二进制复制给Windows。

## 5. ARM依赖和版本来源

### ROS/C++与传感器

目标需ROS Noetic、MAVROS、roscpp/rospy、消息生成、TF2、image_transport/cv_bridge及工程package.xml列出的依赖；C++需要编译器、CMake、Eigen3、PCL、OpenCV开发库、Boost、yaml-cpp、nlopt/nlopt_cxx等。相机UVC/SDK与串口权限按当前设备和`docs/HARDWARE.md`、`docs/CAMERA_AND_FLIGHT.md`确认。

接收者先按目标OS发行版准备ROS及系统开发依赖，再构建源码。不把2026安装命令或版本当永久可用的软件源；本文不提供自动apt/pip升级、不改设备系统。

### Livox SDK2与driver2

参考`docs/deployment/LIVOX_DRIVER2.md`及`patrol_uav_ws-patrol_planner/src/tool/livox_ros_driver2/INTEGRATION.md`。后者记录driver2上游revision `6b9356cadf77084619ba406e6a0eb41163b08039`，本地默认ROS1并显式查找SDK2。官方来源为Livox-SDK/Livox-SDK2与Livox-SDK/livox_ros_driver2；这是仓内记载的来源，不是本次重新联网核验。

SDK2需要目标架构的头文件和库，不能拷x86静态库到ARM。现有资料没有给出可确认的统一SDK2安装revision，不应编造版本；下一届应记录实际安装源commit、构建选项和库落点。默认查找`/usr/local/include/livox_lidar_api.h`和SDK2静态库，非标准路径通过`LIVOX_LIDAR_SDK_INCLUDE_DIR`/`LIVOX_LIDAR_SDK_LIBRARY`传给CMake。

SDK2安装按其匹配版本说明先在aarch64原生构建并由维护者安装到目标前缀；驱动本身走工程统一构建，**不要在集成工作区运行上游driver2的build.sh**。硬件CustomMsg与Gazebo PointCloud2是不同输入，参数权威为`mid360_hardware.yaml`及对应launch，不以改成别的lidar_type来掩盖消息不匹配。

### RKNN/NPU

模型导出`export.json`记录Toolkit2 **2.3.2**。10月8日已归档板端日志记录librknnrt **1.6.0**、驱动 **0.9.8**，模型load/init/单合成帧推理成功，同时有toolkit/runtime版本不匹配警告。这是特定旧板端记录，不是建议新机器必须安装该组合，更不是当前内核和v8整机兼容性保证；日志未给出可确认的RKNNLite Python包版本，不能以librknnrt版本代替它。

新板应从对应OrangePi镜像/厂商内核和Rockchip RKNN工具链发行说明配套获得NPU驱动、aarch64 librknnrt和RKNNLite2，保留实际版本与来源。不要只替换Python wheel而忽略系统动态库/内核驱动，不把x86 Toolkit模拟器当RK3588硬件运行环境。受控真实NPU检查留给后续另行授权；本指南无推理启动命令。

## 6. 先安装依赖，再在目标机原生构建

源码移植不携带可当作通用产物使用的x86 `build/`、`devel/`、`install/`。它们含架构相关二进制、CMake缓存及旧绝对路径；ARM必须新建自身产物。即使同为x86的新WSL目录，旧overlay也可能指向原工作树。重建不意味着删除任何原始工程资产；如收到历史产物，先保留记录，用新源码副本构建。

统一脚本只编译，不启动ROS节点或PWM。依赖准备完成后，在工程根执行：

```bash
BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
```

它先编译`vision_ws`，再用其overlay编译导航工作区，明确ROS系统Python，FAST_LIO匹配线程默认由脚本设置。BUILD_JOBS用于控制编译资源，不是飞行性能参数。ARM内存不足先减并行数，不复用x86 build。

Windows宿主上的同一步只用于构建WSL x86工程：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace && BUILD_JOBS=2 bash top_level_scripts/build_competition.sh'
```

这条成功不等于板端ARM编译成功。板端需要在其真实工程根再次原生构建。任何环境source错误或Wrong overlay提示先修好，不能用手写整条ROS_PACKAGE_PATH强行遮盖。

当前实体舵机由统一Catkin树编译；已有正确架构/路径的导航构建目录中，可单独编译目标：

```bash
cmake --build patrol_uav_ws-patrol_planner/build --target pwm_node1 -j2
```

编译pwm_node1不会执行它。不要把文档中旧独立候选的build目录或另板型源码当当前目标。

## 7. 安装默认v8模型时，metadata必须配对

当前main两份环境脚本均选择以下工程内路径：

- 权重：`runtime_models/flight_5cls_new_video_v8_20261011/flight_5cls_new_video_v8_20261011_fp16.rknn`
- metadata：`vision_ws/src/uav_vision/config/flight_5cls_new_video_v8_20261011_metadata.yaml`

将外置模型包的同源RKNN放到权重落点，并核对工程配置metadata与该包契约。不要误把包内通用名`metadata.yaml`当默认路径已经自动切换；特殊版本通过所选入口帮助中提供的模型/metadata参数成对指定。`environment.sh`可能重设默认路径，单独提前export未必生效。

| 项目 | v8契约 |
| --- | --- |
| 版本 | `flight_5cls_new_video_v8_20261011`，YOLO11n |
| 类别 | 0 bridge、1 panzer、2 pillbox、3 tent、4 red_cross；无tank和H正类 |
| RKNN内部精度 | FP16非INT8；输入float32不代表内部FP32 |
| 输入 | 640×640、RGB/NHWC、归一化一次；等比例letterbox，边框114 |
| 输出 | [1,9,8400]；xywh加5类概率，无objectness/内置NMS |
| H | 独立landing_detector，不通过给YOLO metadata加H类实现 |

最终PT在旧本机`/home/xhj/liftrace-deliverables/new_video_20261011/final/best.pt`，原同源导出目录`/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/`。下一届优先使用可移植交付包而非依赖这些旧主机绝对路径。

导出metadata保留candidate/hardware_validated旧字段；后续运行归档确认实际用过v8，不代表严密五类精度、10min稳定性、带载整场已全部通过。检查实际版本优先看当轮run_metadata顶层model_path/metadata_path和启动记录，不用嵌套的旧0928部署manifest。回滚必须同时回滚模型、metadata和相应选择配置。

## 8. 先量实物，再填actual config

不存在一张可无条件照抄的“全工程参数表”。实际配置由入口settings、选定现场YAML、CLI覆盖、生成结果及对应二进制共同决定。优先保存/读取本轮`run_metadata.json`、`runtime.yaml`、`control.yaml`、overrides、ground_reference；有些目录名称是run生成物而非仓库常驻文件。

### 坐标与场地

2026-10-16实测场地约定为起飞H原点、+X朝走廊、+Y向左、单位米。对应配置：

- `deployment/board_trials_4x4/04_corridor_landing/site_waypoint_map_20261016.yaml`
- `deployment/board_trials_4x4/08_full_mission/site_waypoint_map_20261016.yaml`
- `deployment/board_trials_4x4/10_low_coverage_full/site_lawnmower_20261016.yaml`
- 同目录`site_nested_rectangles_20261016.yaml`及两份`*_shiftback_1m_20261016.yaml`。

这些只对应旧现场。新场地必须重新测起飞点、航向、墙面、门前后净空、障碍、H和边界；不是只改地图图片。门墙坐标不是实际开口中心；航点在范围内也不代表整段轨迹被全程硬围栏约束。后移1米是选项而非默认。牛耕端点后改已在当前YAML内，旧报告表格不应覆盖actual config。

独立正赛模板位于`deployment/competition/field.example.yaml`和`candidates/`，未填门/H或site_confirmed=false时不作为可飞现场。工作台手动地图另生成覆盖文件，核对来源工程/组/文件路径，不能把旧草稿自动带到新场地。

### FC AGL、相机与TF

FC AGL是飞控中心相对地面的高度，不是local Z，也不是镜头高度。关系为`FC_AGL = FC_local_Z - ground_z`；ground_z依据本轮静置参考和实测FC离支撑面高度计算。镜头高度由完整安装外参计算，机体倾斜时不能只作固定Z相减。

参考机架有历史0.16m相机下移等测量，但换机架、支腿或安装位置就必须重测。当前值查实际rig/TF配置、相机`param/`和生成control，不把旧`ground_z=-0.22`作为固定常量。相机分辨率、K/D和frame需同源；旋转、裁切、缩放后要更新标定和投影契约。ROS frame map/camera_init、MAVROS ENU及raw控制变换不可混用。

### 槽位与机构

当前约定req=1后、2右、3左，需与机械实物逐槽确认。测量的是FC中心到投口的机体系三维杆臂；补偿关系为`FC目标 = 靶心 - R(机体到任务系) × FC到投口杆臂`，不是固定在地图X/Y直接加偏移。换槽或重装后重新测量，不能直接复制历史12cm。

机械参数权威：

- `patrol_uav_ws-patrol_planner/src/actuator_pwm/include/actuator_pwm/SlotCalibration.h`
- 同包`src/pwm_node1.cpp`、`src/PWMController.cpp`和`launch/`
- 同包`README.md`（含日期修订，历史段落按日期阅读）。

当前包按OrangePi 5 Plus适配；旧OrangePi 5、D2/D3候选和历史引脚表不是通用接线指南。这里不复制针脚或脉宽数字：实物板型、物理针脚/排线、设备树overlay、PWM硬件地址、权限、电源/共地与最终行程必须由现场负责人按当前源码和硬件资料逐项核对。pwmchip编号会变化，不能写死枚举号。

`initialize_on_startup=false`是被动检查设计；显式true会复位机构。`init_pwm.sh`会写配置/PWM，不是检查命令。正常`/Servo`由许可代理使用，raw `/legacy/Servo_raw`为底层执行机构；同类型mock服务不能凭服务类型自动判定真实后端。硬件锁被占用先查现有实例，不启动第二份驱动。PWM ACK只证明软件/电气调用，不证明载荷真的松脱或落点正确。

## 9. 只做帮助与配置检查：新人第一轮命令

以下命令不应启动飞行或机构；依赖已安装、构建已完成时，在工程根使用。`--check-config`不可省略。help/check-config仍可能需要读取overlay和Python依赖，失败应按错误修复，不改成flight来“试一下”。

```bash
bash deployment/competition/start.sh --help
bash deployment/competition/start.sh preview \
  --site-config deployment/competition/field.example.yaml --check-config

bash deployment/board_trials_4x4/04_corridor_landing/start.sh preview --check-config
bash deployment/board_trials_4x4/08_full_mission/start.sh preview --check-config
bash deployment/board_trials_4x4/10_low_coverage_full/start.sh preview --check-config
bash deployment/board_trials_4x4/11_low_release_test/start.sh preview --slot 1 --check-config
```

正赛示例模板未确认可能只给预览或拒绝生成，按实际输出解释；不能为拿到CONFIG_VALID随便改site_confirmed。配置通过只说明静态参数/接线校验通过，不证明路径上的权重已能NPU加载、相机/雷达在线、固件匹配、实际可达或可飞。

**preview这个词本身不是统一的“零动作”承诺。** 一般试飞preview可能建立ROS、定位、相机等准备链，纯离线必须带check-config。模块11目前preview分支专门做离线处理，但仍建议统一保留check-config避免误用其他入口。flight即使释放mock，也可发送真实飞行目标；模块11不要求靶标且走独立实验释放条件，绝不可当正常许可链的快捷替代。

本指南不提供roslaunch、flight、real-release、舵机初始化、解锁、接管或飞控参数写入命令。后续实机运行按现场负责人批准的分级流程和对应入口操作说明进行；仿真也须有当轮明确授权并用统一sim_run包装，不能从旧README裸启动。

## 10. H降落：模式名和能力边界

H检测、H高位确认、下降方式、模式交接、触地和上锁是不同阶段。`landing_alignment_mode`的vh/non_vh与`landing_descent_mode`、`landing_handoff_mode`是独立开关；投递drop_descent_mode和H模式也不能互换解释。

当前配置查03/04/08的settings、所选site YAML、共用run_trial生成器及生成控制配置。03会继承08部分landing策略，不能只改03的一份镜像字段就认定最终生效。现场YAML可覆盖捕获高度，settings静态值不是完整答案。所谓max_vertical_speed等字段在某些路径仅保留兼容含义，需按生产控制逻辑而非字段名称推断是否执行。

- 已采用的POSCTL路径：先OFFBOARD视觉对准/下降，再请求POSCTL，由飞手完成末段。POSCTL不等于AUTO.LAND，也不等于无人干预自主落地得分通过；交接后不继续消费H视觉目标。
- AUTO.LAND：历史SITL/旧配置中存在，依赖对应PX4逻辑和版本；不能因为旧seed成功就自动给当前硬件启用。
- H OFFBOARD模式4：说明在`docs/verification/h_offboard_touchdown_20261012/README.md`，候选覆盖文件`deployment/competition/candidates/h_offboard_mode4.override.yaml`。固定H XY、竖直速度下降、触地确认后普通上锁是候选设计，板端固件和触地参数、实物末段仍需验收，不作为默认保证。
- 坐标H兜底：是否启用及等待条件查`landing_coordinate_fallback`和生产实现；它用实测预设H位置，不是无视觉时任意下降许可。

不要用强制上锁补救不明触地状态，不把模式回执当触地成功。旧落点偏差报告只能代表对应机架、参数和现场。

## 11. 所有工具都在源码中，按用途选择

| 工具 | 说明入口与限制 |
| --- | --- |
| 飞行工作台 | `tools/flight_workbench/README.md`、`README_DISTRIBUTION.md`、`WAYPOINT_EDITOR.md`；Windows入口start_windows.cmd，Linux入口start_workbench.sh；本指南不启动服务 |
| 离线bag回放 | `tools/bag_replay/README.md`；只读记录、不重推理；已删bag无法重新生成 |
| 分包合并 | `tools/bag_replay/merge_bags.py`；需要仍然存在的输入bag；现有报告不是替代输入 |
| Windows原生回放 | 工作台`offline_replay/`，依赖rosbags/OpenCV/SciPy/FFmpeg；过去Windows缺依赖时未完成新bag生成，不把UI成功当全能力通过 |
| ULog | `tools/flight_logs/README.md`及`ulg_motor_viewer/`；读取已有ULog，不能凭ROS视频补出飞控ULog |
| 模型发布 | `tools/model_release/README.md`、`README_candidate_zh.md`；转换/包验证，输出精度与版本必须如实记录 |
| 阈值回放 | `tools/threshold_replay/README.md`；用于离线分析，不能把调参后同数据表现当独立验收 |
| 障碍柱 | `tools/obstacle_columns/README.md`；可视化/诊断不等于安全脱困策略通过 |
| 舵机诊断 | `tools/servo_diagnostics/README.md`；部分会复位/驱动真实机构，绝不是普通离线工具 |

工作台发行目录`/home/xhj/liftrace-deliverables/workbench_distribution_20261016/`保留routes_release/latest ZIP；Git源码和结项发行更新优先于旧ZIP。工作台ZIP不自动安装ARM代码、模型、PWM驱动或设备环境。换电脑要明确root、环境、板端IP和资料库路径；历史Wi-Fi IP不是设备固定地址。

保留的replay/index.html或analysis/index.html应连同同目录视频和资源搬迁。曲线是已有定位/记录结果，不是外部测量真值；检测框按时间匹配回贴，视频闪框不必然等于在线掉检，离线回贴先后也不能直接量在线延迟。

## 12. 常见问题先查什么

| 症状 | 优先检查 | 避免的误处理 |
| --- | --- | --- |
| 构建缺SDK2或链接架构错误 | SDK2头/库路径、ARM原生安装、driver2 INTEGRATION、CMake缓存架构 | 拷x86库或运行上游build.sh覆盖工程 |
| Wrong package overlay | 当前root、两套workspace环境、rospack解析报错和旧终端环境 | 手写全ROS_PACKAGE_PATH拼顺序 |
| cv_bridge/NumPy/cv2导入失败 | ROS系统Python与conda依赖边界、PYTHONPATH混载 | 系统pip升级全部包 |
| RKNN加载失败/空输出 | 实际模型路径、同源metadata、目标rk3588、驱动/runtime/SDK版本、输入输出契约 | 只改names伪装兼容或把模拟器PASS当硬件PASS |
| 有画面却检测慢/持续陈旧 | 原始图像时间戳、queue/latest-frame行为、相机FPS、全机CPU/NPU负载、记录吞吐 | 把消息时间戳改处理完成时间、放宽时效掩盖积压 |
| 雷达无数据 | MID360实际JSON、专用网卡/IP、设备连通、driver2消息类型及topic | 把雷达专网IP改成Wi-Fi地址或多开驱动 |
| LIO/EV中断/ABORT | 当轮自报年龄、队列、实际二进制、线程/亲和性、全机负载和位姿跳变 | 把检测器绑核日志当LIO证据，直接启用冻结EV预测 |
| 规划start_occ=1 | 同高度占用、柱化/额外膨胀与实体区别、当前点/目标点、场地和时间 | 清图/删实体/强制向下当万能脱困 |
| CAPTURE等旧类别 | 同id改类证据、冻结/ALIGN/机构调用阶段、最新补丁和实际运行二进制 | 改掉已执行事务类别或重复投递 |
| 舵机服务不存在/锁占用 | 当前真实pwm节点、被动检查错误、板型/overlay/权限、是否第二实例 | 绕锁、自动初始化或不明raw服务直接试投 |
| 只有PWM ACK却未投出 | 机构物理行程、供电、线缆、真实松脱时刻、载荷 | 用ACK/相机估计宣称落点达标 |
| H不下降或不交接 | 实际模式组合、有效H年龄/连续帧、ground_reference、生成参数和控制保护 | 混用drop与landing参数或先强制上锁 |
| 回放报输入不存在 | bag删除清单与资源现状；优先看保留播放器和分析 | 承诺从视频重建原始bag |

## 13. 下一届继续前保留的验收边界

历史固定seed11完整37/37和随机矩阵是SITL；特定现场08链路成功含实际POSCTL/飞手末段边界。最新低位首投ACK、高位规划占用接管、v8高置信错类、LIO负载波动、真实落点/松脱真值缺口、H模式4和三类恢复候选，均不能因main结项而写成“全部解决”。

CAPTURE最小修复已集成并有板端部署及离线测试，修复后实飞仍缺。解锁后建图/人员撤场一次启动只是讨论与时序核查，未实现。EV预测保持冻结；全机十分钟性能、真实H负样本和独立五类数据仍需单独验证。

下一届换规则、机架、固件、传感器、相机分辨率或场地后，要建立自己的实际配置、构建和分级验收记录。保留原结果及来源，不将旧参数、模拟成绩或软件回执冒充新机实测。

本文命令仅为帮助、check-config和构建；本次撰写没有安装依赖、启动设备/仿真、驱动机构或修改共享仓库。