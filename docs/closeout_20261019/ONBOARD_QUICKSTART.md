# 下一届机载工程上手（运行包专用）

本页对应`onboard_runtime_source`分发范围。原始源码与历史在远端main；运行包仅带当前飞机链路及必要工具，包内代码版本见ARCHIVE_MANIFEST.json。

## 1. 文件结构

| 路径 | 内容 |
|---|---|
| patrol_uav_ws-patrol_planner/src | 导航、建图、任务、控制、舵机与ROS依赖源码 |
| vision_ws/src | uav_vision、uav_high_view、camera_sdk与试飞接口软链 |
| runtime_models/flight_5cls_new_video_v8_20261011 | FP16 RKNN、metadata和说明 |
| deployment/competition | 独立正赛入口、环境、场地与候选配置 |
| deployment/board_trials_4x4 | 分模块试飞，03/04/08/10/11等入口 |
| deployment/site_20260928、site_20261006 | 现场基础参数、环境与相机/雷达接线 |
| deployment/low_hover_observation | 低空观察、临时单投共用逻辑 |
| tools | 工作台、replay、ULog、导出与诊断工具 |
| top_level_scripts | 构建/驱动/检查脚本；仿真专用脚本需要另取仿真资产 |

专项配置生成器仍读取`docs/verification/fov_landing_inner_20260919/seed_2672/fast_runtime.yaml`，因此只带这一份实际运行模板，不带该目录的历史报告。

保留当前依赖包内的辅助源码和测试，避免截断CMake引用；这不代表把历史工程副本一起运行。

## 2. 环境及构建

目标平台为Ubuntu20.04/ROS Noetic、OrangePi5 Plus/RK3588、MID360。需安装MAVROS、PCL/Eigen/OpenCV/nlopt等系统依赖，Livox SDK2、相机SDK以及与板端驱动兼容的RKNN Lite运行时。具体版本/来源见NEXT_TEAM_README.md与docs/ENVIRONMENT.md。系统/固件不打包，新机器须单独准备。

将目录放在板端的独立工程路径，不依赖旧0928工作区。先编译：

```bash
cd /path/to/liftrace_competition_20261019
BUILD_JOBS=2 bash deployment/competition/build.sh
```

该构建按当前实机包及依赖编译，不启动飞机。保留当前FAST_LIO_MATCH_THREADS默认值3；CPU亲和绑定还由运行脚本设置，不能只看编译参数判断已生效。

## 3. 模型与外参

正式类别顺序：bridge、panzer、pillbox、tent、red_cross，H不是YOLO第六类。环境脚本默认同源v8 RKNN与metadata，切模型必须成对替换。PT/ONNX、数据集、训练记录另取独立模型数据包。

必须实测并核对：相机标定/分辨率、相机相对FC安装、静置FC离地高度、任务坐标轴、每个投口的物理位置、舵机锁止/释放脉宽与引脚。投口约12cm是机体系杆臂，不能当任务XY同符号偏移；控制补偿从靶心减去旋转后的杆臂。

## 4. 入口与配置

| 用途 | 入口 | 配置入口 |
|---|---|---|
| 正赛 | deployment/competition/start.sh | field_10x10_default.yaml或显式--site-config；默认未确认现场 |
| 03 H专项 | deployment/board_trials_4x4/03_h_landing/start.sh | deployment/site_20260928/h_landing_test_area.yaml及入口覆盖 |
| 04走廊接H | deployment/board_trials_4x4/04_corridor_landing/start.sh | 同目录site_waypoint_map_20261016.yaml |
| 08整场 | deployment/board_trials_4x4/08_full_mission/start.sh | 同目录site_waypoint_map_20261016.yaml |
| 10牛耕/回字形 | deployment/board_trials_4x4/10_low_coverage_full/start.sh | 同目录四份site_*_20261016.yaml；后移版是可选项 |
| 11无靶临时单投 | deployment/board_trials_4x4/11_low_release_test/start.sh | 同目录config.yaml；不是正赛入口 |

只检查正赛配置，不连接飞机、不启动ROS：

```bash
bash deployment/competition/start.sh preview --check-config
```

CONFIG_VALID仅表示格式/范围检查通过；site_confirmed=false仍需重新实测并确认。各组可用`--help`查看参数；纯离线检查必须带`--check-config`，不要把裸preview一概理解为没有节点或连接。

各轮实际运行参数以生成的runtime.yaml、control.yaml、run_metadata.json为准。投递目标高度、允许释放窗、补偿模式、H模式和兜底时间等不得凭旧报告标题推断，先在工作台生成/应用配置并检查实际命令。运行阶段仍保留任务身份、取消和防重复执行约束。

## 5. 工作台、日志与试飞顺序

Windows解压distributions工作台ZIP后按其README安装Python依赖并运行start_windows.cmd；WSL/Linux用tools/flight_workbench/start_workbench.sh。默认端口8771，先设置SSH地址、板端实际工程目录和模型路径，不随包传私钥/口令。

建议顺序：离线配置检查→地面驱动/定位/视觉状态检查→地面机构校准→专项→完整场景。真实舵机/起飞前由现场人员确认。默认模板不是下一届自动验收的场地。

回放/ULog工具可处理新录包；本项目原始bag已清理，旧视频不能代替原始消息。工具说明见TOOLS.md。历史分析与Git备份另存，不属于本运行包。

## 6. 已知边界

本次本机整机构建和入口离线检查通过，历史广域测试仍有失败；CAPTURE最新修复部署后未获新的整场验收；模型仍可能错类；导航恢复及部分H模式有候选状态。详见VALIDATION.md。下一届先继承已测的运行入口和配置，再逐项试飞确认，不能直接把所有候选开关全打开。

本包不含完整仿真世界/eval与历史报告。保留的旧仿真、历史打包脚本仅作为源码参考，不保证在精简包中直接运行；不能为运行这些旧脚本而误用真实硬件节点。重新生成发行包应在完整Git源码仓执行tools/archive/build_handover.py。
