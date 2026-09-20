# 本地4×4板端专项测试：三套独立入口

本地分支 `feat/board-4x4-vision-trials-local`。本目录不推远端，不覆盖上次4×4避障入口，也不自动部署/启动实机。复用当前研究版导航、视觉与旧投递事务，新增外围测试适配器；未修改正式任务核心或解除仿真专用节点的保护。

2026-09-20用户授权SSH部署：独立板端目录为`/home/orangepi/liftrace_board_trials_20260920`。已先比对现场旧4×4修改，继承地图发布与局部地图资源配置，详见[继承说明](board_inheritance_20260920/README.md)。本段是对早期“尚未部署”的更新；飞行仍需现场启动，不自动执行。

| 目录 | 任务 | 正常结束 |
|---|---|---|
| [01_visual_interrupt](01_visual_interrupt/README.md) | 1.4m直飞，遇前方高权重靶中断、对齐、模拟投递一次 | 恢复稳定后直接原地AUTO.LAND，不继续直飞或返航 |
| [02_high_view_revisit](02_high_view_revisit/README.md) | 2.6m完整一圈，冻结1–3个有效目标，1.4m逐个重访/对齐/模拟投递 | 在最后目标处AUTO.LAND，不返回起飞点 |
| [03_h_landing](03_h_landing/README.md) | H放正前方约2m，低空接近、定点升高、CV对齐 | 降到H上；不启动投递服务 |

每轮开始前把飞机放回近侧边中点、机头朝场内，再启动应用重新采样；上一轮末目标不是下一轮场地原点。

模拟投递经过原视觉新鲜度、对准、释放许可和任务确认链，但控制器的Servo调用重映射到`/board_trials/Servo`，最终调用 `/board_trials/mock_servo`，没有PWM/GPIO/真实舵机输出，也不人为阻塞服务等待10秒。此前10s机构预算尚不等于真实机构耗时验收。

## 场地、外参与高度

- 总面积4×4m，固定起飞坐标约定 **+X为初始机头正前方、+Y为左侧、+Z向上**；名义X∈[0,4]、Y∈[-2,2]。起飞点在近侧边中点，初始机体会跨出边缘，后方也要留出机体空间。
- 沿用上次板端安装：机体到IMU平移 `[0,0,0.05]`、IMU到相机 `[0,0,-0.21]`，相机相对FC下方16cm；四元数 `[-0.70710678,0.70710678,0,0]` 与像素矩阵 `[0,-1,-1,0]`配套，集中在 `common/uav_board_trials/config/known_rig.yaml`。
- 仿真中新FOV曾假设相机机械旋转；这里继承**上次板端已知安装**，不把仿真旋转后的外参无条件套到实物。只有实际安装变化时才改这一份公共rig配置，不需要每轮手填外参。
- 地面时飞控local Z约0是正常的。本入口在**未解锁、静置、有新鲜相机/位姿**时自动采样`/navigation/local_pose`中的FC高度，再减去继承的FC静置离地高度0.22m得到地面平面。`camera_init`中的FC零点可能与MAVROS `map`略有平移，因此不能直接把原始MAVROS Z当任务Z。
- 低空FC AGL默认1.4m，高位2.6m；模拟投递FC AGL默认 **0.60m**，在各自`settings.yaml`的`drop_agl`独立调整。降落专项低空接近1.0m、定点升高1.8m。脚本自动统一任务、控制、投递许可、恢复和视觉投影的local Z，**启动命令不再要求ground_z**。
- 自动采样使用已知0.22m机体离地结构值，并非自动测量机架尺寸；更换起落架后应一次性更新公共rig。启动时坐标异常、初始机头偏离camera_init的+X过多或图像/CameraInfo frame不一致会拒绝进入应用阶段。

## 保留上次坐标修复

任务/规划使用`camera_init`；MAVROS原始反馈/设定点使用`map`。`map_camera_alignment`通过同一机体两路实测位姿建立变换；`navigation_frame_adapter`对位姿、里程计和反向设定点做数值转换，保留时间戳、机体系速度及协方差规则。没有只改`frame_id`，没有同时发布第二套固定`map→camera_init`变换，也没有打开latest-TF兜底。

任务、控制、规划桥、释放许可都消费`/navigation/local_pose`或`/navigation/local_odom`；控制设定点先进入`/navigation/setpoint_mission`再转换回MAVROS。相机TF仍使用`map→vision_body→mapping_imu→optical`，保持地图与融合机体的来源一致。

上次资料证明了这套转换的离线测试与板端静态展开，不等于本次新入口已实飞验收。本次重新检查入口与结束条件，不宣称消除了所有定位误差。

## 上板准备（整套独立源码工作树）

三套共享`common/uav_board_trials`以及本工作树的`uav_mission/uav_high_view/uav_vision`等源码。**不要只把某一个小目录复制进旧405bda42包就直接启动**：高位策略、消息和新规划器版本需要一起编译。保留当前独立工作树目录结构；新包通过 `vision_ws/src/uav_board_trials` 相对符号链接参与Catkin。

在板端独立工程根目录完成一次构建：

```bash
OPENCV_CMAKE_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash --extend
export UAV_VISION_RKNN_MODEL_PATH="$PWD/runtime_models/merged_standard_fp32.rknn"
```

模型复用已部署RKNN；若模型放在旧工程，可把上述变量指向那个实际文件，不需要重新训练。板端使用既有可导入RKNN Lite2的ROS Python环境，应用不启动PyTorch。`BOARD_PYTHON`只选择监督脚本解释器，不会重写已生成的Catkin节点shebang；默认构建使用板端系统ROS Python。若原板端实际使用其他既有解释器，需要以相同`PYTHON_EXECUTABLE`重新构建两工作区，不在系统Python临时安装模型包。

按上次方式先启动MAVROS、MID360 **driver2** 和已标定相机；相机应提供`/camera/image_raw`与`/camera/camera_info`。旧整机/旧4×4应用先退出，设备驱动保留。脚本会拒绝已有LIO/规划/控制/任务应用节点以及错误源码overlay。

各子目录均有：

```bash
bash deployment/board_trials_4x4/01_visual_interrupt/start.sh preview
# 退出preview后，二选一启动flight；不能同时启动两套。
bash deployment/board_trials_4x4/01_visual_interrupt/start.sh flight
```

`preview`有定位、建图、规划和视觉/录像，没有patrol_control控制输出、飞控设定点适配出口或模拟投递服务。`flight`接通控制输出和本测试所需的mock链，**不自动解锁，也不自动调用任务启动服务**。飞手按现场流程切模式/解锁、确认起飞悬停后，另一个终端启动任务：

```bash
rosservice call /navigation/start_mission "{}"
```

首次`flight`按用户正常试飞授权现场执行。本次开发没有运行这些实机命令。运行期间遥控接管优先；自动落地后检测到ON_GROUND且已解除武装会自动收尾，若飞控不自动解除武装，可确认实际落地后Ctrl+C收尾。脚本不发强制停桨/解锁/重解锁命令，不改PX4参数。

## 每次相机与视觉链回看

每次应用运行写入 `logs/board_<专项>_<时间>/`：

- `camera_raw.mp4`：原始相机内容的640宽、5fps存档；不录整包bag。
- `camera_annotated.mp4`：YOLO橙框、几何精修/地图投影绿框与精修中心，加任务阶段、记忆、像素偏差、释放证据与模拟投递次数。框与图像时间差必须≤30ms，陈旧框不强行画到新图像上。
- `vision_events.jsonl`：检测类别/置信度、几何/关联/拒绝原因、投影坐标、目标记忆、对准证据、mock调用、落地请求和任务状态。
- `camera_frames.csv`、`navigation_pose.csv`：源图像/记录时间和任务系轨迹；视频存在丢帧/保持时以CSV为准。
- `ground_reference.json`、`camera_info.json`、运行YAML、ROS日志：自动高度基准和实际配置。
- `result.json`、`index.html`：正常收尾生成结果与两个视频索引。失败、中止、预览或缺靶保留为INCOMPLETE，不伪报PASS。若已装ffmpeg，收尾后额外转H.264方便浏览器播放。

相机缓存最多16帧、录制5fps、OpenCV单线程；默认最长记录15分钟，剩余空间低于512MiB时先正常关闭录像，启动前要求至少2GiB空闲。录像线程不向控制链发布运动指令。板端实际并发帧率/温度仍需现场观察。

## 共同终止与验收边界

局部运动期限30s、目标事务60s、总任务300s；没有为了等任务而无限延期。搜索初次规划12s保护保留，但不能据此宣称全工程停滞已根治。缺靶、全圈中途不可达、记忆目标复访失败、旧帧/缺TF均保留失败并交由飞手处理；不冒充模拟投递成功，不改用真实舵机，不临时追加全覆盖补搜。

三套属于板端待验收测试配置。文档中单元/静态/录制自检与实机动态试飞分别记录，不能把笔记本检查当作板端飞行PASS。

## 本地完成的验证

- 8项专项逻辑/配置测试：一次模拟投递后直接LAND；全圈不提前中断；1/2/3个记忆结束不虚构槽位；无返航边的三点代价；统一地面基准；无靶失败；落地上下文匹配；Python3.8语法。
- 11项继承的双向坐标适配/实测map-camera对齐回归通过。
- 3套×preview/flight共6个应用入口，以及反馈/控制两种定位入口的离线展开通过；第一、二套全部指向独立mock服务；H套不加载投递服务。
- 新增ROS包在独立临时Catkin工作区实际构建通过，未重编或替换正在跑矩阵的工作树。
- 合成图像录制/标注/视频解码自检通过（不是实机或仿真飞行录像），见`validation_recording.json`。实际板端RKNN并发、相机流和飞行尚待上板验证。

规划区域和名义航点不能当作飞控硬围栏，末端CV修正与跟踪误差仍需场地余量。图像标框按图像时间匹配，底栏为记录时刻的任务状态，源时刻在CSV中保留。

如需拷上板，可在本独立工作树已提交后用`git archive`导出**一个完整源码包**，保留相对符号链接；不要复制工作树的`.git`指针或笔记本build/devel。当前只保留本地三个目录，没有额外自动打包、推远端或上传板端。
