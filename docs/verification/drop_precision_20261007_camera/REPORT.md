# 投递研究相机一致性准备与离线检查（2026-10-07）

**离线检查 PASS，可交接 operator；没有启动 ROS / Gazebo / SITL，没有渲染实测或新增飞行精度结果。** 本次基于 H=`/home/xhj/liftrace-worktrees/r2026-high-view-search`、HEAD=`82a91e453ab752f5751f2dd4dc0a1b6493e64cdb`。13:32（Asia/Shanghai）已完成生成、数学检查、两 seed 预检和只读客户端编译。

最新运行安排仅为**新版 seed31/38 两轮 exact ON，复用历史结果**。启动由主代理 review 完成后的 operator / Bernoulli runner 负责；本相机任务没有自行开跑。接口和命令见 [OPERATOR_INTERFACE.md](OPERATOR_INTERFACE.md)。通过该文件及 `prepared.json` 对齐配置，随后已只读核对 Bernoulli runner 对同一生成器/相机契约和 `--camera-dir` 的使用。

## 判断与实现

采纳居中无畸变的简单针孔对照。原模型渲染基础针孔按 FOV 居中，插件发布偏心主点；其非零畸变实际参与渲染，所以仅将 CameraInfo D 清零不构成一致性修复。依据为已读的 [历史 renderer 复核](../drop_precision_20261006/independent_review/drop31_renderer_intrinsics_notes.md)，并核对对应版本的 [Gazebo Camera](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Camera.cc#L192)、[Distortion](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Distortion.cc#L488)、[ROS CameraInfo 发布](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_camera_utils.cpp#L516)。这是配置/源码依据，尚未对本机运行时投影矩阵或渲染像素做验收。

新增 `uav_vision_eval/config/drop_camera_centered_d0.json` 与 `scripts/generate_drop_research_camera.py`。生成器读取既有 `iris_mid360_start_fov/model.sdf`，保留尺寸/FOV、相机安装姿态、iris/mid360 include、质量惯量、动力学、接触模型、话题/frame；仅替换相机光学及发布参数。没有另一份手工维护的完整 SDF，没有复制 meshes/动力学目录，没有改真实相机标定或既有模型/场景。

| 项目 | 研究配置 |
| --- | --- |
| 尺寸 | 1280 × 720 |
| HFOV | 1.44593453190313 rad，继承原模型 |
| fx=fy | 1280 / (2 tan(HFOV/2)) = 725.3510059644452 px |
| cx / cy / CxPrime | 640 / 360 / 640 px |
| 渲染 k1/k2/p1/p2/k3 | 全零 |
| 渲染 distortion center | 0.5 / 0.5 |
| 插件 D / CameraInfo 预期 D | 全零，plumb_bob |
| autoDistortion / borderCrop | true / false；从零畸变渲染端复制 D，不裁边 |
| 基础针孔投影 | 保留默认居中 FOV 路径，不增加 custom lens/intrinsics |

必须显式 `--enable-drop-camera-research`，且输出目录必须全新。operator 唯一入口 `prepare_camera.py` 运行时从已有三层模板派生独立 launch，将实际 vehicle SDF 传入研究模型；旧 launch/报告原样保留。产物含 `prepared.json`、相机 SDF/配置、**预期** CameraInfo 契约与三层生成 launch。相机只供投递对照入口选用，不进入默认/板端配置。

## 已完成检查

1. [离线数值](offline_results.json)：9 个固定 world 地面点 × 相机 AGL 0.5/1/2/3m × 5 组机体姿态，共 **180** 样本；roll/pitch 包含 +10°、−12° 及组合倾角/航向。点在全部姿态/高度之间保持固定。1m 水平时包含主点、四边、四角；119 样本在视野内、61 样本在视野外，后者仅作射线数学检查，共15个视野内边缘样本。
2. 正向使用从渲染 FOV 独立推导的针孔 K；反向使用插件声明 K/D，经 OpenCV 去畸变射线和生产 `uav_vision.ground_projection.intersect_ground` 求交。SDF RPY/传感器坐标独立推导 optical 外参，与既有 `[0,1,0,0]` 对齐。最大地面往返误差 **1.047e−15m**、最大像素往返误差 **3.216e−12px**；这些是浮点自洽结果，**不是渲染/检测/飞行精度**。
3. **12 项负例**均拒绝：未显式启用、覆盖已有输出、偏心 K、错误焦距/P、非零发布 D、错误 frame/尺寸、裁剪、渲染 D 非零而发布 D0、历史 SDF、错误运行时模型。模板源内容及非光学内容保持相同，未做无关哈希。另固定原偏心 K、D0 的数学负对照最大误差17.31cm，仅用于证明检查可识别该失配，不重算历史飞行精度。
4. [seed31 预检](preflight_seed31.json) / [seed38 预检](preflight_seed38.json)：每轮复用原 **32 项**投影/冻结参数检查及8份冻结场景文件核对，追加光学契约、H overlay、实际 spawn `-file` 路径检查，均 PASS。展开的全部 ROS 参数与原 exact ON 包装器相同；实际 `spawn_model_delayed.py` 指向新研究 SDF。两轮均明确 `runtime_camera_info=NOT_OBSERVED`。
5. `read_runtime_world_sdf.cpp` 已用本机 Gazebo headers/libs **编译成功**；生成、预检、数值及运行观察脚本可解析。只编译该客户端，没有运行它，没有启动 master/节点/模拟器。

**13:35 新增接入核对：相机独立预检 PASS，Bernoulli runner 整体 dry-run 尚未 PASS。** 对 `drop_precision_ab_20261007/run_case.py --case 0/1 --camera-dir /tmp/drop_camera_20261007_82a91e45_reviewed/camera --dry-run` 均在 runner 的 spawn 类型断言处失败：其预检要求 `gazebo_ros/spawn_model`，当前历史 include 链实际使用 `patrol_control/spawn_model_delayed.py`。相机 SDF 路径/契约已被 runner 前置验证接受。交给 runner 所有者修正该节点筛选并复验，保留 `-file` 精确路径和模型名检查；无需改本研究相机配置。本任务未改 runner，也没有执行其授权运行分支。

本次检查产物位于 `/tmp/drop_camera_20261007_82a91e45_reviewed/`，客户端二进制位于 `/tmp/read_drop_camera_world_sdf_20261007`。这些是离线临时产物；正式 runner 应在自己的新目录重新准备一次，记录 `source_head`、`source_git_status` 和当前控制修复 review 后的版本，两轮共享该产物，不在轮次之间调整校准。

## 运行检查及必要验收

`runtime_check.py` 已提供只读入口：对已运行场景订阅 Image/CameraInfo，每帧检查尺寸/frame、K/P/R/D、裁剪/binning，并要求至少3组不同源 stamp 精确配对；经已编译客户端向真实 Gazebo 请求 world_sdf，选唯一实际 vehicle，核对服务器保存的相机渲染 D0、居中 FOV、插件 K/D 与安装姿态。结果与 runtime SDF 分别写入新文件；缺采样、不一致或请求失败均非零。observer 纳入 runner 自身收尾，FAIL 由 runner 处理；工具不发布控制/GT，不执行 stop/arming/投递。

**上述运行工具尚未在 Gazebo 中执行。** 下一轮须保存启动稳定后的短窗口 PASS，并覆盖各槽捕获/释放附近或整段运行；仅磁盘 SDF/预期 JSON 不足以声称实际选用正确。具体命令见 operator 接口。

此外仍须采集固定已知地面点的真实渲染图像、同源 CameraInfo、曝光姿态与安装外参，在中心/边缘、多高度、roll/pitch 下检查像素及地面残差，明确像素中心约定、地面高度和时间对齐。**运行配置观察也不替代此像素层标定实测**。真值只进入评测，不进入规划/选靶/控制。

历史轮使用原偏心主点/非零渲染畸变，新轮使用 centerK/D0，同时控制修复由另一 agent 提供。新两轮与历史结果可作整体对照，但**不能单独归因于完整姿态投影算法**，不能将数学误差当释放精度，也不能外推为真实相机或实物落点验收。

## 变更边界

仅新增本报告目录内工具/说明/小型结果、视觉评测包的一份研究 JSON 和生成脚本。本任务未修改 `drop_precision_20261006` 历史内容、`run_case.py`、`patrol_control`、共享变更记录或 ROADMAP，未 commit、切换/修改其他分支。工作区中控制源码/测试及 legacy 快照为并行 agent 的改动，本任务没有碰这些文件。


## 运行后补充：实际相机检查与序列化兼容

两轮飞行源码固定为66c36a9e：seed31=`logs/drop_precision_new_seed31_20261007_135022`，seed38=`logs/drop_precision_new_seed38_20261007_140702`。实际图像/CameraInfo与world SDF均已读取。seed38取得118组同源时间戳配对；两轮均使用居中主点640/360、D0。此结论是运行配置核对，不替代像素层标定或释放误差评测。

seed31原runtime检查FAIL保留：Gazebo序列化的FOV小数和等价Euler形式使原严格文本比较误报；实际序列化角差1.0985e-5rad，实时link-state与安装TF差5.2068e-6rad、平移6.12e-8m。修正只针对评测解析：runtime旋转角差2e-5rad、FOV差1e-5rad，兼容bool 1/0；源SDF及CameraInfo严格检查保持。运行期间用/tmp评测副本重查，三份31快照及38运行检查PASS，未改飞行控制或其参数。

两轮收尾后才将这项评测工具修正纳入仓库，原失败/原始SDF/补充PASS均保留；保存原始响应移至验证之前，避免失败时丢失诊断内容。主代理再次执行180组数学与12项负例，并离线重验四份实际world SDF及两项人为失配，全部符合预期，见runtime_snapshot_review.json。
