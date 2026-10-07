# 研究相机 operator / Bernoulli 接口（2026-10-07）

最新安排：**只准备新版 seed31 / seed38 两轮 exact ON**，复用历史结果，不重跑 OFF。本相机任务没有启动仿真。runner 由 Bernoulli 负责，不改本配置，也不修改旧 `run_case.py`。本接口只允许研究入口使用，不接板端。

唯一准备入口为本目录 `prepare_camera.py`：

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python -B docs/verification/drop_precision_20261007_camera/prepare_camera.py \
  --enable-drop-camera-research --output-dir NEW_PREPARED_DIR
```

`NEW_PREPARED_DIR` 必须不存在。准备阶段只写新目录，绝不启动 ROS / Gazebo / PX4。一次准备产物供两轮共同使用；准备之后不修改产物。生成器复用当前 `iris_mid360_start_fov/model.sdf` 与原有三层 launch 模板，不维护另一份机架/场景副本。

`NEW_PREPARED_DIR/prepared.json` 是 runner 的输入：

| 字段 | 用途 |
| --- | --- |
| `launch_file` | 新生成的唯一研究 `replay.launch`，保持 exact ON |
| `vehicle_sdf` | 该入口展开后必须实际传给 vehicle spawn 的 SDF |
| `camera_contract` | 预期 CameraInfo K/D/P/R 与尺寸/frame；不是已采集 CameraInfo |
| `source_head` / `source_root` | 准备时源码；本次基于 `82a91e45`，runner 仍须记录自己的实际工作区状态 |
| `operator_preflight` | 下述离线预检入口 |
| `operator_runtime_check` | 下述运行中只读检查入口 |
| `runtime_world_sdf_helper_source` | Gazebo world_sdf 只读客户端源码 |

直接 operator 可使用 `launch_file`；Bernoulli 的任务 runner 可消费同一准备产物的 `--camera-dir NEW_PREPARED_DIR/camera`，在自己的研究 replay 中接入任务启动 CameraInfo guard。这是 runner 外围适配，不另建相机配置。仅传原有 `scene_dir`、`field_seed`、`target_model_path`、`high_agl:=2.16`、`resume_survey_enabled:=false` 等已冻结参数；场景仍来自 `snake3_camera2m_20261005/generated/snake3_{31,38}/snake3_seed{31,38}`。**不要回到旧 replay，也不要用 rosparam 覆盖 SDF 或单独发布一份“修正” CameraInfo。** 原入口里的 optical TF `[0,1,0,0]`、相机 FC 下 16cm、zero 槽位、关闭质量 NMS、许可/门槛保持模板定义。

### 13:35 接入核对：runner 尚有待修预检

已只读核对 `drop_precision_ab_20261007` runner：其 `camera_inputs()` 使用本任务的 `generate_drop_research_camera.py.make_model()` 验证 SDF/契约，输入 `--camera-dir` 与上表兼容。无需复制或改写本研究配置。

对当前 runner 的 case0/1 实际 dry-run 均在 `preflight.py` 的 spawn 类型断言处退出：它要求 `gazebo_ros/spawn_model`，实际 include 链为 `patrol_control/spawn_model_delayed.py`（相机独立预检已展开并核对其 `-sdf -file ... -model iris_mid360`）。**交给 Bernoulli 修正节点选择/类型断言并重做 dry-run**；保留唯一 vehicle、精确 `-file` 路径和模型名检查，不要为通过预检而绕开原延迟 spawn。此问题属于 runner 预检，不是相机 SDF 不一致；本任务未修改 runner。

runner 自带 CameraInfo guard 核对发布和磁盘输入；还应接入下面的 `runtime_check.py` 读取服务器 world_sdf，保留像素层实测未验收的边界。此处不声称 runner 已通过整体接入检查。

## 每轮启动前

通过已有 setup.bash 顺序 source H 的视觉/导航 overlay，并激活 `rl_drone`，运行：

```bash
python -B docs/verification/drop_precision_20261007_camera/preflight.py \
  --prepared-dir NEW_PREPARED_DIR --seed 31 --model SAME_DETECTOR_WEIGHT \
  --output NEW_PREFLIGHT31_JSON
```

seed38 同理。此命令只用 XmlLoader 展开 launch，不启动节点或 ROS master。它复用旧的冻结场景/32项投影参数检查，追加 SDF 渲染系数、插件 K/D、实际 spawn `-file` 路径和 H 包解析检查；所有项须 PASS。CameraInfo 字段此时只是**预期发布值**，预检明确 `runtime_camera_info=NOT_OBSERVED`。保留结果至相应 run；两轮应引用同一研究产物和同版工作区。

runner 的启动/互斥/收尾仍走 `sim_run.sh`，使用明确设置的 `UAV_WS` / `VISION_WS`，仅在受授权的那条命令临时设置 `SIM_RUN_AUTHORIZED=1`。本接口不替 runner 发起任何启动。

## 运行中：真实选用模型与 CameraInfo

只读客户端先离线编译（可在没有 Gazebo 时执行）：

```bash
g++ docs/verification/drop_precision_20261007_camera/read_runtime_world_sdf.cpp \
  -o NEW_WORLD_SDF_HELPER $(pkg-config --cflags --libs gazebo)
```

待研究场景已经由 operator 启动并稳定发布相机后，runner 在自己的 wrapper 生命周期内调用下面的观察工具；ROS 观察脚本用已有 overlay 下的系统 Python，不装包：

```bash
/usr/bin/python3 docs/verification/drop_precision_20261007_camera/runtime_check.py \
  --observe-existing-run --contract NEW_PREPARED_DIR/camera/camera_contract.json \
  --world-sdf-helper NEW_WORLD_SDF_HELPER --world ACTUAL_WORLD_NAME \
  --model-name iris_mid360 \
  --camera-info-topic /downward_camera/camera_info \
  --image-topic /downward_camera/image_raw \
  --duration-wall 30 --max-gap-wall 10 --min-samples 3 \
  --world-sdf-output NEW_RUNTIME_WORLD_SDF --output NEW_RUNTIME_CHECK_JSON
```

`ACTUAL_WORLD_NAME` 从本轮冻结 `field.world` 的 `<world name>` 读取；必须显式传，不能猜 `default`。本工具只订阅 Image / CameraInfo，并向已运行 Gazebo 发 `world_sdf` 请求，不启动服务器、不 spawn、不发布 GT/控制话题。它从服务器返回的 world SDF 选中唯一 `iris_mid360`，核对其中的相机渲染 D0、居中 FOV、插件 K/D、安装姿态；这比仅核对磁盘 SDF/标记参数更强，但仍不是渲染像素标定实测。

CameraInfo 每帧检查尺寸、frame、plumb_bob、K/P/R/D、裁剪/binning，并要求至少三组不同源时间戳的 Image/CameraInfo 精确配对；无消息、间断、数值变化或模型不符返回非零。结果和服务器 SDF使用新路径写入，不覆盖已有记录。

落地建议：启动稳定后先取一次短窗口 PASS；整场另运行观察窗口，覆盖全部投递阶段，或每槽捕获/释放附近再取独立窗口。`duration-wall` 是宿主墙钟秒，慢 RTF 下按本轮实际预算设置。runner 将观察子进程纳入自身收尾；任何检查 FAIL 按已有 runner 失败处理与收尾，不能继续并报告相机通过。本观察工具自己不控制飞机、不停止仿真，也不负责动作准入。

## 仍需像素层验收

在后续获授权的静态观测中，用已知**固定地面点**采集中心/四边/多高度/roll-pitch 的真实 Image、同源 CameraInfo、曝光时间姿态与外参。用像素测量残差核对正向投影及反投影，包含边缘、像素中心约定、地面高度、时间对齐；GT 只进入评测进程。缺少此项不能声称 Gazebo 渲染与数学模型已实测一致。

历史 seed31/38 用原有偏心主点及非零渲染畸变，新两轮用 centerK/D0，且控制候选可能由其他 agent 修复。报告必须分别记录源码、校准和控制变化；历史对照仅作整体结果对比，**无法单独识别算法收益**。真实相机标定与实物落点不受本研究配置影响。
