# AGENTS.md

本文件面向后续参与 RoboCup 无人机识别与快递运输项目开发的 coding agent / automation agent。进入本仓库后，优先阅读本文件，再阅读 `README.md`、`VISION_2026_ROADMAP.md`、`VISION_WORKSPACE_GUIDE.md`、`SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md`、`VISION_MIGRATION_CHECKLIST.md`。当前任务优先级只在 `VISION_2026_ROADMAP.md` 维护。

> 项目当前定位：以 2025 年机载工程代码为基线，面向 2026 年 RoboCup 无人机投递赛规则进行迁移、仿真、视觉搜索与投递精定位能力升级。

---

## 1. 对 agent 的总要求

1. **先确认上下文，再改代码。** 每次任务开始先说明将查看哪些文件、要验证什么，不要凭印象直接改。
2. **小步修改、可回滚。** 每次只解决一个明确问题，修改后必须说明改了哪些文件、为什么改、如何验证。
3. **禁止无保护地执行实机危险动作。** 不得擅自执行解锁、起飞、舵机投递、PWM 输出、遥控接管相关命令。任何涉及实机运动或执行机构的操作都必须先让用户确认。
4. **不要把仿真和实机混为一谈。** `actuator_pwm`、真实 MAVROS 链路、Livox 驱动、相机 SDK 等涉及硬件的节点，在仿真中默认不启动，除非有 mock 或用户明确要求。
5. **不要污染基线。** `liftrace/` 是从香橙派机载电脑拷贝回来的基线/集成工程；视觉日常开发优先在 `vision_ws/src/uav_vision` 中进行。
6. **不要删除原始资产。** 不要删除 `patrol_uav_ws-patrol_planner`、`Desktop_patrol_uav_ws-patrol_planner`、`Visual`、`detect_ws`、`Desktop_misc`、`top_level_scripts` 等原始目录。若要清理，只能清理可再生的 `build/`、`devel/`、`.ros/log`、临时日志，并在执行前说明。
7. **不硬编码新路径、新话题、新内参。** 新增代码必须通过 launch/yaml 参数配置相机话题、相机内参、TF frame、模型路径、输出话题。
8. **不在 OrangePi 5 Plus 上跑 PyTorch 大模型作为主路径。** 板端推理路线优先 RKNN/NPU；OpenCV CPU 只做 ROI 几何精修与轻量检测。
9. **保留旧接口兼容。** 改造视觉系统时，内部可以新增 `/uav_vision/*`，但短期必须兼容旧 `/detect/*` 话题，避免破坏 `patrol_control`。
10. **遇到规则不确定项要标注。** 例如规则书对某类靶标的得分权重表述存在缺口时，应先标注并确认，不能擅自假定其最高优先级（`tank` 权重已确认 = 5，不再作为不确定项）。
11. **Python 开发和模型训练优先使用已有 conda 环境。** 本机已配置 miniconda3 和 `rl_drone` 环境（含 PyTorch 2.5.1、Ultralytics 8.4.33、OpenCV 4.13 等），激活方式：`source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone`。禁止在系统 Python 中另行 pip install。如果确有需要新建 conda 环境，必须先申请并经用户确认后再执行。
12. **每次变更完成后必须追加联调变更记录。** 代码、脚本、launch、配置、文档的任何修订，都必须在 [docs/仿真联调变更记录.md](/home/xhj/liftrace/docs/仿真联调变更记录.md) 文末按模板追加一条记录，至少包含：日期、改动范围、具体改动、验证结果、遗留问题、下一步。
13. **WSL shell 命令必须用 `wsl -e bash -c '...'` 执行。** 本机宿主为 Windows 11，开发环境在 WSL2 Ubuntu 20.04 中。Claude Code 的 Bash 工具默认使用 Windows 原生 bash（而非 WSL 内 bash），会导致以下问题：
    - `dirname` 等外部 GNU 命令解析失败或返回空字符串
    -  环境变量（`ROS_PACKAGE_PATH` 等）不完整
    - ROS 和 PX4 命令行工具不可用
    - **正确做法：所有需要操作 WSL 文件系统或执行 ROS/PX4 命令时，必须用 `wsl -e bash -c '...'`**，其中 `-e` 标志禁止 Windows PATH 污染。
    - **文件访问工具（Read / Write / Edit / Glob / Grep）** 使用 `\\wsl.localhost\Ubuntu-20.04\...` 前缀访问 WSL 文件系统。
    - **脚本内（`.sh` 文件）** 避免使用 `dirname` 外部命令，改用 bash 内置参数展开：`${BASH_SOURCE[0]%/*}` 获取脚本所在目录。
    - **编码边界（Windows→WSL argv）**：Windows 侧 pwsh 以系统 ANSI/GBK 编码把参数传给 `wsl.exe`，WSL 内 bash 按 UTF-8 解析，命令行内联中文会乱码/丢字。commit message 与含中文的文件内容一律先写 UTF-8 文件再消费：中文提交用 `top_level_scripts/git_commit_cn.sh <消息文件> [git add 路径...]`（自动去 BOM/统一 LF，内部走 `git commit -F`）；文件级中文替换用 python 读 UTF-8 文件改写。禁止在 `wsl -e bash -c '...'` 内联中文。
14. **保留旧任务/投递链原貌。** 新视觉闭环优先通过独立 Mission Manager、接口适配器、
    服务代理和新 launch 复用旧 `patrol_control`，不得借重构之名随意删除、合并或改写旧
    状态机。确有必要修改旧链源码时，先在不参与编译的 `legacy_baseline/<日期>/` 保存原包
    快照、文件清单和 SHA256，再实施最小补丁；禁止用散落 `.bak` 代替可审计快照。
15. **git 是唯一版本控制，纳入自动工作流。**
    - 每次改动前先 `git status` 确认工作区；每完成一个可独立验证的改动即提交一次，
      commit message 用**中文**并遵循 conventional 标记（`feat:`/`fix:`/`docs:`/`chore:`/
      `refactor:`/`test:`），如 `fix: 走廊航点可达性修正`；中文消息必须经 UTF-8 文件 + `git commit -F`
      提交（编码边界见规则 13，禁止命令行内联中文）。
    - GitHub 凭据只存于 WSL `~/.git-credentials`（chmod 600，credential.helper store），
      **禁止**把 token 写入仓库内任何文件、commit message、launch/yaml/脚本/日志；
      `.gitignore` 已含 `*token*`、`*.credentials`、`.git-credentials` 双保险。
    - `build/`、`devel/`、`install/`、日志、bag、模型权重、数据集等大文件一律不入库
      （由 `.gitignore` 维护，新增例外需在变更记录说明）。
    - 变更记录（规则 12）**先于** commit 更新，commit 与文档记录一一对应；
      每阶段 push 前确认 `git status` 干净；`main` 只经合并进入（规则 18）。
16. **仿真必须走统一 run 目录规范。** 每次经当前用户请求明确授权的仿真启动用
    `SIM_RUN_AUTHORIZED=1 top_level_scripts/sim_run.sh <场景名> roslaunch ...`，它自动生成
    `logs/<场景名>_<时间>/`（含 run.log、manifest.yaml、screenrecord.mp4、
    roslog 归档、timeline.txt）；收尾由脚本内自动执行（ffmpeg SIGINT 优雅
    停止写 moov、校验录屏、归档 `~/.ros/log/latest/`）。禁止再直接
    `roslaunch ... > /tmp/xxx.log &` 裸跑；手动强杀仿真用
    `stop_toudi3_sim.sh`（TERM 后 KILL）。录屏不可用时 `SIM_NO_RECORD=1`。
17. **以普通无人机开发者身份工作，不做过度审计。** 本项目的定位是 RoboCup
    无人机投递赛的工程开发，不是安全审计或取证。因此：
    - **禁止无意义的哈希校验**：不主动对文件/目录做 sha256/md5 校验（除非
      规则 14 的 legacy 快照场景明确要求、或排查"文件是否被意外改动"确有
      必要）；git diff/git status 就是足够的变更追踪手段。
    - **不做过度审计活动**：不反复核对"证据链"、不把每次改动都写成审计报告、
      不检查与开发无关的敏感信息。专注：能编译、能仿真、能飞、能投递。
    - 安全底线仅保留：token/凭据不入仓库（规则 15）、不执行实机危险动作
      （规则 3）、仿真与实机分离（规则 4）。
18. **多人协作分支策略（2-4 人小团队 + agent 辅助）。**
    - **主干模型**：`main` 是唯一验收基线，只接受经过实跑验证的内容；任何成员
      不得直接 push `main`，必须走 feature 分支合并（GitHub PR 或本地
      `git merge --no-ff`，保留分叉痕迹；禁止 squash 与 fast-forward）。
    - **分支命名**：沿用现有语义风格，两种均可：
      `feat/<语义主题>-<简述>`（如 `feat/external-mission-coverage`、
      `feat/new-vision-coverage-search`），或带 ROADMAP 阶段编号
      `feat/<阶段号>-<简述>`（如 `feat/vcl04-r6-bridge`、`feat/vsim04-stability`）；
      阶段号必须引用 `VISION_2026_ROADMAP.md` 中的既有编号（V-CL/V-SIM/V-ALG/
      V-DEPLOY/R 系列）。
    - **分支生命周期**：从最新 `main` 拉出 → 小步提交并 push 同名远程分支 →
      实跑验证（`logs/<场景>_<时间>/` + `gate_status.json`）→ 合并回 `main`
      （--no-ff）→ 分支保留不删除（供溯源），分支负责人确认无后续工作后自行清理。
      一个分支只承载一个 Gate/里程碑，禁止长期分支积压。
    - **并发分治**：按包划分所有权，避免两人同时改同一处：视觉链
      `vision_ws/src/uav_vision*`（视觉组）、任务/控制链
      `patrol_uav_ws-patrol_planner/src/uav_mission` 与 `patrol_control`
      （控制/规划组）、`top_level_scripts` 与板端工具（板端组）。跨组改动
      （话题/消息/接口）须先在变更记录或组间对齐，接口变更分支优先合并。
    - **共享文档冲突约定**：`docs/仿真联调变更记录.md` 与 `AGENTS.md` 是多分支
      追加型共享文件。合并冲突时**双方条目都保留**、按日期排序，不得覆盖他人
      条目；修改 AGENTS.md 规则条款必须在变更记录说明动机。
    - **仿真资源独占**：SITL 实跑按规则 16 走 `sim_run.sh`；同机禁止多套
      roscore（规则 12）；多人实跑以 `logs/` 目录时间为序排队，开跑前先检查
      他人最近的 run 目录，避免端口/资源争用。
    - **agent 使用边界**：agent 可代成员完成提交（遵守规则 15 的 commit 规范）；
      但**合并 main、推送 main、删除分支必须由人工确认后执行**；agent 不得自行
      决定跨组接口变更。
    - **合并前检查清单**：`git status` 干净；相关 Gate/断言有 PASS 或实跑记录
      （变更记录引用 logs 路径）；无 token/大文件入库；变更记录条目已存在且
      先于合并。
    - **合并后打 tag**：每次 `--no-ff` 合并回 `main` 后立即打 annotated tag 并
      推送远端，便于复盘定位节点。Gate 合并用 `gate/<Gate-ID>-<简述>`（如
      `gate/vcl04-r6-bridge`），非 Gate 合并用 `chore/<简述>`；tag 消息写清该
      合并对应的 Gate/通过证据（按规则 15 的 UTF-8 文件方式，禁止命令行内联中文）。
19. **仿真启动必须显式授权、单实例且强制收尾。** `gzserver`、PX4、ROS 和 RViz 属于同一套
    本机独占仿真资源，agent 必须遵守以下运行边界：
    - 只有用户在**当前请求**中明确要求“启动、运行、重跑或继续某轮仿真”才构成授权；“恢复并继续”、
      “修复后提交”、“查看/诊断/汇报”及历史请求均不授权自动续跑。停止仿真后，除非收到新的明确
      启动请求，不得因自动 continuation、恢复会话或验证代码而再次设置 `SIM_RUN_AUTHORIZED=1`。
    - 禁止直接运行 `roslaunch`、`gzserver`、PX4 SITL 或绕开包装器设置后台进程。获授权后也只能在
      单条命令上临时设置 `SIM_RUN_AUTHORIZED=1` 并调用规则 16 的 `sim_run.sh`；不得把该变量写入
      shell profile、launch、脚本默认值或长期环境。
    - `sim_run.sh` 必须先取得本机互斥锁并确认不存在 `roscore/rosmaster/rosout/roslaunch/gzserver/
      gzclient/px4/mavros_node/rviz` 残留；任一存在时拒绝启动，不得叠加第二套仿真。
    - 联合工作树运行时，调用者显式传入的 `UAV_WS` 与 `VISION_WS` 是唯一源码 overlay 权威；
      `sim_run.sh` 必须拒绝 `uav_mission`/`uav_vision` 解析到其他根仓或旧工作树，并在 manifest
      记录双仓 HEAD、工作区状态及实际包路径。不得再用手写整条 `ROS_PACKAGE_PATH` 修补包顺序。
    - 手工核查仿真残留必须调用 `top_level_scripts/check_sim_processes.sh`，或逐个使用
      `pgrep -x <精确进程名>`。禁止在 Windows→WSL 命令中使用 `pgrep -af 'a|b|...'`、未受保护的
      `|` 进程名组合或等价写法；宿主侧引号一旦丢失，shell 会把它解释成管道并实际执行
      `gzserver`、`gzclient` 等命令。进程核查必须保持只读，不能以“检查”为名调用仿真程序。
    - 正常结束、Gate FAIL、命令失败、超时、`HUP/INT/TERM` 中断都必须执行同一收尾 trap：停止辅助
      进程和录屏、调用 `stop_toudi3_sim.sh`，并以精确进程名复查零残留。清理复查失败时整轮返回
      非零并报告残留 PID，不能在尚有 `gzserver` 时宣布仿真已停止。
    - 同一 FAIL 不得无分析连续重跑。先从现有 run 目录确定首个失败阶段；只有代码/配置发生相关
      改动，或用户明确要求测量同版本波动时，才允许下一轮。汇报和文档验收只读取既有产物。

---

## 2. 本机开发环境

以下环境均位于 WSL Ubuntu 20.04 中。

### 2.1 核心工具链

```text
OS:       Ubuntu 20.04 (WSL)
ROS:      Noetic (full desktop, /opt/ros/noetic)
PX4:      /home/xhj/PX4-Autopilot (SITL + Gazebo Classic)
Gazebo:   11.15.1
QGC:      /home/xhj/QGC/QGroundControl.AppImage
AstraDroneOpen: /home/xhj/AstraDroneOpen/ (仿真模型底座)
```

### 2.2 C/C++ 编译环境

```text
GCC:      9.4.0
CMake:    3.28.3
Make:     4.2.1
```

### 2.3 Python 环境（唯一允许使用的 conda 环境）

```bash
# 激活方式（每次使用 Python 前必须执行）
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
```

`rl_drone` 环境关键包：

| 包名 | 版本 | 用途 |
|------|------|------|
| Python | 3.9.25 | |
| PyTorch | 2.5.1+cu121 | 模型训练/推理 |
| Ultralytics | 8.4.33 | YOLO 训练/推理 |
| OpenCV | 4.13.0.92 | 图像处理 (headless) |
| NumPy | 2.0.2 | 数值计算 |
| SciPy | 1.13.1 | 科学计算 |
| Numba | 0.60.0 | JIT 加速 |
| Matplotlib | 3.9.4 | 可视化 |
| Rich | 14.3.3 | 终端美化输出 |
| PyYAML | 6.0.3 | YAML 解析 |
| rospkg | 1.6.1 | ROS 包解析 |
| catkin-pkg | 1.1.0 | Catkin 包工具 |

系统 Python（`/usr/bin/python3`, 3.8.10）仅用于 ROS 脚本，**不安装 ML 包**。

### 2.4 关键系统库

```text
OpenCV:   4.2.0 (C++, dpkg)
PCL:      1.10.0 (libpcl-dev)
Eigen3:   3.3.7
nlopt:    2.6.1 (libnlopt-dev, libnlopt-cxx-dev)
Boost:    系统自带
yaml-cpp: 0.6.2 (libyaml-cpp-dev)
Protobuf: 3.6.1 (libprotobuf-dev)
MAVROS:   1.20.1 (ros-noetic-mavros*)
```

### 2.5 已安装的 ROS 关键包

MAVROS、Gazebo、PCL、CV-Bridge、TF2、Nodelet、Rviz、RQT、rosbag、Controller、Image-Transport、Robot-State-Publisher 等全套 Noetic 桌面组件已安装。

### 2.6 实用工具

```text
tmux:   3.0a
rsync:  3.1.3
注：未安装 expect 和 sshpass，SSH 自动化需用 pexpect 或手工输入密码
```

### 2.7 宿主环境与 WSL2 shell 约定

- **宿主**：Windows 11 Home China，所有开发工作在 WSL2 Ubuntu 20.04 中进行。
- **Claude Code 的 Bash 工具**运行在 Windows 侧，**不能**直接执行 `bash -c` 或 `wsl bash -c`（会导致 PATH 污染、`dirname` 等 GNU 工具失败、ROS 环境缺失）。
- **正确的 shell 命令前缀**：`wsl -e bash -c '...'`。关键参数 `-e`：不继承 Windows PATH，保证所有命令解析为 WSL 内原生版本。
- **文件操作工具**（Read / Write / Edit / Glob / Grep）使用 `\\wsl.localhost\Ubuntu-20.04\home\xhj\...` 路径前缀。
- **脚本文件内**（`.sh`）：避免使用 `dirname`，用 `${BASH_SOURCE[0]%/*}` 替代。

---

## 3. 项目根目录和主工作区

默认工程根目录：

```bash
/home/xhj/liftrace
```

当前目录职责：

```text
/home/xhj/liftrace/
  README.md                                      # 项目总览
  VISION_2026_ROADMAP.md                         # 视觉组唯一任务、架构与验收来源
  SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md      # 本机 ROS Noetic + PX4 + Gazebo + QGC 仿真手册
  VISION_WORKSPACE_GUIDE.md                      # 视觉工作区方案
  VISION_MIGRATION_CHECKLIST.md                  # 视觉迁移清单
  VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md    # RKNN / OrangePi 板端部署 Gate
  docs/
    docs/仿真联调变更记录.md                       # 长期联调变更台账
  patrol_uav_ws-patrol_planner/                  # 主集成工作区：导航、建图、规划、控制、投递执行
  Desktop_patrol_uav_ws-patrol_planner/          # 历史/次级副本：只用于对照
  Visual/                                        # 原视觉工作区：yolov5_detect、depth 等
  detect_ws/                                     # 相机与检测实验工作区
  Desktop_misc/                                  # 桌面散落源码、launch、图像检测脚本
  top_level_scripts/                             # 板端一键启动脚本
  vision_ws/                                     # 视觉组推荐日常开发工作区
```

### 3.1 主集成工作区

```bash
/home/xhj/liftrace/patrol_uav_ws-patrol_planner
```

关键模块：

```text
src/FAST_LIO/           # Livox MID360 + IMU 定位
src/FreeDOM/            # 点云地图处理，输出 /freedom/static_pointcloud
src/Fast-Planner/       # 在线建图、路径搜索、轨迹优化、轨迹服务器、内部仿真器
src/patrol_control/     # 任务状态机、航点逻辑、检测点逻辑、投递/降落流程
src/actuator_pwm/       # 舵机/投递机构控制，实机相关
src/tool/               # cv_bridge、catkin_simple、Livox 驱动等支撑代码
```

### 3.2 视觉开发主工作区

```bash
/home/xhj/liftrace/vision_ws
```

推荐视觉组只在这里做日常开发：

```text
vision_ws/src/uav_vision/      # 正式沉淀包，新代码优先放这里
vision_ws/src/yolov5_detect/   # 历史 YOLO/RKNN 参考
vision_ws/src/detect_pkg/      # 历史检测消息、脚本、图像发布工具
vision_ws/src/camera_sdk/      # 相机输入
vision_ws/migration_refs/patrol_control_visual/  # 从 patrol_control 抽取出的视觉节点参考快照，不参与编译
```

`migration_refs` 只作为参考，不要直接放进 `src/` 编译。

### 3.3 代码仓库、链路来源与回流方向

本目录是**视觉组主仓兼整机集成仓**，并不表示目录内所有导航、控制和第三方代码都由
视觉组维护。后续接手者必须先按下表确认 source of truth，再决定改动应提交到哪个仓库：

| 链路/资产 | 权威来源 | 当前本地位置或集成落点 | 维护归属与改动去向 |
| --- | --- | --- | --- |
| 视觉组主仓、完整集成基线 | `https://github.com/Qinling-Melon-Farmers/liftrace-visionwork.git` | `/home/xhj/liftrace`，Git remote `origin` | 视觉链、视觉接口、项目专用仿真资产、集成适配器和文档在本仓 feature 分支开发；验证后由人工确认合并 `main` |
| 导航组开发仓 | `https://github.com/sakelier/liftrace-controlwork.git` | 只读参考 clone `/home/xhj/liftrace-controlwork-nav`；经确认的成果集成到 `patrol_uav_ws-patrol_planner/` | 2026 搜索/任务 manager、候选策略及导航组后续规划改动以该仓为准；应先在导航组仓 feature 分支修改和验证，再记录 branch/commit 同步到本集成仓，禁止在本仓悄悄形成导航组源码分叉 |
| 2025 机载旧链基线 | 香橙派机载电脑拷回的工程；当前未登记独立权威远端 | `patrol_uav_ws-patrol_planner/`、`Desktop_patrol_uav_ws-patrol_planner/`、`Visual/`、`detect_ws/` 等 | 作为历史兼容和回归基线保留；旧 `patrol_control`、`actuator_pwm` 等修改遵守规则 14，并由控制/规划组确认去向 |
| PX4 SITL/飞控依赖 | `https://github.com/PX4/PX4-Autopilot.git` | `/home/xhj/PX4-Autopilot`，位于本仓外 | 保持独立仓库；不要把 PX4 源码复制进视觉主仓。确需修改时在其独立分支维护，并在集成文档记录 revision |
| AstraDroneOpen 仿真底座 | `https://gitee.com/lulese/AstraDroneOpen.git` | `/home/xhj/AstraDroneOpen`，位于本仓外 | 通用底座修改留在独立仓；本项目派生的 `toudi4_copy.world`、单下视机架和模型资产留在视觉主仓并记录来源 |
| ROS Noetic/MAVROS/Gazebo 等系统依赖 | Ubuntu/ROS 系统包与本机安装 | `/opt/ros/noetic` 及系统路径 | 不纳入项目 Git；版本或安装变更写入环境/仿真文档 |
| 仿真日志、bag、模型权重和数据集 | 运行或训练生成物 | `logs/`、训练输出目录及外部数据目录 | 不进入 Git；通过 run 目录、报告或单独 ZIP 交接，仓库只提交可复现配置和结果摘要 |

当前完整联调链的代码来源关系为：

```text
PX4/Gazebo/Astra（外部依赖与仿真底座）
  -> MAVROS
  -> FAST_LIO -> FreeDOM -> Fast-Planner（2025 集成基线，后续导航组主责）
  -> 导航组 Search/Mission Manager（liftrace-controlwork）
  -> 本仓集成适配器 -> 旧 patrol_control / 安全投递代理

单下视相机
  -> uav_vision 检测/精修/地图投影/记忆/对准（liftrace-visionwork）
  -> ROS 消息接口交给导航 manager 和旧控制兼容层
```

跨仓同步必须遵守以下规则：

1. 同步前记录来源仓库 URL、branch 和 commit；只迁移已确认的文件，不用整目录覆盖。
2. 导航组拥有的 `uav_mission` manager/策略、Fast-Planner 和任务调度改动，优先回到
   `liftrace-controlwork` 的 feature 分支；本仓只集成明确 revision，并把视觉—导航胶水与
   Gate 保持在可辨识的外围文件中。
3. `uav_vision` 的正式实现和消息契约以 `liftrace-visionwork` 为准；导航仓应通过 ROS
   接口消费，若需复制联调版本必须记录来源 revision，不得在两仓分别演进同名视觉源码。
4. 跨组公共接口变更先形成书面约定，并在两个仓库各自的变更记录/说明中互相引用；不得由
   agent 单方面推送另一个组的 `main`。
5. 仓库 URL 可以写入文档，token、credential 和带凭据 URL 仍严格按规则 15 禁止入库。

---

## 4. 2026 规则驱动的技术重点

2026 年相对 2025 年的重要变化：

1. **不再给出靶标的大致位置坐标。** 视觉系统不能只做末端识别，必须支持自主搜索、候选记忆、目标确认和排序。
2. **避障分只有不发生任何碰撞才能得到。** 规划/控制要降低无效穿行，视觉要尽早给出目标候选和搜索终止依据。
3. **任务描述强调“自主搜索并找到合适靶标”。** 2025 工程中“飞到预设检测点再识别”的思路需要升级。
4. **投递得分高度依赖落点。** 视觉不能只输出类别，还要输出可用于投递的靶心/中心偏差/释放许可。

关键规则参数：

```text
场地：10 m × 10 m × 4 m
隔离墙高度：1.5 m
障碍通道宽度：1.5 m
障碍门宽：约 0.8 m，可左右调整
标准投放区：4 个，单个 1 m × 1 m，蓝白圆环 + 中心图案
随机投放区：红色十字，0.35 m × 0.35 m
快递盒：单个约 100 g，允许 ±5% 误差
单次飞行：不超过 10 min
飞行高度：不得高于 4 m
```

无人机约束：

```text
必须使用开源飞控
总重量 ≤ 3 kg
轴距 ≤ 500 mm
必须配备桨叶保护装置
必须配备遥控器和紧急停止开关
不允许使用差分 GPS / 差分北斗等高精度定位设备
严禁使用品牌、商用或成品无人机
```

计分相关重点：

```text
快递携带：1 分
自主起飞：10 分
非障碍区避障：10 分，要求无碰撞
快递投送：最高 72.5 分
障碍区过门：最多 30 分；无撞击通过一扇门 15 分，撞击通过 10 分
自主降落：10 分，压边 5 分
标准图案权重：tent=1, pillbox=1.5, bridge=2, panzer=2.5, tank=5, red_cross=10
```

注意：`tank` 得分权重已确认（tank=5），此前"权重表述存在缺口"的标注已由补充规则确认解除。

---

## 5. 推荐系统架构

### 5.1 现有 2025 工程链路

```text
PX4 + MAVROS
  -> FAST_LIO + FreeDOM
  -> Fast-Planner
  -> patrol_control
  -> 视觉检测 / 投递 / 降落 / actuator_pwm
```

关键话题：

```text
/mavros/local_position/pose
/mavros/local_position/odom
/livox/lidar
/freedom/static_pointcloud
/planning/pos_cmd
/iris_mid360/camera/rgb/image_raw
/detect/waypoint_mark_point
/detect/land_mark_point
```

### 5.2 2026 视觉组推荐架构

`uav_vision` 建议拆成 6 个逻辑节点：

```text
camera/depth/odom/tf
  -> vision_camera_adapter
  -> target_detector_rknn
  -> target_refiner
  -> target_memory
  -> drop_aligner
  -> patrol_control / planner / actuator
```

职责：

```text
vision_camera_adapter   # 统一相机图像、camera_info、frame_id、压缩图开关
target_detector_rknn    # RKNN/NPU 粗检测，输出标准目标和红十字候选框
target_refiner          # ROI 几何精修，圆环/红十字/黑色外圈/靶心中心
target_memory           # 多帧融合、去重、状态机、目标位置记忆
drop_aligner            # 投递中心偏差、释放许可、旧 /detect/* 兼容输出
vision_recorder         # rosbag、debug image、性能统计，可开关
```

新增接口优先使用：

```text
/uav_vision/detections
/uav_vision/targets
/uav_vision/selected_target
/uav_vision/drop_offset
/uav_vision/drop_ready
/uav_vision/debug_image
/uav_vision/perf
```

短期必须兼容：

```text
/detect/waypoint_mark_point
/detect/land_mark_point
/detect/tank_status
/yolo_detect
/detect/status
```

---

## 6. OrangePi 5 Plus 部署约束

面向 RK3588 / OrangePi 5 Plus，默认策略：

```text
相机输入：20-30 FPS，ROS queue_size=1，避免图像积压
目标粗识别：RKNN runtime，搜索阶段 5-10 Hz
红十字精修：HSV + 轮廓 + 形态学，ROI 优先，15-20 Hz
圆环/靶心精修：颜色分割 + 椭圆/圆环拟合 + 质量评分，15-20 Hz
目标记忆：CPU 轻量状态表，5-10 Hz
投影定位：image_geometry + TF + 高度/深度，约 10 Hz
诊断记录：rosbag/debug image/perf 默认关闭，调试时开启
```

禁止作为实机主路径：

```text
OrangePi 上直接跑 ultralytics.YOLO / PyTorch .pt 大模型
多套全图 YOLO 同时常开
SAM、YOLOv8/YOLO11 大模型直接上板端实时路径
无队列限制的图像订阅
把相机内参、camera_link、图像话题写死在源码里
```

---

## 7. 当前进展和已知状态

以下内容来自前序开发会话，应作为后续接力基础，但仍需以当前仓库实际文件为准重新核验。

**本节为历史快照：任务优先级与 Gate 状态一律以 `VISION_2026_ROADMAP.md` 为准（唯一任务来源），本节仅保留前序会话的过程性记录。最新进展见 `docs/仿真联调变更记录.md` 尾部条目。**

### 7.1 已完成

1. 已从香橙派 `orangepi@192.168.3.15` 拷回机载项目，并上移到 `/home/xhj/liftrace`。
2. 已整理项目级文档：
   - `README.md`
   - `VISION_2026_ROADMAP.md`
   - `SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md`
   - `VISION_WORKSPACE_GUIDE.md`
   - `VISION_MIGRATION_CHECKLIST.md`
   - `VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md`
3. 已准备视觉工作区：
   - `vision_ws/src/uav_vision`
   - `vision_ws/src/yolov5_detect`
   - `vision_ws/src/detect_pkg`
   - `vision_ws/src/camera_sdk`
   - `vision_ws/migration_refs/patrol_control_visual`
4. `toudi3.world` 的五类标准靶材质、`red_cross`、`landing_h` 模型和仓库内 `patrol_world.launch` 已重建；旧链专用路线在 `patrol_control/config/patrol_toudi3.yaml`，完整资产状态见 `TOUDI3_SIM_ASSET_CHECKLIST.md`，完整操作见 `docs/TOUDI3_FULL_SIM_GUI_GUIDE.md`。2026-08-12 起默认仿真世界已切换为 `toudi4_copy.world` + 单下视相机机架（见 `docs/TOUDI4仿真机架与世界切换说明_20260812.md`），toudi3 保留作历史回归。
5. 本机上级目录存在 `AstraDroneOpen`，作为 PX4/Gazebo 仿真底座；PX4 中已有 `iris_mid360` 和 `mid360` 模型，并存在 `astra_example.launch`。
6. 前序会话中主工作区曾经修到可以编译通过，但后续 agent 必须重新执行验证，不可直接假定当前环境一定一致。
7. V-SIM-00～03 最小基建已落地：联合 overlay、`uav_vision_eval` 独立真值/报告和
   headless shadow 均已有自动验证；固定场景直接复用 world 现有标准靶/H，红十字按场景插入。
8. 当前所有模型推理、Gazebo 识别和视频回放都运行在笔记本 WSL2/RTX；统一六分类链
   已于 2026-08-10 在 OrangePi 5 Plus 以 FP32 RKNN 离线实测（16fps，五路对比推荐
   `merged_standard_fp32.rknn`，详见 `docs/板端推理性能对比报告_20260810.md`），但
   板端 ROS 实时视觉链（CameraInfo/TF 接线、10 min 稳定性）仍未验收。任何 agent
   不得把笔记本结果写成板端结果。
9. `uav_mission` 已建立第一版任务层释放许可、旧 `/Servo` 安全代理和 raw mock；确定性
   回归已验证顺序三槽、错槽、过期、重放及重复目标拒绝。旧控制三投 SITL（V-CL-02）
   已于 2026-08-06 通过（新视觉+旧控制固定三投 PASS）。2026-08-13（0869c37）对
   `patrol_control` 打了外部任务模式最小补丁（修复前快照存于
   `legacy_baseline/20260813/`）；`actuator_pwm` 与 `Servo.srv` 仍保持未改。
10. V-CL-00B/01 的 L0 已落地：Phase D/板端以 `camera_init` 为默认 mission frame，消息
    显式携带中心来源、关联、TF 年龄和拒绝原因；无效 TF 不进入候选；物理地图 ID 使用
    连续帧、置信度和类别投票，地图点使用质量加权融合。完整 SITL/跨视角仍待验收。

### 7.2 前序编译修复点

如当前仓库仍保留前序修改，修复点大致包括：

```text
src/tool/cv_bridge/CMakeLists.txt
  - 修 OpenCV 版本硬编码，优先适配 OpenCV 4，再回退 OpenCV 3

src/Fast-Planner/uav_simulator/local_sensing/CMakeLists.txt
src/Fast-Planner/uav_simulator/local_sensing/package.xml
  - 去掉失效的 cmake_modules 依赖

src/actuator_pwm/CMakeLists.txt
  - 补 actuator_pwm 对 patrol_control 服务/消息生成的显式依赖

src/Fast-Planner/fast_planner/bspline_opt/CMakeLists.txt
  - 去掉 /usr/local/lib/libnlopt.so 硬编码，改为标准查找 nlopt/nlopt_cxx

src/patrol_control/launch/patrol_control_sim.launch
src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_sim.launch
  - 修内部仿真里程计话题错配，避免默认拿 /mavros/local_position/odom 跑内部 simulator
```

### 7.3 当前主要卡点

1. 视觉任务闭环已打通：V-CL-02 固定三投、V-CL-03 外部单候选通过；V-CL-04 覆盖
   搜索三投闭环实跑（v2：12/12 覆盖、tank/panzer/bridge 3/3、0 碰撞、377.6 s 降落，
   Gate 断言已按全程事实修复）；V-CL-05 高权重中断投递 + red_cross 统一入队 +
   随机红十字摆放已落地，中断机制实跑验证（vcl05b：tank 中断→恢复搜索、pillbox
   端到端投出）。剩余阻断：Fast-Planner 低空下降/返航可达性波动（规划组）、
   H 视觉降落与北区走廊 Gate（联合）、30-seed/实拍真值/板端 ROS 链（量化/部署组）。
2. 30-seed、10 min 和延迟继续记录，但不再阻塞 V-CL；搜索阶段统一 P95 `<=200 ms`
   后移，陈旧数据、持续积压和错误释放仍是硬失败。
3. 实拍圆环 letterbox 回放自洽关联率 58.80%，但缺人工实例/中心真值和同步 pose，
   不能宣称关联召回或地图误差达标。
4. 地图新鲜度/stable ID/reset 已有 mock 和固定场景，仍缺 30-seed 跨视角与实拍同步位姿。
5. H 内部结构和阶段门控已实现，仍缺真实 H、普通黑圈、残圈负样本 Gate。
6. `release_evidence` 已实现；任务层第一版 `release_permission` 和旧 Servo 安全代理已通过
   mock 回归；旧控制三投 SITL 已通过（见 7.1-9），剩余为更多飞行/机构互锁与
   toudi4 三投完整验收。
7. 旧控制没有全场自主搜索。Search/Mission Manager 属于控制/规划组，视觉组交付候选、地图点、质量、年龄和拒绝原因。
8. 内部仿真的 Fast-Planner 锁异常和四元数问题仍在，但不作为视觉仿真评测基建的前置阻塞；LIO/Planner 当前不替换。

---

## 8. 常用命令

### 8.1 环境变量

```bash
export PROJECT_ROOT=/home/xhj/liftrace
export UAV_WS=$PROJECT_ROOT/patrol_uav_ws-patrol_planner
export VISION_WS=$PROJECT_ROOT/vision_ws
export PX4_ROOT=/home/xhj/PX4-Autopilot
export QGC_APP=/home/xhj/QGC/QGroundControl.AppImage
source /opt/ros/noetic/setup.bash
```

### 8.2 编译主工作区

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
cd /home/xhj/liftrace/patrol_uav_ws-patrol_planner
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES="" -j1
```

若并行编译出现消息生成抢跑，先用串行定位：

```bash
catkin_make -DROS_EDITION=ROS1 -j1
```

若需要抓取日志：

```bash
catkin_make -DROS_EDITION=ROS1 2>&1 | tee /tmp/liftrace_catkin_make.log
tail -n 120 /tmp/liftrace_catkin_make.log
```

### 8.3 编译视觉工作区

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/vision_ws
catkin_make
```

### 8.4 启动 QGroundControl

```bash
chmod +x /home/xhj/QGC/QGroundControl.AppImage
/home/xhj/QGC/QGroundControl.AppImage
```

### 8.5 PX4 + Gazebo + MAVROS 基础链路

优先使用本机已有 PX4 与 AstraDroneOpen 模型。启动前必须保证 ROS 包路径没有覆盖掉 `/opt/ros/noetic/share`。

```bash
source /opt/ros/noetic/setup.bash
export PX4_ROOT=/home/xhj/PX4-Autopilot
export SITL_GAZEBO=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export ROS_PACKAGE_PATH=/opt/ros/noetic/share:$ROS_PACKAGE_PATH:$PX4_ROOT:$SITL_GAZEBO
export GAZEBO_MODEL_PATH=$SITL_GAZEBO/models:${GAZEBO_MODEL_PATH}
roslaunch px4 astra_example.launch vehicle:=iris_mid360
```

验证：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/odom
```

### 8.6 启动 liftrace 内部仿真链路

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/patrol_uav_ws-patrol_planner
source devel/setup.bash
roslaunch patrol_control patrol_control_sim.launch
```

若崩溃，先清理 ROS 残留：

```bash
pkill -f roscore || true
pkill -f roslaunch || true
rm -rf ~/.ros/log
mkdir -p ~/.ros/log
```

然后重新单独启动，不要和 PX4/Gazebo 另一路 launch 混跑。

### 8.7 检查话题

```bash
rostopic list | grep -E 'mavros|livox|freedom|planning|detect|uav_vision|camera'
rostopic echo -n 1 /freedom/static_pointcloud
rostopic echo -n 1 /planning/pos_cmd
rostopic echo -n 1 /iris_mid360/camera/rgb/image_raw/header
rostopic echo -n 1 /detect/waypoint_mark_point
rostopic echo -n 1 /detect/land_mark_point
```

---

## 9. 建议优先级

具体 ID、依赖和阈值只在 `VISION_2026_ROADMAP.md` 维护。接手时按下列 Gate 顺序：

### P0：打通安全的新视觉任务闭环

1. 保持 `uav_mission` 释放许可/旧 Servo 安全代理回归通过，接入完整旧控制固定三投 SITL。
2. 统一 mission frame、图像源时间、TF 年龄、地图质量和无效原因契约。
3. target memory 改为按物理地图位置保持 stable ID，类别使用时序投票。
4. 三个固定检测点依次完成视觉证据→任务许可→mock ACK，禁止超时和重复投放。

### P1：复用导航完成候选接近与搜索恢复

1. 为旧控制提供独立外部任务模式，不删除旧固定路线；保证 Mission Manager 独占目标入口。
2. 用 `/fastplanner/goal` 完成单候选接近、重捕获、投放和恢复搜索。
3. 扩为非靶标坐标覆盖航线、候选权重队列和三次投放。

### P2：闭环稳定后的量化、实拍和部署

1. 根据闭环失败阶段处理低召回、地图误差、30-seed 和 10 min 稳定性。
2. 补实拍实例/中心/H/红十字负样本与同步 CameraInfo/pose。
3. 修复 PT/ONNX 差异；获得板端后做 RKNN/OrangePi 验收。

---

## 10. 代码风格与工程约定

1. ROS1 / Catkin 工程保持现有 C++14 风格，除非包内已有更高标准要求。
2. C++ 节点新增参数必须通过 private nh 读取，例如：

```cpp
ros::NodeHandle pnh("~");
pnh.param<std::string>("image_topic", image_topic, "/camera/color/image_raw");
```

3. Python 节点必须有可执行 shebang，并在 `CMakeLists.txt` 中正确安装或说明运行方式。
4. launch 文件中不要写死绝对路径，优先使用 `$(find package_name)`、`arg` 和 yaml。
5. yaml 参数要保留注释，说明单位、frame、默认值来源。
6. 新增视觉调试图默认关闭：

```xml
<arg name="enable_debug_image" default="false" />
```

7. 图像订阅默认 `queue_size=1`，防止实机积压。
8. 修改 `CMakeLists.txt` 后必须实际跑 `catkin_make` 或至少说明未跑的原因。
9. 发现历史重复文件、`.bak`、`.beifen`、`.back`，先列清单，不要直接删。
10. 不要把模型权重、bag、大型视频、build/devel 产物加入版本管理建议中。

---

## 11. Agent 工作流模板

每次执行开发任务时，按以下格式向用户汇报：

```text
目标：本次要解决什么问题。
查看：读取了哪些文件/launch/yaml/log。
判断：当前问题边界是什么，哪些不是本次问题。
修改：改了哪些文件，每个文件为什么改。
验证：执行了哪些命令，结果是什么。
风险：还有哪些未验证点或潜在副作用。
下一步：建议接着做的一件事。
```

当任务涉及实机、舵机、飞控模式、起飞、投递时，必须先停止并请求确认。

---

## 12. 不要做的事

```text
不要直接把 vision_ws/migration_refs 放进 src 编译。
不要在 patrol_control 中继续堆新的视觉逻辑，除非是短期兼容桥。
不要把 PyTorch/ultralytics 当成 OrangePi 实机主推理路径。
不要把现有 toudi3 新视觉 GUI 烟测误写成带真值的算法验收。
不要把 source / setup 脚本导致的 ROS_PACKAGE_PATH 问题误判为 ROS 损坏。
不要同时跑多套 roscore/roslaunch 再调内部仿真。
不要在未确认的情况下运行 actuator_pwm、解锁、起飞、投递。
不要根据 2025 固定靶标坐标逻辑设计 2026 搜索系统。
```

---

## 13. 必读文件顺序

建议新 agent 按以下顺序接手：

```text
1. AGENTS.md
2. README.md
3. VISION_2026_ROADMAP.md
4. VISION_WORKSPACE_GUIDE.md
5. vision_ws/src/uav_vision/README.md
6. SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md
7. docs/TOUDI3_FULL_SIM_GUI_GUIDE.md
8. VISION_MIGRATION_CHECKLIST.md
9. docs/当前问题与责任边界.md
10. VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md
11. docs/仿真联调变更记录.md（读取文末最近记录）
```

---

## 14. 当前最推荐的下一步

当前最务实的下一步是把视觉结果变成可审计的任务闭环：

1. 在保持旧控制源码不变的前提下，用 guarded `/Servo` 完成三个固定检测点 mock 三投；
2. 统一 mission frame 与地图投影有效性，避免错误地图点进入导航；
3. 完成物理 stable ID、类别投票和任务侧 delivered/failed 去重；
4. 再用 Fast-Planner 实现单候选接近、恢复搜索和全场三投；
5. 闭环稳定后再按真实失败阶段收紧 30-seed、召回、延迟和板端性能。

任何 agent 接手时，应先核对 `VISION_2026_ROADMAP.md` 的首个未完成任务和 `VISION_MIGRATION_CHECKLIST.md` 对应 Gate，避免重新实现已存在的节点或把 GUI 有输出误当算法通过。
