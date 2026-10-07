# 前线试飞工作台：先读这里

本包只装笔记本侧工作台。无需克隆完整仓库，也不需要在笔记本安装 ROS、模型、相机或雷达驱动。启动后只监听 `127.0.0.1`，不会自动连接飞机；由用户在网页输入连接信息并点击连接。

## Windows 使用

**支持 Windows 浏览器 + WSL Linux 后端，不支持 Windows 原生 Python。** 终端层依赖 Linux PTY、bash 和 OpenSSH。须先有可用 WSL Ubuntu，并设为默认发行版（`wsl --list --verbose` 可查看）。已装 WSL 的队员不用重复安装。

1. 将 ZIP 完整解压到普通本地目录，例如 `C:\flight_workbench`；中文目录也可。不要在压缩包内直接双击，不要把解压目录放在 WSL 的网络路径下使用 Windows 启动脚本。
2. 检查下节依赖已经安装在默认 WSL 中。双击 `start_windows.cmd`，保留弹出的终端窗口。
3. 在 Windows 浏览器打开 `http://127.0.0.1:8791/`。默认端口被占用时，以终端打印的地址为准；更换端口可在 PowerShell 执行 `./start_windows.ps1 -Port 8792`。
4. 只看界面且禁止设备命令：`./start_windows.ps1 -Transport local`。现场正常使用默认 `ssh`；这仍需网页点击连接后才会联络板端。
5. 关闭工作台：先完成现场接管、落地与原入口收尾，再在本窗口按 Ctrl+C。刷新网页只重载页面，不重启设备/探针或应用链。

Windows 可通过 WSL localhost 转发访问后端。若系统禁用了此功能，需恢复 WSL localhost 转发；本包不开放局域网监听作为替代。

## Linux / WSL 依赖与启动

后端需要 Python 3.9–3.12、`pexpect`、`PyYAML`，以及系统命令 `bash`、`ssh`、`scp`；Windows 启动脚本还用到 `wslpath` 和 `base64`。依赖版本范围见 `requirements.txt`。前端为随包原生 HTML/CSS/JS，无 Node、npm、CDN 或构建步骤。

已有本项目环境优先使用：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
cd /实际解压目录/liftrace_flight_workbench_时间戳
bash start_workbench.sh
```

Windows 脚本检测到 `~/miniconda3` 时使用其中已有的 `rl_drone`；不存在该环境则报错，须由本机维护者配置，不会自动新建环境。本机 conda 缺 `pexpect` 时，启动脚本可复用已安装的 `/usr/lib/python3/dist-packages` 纯 Python 包。

新队员的普通 Ubuntu（非项目开发机）可由本机维护者通过发行版包管理器准备 `python3`、`python3-pexpect`、`python3-yaml`、`openssh-client`；Python 须满足上述版本。自行管理的 Python 环境可参考 `requirements.txt`。本包不会安装依赖，也不要在项目系统 Python 中 pip 安装。

若本机已配置其他允许使用的 Python，用 `WORKBENCH_PYTHON=/实际/python bash start_workbench.sh`。Linux 原生用户在解压目录运行同一脚本；无需 Windows 启动脚本。

## 页面功能和现场使用

主页保留现有现场任务卡、设备终端、配置检查、命令预览、状态/READY、运行日志、产物浏览、回报、手填坐标和搜索航线/FOV 图；`/observe` 为实时观察，`/motor` 为电机诊断。低空观察区三张卡片依次为 **FC AGL 0.60m 悬停、短程前移、矩形**，复用板端 `deployment/low_hover_observation/start.sh` 的 `hover / forward / square`。

选低空卡片再点一键设备，使用已有独立定位/EV 入口；不自动启动相机、视觉、规划、任务管理器或舵机。低空卡片的配置检查和 preview 都是原入口离线 preview。常规任务的 preview 可能启动地面应用链，先用独立配置检查。

网页连接设置需核对 SSH 地址、用户名、端口、板端部署根和环境脚本。默认值来自现有现场工程，换飞机/网络必须改成实际值。密码默认只在内存中；不要勾选“记住口令”后把个人状态目录发给他人。连通网络和 OpenSSH 主机指纹问题需按现场既有流程解决。

飞行与舵机按钮仍要求原确认；READY 只表示应用链就绪。低空观察等到 `READY_FOR_MANUAL_ARM_AND_OFFBOARD` 后由飞手人工解锁并重新拨入 OFFBOARD，结束继续悬停，由飞手接管落地上锁。Ctrl+C 请求保持接管，必须等 `OBSERVATION_CLOSED` 后才全停/断连。观察页的“本地片段”只记录浏览器已收到的数据，不触发飞机动作。

手动坐标/自动搜索路线使用现有 API，板端仍需具备对应生成器与版本；理想 FOV 覆盖不是实际避障覆盖或目标召回率。电机映射只对配置登记的目标有效，RC 输出不能当作电流或 RPM。

## 包内容与限制

- 服务、SSH 会话层、状态/板端接口、几何/航线工具、只读探针、现场 YAML、完整静态网页、电机接线 JSON。
- Linux 与 Windows→WSL 启动脚本、依赖说明、本说明和 `manifest.json`。清单标明来源 HEAD 及未提交工作台文件；不是板端发布版本。
- 不含密码、私钥、个人 profile、会话、日志、bag、视频、模型、飞行工程或舵机包。不要把此包覆盖到板端。

已有板端部署必须具备当前专项入口、ROS 环境、消息/服务与生成器。启动本地网页不能安装或更新板端，也不能证明板端已实飞验收。板端大 bag 用现场既有 scp/rsync 回传；工作台仅预览及下载小文件。探针更新需明确重连，网页刷新不会替换已有探针。

个人状态默认保存在 `~/.config/liftrace-flight-workbench/`，与解压目录分开；不同用户的口令/记录不会从本包导入。本机已有 profile 可能覆盖包内默认地址，连接前应在页面核对。Linux 需要独立空状态可加 `--profile-dir /本机私有状态目录`；Windows 则用 `./start_windows.ps1 -ProfileDir /home/你的WSL用户名/workbench_state`，此参数填写 WSL 内路径。只监听 localhost；本包不提供群内公网访问链接。
