# 飞行工作台分发包

本包只包含笔记本侧工作台。无需完整工程、ROS、模型或 WSL。启动监听 localhost，等待用户点击连接；不会自动 SSH、启动设备、解锁或起飞。

Bag 回放入口 `/replay` 可发现、播放已有结果，也可导入本机 bag 生成回放和分析。
前线新增的原生离线后端支持 Windows/Linux，不要求 WSL；需现有 Python 3.10+、
rosbags、NumPy、SciPy、OpenCV、Matplotlib，以及 FFmpeg/ffprobe。依赖不足时页面会提示；
工作台不自动安装依赖。WSL 可使用已有 ROS1 读取环境。原 ULog 上传、解析和导出保留。

## Windows 原生启动

使用已有 Windows Python 3.10+ 环境及 PyYAML、pexpect、Paramiko（版本范围见 requirements.txt）；离线 ULog 分析另需已有 numpy、matplotlib。pyulog 1.2.4 的读取源码及 BSD 许可随包提供，优先使用环境内已安装版本，无需另装 pyulog。pexpect 仅复用异常类型，Windows 不使用 Unix PTY。脚本检查依赖，不安装包或新建环境。本机已用 Windows Anaconda Python 3.12 验证，未在未安装 Python 的干净 Windows 上验证。

1. 完整解压 ZIP 到本地目录，例如 C:\flight_workbench；支持中文和空格目录。
2. 双击 start_windows.bat（start_windows.cmd 同效），或 PowerShell 执行：
   `./start_windows.ps1 -Python D:\Anaconda3\python.exe -Open`
3. 仅离线看页面：`./start_windows.ps1 -Transport local`。Windows local 模式禁止运行 Linux 设备命令。
4. 默认请求 8771；以终端实际打印的 URL 为准。日志链接 /logs 与实时观察 /observe 跟随当前页面 host/port。端口被占用会顺延；可用 -Port 指定。
5. 需要独立状态时用 `-ProfileDir C:\你的私有目录\workbench_state`。此处是 Windows 路径。包内没有密码、密钥或个人 profile。
6. 正常收尾后 Ctrl+C；不在设备会话运行时替换后端。独立日志后端可用 `-LogsOnly -Port 8772`，不启动探针或设备。

Windows 用 Paramiko SSH channel 复用终端、Ctrl+C、resize 和二进制下载。支持内存密码、已有密钥/可用 agent；known_hosts 位于本用户 ~/.ssh/known_hosts。只在配置 StrictHostKeyChecking=accept-new 时接受并保存新指纹，指纹变化拒绝连接。没有自动导入 PuTTY saved sessions 或完整 OpenSSH config；页面须填准确 user@host/端口。加密私钥单独解密和不兼容的 agent 未验收。

连接设置支持`identity_file`本机私钥路径：WSL/OpenSSH使用该文件并限制密钥选择，Windows/Paramiko将其作为key_filename；留空恢复默认密钥/agent。路径保存到使用者的本机profile，ZIP不含私钥或个人profile，Windows需填写Windows可访问路径，不能直接沿用WSL的/home路径。仅保存连接参数不自动连接、不启动探针或设备。

独立正赛环境的通用预检检查根内field.example.yaml的存在及site_confirmed文件声明，不要求08的test_area.yaml或专项模块；模板检查不代表现场确认或飞行配置验收。切独立根时model/metadata必须属于该根，根内绝对路径会转成相对路径，根外或越界路径拒绝保存。

Windows 无法核查远端 PTY 的 echo，因此不会自动填写 sudo 口令，5a 经确认后需终端手工认证；Linux 保留原有有限次数且确认 echo 关闭的应答。SSH 登录密码不出现在命令行、ZIP、终端日志或 URL。记住口令仍是用户明确可选操作，只写个人状态目录；不要把状态目录打包分享。

## Linux / WSL

使用已有 rl_drone：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
bash start_workbench.sh
```

Linux 需要 Python 3.9+、pexpect、PyYAML、bash、OpenSSH，无需 Paramiko。可用 WORKBENCH_PYTHON 指向已有环境。脚本可复用系统纯 Python 依赖，不安装任何包。

## 比赛与测试

独立正赛卡片调用 deployment/competition/start.sh，默认 YAML 是 deployment/competition/field.example.yaml。入口和模板由整机工程提供，未随本工具包部署。10月8日晚候选为规划速度1.2m/s、加速度1.0m/s²，搜索前视1.0m、其余导航档0.6m，投递下降目标FC离地0.40m（释放窗口0.35～0.45m）、高位软件上限3.2m。基础模板FC高2.6m；矩形/双线候选镜头2.6m（FC2.76m）；三线候选镜头2m（FC2.16m）。具体有效参数以实际生成配置为准。

08与备用10默认使用各自site_waypoint_map_20261015.yaml，无须手填或开启实测航点；默认competition 1.2m/s、1.0m/s²，H捕获/对准FC1.1m。08高位FC2.6m，备用10选择牛耕/回字和FC1.8/1.9m。手动override优先；板端需先更新对应现场版本。

正赛模板必须按 10×10 场地实测填写并确认门口、H、边界及路线。昨晚成功轮属于测试场地，不能作为正赛默认坐标或正赛验收。复现测试时另填 F 代理提供的测试示例路径，完整命令会显示实际文件。

运动优化、高位续扫、障碍柱均有“继承所选 YAML / 显式开启 / 显式关闭”。继承不传覆盖参数；开启/关闭传 --motion-optimization on/off、--resume-survey on/off、--obstacle-columns on/off，须整机入口支持这些 CLI。正式模板默认开启运动优化与高位续扫，历史测试配置保留原值；二者独立。先用“配置检查”查看真实有效参数与来源。卡片分别显示所选请求、“离线配置检查”和“运行已生成配置”；没有监督器结果或改变路径/开关后显示“尚未确认”。不以 UI 选中冒称配置已经生效。障碍柱关闭仅移除配置柱，真实点云避障仍依入口配置。工作台不修改飞行数值。

保留配置检查、preview、实投和原有双重确认/任务互斥。配置检查不启动 ROS；preview 可能启动地面应用链，现场应先配置检查。独立正赛 flight 只允许真实投递，后端拒绝伪装为 mock。

现有现场/专项卡片、低空观察三卡、日志录制下载、终端和状态保持可用。取消独立电机大页入口；/observe 保留实时位姿、姿态、电池与诊断；旧 /motor 跳转 /observe。实时电机面板已移除，ULog离线电机分析保留；浏览器片段只记录已接收数据，不启动动作。

日志录制/索引/取回只在用户点击后调用现有脚本，下载失败或超时丢弃部分文件，没有断点续传。READY 是应用状态，飞手仍须依现场流程解锁/切 OFFBOARD/接管；保留低空观察收尾保护、舵机独立确认和 probe watchdog 边界。

## 2026-10-09：ULog 小工具并入工作台

日志页 `/logs` 新增本地 ULog 查看器，沿用 `tools/飞控日志小工具.zip` 的列表选取与轨迹查看思路；轨迹处理改为无需 Tk 的网页图像导出，复用已有电机/姿态/RC/电池/EKF 分析。原 ZIP 的 SSH 设置和下载目录不导入本包。

离线查看：用 `-Transport local` 启动，打开日志页，选择已完整下载的 `.ulg`（可多选，单份最大 128 MiB），点击“导入并分析”。无需 SSH、ROS 或日志 ID。本机状态目录的 `ulog_library/` 保存原文件和结果；保持同一 `-ProfileDir`，下次从“已导入/下载的 ULog”直接选择。分析在独立子进程串行执行；上次服务中断的分析显示失败，可重新导入。

可查看轨迹俯视/三维/局部高度和电机诊断图，点击图像打开原图；可下载原始 ULog、CSV、摘要及分析 ZIP。轨迹以首个有效位置为原点，NED 转 ENU，位置重置、无效位置及大于 0.5 秒间隙断线。局部位置/高度是估计值；电机是指令量。分析 ZIP 仅含派生图表和数据，原始日志通过单独按钮保存。

现场取回：连接后明确点击“查询飞控日志索引”，从返回列表选择日志，ID 自动带入，再点击“下载选中飞控日志并查看”。保持页面打开，文件封闭后自动取回本机并分析。旧取回脚本无需替换；索引辅助代码通过现有 SSH 链在板端内存中执行。只接受飞控真实 `LOG_ENTRY` ID，部分索引会提示重查；手填 ID 保留为可选入口。飞控查询/下载仍要求新鲜、已连接且未解锁状态，测试只使用 mock，未执行现场取回验收。

本次已使用已有 `board_full_mission_20261008_223632_log70.ulg` 做 WSL 与 Windows 原生离线 HTTP 上传/解析/导出验证；Windows 使用随包 pyulog 副本。真实试飞日志只作为外部测试输入，不随分发包或 Git 提交。

## 包边界

白名单打包：服务、SSH 适配、配置、网页、只读探针、启动脚本和 manifest。没有板端工程、模型、密码、私钥、个人状态或运行日志。manifest 标明源码 HEAD 和未提交文件；此包不是板端发布或实飞验收。

白名单另含 `bag_replay/run.sh`、`bag_replay.py`、`video_encoder.py` 及工具分析脚本/说明；
不打包 `merge_bags.py`、原始 bag、已有回放视频或个人库。解压后把完整结果目录放在包内
`data/`，或在 `workbench.yaml` 配置 `replay.data_roots` 的本地目录后重新启动服务；只能从
这些预定根目录选取。播放器目录需保留 `index.html`、`summary.json`、`validation.json`
和所有视频，相对资源通过目录 URL 提供，支持 HTTP Range 与拖动。渲染中或缺失视频的目录
不列为可用结果。工作台作业存于个人状态目录 `replay_library/`，可从页面查看进度与日志。
普通浏览器刷新不重启任务；服务退出中断的任务不会误报完成。
新增原生离线实现位于 offline_replay/；打包只包含程序，不含原始飞行bag或视频。

本轮 Windows 原生离线 UI/HTTP、解压启动及本机 SSH fixture 的验证结果见仓库 docs/verification/workbench_release_20261008/REPORT.md。未连接真实板端，未验收真实网络、ROS、舵机、飞行或大 bag 的现场传输。


## 2026-10-13：前线界面与正赛地图编辑

已整合前线“飞行工作台.zip”的浅色导航、04/08地图编辑、下载及离线分析改进，
保留本地 H 候选配置参数。Windows 与 WSL 使用同一份源码和分发包。

独立正赛卡片也可点击“地图调整航点”：选择搜索点、走廊门前/门后点、准备点、H点后
输入坐标或拖动；可编辑飞行/目标/搜索/低空覆盖范围与地图尺寸。
已连接时读取所选板端 YAML；离线只使用包内对应参考模板并明确提示，未知文件不以其他
模板代替。模板的空走廊和 H 必须填完；“现场已实测确认”由操作者明确勾选，保存不自动
确认。草稿按板端、工程、组别和所选 YAML 隔离，切换来源不携带旧草稿。

保存并用于本组仅保存本机草稿、生成命令。配置检查/preview/flight 使用同一坐标覆盖；
执行时在工程 logs/flight_workbench/site_overlays/ 生成独立 YAML，不覆盖原文件。
正赛覆盖经原 competition_config 校验，调用 deployment/competition/start.sh；
不依赖板端的08测试模块或新工作台模块，也不改变速度、投递补偿、H策略和模型。
只读配置检查不启动 ROS，preview/flight 仍沿用原确认与互斥。

实时状态曲线 /observe 已移除电机接线、RC OUT、ESC和电机曲线面板，保留位姿、
姿态、电池、LIO诊断及浏览器片段导出。ULog离线电机分析和板端录制不受此界面调整影响。

本包是笔记本侧工作台，不能代替板端工程部署；没有自动连接、解锁或起飞。
