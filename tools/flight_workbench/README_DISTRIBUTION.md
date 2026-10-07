# 飞行工作台分发包

本包只包含笔记本侧工作台。无需完整工程、ROS、模型或 WSL。启动监听 localhost，等待用户点击连接；不会自动 SSH、启动设备、解锁或起飞。

## Windows 原生启动

使用已有 Windows Python 3.9+ 环境及 PyYAML、pexpect、Paramiko（版本范围见 requirements.txt）。pexpect 仅复用异常类型，Windows 不使用 Unix PTY。脚本检查依赖，不安装包或新建环境。本机已用 Windows Anaconda Python 3.12 验证，未在未安装 Python 的干净 Windows 上验证。

1. 完整解压 ZIP 到本地目录，例如 C:\flight_workbench；支持中文和空格目录。
2. 双击 start_windows.bat（start_windows.cmd 同效），或 PowerShell 执行：
   `./start_windows.ps1 -Python D:\Anaconda3\python.exe -Open`
3. 仅离线看页面：`./start_windows.ps1 -Transport local`。Windows local 模式禁止运行 Linux 设备命令。
4. 默认请求 8771；以终端实际打印的 URL 为准。日志链接 /logs 与实时观察 /observe 跟随当前页面 host/port。端口被占用会顺延；可用 -Port 指定。
5. 需要独立状态时用 `-ProfileDir C:\你的私有目录\workbench_state`。此处是 Windows 路径。包内没有密码、密钥或个人 profile。
6. 正常收尾后 Ctrl+C；不在设备会话运行时替换后端。独立日志后端可用 `-LogsOnly -Port 8772`，不启动探针或设备。

Windows 用 Paramiko SSH channel 复用终端、Ctrl+C、resize 和二进制下载。支持内存密码、已有密钥/可用 agent；known_hosts 位于本用户 ~/.ssh/known_hosts。只在配置 StrictHostKeyChecking=accept-new 时接受并保存新指纹，指纹变化拒绝连接。没有自动导入 PuTTY saved sessions 或完整 OpenSSH config；页面须填准确 user@host/端口。加密私钥单独解密和不兼容的 agent 未验收。

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

独立正赛卡片调用 deployment/competition/start.sh，默认 YAML 是 deployment/competition/field.example.yaml。入口和模板由整机工程提供，未随本工具包部署。比赛参数保持既有高位 2.6m、速度 1.2m/s、加速度 1.0m/s²、前视 1.0/0.4/0.15m，具体有效参数以整机 YAML 展开为准。

正赛模板必须按 10×10 场地实测填写并确认门口、H、边界及路线。昨晚成功轮属于测试场地，不能作为正赛默认坐标或正赛验收。复现测试时另填 F 代理提供的测试示例路径，完整命令会显示实际文件。

运动优化、障碍柱均有“继承所选 YAML / 显式开启 / 显式关闭”。继承不传覆盖参数；开启/关闭传 --motion-optimization on/off、--obstacle-columns on/off，须整机入口支持这两个 CLI。先用“配置检查”查看真实有效参数与来源，不以 UI 选中冒称配置已经生效。障碍柱关闭仅移除配置柱，真实点云避障仍依入口配置。工作台不修改飞行数值。

保留配置检查、preview、实投和原有双重确认/任务互斥。配置检查不启动 ROS；preview 可能启动地面应用链，现场应先配置检查。独立正赛 flight 只允许真实投递，后端拒绝伪装为 mock。

现有现场/专项卡片、低空观察三卡、日志录制下载、终端和状态保持可用。取消独立电机大页入口；/observe 保留实时位姿、电机原始输出、映射与 ESC 摘要；旧 /motor 跳转 /observe。RC OUT 是输出指令，不能等同电流/RPM；浏览器片段只记录已接收数据，不启动动作。

日志录制/索引/取回只在用户点击后调用现有脚本，下载失败或超时丢弃部分文件，没有断点续传。READY 是应用状态，飞手仍须依现场流程解锁/切 OFFBOARD/接管；保留低空观察收尾保护、舵机独立确认和 probe watchdog 边界。

## 包边界

白名单打包：服务、SSH 适配、配置、网页、只读探针、启动脚本和 manifest。没有板端工程、模型、密码、私钥、个人状态或运行日志。manifest 标明源码 HEAD 和未提交文件；此包不是板端发布或实飞验收。

本轮 Windows 原生离线 UI/HTTP、解压启动及本机 SSH fixture 的验证结果见仓库 docs/verification/workbench_release_20261008/REPORT.md。未连接真实板端，未验收真实网络、ROS、舵机、飞行或大 bag 的现场传输。
