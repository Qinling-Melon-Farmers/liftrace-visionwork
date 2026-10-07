# 工作台升级交付与验证（2026-10-08）

本轮只修改 B 工作树 `/home/xhj/liftrace-worktrees/r2026-board-vision-tests` 的 `tools/flight_workbench` 及相关 tests/docs。未提交、未推送、未合并，未改共享 ROADMAP/总联调台账。由主代理审查及统一处理；工作台留在 B，不能把整条分支合入视觉 main。

开始时已读 B 的 AGENTS、README、ROADMAP、VALIDATION、ENVIRONMENT、INTERFACES，确认 Windows 宿主及 WSL 开发目录，所有 Linux 命令经 `wsl -e bash -c` 执行。初轮B HEAD为 `6fa70dee`；用户要求最终回归时，主代理已更新基线至 `fa5d6ead`，最终ZIP的manifest记录此HEAD。工作台改动仍未提交，本代理未执行commit。原有 bag_replay/试飞资产和其他代理新增报告未修改。

## 实际改动

| 范围 | 最终行为 |
| --- | --- |
| `workbench.yaml`、`wb_board.py`、`server.py`、`web/app.js` | 新增独立正赛卡片，调用 `deployment/competition/start.sh`；默认仅指向未确认的 `deployment/competition/field.example.yaml`，不走08专项，也不复制昨晚测试坐标。 |
| 正赛配置来源 | 默认比赛模板与 `field_20261007_validated.yaml` 可选测试复现明确分开；也可输入自定义 YAML。选测试会显示“不是正赛参数/验收”，切换来源清除飞行与实投确认。 |
| 运动优化/障碍柱 | 三态“继承所选YAML / 显式开启 / 显式关闭”；继承省略 CLI，on/off 分别接 F 代理的 `--motion-optimization` / `--obstacle-columns`。命令预览展示实际文件和覆盖值，实际生效以配置检查输出为准。关闭配置柱不等同关闭真实点云避障。 |
| 运行保护 | 保留配置检查、preview、实投词确认、二次确认、后端任务锁与预览一致性检查；正赛 flight 只允许实投，拒绝改标签为mock；未确认模板由真实入口拒绝飞行。 |
| 正式数值 | 工作台不覆写高度/速度/加速度/前视。默认说明保留2.6m、1.2m/s、1.0m/s²、1.0/0.4/0.15m。成功现场轮仅为有限测试场地，不能当10×10比赛验收。中文模板与测试示例由 F 代理维护。 |
| 实时观察 | 删除独立电机大页链接，旧 `/motor` 跳转 `/observe`；统一观察页仍保留电机原始输出、映射、ESC、位姿与诊断图表。 |
| 日志地址 | 默认请求端口从8791改为8771；`/logs` 和 `/observe` 使用当前页面同源地址，随实际 host/port 变化。Windows后端禁用HTTPServer的地址复用并启用SO_EXCLUSIVEADDRUSE，避免两个后端共享同一监听端口。 |
| Windows 原生 | 新增 `wb_native_ssh.py`，用已有 Python/Paramiko SSH channel 适配现有会话和二进制接口；Linux仍用原OpenSSH/PTY。板端路径统一POSIX，防止Windows反斜杠进入远端命令。 |
| 打包/启动 | 修 `build_zip.py` 白名单和manifest，新增bat，ps1改原生Python启动；不调用WSL、不安装依赖、不自动SSH，不包含任何用户密码/profile或板端工程。 |

Windows 远端PTY无法核查echo，故不自动填写sudo密码；保留确认后在终端人工输入。Linux原sudo确认/echo检查策略保留。已知主机指纹变化仍拒绝连接；新指纹仅在配置accept-new时保存。本轮没有增加真实复位/释放操作。

## 验证结果

| 验证 | 结果与证据 |
| --- | --- |
| B工作台Python回归 | 最终版本 `201` 项，`OK (skipped=5)`；5项是Windows专用SSH测试，在Linux跳过。已加载ROS环境，使用现有rl_drone及纯Python系统依赖，未安装包。原始输出 `final_linux_tests.log`。 |
| Windows原生SSH fixture | 最终回归 `5/5 PASS`，Windows Python实际连本进程创建的localhost测试SSH服务，覆盖256KiB任意字节传输、失败和超时丢弃部分输出、终端脱敏/退出状态、本地Linux命令拒绝。不是板端连接。原始输出 `final_native_ssh.log`。 |
| 前后端契约 | `426` 组命令一致性/426次mock UI请求通过，涵盖开关、配置路径shell引用、独立检查、确认和互斥。Node为Windows原生。 |
| 其他JS回归 | 观察逻辑通过、坐标解析16项通过、14张卡片选择/嵌套控件/快照保存通过。旧group-selection fixture补齐可选toolbar的DOM方法。 |
| Windows解压启动 | 最终final2 ZIP复制到C盘，解压于中文+空格目录，经包内bat→ps1→Windows Python实际启动。HTTP主页/日志/观察/静态资源、纯命令预览、offline执行阻断、无自动会话全部通过。`windows_smoke.json` / `windows_launcher.log`。 |
| Windows占用端口回归 | 首个后端实际8771，第二个离线后端请求同端口后实际8772；两个页面的 `/logs` 链接均为相对同源，8772的日志页可读取。最终 `windows_smoke.json` 的fallback_url及 `windows_port_fallback.log`。旧默认地址复用导致同端口双绑定的问题已在此轮修复。 |
| Windows Edge实际DOM | `46` 项工作台检查、`27` 项统一观察检查通过；飞行/连接动作在测试里mock，不下发设备命令。`windows_browser.log` / `windows_observe.log`；截图 `windows_ui.png`。 |
| F真实CLI联调（只读） | 最终 `18` 组CLI组合验证：默认正式模板9组，显式选择测试示例9组；其中正式模板motion=on的3组因未测门墙平面按预期拒绝，其余CONFIG_VALID且ros_started=false。测试示例9组还离线生成runtime/control/overrides，检查runtime及planner_bridge运动开关、sdf_map配置柱开关确实变化。`final_competition_cli.json`。 |
| 正式数值/确认 | 读取F正式模板核对2.6/1.2/1.0及前视1.0/.4/.15，site_confirmed=false、门口/H留空；flight配置检查拒绝未确认模板。未修改F文件。 |
| Git/收尾 | `git diff --check`通过；最终ZIP被gitignore排除。WSL ss无工作台监听；验证创建的Windows离线后端已停止，未启动ROS/Gazebo/PX4。 |

F联调来源为 `/home/xhj/liftrace-worktrees/r2026-board-frame-fix` 当前未提交入口（检查时HEAD `7b6ca237`）。此联调直接调用实际supervisor的配置检查/离线生成，不调用preview应用链或run_session，所有生成文件留独立临时目录。整机入口的最终集成、默认参数对齐及发布仍由F/主代理负责。

最终回归在用户通知“F代理完整61回归与构建成功，CLI已落地”后执行。本代理复核实际CLI并完成上述18组检查；61项整机回归及整机构建是F代理交付范围，没有冒称由本代理重新执行。

初次检查Windows与WSL均未发现71/8771/8791/8792已有监听，不能据此认定旧现场服务正在某端口。按用户补充采用8771，实际Windows验证URL为 `http://127.0.0.1:8771`；链接不固定到该数值。工作完成后后端已关闭。

## Windows真正验证的边界

已使用本机Windows 11、`D:\Anaconda3\python.exe`（Python3.12.7）、PyYAML6.0.1、Paramiko2.8.1、pexpect、Windows Node24.19.0和原生Edge。后端和网页资源来自C盘解压包；运行链没有WSL调用，未读取Linux运行依赖。测试脚本/结果位于B的UNC路径，宿主仍安装且运行WSL；未通过卸载/关闭WSL验证“干净新机”。

本包需要用户已有Windows Python和上述轻量Python依赖，不是包含Python解释器的独立exe，也不会自动安装。最终已通过cmd实际调用包内bat，再运行ps1/Python；初轮ps1也通过。没有单独验收用户桌面的鼠标双击行为；cmd为同一ps1的转发包装。

未连接真实OrangePi，未部署、未启动驱动、ROS、舵机或飞行；真实网络重连/大bag传输/真实sudo与接管收尾须现场验收。Windows密钥认证、加密私钥解锁、不兼容agent、PuTTY saved sessions及完整OpenSSH config自动导入未验证或未提供；本轮验证的是密码SSH fixture和现有工作台协议。已有Paramiko2.8.1会输出旧算法API弃用提示，但本轮连接/传输测试通过，未升级本机依赖。

## 最终包与启动

最终交付ZIP（28项，166844 bytes；final2是有效最终包，早先r2/final是过程包）：

- B：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/deliverables/liftrace_flight_workbench_windows_20261008_final2.zip`
- Windows下载路径：`C:\Users\ASUS\Downloads\liftrace_flight_workbench_windows_20261008_final2.zip`
- Windows本地副本：`C:\Users\ASUS\AppData\Local\LiftraceWorkbenchReleases\20261008\liftrace_flight_workbench_windows_20261008_final2.zip`
- 验证解压目录：`C:\Users\ASUS\AppData\Local\LiftraceWorkbenchReleases\20261008\中文目录 release final2\liftrace_flight_workbench_20261007_165910`（内部目录时间戳为UTC）。

解压后双击 `start_windows.bat`，或运行 `powershell -NoProfile -ExecutionPolicy Bypass -File .\start_windows.ps1 -Transport local` 做离线预览；正常SSH使用不传 `-Transport local`，仍须在浏览器明确点击连接。默认请求8771，以实际启动打印地址为准。

包仅为工作台，未包含F整机入口、模板或测试示例的部署。板端必须具备最终F入口及两个开关CLI；主代理同步后再按现场流程检查。不要用08专项代替缺失的独立正赛入口。

下一步由主代理审查本报告、选取必要视觉/整机改动及统一提交；工具保持B开发线，本轮不建议整分支合入视觉main。现场确认配置与Windows真实SSH/控制链验收另行安排，启动授权不由本报告推定。
