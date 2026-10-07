# 2026-10-07 前线工作台分发包

工作台本地服务已挂起：`http://127.0.0.1:8791/`，当前仅localhost监听、未建立SSH会话，13张现有任务卡（含低空hover/forward/square）保留。用户手动点击连接/启动，不由打包或网页刷新启动飞机。

分发包在本工作树 `deliverables/liftrace_flight_workbench_frontline_20261007_release.zip`。解压后读README_FIRST.md；Windows需要已装WSL（默认Ubuntu发行版），双击start_windows.cmd。Linux/WSL用bash start_workbench.sh。不是Windows原生exe，不包含或安装ROS/机载工程，不需要完整源码仓；依赖由维护者按说明预先准备。

打包器使用20个明确文件白名单，加manifest共21项：服务、SSH/状态/几何模块、只读探针、现场YAML、静态网页/接线信息、启动入口与依赖说明。没有密码、私钥、个人profile、日志、bag、模型或板端包。Windows入口用UTF8→base64跨WSL参数边界，支持中文/空格解压路径；PowerShell包文件带BOM，cmd用CRLF。未新增前端构建工具、安装器或自动部署。

## 验证

- 子代理验证独立ZIP解压启动、HTTP/SSE、Windows中文/空格路径入口、低空卡片回归、356命令组合；既有浏览器主页面35项、观察页27项、坐标/航线18项通过。后18项针对初版ZIP，最终静态页面未改。
- 主代理独立重跑tests/test_distribution.py通过：白名单、ZIP完整性、独立空状态启动、13卡片与3低空卡、页面/资源HTTP200、低空命令preview正确，离线模式拒绝实际启动，不建立终端会话。
- 主代理读取8791实际snapshot：connection=unknown、sessions为空、13卡片。保留后台服务，仅关闭自检临时服务。
- Windows原始启动日志在C:/Users/ASUS/AppData/Local/Temp/liftrace final 中文 2fa3124c590241e5ba595d36e2fb6c8b/；其余子代理stdout会话转录在/tmp/liftrace-workbench-review-20261007，明确不是原始进程日志。
- 只改打包/启动依赖选择/文档，不改任何飞行速度、高度、任务策略或遥控器模式逻辑。

本包仍依赖板端已有专项入口与匹配消息；本地服务能打开不等于各飞行组已READY。前线输入当前SSH地址192.168.43.59及实际账户；密码由操作者输入，不随包发群。RC输出观察不等于电机RPM/电流遥测。

源码提交后重新构建正式ZIP，manifest记录对应HEAD及modified_package_files为空。zip/运行状态/日志均不入Git。
