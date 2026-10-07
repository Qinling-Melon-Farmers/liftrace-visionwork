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


## 2026-10-07：5a初始化成功灯与sudo提示修正

- 改动范围：工作台前端状态、SSH会话认证、操作说明及定向测试。
- 具体改动：5a为一次性命令，exit 0在终端标签和步骤栏均显示绿色“初始化成功”；失败红色、执行中待完成、未执行灰色。5b等常驻服务退出不套用成功状态。现场确认日志来自两次实际初始化，未增加日志去重或修改执行次数。仅经已有现场确认的5a可复用内存连接口令响应中英文sudo提示，最多一次；普通终端不自动填写sudo，口令不保存、不回显，SSH认证单独计次。
- 验证结果：子代理认证/PTY12项、浏览器42项、HTTP/SSE20项、既有18+18+34项、356组命令对照及请求通过，独立解压启动通过。主代理独立认证测试11项（追加私钥手输用例之前）及终端9类状态/5类常驻退出验证通过。当前8791静态JS/CSS含修复，刷新即可生效。
- 遗留问题：绿色只代表初始化命令软件成功，不是舵机物理反馈；当前后台未重启，新sudo应答需正常结束设备会话后重启工作台才生效，不能通过刷新Python后台生效。
- 下一步：用提交版重新打包前线ZIP；保留现场SSH与5b服务，继续投递研究两轮仿真。

新包：`deliverables/liftrace_flight_workbench_frontline_20261007_initfix.zip`，提交后由build_zip.py生成；旧包保留。测试原始日志：`/tmp/liftrace-workbench-sudo-review-20261007/`。
