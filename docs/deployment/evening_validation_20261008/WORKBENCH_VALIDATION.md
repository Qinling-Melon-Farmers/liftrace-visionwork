# 工作台实际验证与交付（2026-10-08）

0928 ARM于21:29 BUILD_PASS，独立正赛ARM于21:43:41 BUILD_PASS。按随后明确授权，WSL8771使用本机显式SSH密钥完成三次HTTP命令生成和`preview --check-config`，均退出0。没有调用自动探针的连接接口，没有启动应用/READY、master、MAVROS、舵机或飞行；主代理11418纯软件验证独立执行。

| 检查 | 实际来源与结果 |
| --- | --- |
| mod08 competition | 0928 root + site_20260928环境 + 221730现场YAML；CONFIG_VALID，规划1.2m/s、1.0m/s²，巡航/精调前视1.0/0.4m，走廊快/慢0.6/0.4m，FC搜索2m、投递0.35m；运动优化/续扫开启 |
| mod08 limited | 同一0928来源和现场几何；CONFIG_VALID，规划0.5m/s、0.35m/s²，巡航/精调前视0.5/0.4m，走廊快/慢0.6/0.4m，FC搜索2m、投递0.35m；运动优化/续扫开启 |
| 独立正赛模板 | `/home/orangepi/liftrace_competition_20261008` + competition环境 + 根内field.example.yaml；CONFIG_VALID、ros_started=false，site_confirmed=false、generation_ready=false，运动优化/续扫开启。入口未输出planning/drop数值，不能宣称已验证这些数值或实测场地 |

此前无密钥时的退出255认证失败已由identity_file接入解决，原日志保留；此次成功不是绕过配置/来源检查。model/metadata均使用根内相对路径。独立MID360实测配置由主代理部署在独立根内，不借用旧root。

工作台增加可留空的identity_file：OpenSSH传-i并限制密钥选择，Windows Paramiko使用本机key_filename。指定的真实本机key路径仅在仓库外profile；没有复制私钥，没有使用或保存备用口令。Windows包默认身份文件留空，可用自己的本机密钥或内存口令；包不包含profile、私钥或设备工程。

验证范围：B的63项相关Python回归通过，F的identity7/project_preflight11项定向通过；Windows原生identity7及SSH fixture6项通过，含真实fixture私钥认证；568组前后端命令及568组模拟UI请求通过。白名单ZIP解压独立启动通过，Windows BAT/Python、真实Edge页面/观察/有效配置、端口占用顺延与同源日志共19项通过，均未连接真实设备。

两套板端root各同步31个明确清单中的`tools/flight_workbench/*`文件（产品白名单、打包脚本及相关测试），仅同路径写入，无删除，无设备/服务启动。不触碰environment/package.sh、planner XML或主代理FIELD_REPORT。

最终工作台ZIP输出到 `C:/Users/ASUS/Downloads/liftrace_flight_workbench_windows_20261008_key_preflight.zip`，保留旧包。最终产品payload与通过Windows验证的候选逐文件一致，manifest记录工作台提交版本；随后只追加文档不会改变该代码版本。此工具包不包含板端ROS工程，整机runtime由主代理两套板端部署交付。

原始小报告位于本机仓库外`~/.config/liftrace-flight-workbench/reports/`：`http_08_check_20261008.json`、`http_formal_check_20261008.json`、`board_workbench_sync_20261008.json`及`windows_key_preflight_20261008/windows_smoke.json`；成功SSH终端日志为21:42:06、21:42:09和21:46:18三次trial日志。

收尾配置：host=orangepi@192.168.3.126、root=/home/orangepi/liftrace_board_trials_20260928、env=deployment/site_20260928/environment.sh；保留本机identity_file，模型与metadata为根内相对路径。无starting/running会话，probe未上传，设备编排未运行。用户后续先选08并显式选择competition速度；配置检查通过不构成实飞授权。
