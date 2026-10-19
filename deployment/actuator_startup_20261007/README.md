> 2026-10-07晚间更新：后仓释放改2100000ns，锁止1700000ns。板端已重编译并仅重启被动舵机服务；没有测试新脉宽。下方旧部署段落为历史记录，以本条为准。

# 2026-10-07 舵机被动启动补丁交付

当前工作树：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests`。
只在本目录和 `legacy_baseline/20261007_servo_startup` 新增文件；不改GPIO1_D2或20261003冻结配置，
原包完整快照、manifest、文件清单与SHA256已先保存。运行源码与新二进制已在板端部署，详见下方现场状态。

## 行为

- `initialize_on_startup=false` 同时是main和launch默认：周期20000000ns、normal极性、enable0的只读校验；
  检查四个属性的可写权限。启动、失败及被动退出零sysfs写入，不改duty，不补配置，不开启PWM。
- 三槽全部检查成功才注册服务；错误配置/权限/地址解析失败返回非零，不注册服务。
- 显式true才允许原人工复位事务，**仅非装载机构准备使用**。不会因为启动自动发复位脉冲。
- 释放事务仍是checkedPulse、1s等待、每一步返回值/读回检查。
- 后仓现场确认“伸出才锁止，缩回会开仓”：initial/lock=1700000ns，release=700000ns。
  主代理已独立1700000ns 1s后关PWM，现场确认已伸出并锁止；本代理未执行任何机构动作。
- 右仓initial=1000000/release=2100000，左仓initial=1100000/release=2100000，均未改。
  后/右/左设备仍分别febf0020/febf0030/fd8b0030.pwm；左槽Pin15 PWM3_M0保留。

**当前运行PID保持原样。** 当前oldservice仍使用旧编译常量：启动后仓700000ns、释放1700000ns。
覆盖文件/编译/设参数不会自动切换其常量。本补丁仅在下次使用新二进制的新进程中生效；
本次不重启、不启停节点、不发真实释放请求。

## 文件

- `source/actuator_pwm/`：从实际板端包复制后修订的实体包，原init_pwm.sh/package.xml保留。
- `actuator_startup.patch`：运行代码/launch/说明补丁，以板端原包目录为根。
- `actuator_startup_tests.patch`：独立测试与CMake测试target补丁，可另行审阅。
- `DEPLOY_FILES.txt`：最小运行必需的7个待上板文件；`CHANGED_FILES.txt`：全部修订文件。
- 快照：`../../legacy_baseline/20261007_servo_startup/{actuator_pwm,manifest.json,FILES.txt,SHA256SUMS}`。
- 构建日志本地保留于`build_local.log`；测试详情在`build/Testing/Temporary/LastTest.log`。

## 本地独立构建

Windows宿主执行：

```powershell
wsl -e bash -c 'bash /home/xhj/liftrace-worktrees/r2026-board-vision-tests/deployment/actuator_startup_20261007/build_local.sh'
```

脚本只消费现有patrol_control/Servo生成头和ROS依赖，直接CMake该包，
输出build/devel/install均在本交付目录；只编译pwm_node1与actuator_startup_tests并运行CTest。
未运行pwm_node1、ROS节点、init_pwm.sh、SSH、仿真或真实PWM。临时测试文件模拟sysfs。
不能把WSL编译的x86_64二进制复制上板，主代理须在板端编译。

## 主代理最小部署清单（本代理未执行）

1. 使用实际板端目标包，不整仓覆盖；先保留板端旧源与旧二进制，保持当前PID运行。
2. 从`source/actuator_pwm/`按`DEPLOY_FILES.txt`覆盖7个运行必需文件。
   建议另同步未编译的根目录pwm_node1.cpp及README，避免旧副本误导。
   需要板端无硬件测试时再同步CMakeLists.txt、tests/test_startup.cpp（第二份patch）。
   不覆盖init_pwm.sh、package.xml、GPIO1_D2、任何冻结配置或设备树。
3. 加载板端既有环境（非此处的WSL路径）后，只构建已有target：

   ```bash
   cmake --build patrol_uav_ws-patrol_planner/build --target pwm_node1 -j2
   ```

   此target按现有CMake读取src/pwm_node1.cpp和src/PWMController.cpp；构建需成功，
   新增include文件必须一并到位。不发送服务请求，不执行初始化脚本，不restart。
4. 交接记录明确“源码和新二进制已准备，运行oldservice未切换”；
   后续由现场另行安排使用新二进制，装载时保持initialize_on_startup=false。
   当前参数服务器如残留true会触发显式复位；下次入口必须保持launch默认false。

## 验证范围

本地pwm_node1独立编译和CTest PASS（46项用例，0.12s）；测试包括默认三槽readchecks/零writes、错误配置/权限/缺字段、
后仓反向标定与左右不变、显式复位事务8个故障点、checkedPulse4个故障点。
生产PWMController通过临时根目录测试构造/校验/析构；inotify确认被动正常和错误配置退出无IN_MODIFY，
配置读取有IN_ACCESS；错误设备地址构造失败时，已构造对象展开析构亦零写入。
有写事务后的析构仍会关闭输出（只在临时文件中测试）。真实机构释放及下次启动尚未验收。

两份补丁已在原包副本实际应用，逐文件比对与待上板source完全一致；原包SHA256核对全部通过。

## 现场部署状态（2026-10-07）

10.75.120.193的0928工程已同步运行文件，在logs/servo_passive_build_20261007独立ARM构建成功，再原子替换devel/lib/actuator_pwm/pwm_node1。部署前后运行PID均12271，没有重启原服务。该旧进程仍使用旧常量；下一次正常启动新进程才启用被动检查及后仓反向标定。旧源包、旧可执行及构建日志保留在同一logs目录。