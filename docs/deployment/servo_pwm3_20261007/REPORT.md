# 2026-10-07 板端PWM3接线回收与地面工程链验收

已从 `orangepi@192.168.43.59` 的0928工程取回实际舵机源码。两轮均按1→2→3执行，每槽间隔8秒；第二轮后操作者明确确认“三个仓均正常动作”。

## 当前接线

| 槽位 | 仓位 | 物理针脚 | GPIO / PWM | 硬件地址 | 本次枚举 |
|---|---|---|---|---|---|
| 1 | 后仓 | Pin11 | GPIO4_B2 / PWM14_M1 | febf0020.pwm | pwmchip5 |
| 2 | 右仓 | Pin7 | GPIO1_C6 / PWM15_M2 | febf0030.pwm | pwmchip6 |
| 3 | 左仓 | Pin15 | GPIO0_D4 / PWM3_M0 | fd8b0030.pwm | pwmchip2 |

板端overlay含pwm15-m2、pwm14-m1、pwm3-m0（另保留pwm1-m1）。驱动和初始化脚本按硬件地址解析pwmchip，不依赖5/6/2本次编号。新增overlay改变枚举顺序时不应回到旧硬编码4/5/0。GPIO名称来自现场源码、设备树overlay与sysfs映射；实际三仓动作由现场确认。

## 本地回收与检查

完整源码及板端原许可节点/代理/服务定义保留在 `logs/servo_field_pull_20261007_130155/`，第二轮为 `logs/servo_field_pull_20261007_130608/`。板端包是实体目录，不是链接。已核对实际运行二进制包含fd8b0030.pwm，运行时用的是板端现有编译程序，本轮未替换或重新编译它。

[board_actuator.patch](board_actuator.patch)记录相对本试飞分支 `deployment/onboard_actuator_reference_20261003/actuator_pwm` 的源码差异，基线提交c09129cc；从该包根目录可用 `git apply --check` 检查。包括README、init_pwm.sh、src/pwm_node1.cpp，以及现场保留的包根pwm_node1.cpp。CMake只编译src/pwm_node1.cpp，包根旧文件不参与构建。此补丁是回收档案，不自动改变别台电脑的针脚。

初始化三槽成功后才开放raw服务、PWM写入/读回检查、无效槽拒绝、脉冲结束关闭输出均保留；raw服务本身仍同步等待约1秒；本次由fixture调用代理，不包含patrol_control异步控制定时器的实跑。这里的raw成功是内核PWM操作成功，机械动作另由现场观察确认。

本地诊断工具新增显式 `--left-pwm-device fd8b0030.pwm`，保留D2/D3选项和原默认，参数仅选择检查/收尾通道，不修改真实驱动映射。

## 两轮测试

测试使用隔离ROS master 11329与/ground_servo_check命名空间：模拟对准证据→板端原release_permission_arbiter→guarded_servo_proxy的带身份ServoAction→板端真实/legacy/Servo_raw→PWM。没有启动MAVROS、飞行控制器、视觉识别或仿真飞行，不把此结果当空中连续投递/识别精度验收。

- 先验证没有有效许可时拒绝，未启动raw驱动前不会产生释放。
- 启动真实驱动依次复位，然后合法三槽各实际调用一次，每次约1.02–1.05秒。
- 错槽、同动作重复请求均拒绝，许可端最终推进到payload_exhausted。
- 两轮软件PASS；第二轮操作者确认三仓均正常。
- 两轮结束测试子进程全退出、三路enable=0，独立复查无飞行/舵机进程。未修改板端飞行参数、设备树或原硬件代码，只上传隔离测试脚本和产生日志。

逐槽数据见[RESULTS.json](RESULTS.json)。本次只回收并验收现场改动，不向其他硬件配置自动套用PWM3；旧默认与档案仍保留。
