# D3 正式舵机地面调用结果（2026-10-06）

用户要求使用板端正式 D3 链路调用测试，并确认左仓信号线已改接回 GPIO1_D3。正式程序完成三槽调用，软件链检查 PASS；用户反馈“左仓仍无动作或异常”，随后明确结束本轮。左仓实际动作未通过，不能宣布机构验收成功。三路 PWM 输出已关闭，四个测试子进程均已退出，不自动续测。

## 当前板端状态

- 地址 orangepi@192.168.3.126，工程 /home/orangepi/liftrace_board_trials_20260928。
- /boot/orangepiEnv.txt 已从 pwm0-m1 恢复 pwm1-m1，并重启核验；UART、WiFi、PWM14/15等其他 overlay 保留。恢复前启动文件备份在本轮 logs 目录的 orangepiEnv_before_D3.txt。
- 设备树 PWM1/fd8b0010 status=okay，pinctrl 为 bank1 offset27 mux11，即 GPIO1_D3/PWM1_M1。正式程序使用的固定编号均吻合：后仓 chip4/febf0020、右仓 chip5/febf0030、左仓 chip0/fd8b0010。
- 真实调用使用原正式二进制 patrol_uav_ws-patrol_planner/devel/lib/actuator_pwm/pwm_node1。四个正式运行文件及该二进制在测试前后均与 D2 修改前板端备份一致，没有替换正式源码或二进制。
- 正式 init_pwm.sh 仅导出通道、设置20ms周期及访问权限，执行后 enable 均0；随后启动正式驱动，依次复位三槽，再执行释放测试。测试结束后不留驱动服务常驻。
- 本地 D2 候选保留，不部署正式板端。当前硬件接线与 boot 按用户本轮要求保持 D3。

## 本轮测试

时间为北京时间 20:57:50—20:58:25。使用独立 ROS Master 11329 和 /ground_servo_check 话题域，合成 typed 对准证据输入板端生产 strict arbiter，再经现有 fenced ServoAction 代理调用 /legacy/Servo_raw 和原正式 ARM 舵机驱动。未启动飞控、MAVROS、相机或飞行任务，没有解锁、起飞或改变飞行许可参数。低高度等诊断参数仅作用于本轮独立测试域，不代表完整实飞识别链验收。

| 槽 | 位置/引脚 | 复位 → 释放 ns | 调用耗时 s | 软件结果 |
|---|---|---|---:|---|
| 1 | 后仓 GPIO4_B2 | 700000 → 1700000 | 1.045 | ACK |
| 2 | 右仓 GPIO1_C6 | 1000000 → 2100000 | 1.030 | ACK |
| 3 | 左仓 GPIO1_D3 | 1100000 → 2100000 | 1.050 | ACK；现场仍异常 |

周期20ms、每次1s脉冲、释放按1→2→3且间隔8s，沿用正式驱动标定值。共3次 RAW_CALL_STARTED、3次 COMPLETED/ACK，无许可1项、错槽3项、重复请求3项拒绝，最终 payload_exhausted。收尾后 chip4/chip5/chip0 的 enable 全0，所有测试子进程 exit code=0；随后独立复查正式文件未变、输出关闭及零残留。

用户现场只明确左仓仍无动作或异常，其余槽本轮实际动作未单独确认。D2 候选与 D3 正式程序均得到软件 ACK，但左仓实际动作都未通过；ACK只能证明软件 PWM 设置和服务调用完成，不能证明插针端信号、舵机供电/共地或机械机构正常。本轮不进一步判定故障元件，按用户要求结束。

## 产物

板端：/home/orangepi/liftrace_board_trials_20260928/logs/servo_d3_formal_20261006_205129，包含恢复前启动文件、boot_restore.json、ground_chain.py、test_launch.json、test/results.json及fixture/raw/arbiter/proxy日志、finalcheck.json。

本地：试飞工作树 logs/servo_d3_formal_20261006_205129，另含正式文件读取结果、D3设备树/初始化、三槽计数和时长、用户反馈。原始日志不入 git；地面工具来源 revision 6e73791c，明确传入 --left-pwm-device fd8b0010.pwm 和原正式 --raw-node 路径。

D2 前轮见 [D2结果](../servo_gpio1_d2_20261006/REPORT.md)。本报告更新其“当前板端 boot”状态为 D3，不覆盖前轮 D2 测试历史。后续任何新增真实动作必须先获得新的用户授权。
