# Orange Pi 5 Plus 舵机配置（2026-10-11）

本包适配当前 RK3588 OPi 5 Plus，禁止用于 Orange Pi 5。启动时检查机型，按硬件地址解析 pwmchip，不写死编号。

| req | 舱位 | 40-pin物理针脚 | GPIO | PWM | 地址 | 初始/释放脉宽ms |
|---|---|---|---|---|---|---|
| 1 | 后仓 | 16 | GPIO3_B5 | PWM12_M0 | febf0000.pwm | 0.6 / 1.8 |
| 2 | 右仓 | 7 | GPIO1_D6 | PWM14_M2 | febf0020.pwm | 1.0 / 2.1 |
| 3 | 左仓 | 18 | GPIO3_B6 | PWM13_M0 | febf0010.pwm | 1.1 / 2.1 |

这是新配置要求的接线，不代表实际线缆已核实或移动。针脚为物理编号，不是wPi编号。
PWM12/PWM13针脚依据当前板子的gpio readall与设备树核对；部分旧官方PWM表针脚有误，Pin14是GND，不能接信号。
舵机电源按舵机额定电压由独立电源供给；舵机电源GND与板子GND共地（例如Pin6），信号接表中针脚。不得接3.3V针脚供电。

/boot/orangepiEnv.txt 当前 overlays=pwm12-m0 pwm13-m0 pwm14-m2，三路已存在，不需本次修改或重启。
2026-10-11只读枚举：后仓pwmchip2、右仓pwmchip4、左仓pwmchip3；编号可变化，代码仍以地址为准。

周期20ms（50Hz），normal极性，释放脉冲1秒。SlotCalibration.h为脉宽权威；本次保留原代码值，未做5 Plus实物行程标定。

## 启动

默认 initialize_on_startup=false：只读检查三槽period=20000000、polarity=normal、enable=0及写权限，不输出脉冲；失败不注册服务。
init_pwm.sh需要sudo，会导出通道并配置PWM，不能视为只读检查。本次未执行。
initialize_on_startup=true会输出复位脉冲，必须另行确认后用于未装载机构准备。

roslaunch actuator_pwm launch_all.launch

启动入口将Servo重映射为/legacy/Servo_raw（patrol_control/Servo）；对外/Servo由许可代理独占。
服务成功只表示PWM操作确认，不证明机械投放成功。
本次只改源码、离线检查和编译，不运行节点、不写PWM、不调用服务。
旧包已保存于 ~/board_deploy_backups/20261011_actuator_before_5plus/。

2026-10-14：按用户现场反馈与指定值，1号归位保持1.7ms，释放改2.2ms；2/3号及1秒输出时间保持。尚未编译或执行新脉宽动作。

2026-10-14后续：按用户要求1号归位/释放改0.5/1.7ms，已构建pwm_node1；执行归位→等5秒→释放→等5秒→归位，每次输出1秒，PWM读回成功，机械状态由现场确认。

2026-10-14最终参数：用户指定1号归位0.6ms、释放1.8ms用于实机投递；写入SlotCalibration.h权威表，2/3号及1秒输出时长不变。本次不执行机构动作。
