# GPIO1_D2 地面舵机测试：停止，未通过机构验收（2026-10-06）

板端地址为 orangepi@192.168.3.126，部署根目录 /home/orangepi/liftrace_board_trials_20260928。用户授权写板及合成对准许可后的真实调用，并确认左仓信号线已改接 GPIO1_D2。最终用户反馈“左仓仍未动作或动作异常”，随后要求停止测试。已复查本轮全部节点退出，三路 PWM enable=0；未激活正式舵机包或二进制，禁止将软件 ACK 记为机械验收成功。

## 候选与通道

实际板型是 Orange Pi 5。GPIO1_D2 对应 PWM0_M1/fd8b0000.pwm，原 D3 是 PWM1_M1/fd8b0010.pwm。已备份 /boot/orangepiEnv.txt，将 overlay 的 pwm1-m1 替换为 pwm0-m1，其余 overlay 保留，重启后设备树确认 bank1 offset26 mux11 和 PWM0 status=okay。启动配置已经改变；正式旧舵机代码和二进制仍未替换，当前不能宣称正式投递链可用。停止测试后未额外回滚启动配置或发出动作。

| 槽位 | 位置 | 当前 GPIO / PWM | 设备地址 | 本轮 pwmchip | 复位 → 释放脉宽 ns |
|---|---|---|---|---|---|
| 1 | 后仓 | GPIO4_B2 / PWM14_M1 | febf0020.pwm | 4 | 700000 → 1700000 |
| 2 | 右仓 | GPIO1_C6 / PWM15_M2 | febf0030.pwm | 5 | 1000000 → 2100000 |
| 3 | 左仓 | GPIO1_D2 / PWM0_M1 | fd8b0000.pwm | 0 | 1100000 → 2100000 |

周期 20ms、动作脉冲 1s；三路均通过设备地址选唯一控制器，不能依赖 Linux 枚举编号。驱动修改 revision 05218459；地面测试断言修正 revision 45e5784c。原包快照位于 frame-fix 工作树 legacy_baseline/20261006_servo_gpio1_d2（5457e9e1）；试飞分支仍保留冻结的 20261003 参考，不覆盖历史内容。

## 验证与限度

- 本地 pwm_node1 Catkin 构建、实际生产 PWMController 设备解析五项回归、脚本语法检查通过；板端候选 ARM 编译链接通过。
- 地面链路使用独立 ROS Master 11329 和 /ground_servo_check 话题域：合成 typed ReleaseEvidenceContext → 现有生产 strict release_permission_arbiter → fenced ServoAction 代理 → 真实 ARM pwm_node1。未加载飞控、MAVROS、相机，也未解锁/起飞；诊断的低高度许可只在独立测试命名空间设置，不更改飞行配置。
- 首次测试完成后仓/右仓真实 PWM 调用，左仓完成启动复位；第三槽错槽请求指向已锁定槽1时，代理正确拒绝且返回 EXECUTION_UNKNOWN，脚本错误要求 NOT_STARTED，导致测试误报失败。仅修正诊断断言及失败 traceback，未改生产代理。
- 修正轮完成三次 RAW_CALL_STARTED、三次 COMPLETED/ACK，各约1.02–1.05s；无许可1项、错槽3项、重放3项均拒绝，最终 payload_exhausted。三个 PWM 输出均关闭，所有四个子进程退出，软件链检查 PASS。
- 用户现场只明确反馈左仓仍未动作或异常，因此左仓实际动作失败、整组三槽机械验收未通过。PWM 设置和服务返回不能证明插针端波形、舵机供电、共地或舵机自身正常，尚不能定位具体故障元件。

## 产物与当前部署状态

板端候选/备份目录：/home/orangepi/liftrace_board_trials_20260928/logs/servo_d2_candidate_20261006_202105。backup 包含原包、原 ARM 二进制、原启动配置；candidate_devel 是实测候选二进制。第一次测试在 test，修正轮在 test_retry_1，均保留 results.json、fixture/raw/arbiter/proxy 日志。stop_result.json 保存最后停机复查。

本机原始产物在试飞工作树 logs/servo_gpio1_d2_20261006（不入 git），包含两个测试目录、ARM 构建/设备树/初始化结果及用户反馈。当前正式板端 src/actuator_pwm 和 devel/lib/actuator_pwm/pwm_node1 未替换，测试结束后也未启动正式服务。

候选源码以整机候选分支实际 actuator_pwm 为权威；试飞分支仅在 deployment/actuator_gpio1_d2_20261006 保存本轮候选交接包和来源，保持机械代码不参与该精简分支编译。现有飞行许可、槽顺序和三仓脉宽不变。

## 后续条件

本轮按用户要求停止，不再自动重测或启动机构。后续如继续排查，应先在用户重新授权下检查 D2 插针波形、供电/共地和机构；只有现场实际复位/释放均正常，才允许把候选替换到正式板端链路。本轮没有证明电机、飞控或视觉投递算法存在新的问题。


## 后续：正式 D3 复测结束

用户随后改接回 D3，并要求测试原正式链路。板端 boot 已恢复 pwm1-m1，正式驱动仍原版；三槽软件调用通过，但左仓仍无动作或异常。用户结束本轮，输出关闭、节点退出。当前板端状态以 [D3结果](../servo_d3_formal_20261006/REPORT.md) 为准，前文保留 D2 停测时的历史状态。
