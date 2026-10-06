# Orange Pi 5：GPIO1_D2舵机候选（2026-10-06）

槽1后仓保留GPIO4_B2/PWM14_M1、febf0020.pwm，700000→1700000ns；槽2右仓保留GPIO1_C6/PWM15_M2、febf0030.pwm，1000000→2100000ns。槽3左仓改为GPIO1_D2/PWM0_M1、fd8b0000.pwm，1100000→2100000ns。周期20ms、脉冲1s、顺序启动复位和CheckedPulse返回检查保持。

GPIO1_D2不是旧PWM1的另一根线。启动overlay必须选pwm0-m1，旧pwm1-m1对应GPIO1_D3；启用后Linux PWM编号可能改变，节点和init_pwm.sh均按设备地址查唯一通道，缺失或重复立即拒绝，不回退到旧D3。映射来源为[厂商pinctrl](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-5.10-rk35xx/arch/arm64/boot/dts/rockchip/rk3588s-pinctrl.dtsi#L1965-L2007)及[PWM寄存器地址](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-5.10-rk35xx/arch/arm64/boot/dts/rockchip/rk3588s.dtsi#L2862-L2883)。

本地与板端候选 ARM 构建通过，设备地址解析/真实文件写入5项离线回归通过。板端已启用pwm0-m1并确认D2 pinmux；真实strict许可链三槽PWM调用通过，但用户反馈左仓仍未动作或异常，因此机构验收未通过。用户已要求停止：三路输出均关闭、测试节点全退出，正式板端代码/二进制未替换。详见docs/deployment/servo_gpio1_d2_20261006/REPORT.md；不得把PWM ACK当作机构动作成功。

tools/servo_diagnostics/ground_chain.py仅供获授权的地面真实机构测试。它使用独立ROS Master和话题域，合成ReleaseEvidenceContext输入现有生产arbiter及ServoAction代理，再调用指定的真实pwm_node1；拒绝无许可、错槽及重复请求，记录三槽结果。没有相机识别和飞控参与，PWM ACK不能替代人工看到机构动作。运行会先依次复位三槽；收尾关闭所有实际映射通道并停止本轮节点。

原10月3日说明与旧接线见legacy_baseline/20261006_servo_gpio1_d2；历史档案不作为新D2映射的启动配置。


后续：用户已改接回GPIO1_D3并测试原正式链路。板端启动配置已恢复pwm1-m1，正式源码/二进制仍原版；软件三槽ACK通过，但左仓仍异常，用户结束本轮。当前板端状态见docs/deployment/servo_d3_formal_20261006/REPORT.md；本目录D2代码仅保留为未验收候选。
