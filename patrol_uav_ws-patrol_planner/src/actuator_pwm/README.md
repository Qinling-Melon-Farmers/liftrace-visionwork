# Orange Pi 5：GPIO1_D2舵机候选（2026-10-06）

槽1后仓保留GPIO4_B2/PWM14_M1、febf0020.pwm，700000→1700000ns；槽2右仓保留GPIO1_C6/PWM15_M2、febf0030.pwm，1000000→2100000ns。槽3左仓改为GPIO1_D2/PWM0_M1、fd8b0000.pwm，1100000→2100000ns。周期20ms、脉冲1s、顺序启动复位和CheckedPulse返回检查保持。

GPIO1_D2不是旧PWM1的另一根线。启动overlay必须选pwm0-m1，旧pwm1-m1对应GPIO1_D3；启用后Linux PWM编号可能改变，节点和init_pwm.sh均按设备地址查唯一通道，缺失或重复立即拒绝，不回退到旧D3。映射来源为[厂商pinctrl](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-5.10-rk35xx/arch/arm64/boot/dts/rockchip/rk3588s-pinctrl.dtsi#L1965-L2007)及[PWM寄存器地址](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-5.10-rk35xx/arch/arm64/boot/dts/rockchip/rk3588s.dtsi#L2862-L2883)。

本地控制器已构建，设备地址解析/真实文件写入5项离线回归通过；尚待板端候选构建、overlay启用及真实机构验收。正式板端路径先保持旧版，候选完整strict许可测试通过且现场确认三槽作动后再同步。

tools/servo_diagnostics/ground_chain.py仅供获授权的地面真实机构测试。它使用独立ROS Master和话题域，合成ReleaseEvidenceContext输入现有生产arbiter及ServoAction代理，再调用指定的真实pwm_node1；拒绝无许可、错槽及重复请求，记录三槽结果。没有相机识别和飞控参与，PWM ACK不能替代人工看到机构动作。运行会先依次复位三槽；收尾关闭所有实际映射通道并停止本轮节点。

原10月3日说明与旧接线见legacy_baseline/20261006_servo_gpio1_d2；历史档案不作为新D2映射的启动配置。
