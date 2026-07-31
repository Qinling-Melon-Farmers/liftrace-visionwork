#!/bin/bash

# PWM初始化脚本 - 用于舵机控制
# 每次启动舵机控制前运行此脚本

echo "开始初始化PWM接口..."

# 检查是否以root权限运行
if [[ $EUID -ne 0 ]]; then
   echo "此脚本需要root权限运行"
   echo "请使用: sudo ./init_pwm.sh"
   exit 1
fi

# 导出PWM通道
echo "导出PWM通道..."
echo 0 > /sys/class/pwm/pwmchip2/export 2>/dev/null || echo "pwmchip2已导出"
echo 0 > /sys/class/pwm/pwmchip3/export 2>/dev/null || echo "pwmchip3已导出"
echo 0 > /sys/class/pwm/pwmchip4/export 2>/dev/null || echo "pwmchip4已导出"

# 等待一小段时间，确保设备文件创建完成
sleep 0.5

# 设置PWM周期为20ms（20000000ns）
echo "设置PWM周期..."
echo 20000000 > /sys/class/pwm/pwmchip2/pwm0/period
echo 20000000 > /sys/class/pwm/pwmchip3/pwm0/period
echo 20000000 > /sys/class/pwm/pwmchip4/pwm0/period

# 修改文件权限，允许普通用户访问
echo "设置文件权限..."
chmod 666 /sys/class/pwm/pwmchip2/pwm0/*
chmod 666 /sys/class/pwm/pwmchip3/pwm0/*
chmod 666 /sys/class/pwm/pwmchip4/pwm0/*

echo "PWM初始化完成！"
echo "现在可以运行launch脚本了。" 