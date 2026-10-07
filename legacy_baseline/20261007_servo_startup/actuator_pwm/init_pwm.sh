#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run: sudo bash init_pwm.sh'; exit 1; }
# 舵机三路 PWM 初始化。按硬件地址解析 pwmchip 编号，不写死编号：
#   后仓 febf0020.pwm / 右仓 febf0030.pwm / 左仓 fd8b0030.pwm (PWM3_M0, Pin15)
# 只导出通道、设周期与极性、放开权限并关闭输出；本脚本不驱动舵机。
chips=()
for dev in febf0020.pwm febf0030.pwm fd8b0030.pwm; do
  found=''
  for p in /sys/class/pwm/pwmchip*; do
    resolved=$(readlink -f "$p")
    if [[ "$resolved" == *"/$dev/"* ]]; then found=$p; break; fi
  done
  [[ -n "$found" ]] || { echo "Required hardware PWM unavailable: $dev. No PWM initialization performed."; exit 2; }
  chips+=("$found")
done
for base in "${chips[@]}"; do
  [[ -d "$base/pwm0" ]] || echo 0 > "$base/export"
  p=$base/pwm0
  # 已是关闭且占空为0的新通道不要再写，内核会拒绝这种冗余写入
  [[ $(cat "$p/enable") == 0 ]] || echo 0 > "$p/enable"
  [[ $(cat "$p/duty_cycle") == 0 ]] || echo 0 > "$p/duty_cycle"
  echo 20000000 > "$p/period"
  echo normal > "$p/polarity"
  owner=${SUDO_USER:-root}
  chown "$owner" "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
  chmod 600 "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
  echo "$(basename "$base") ($(readlink -f "$base" | sed 's#.*/##')) initialized: period=20000000, duty=0, output disabled"
done
