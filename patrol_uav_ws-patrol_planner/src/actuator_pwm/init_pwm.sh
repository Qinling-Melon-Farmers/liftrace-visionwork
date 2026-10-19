#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run: sudo bash init_pwm.sh'; exit 1; }
# Orange Pi 5 Plus: rear Pin16 PWM12_M0, right Pin7 PWM14_M2, left Pin18 PWM13_M0.
# Configuration only: output is disabled; never use this script as a loaded-mechanism reset.
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" == *"OPi 5 Plus"* || "$model" == *"Orange Pi 5 Plus"* ]] || { echo 'Orange Pi 5 Plus required'; exit 2; }
# One permanent inode shared with raw/debug services. Never unlink/replace it.
lock_path=/run/lock/liftrace-servo.lock
[[ ! -L "$lock_path" ]] || { echo 'Refusing symlink PWM lock'; exit 2; }
exec {servo_lock_fd}>>"$lock_path"
flock -n "$servo_lock_fd" || { echo 'PWM occupied by raw/debug/initialization; no PWM changes'; exit 2; }
owner=${SUDO_USER:-root}
chown "$owner" "$lock_path"
chmod 600 "$lock_path"
chips=()
for dev in febf0000.pwm febf0020.pwm febf0010.pwm; do
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
