#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run: sudo bash init_pwm.sh'; exit 1; }
# Orange Pi 5 Plus: rear=Pin16, right=Pin18, left=Pin7.
# Check all addresses before making any writes.
for spec in '2 febf0000.pwm' '3 febf0010.pwm' '4 febf0020.pwm'; do
    read -r chip device <<< "$spec"
    actual=$(readlink -f "/sys/class/pwm/pwmchip$chip")
    [[ "$actual" == *"/$device/"* ]] || { echo "PWM address mismatch: chip$chip expected $device"; exit 2; }
done
for chip in 2 3 4; do
    base=/sys/class/pwm/pwmchip$chip
    [[ -d "$base/pwm0" ]] || echo 0 > "$base/export"
    p=$base/pwm0
    [[ $(cat "$p/enable") == 0 ]] || echo 0 > "$p/enable"
    [[ $(cat "$p/duty_cycle") == 0 ]] || echo 0 > "$p/duty_cycle"
    echo 20000000 > "$p/period"
    echo normal > "$p/polarity"
    owner=${SUDO_USER:-root}
    chown "$owner" "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
    chmod 600 "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
    echo "pwmchip$chip initialized: period=20000000, duty=0, output disabled"
done
