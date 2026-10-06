#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run: sudo bash init_pwm.sh'; exit 1; }
# rear=GPIO4_B2/PWM14_M1, right=GPIO1_C6/PWM15_M2, left=GPIO1_D2/PWM0_M1.
# PWM enumeration can change after enabling pwm0-m1. Resolve every address,
# and reject missing/ambiguous devices before making any writes.
chips=()
for device in febf0020.pwm febf0030.pwm fd8b0000.pwm; do
    matches=()
    for candidate in /sys/class/pwm/pwmchip*; do
        actual=$(readlink -f "$candidate") || continue
        [[ "$actual" == *"/$device/"* ]] && matches+=("$candidate")
    done
    [[ ${#matches[@]} -eq 1 ]] || { echo "Expected one PWM controller for $device, found ${#matches[@]}"; exit 2; }
    chips+=("${matches[0]}")
done
for base in "${chips[@]}"; do
    [[ -d "$base/pwm0" ]] || echo 0 > "$base/export"
    p=$base/pwm0
    [[ $(cat "$p/enable") == 0 ]] || echo 0 > "$p/enable"
    [[ $(cat "$p/duty_cycle") == 0 ]] || echo 0 > "$p/duty_cycle"
    echo 20000000 > "$p/period"
    echo normal > "$p/polarity"
    owner=${SUDO_USER:-root}
    chown "$owner" "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
    chmod 600 "$p/period" "$p/duty_cycle" "$p/polarity" "$p/enable"
    echo "${base##*/} initialized: period=20000000, duty=0, output disabled"
done
