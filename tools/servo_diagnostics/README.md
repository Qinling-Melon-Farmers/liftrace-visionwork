# Ground servo chain fixture

`ground_chain.py` runs an isolated ROS master with synthetic ground evidence,
the production release arbiter and guarded action proxy, and the specified raw
PWM executable. It initializes and pulses real servos and requires explicit
operator authorization and `--real-output`.

Select the left channel explicitly to match the raw executable and field wiring:

| `--left-pwm-device` | Left channel label |
| --- | --- |
| `fd8b0000.pwm` | GPIO1_D2 (PWM0; existing default) |
| `fd8b0010.pwm` | GPIO1_D3 (PWM1; previous wiring) |
| `fd8b0030.pwm` | GPIO0_D4/Pin15 (PWM3_M0; 2026-10-07 wiring) |

For the new field mapping, pass `--left-pwm-device fd8b0030.pwm` alongside the
existing required `--root`, `--out`, `--raw-node` and authorized `--real-output`
arguments. The default remains `fd8b0000.pwm`; there is no automatic fallback.

This option selects the channels inspected, logged and disabled during cleanup;
it does **not** reconfigure the raw executable. `--raw-node` must point to the
driver built for the same wiring. Rear/right channels remain `febf0020.pwm` and
`febf0030.pwm`; all three controllers are resolved by device address rather than
fixed `pwmchip` numbers. PWM3 requires the field `pwm3-m0` overlay.

PWM acknowledgements confirm electrical operations, not physical payload release.
