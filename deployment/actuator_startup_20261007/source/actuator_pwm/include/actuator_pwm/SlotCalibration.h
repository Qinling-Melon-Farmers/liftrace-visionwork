#pragma once
namespace actuator_pwm {
// req order: rear / right / left. All values in ns; period stays 20 ms.
// Rear physically confirmed 2026-10-07: extended=locked, retracted=release.
const unsigned kInitialDutyNs[] = {1700000, 1000000, 1100000};
const unsigned kReleaseDutyNs[] = {2100000, 2100000, 2100000};
}
