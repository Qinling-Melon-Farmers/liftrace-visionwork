#pragma once
namespace actuator_pwm {
// 请求顺序为后仓、右仓、左仓；脉宽单位 ns，周期保持 20 ms。
// 2026-10-07 现场确认后仓伸出锁止：锁止 1700us、释放 2100us。
const unsigned kInitialDutyNs[] = {1700000, 1000000, 1100000};
const unsigned kReleaseDutyNs[] = {2100000, 2100000, 2100000};
}
