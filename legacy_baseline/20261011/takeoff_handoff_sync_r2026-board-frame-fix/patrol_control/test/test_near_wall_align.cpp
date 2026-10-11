#include "patrol_control/near_wall_align.h"

#include <cassert>
#include <cmath>

int main() {
    patrol_control::NearWallAlignFence fence;
    fence.enabled = true;
    fence.bounds = {{-0.5, 7.4, -4.8, 4.8}};
    assert(fence.wellFormed());
    const auto safe = fence.clamp(0.93, -4.52, 0.0, 0.0);
    assert(std::abs(safe.first - 0.93) < 1e-9);
    assert(std::abs(safe.second - (-4.8 + fence.margin(0.0))) < 1e-9);
    const double expected = 0.5 * 0.55 *
        (std::cos(10.0 * M_PI / 180.0) + std::sin(10.0 * M_PI / 180.0)) + 0.03;
    assert(std::abs(fence.margin(0.0) - expected) < 1e-9);
    assert(safe.second > -4.46);
    assert(fence.contains(safe.first, safe.second, 0.0));
    assert(!fence.contains(0.93, -4.52, 0.0));
    assert(!fence.contains(safe.first, safe.second, 0.7853981633974483));
    const auto rotated = fence.clamp(0.93, -4.52, 0.7853981633974483, 0.0);
    assert(fence.contains(rotated.first, rotated.second, 0.7853981633974483));
    fence.valid = false;
    assert(!fence.wellFormed());
    assert(!fence.contains(0.93, -4.3, 0.0));
}
