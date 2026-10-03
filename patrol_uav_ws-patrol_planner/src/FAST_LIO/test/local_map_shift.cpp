#include "local_map_shift.h"
#include <iostream>
#include <limits>

namespace {
void expect(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

// One-axis reproduction of the existing box translation; deliberately excludes
// ikd-tree deletion. Position is held constant after crossing a trigger edge.
int stationaryShifts(double length, double threshold, double step, double pos) {
  double low = -length / 2, high = length / 2;
  int shifts = 0;
  for (int frame = 0; frame < 100; ++frame) {
    if (std::abs(pos - low) <= threshold) {
      low -= step; high -= step; ++shifts;
    } else if (std::abs(pos - high) <= threshold) {
      low += step; high += step; ++shifts;
    }
  }
  return shifts;
}

void invalid(double length, double range, double threshold) {
  try {
    fast_lio::localMapShiftDistance(length, range, threshold);
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Invalid configuration was accepted");
}
}  // namespace

int main() {
  try {
    expect(stationaryShifts(20, 9, 3, 1.2) == 100, "20m baseline no longer reproduces");
    expect(stationaryShifts(20, 9, 3, -1.2) == 100, "Negative baseline no longer reproduces");
    const double step = fast_lio::localMapShiftDistance(20, 6, 1.5);
    expect(std::abs(step - 1.8) < 1e-12, "Expected 1.8m capped step");
    for (double length : {20.0, 24.0, 30.0, 200.0}) {
      const double distance = fast_lio::localMapShiftDistance(length, 6, 1.5);
      expect(distance > 0 && distance < length - 18, "Step violates interior bound");
      if (length >= 24)
        expect(distance == std::max(0.45 * (length - 18), 3.0), "Valid original step changed");
      expect(stationaryShifts(length, 9, distance, 0) == 0, "Center moved");
      for (double sign : {-1.0, 1.0}) {
        const double edge = length / 2 - 9;
        expect(stationaryShifts(length, 9, distance, sign * (edge - 1e-6)) == 0,
               "Interior point moved");
        for (double overshoot : {0.0, 1e-6, 0.2})
          expect(stationaryShifts(length, 9, distance, sign * (edge + overshoot)) == 1,
                 "Stationary point repeatedly moved the box");
      }
    }
    invalid(18, 6, 1.5);
    invalid(17, 6, 1.5);
    invalid(20, 0, 1.5);
    invalid(20, -6, 1.5);
    invalid(20, 6, 1);
    invalid(std::numeric_limits<double>::infinity(), 6, 1.5);
    invalid(20, std::numeric_limits<double>::quiet_NaN(), 1.5);
    std::cout << "PASS: map shift baseline, both directions, boundaries, unchanged valid steps, invalid config\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
