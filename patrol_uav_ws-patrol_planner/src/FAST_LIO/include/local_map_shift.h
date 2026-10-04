#ifndef FAST_LIO_LOCAL_MAP_SHIFT_H
#define FAST_LIO_LOCAL_MAP_SHIFT_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fast_lio {

// Configuration-only calculation; does not move boxes or delete map points.
// A step strictly smaller than the non-triggering interior prevents crossing
// from one trigger band into the opposite band while the sensor is stationary.
inline double localMapShiftDistance(double cube_length, double detection_range,
                                    double movement_threshold) {
  if (!std::isfinite(cube_length) || !std::isfinite(detection_range) ||
      !std::isfinite(movement_threshold) || detection_range <= 0 ||
      movement_threshold <= 1)
    throw std::invalid_argument("Invalid local-map movement parameters");
  const double interior = cube_length - 2 * movement_threshold * detection_range;
  if (!std::isfinite(interior) || interior <= 0)
    throw std::invalid_argument("Local-map cube has no non-triggering interior");
  const double original = std::max(0.45 * interior,
                                   detection_range * (movement_threshold - 1));
  return std::min(original, 0.9 * interior);
}

}  // namespace fast_lio
#endif
