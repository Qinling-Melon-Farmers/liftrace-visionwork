#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace uav_vision {

struct EllipseSupport {
  double normal_rms_px = 0.0;
  double visible_arc_fraction = 0.0;
  double largest_gap_degrees = 360.0;
};

// Diagnostics only: these do not replace the historically calibrated quality.
// Sample contour segments by arc length, rather than counting compressed
// CHAIN_APPROX_SIMPLE vertices. Off-ellipse chords do not count as visible arc.
inline EllipseSupport ellipseSupport(const std::vector<cv::Point>& contour,
                                     const cv::RotatedRect& ellipse) {
  EllipseSupport result;
  const double a = ellipse.size.width * 0.5;
  const double b = ellipse.size.height * 0.5;
  if (contour.size() < 2 || a <= 0.0 || b <= 0.0) return result;
  const double angle = ellipse.angle * CV_PI / 180.0;
  const double c = std::cos(angle), s = std::sin(angle);
  const double perimeter = cv::arcLength(contour, true);
  if (!std::isfinite(perimeter) || perimeter <= 0.0) return result;
  const int count = std::min(512, std::max(72, static_cast<int>(std::ceil(perimeter))));
  const double step = perimeter / count;
  const double tolerance = std::max(1.5, 0.03 * std::min(a, b));
  std::array<bool, 36> occupied{};
  double squared_sum = 0.0, traversed = 0.0;
  int segment = 0;
  for (int sample = 0; sample < count; ++sample) {
    const double distance = sample * step;
    double length = 0.0;
    while (segment < static_cast<int>(contour.size())) {
      length = cv::norm(contour[(segment + 1) % contour.size()] - contour[segment]);
      if (length > 0.0 && traversed + length > distance) break;
      traversed += length;
      ++segment;
    }
    if (segment == static_cast<int>(contour.size())) break;
    const cv::Point2d p = cv::Point2d(contour[segment]) +
        cv::Point2d(contour[(segment + 1) % contour.size()] - contour[segment]) *
        ((distance - traversed) / length);
    const double dx = p.x - ellipse.center.x, dy = p.y - ellipse.center.y;
    const double x = c * dx + s * dy, y = -s * dx + c * dy;
    const double f = x*x/(a*a) + y*y/(b*b) - 1.0;
    const double gradient = 2.0 * std::hypot(x/(a*a), y/(b*b));
    // First-order normal distance to the fitted ellipse, not an exact nearest
    // point distance. This is well-conditioned near a valid ellipse boundary.
    const double residual = gradient > 1e-12 ? std::abs(f)/gradient : std::min(a,b);
    squared_sum += residual * residual;
    if (residual <= tolerance) {
      double theta = std::atan2(y/b, x/a);
      if (theta < 0.0) theta += 2.0 * CV_PI;
      occupied[std::min(35, static_cast<int>(theta * 36 / (2.0 * CV_PI)))] = true;
    }
  }
  result.normal_rms_px = std::sqrt(squared_sum / count);
  result.visible_arc_fraction = std::count(occupied.begin(), occupied.end(), true) / 36.0;
  int run = 0, largest = 0;
  for (int i = 0; i < 72; ++i) {
    run = occupied[i % 36] ? 0 : std::min(36, run + 1);
    largest = std::max(largest, run);
  }
  result.largest_gap_degrees = largest * 10.0;
  return result;
}

// Keep the existing distance criterion and score scale. Suppression must see
// the best-quality candidate first; contour enumeration order is not evidence.
template<class Candidate>
inline void qualityOrderedCircleNms(std::vector<Candidate>& candidates,
                                   double center_ratio, int max_candidates) {
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
    if (a.quality != b.quality) return a.quality > b.quality;
    if (a.ellipse.center.x != b.ellipse.center.x)
      return a.ellipse.center.x < b.ellipse.center.x;
    if (a.ellipse.center.y != b.ellipse.center.y)
      return a.ellipse.center.y < b.ellipse.center.y;
    if (a.ellipse.size.width != b.ellipse.size.width)
      return a.ellipse.size.width > b.ellipse.size.width;
    if (a.ellipse.size.height != b.ellipse.size.height)
      return a.ellipse.size.height > b.ellipse.size.height;
    return a.ellipse.angle < b.ellipse.angle;
  });
  std::vector<Candidate> kept;
  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (const auto& previous : kept) {
      const double radius = std::max(previous.ellipse.size.width,
                                     previous.ellipse.size.height) * center_ratio;
      if (cv::norm(previous.ellipse.center - candidate.ellipse.center) < radius) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) kept.push_back(candidate);
    if (max_candidates > 0 && static_cast<int>(kept.size()) >= max_candidates) break;
  }
  if (max_candidates <= 0) kept.clear();
  candidates.swap(kept);
}

}  // namespace uav_vision
