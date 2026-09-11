#ifndef LIFTRACE_LOCAL_SEGMENT_PROBE_H
#define LIFTRACE_LOCAL_SEGMENT_PROBE_H

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fast_planner {

// Metadata refers to the actual cloud integration window, not latest odometry.
struct LocalCloudView {
  bool valid = false;
  uint64_t revision = 0;
  double stamp = 0.0;
  std::string frame;
  Eigen::Vector3d lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d upper = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double resolution = 0.0;
};

enum SegmentStatus { CLEAR_IN_INFLATED_MAP=0, OCCUPIED=1, OUTSIDE_WINDOW=2,
                     TOO_LONG=3, NOT_CHECKED=4 };
struct Segment { Eigen::Vector3d start, end; };
struct ProbeLimits {
  size_t max_segments = 32;
  size_t max_voxels = 4096;
  double max_length = 4.0;
  double max_wall_ms = 3.0;
  double max_age = 1.0;
};
struct ProbeResult {
  bool accepted = false;
  std::string reason;
  std::vector<unsigned char> status;
  size_t checked_voxels = 0;
};

// Conservative supercover traversal of a CENTERLINE in an already-inflated
// map. Includes tie-adjacent cells and both sides of voxel boundary planes.
// This is not a trajectory, dynamic feasibility test, or proof of known free.
inline ProbeResult probeSegments(const LocalCloudView& map,
    const std::vector<Segment>& segments, const ProbeLimits& limits,
    double now, const std::function<int(const Eigen::Vector3i&)>& occupancy) {
  ProbeResult result;
  if (segments.empty() || segments.size() > limits.max_segments ||
      !limits.max_voxels || !(limits.max_length > 0) || !std::isfinite(limits.max_length) ||
      !(limits.max_wall_ms > 0) || !std::isfinite(limits.max_wall_ms) ||
      !(limits.max_age > 0) || !std::isfinite(limits.max_age)) {
    result.reason = "invalid_or_oversized_batch"; return result;
  }
  result.status.assign(segments.size(), NOT_CHECKED);
  if (!map.valid || !std::isfinite(now) || !std::isfinite(map.stamp) || map.stamp <= 0 ||
      now < map.stamp || now-map.stamp > limits.max_age ||
      !map.lower.allFinite() || !map.upper.allFinite() || !map.origin.allFinite() ||
      !(map.lower.array() < map.upper.array()).all() ||
      !(map.resolution > 0) || !std::isfinite(map.resolution)) {
    result.reason = "map_invalid_stale_or_future"; return result;
  }
  for (const auto& s : segments) if (!s.start.allFinite() || !s.end.allFinite()) {
    result.reason = "nonfinite_segment"; return result;
  }
  const auto begin = std::chrono::steady_clock::now();
  bool exhausted = false;
  auto inside = [&](const Eigen::Vector3d& p) {
    return (p.array() > map.lower.array()).all() && (p.array() < map.upper.array()).all();
  };
  for (size_t n=0; n<segments.size() && !exhausted; ++n) {
    const auto& s = segments[n];
    if (!inside(s.start) || !inside(s.end)) { result.status[n]=OUTSIDE_WINDOW; continue; }
    if ((s.end-s.start).norm() > limits.max_length) { result.status[n]=TOO_LONG; continue; }
    const Eigen::Vector3d a=(s.start-map.origin)/map.resolution;
    const Eigen::Vector3d b=(s.end-map.origin)/map.resolution;
    const Eigen::Vector3d d=b-a;
    // Reject pathological coordinates before converting floating values to int.
    if (a.cwiseAbs().maxCoeff() > 1e8 || b.cwiseAbs().maxCoeff() > 1e8) {
      result.reason="invalid_voxel_coordinates"; return result;
    }
    Eigen::Vector3i cell=a.array().floor().cast<int>();
    Eigen::Vector3i step;
    Eigen::Vector3d tmax, delta;
    int parallel_mask=0;
    for (int axis=0; axis<3; ++axis) {
      step[axis]=(d[axis]>0)-(d[axis]<0);
      delta[axis]=step[axis] ? 1.0/std::abs(d[axis]) : INFINITY;
      tmax[axis]=step[axis] ? ((step[axis]>0 ? cell[axis]+1 : cell[axis])-a[axis])/d[axis] : INFINITY;
      if (!step[axis] && std::abs(a[axis]-std::round(a[axis]))<1e-9) parallel_mask |= 1<<axis;
    }
    int status=CLEAR_IN_INFLATED_MAP;
    auto visit = [&](Eigen::Vector3i id) {
      // Cells adjacent to a line lying on a grid plane must also be clear.
      for (int mask=0; mask<8 && status==CLEAR_IN_INFLATED_MAP && !exhausted; ++mask) {
        if (mask & ~parallel_mask) continue;
        Eigen::Vector3i p=id;
        for (int axis=0; axis<3; ++axis) if (mask & (1<<axis)) --p[axis];
        if (result.checked_voxels >= limits.max_voxels ||
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count() > limits.max_wall_ms) {
          exhausted=true; break;
        }
        ++result.checked_voxels;
        const Eigen::Vector3d center=map.origin+(p.cast<double>()+Eigen::Vector3d::Constant(.5))*map.resolution;
        if (!inside(center)) { status=OUTSIDE_WINDOW; break; }
        const int value=occupancy(p);
        if (value<0) status=OUTSIDE_WINDOW;
        else if (value!=0) status=OCCUPIED;
      }
    };
    // Include all cells touching a boundary-aligned start, even when moving away.
    int start_mask=0;
    for (int axis=0;axis<3;++axis)
      if (step[axis] && std::abs(a[axis]-std::round(a[axis]))<1e-9) start_mask|=1<<axis;
    for (int mask=0;mask<8;++mask) if (!(mask & ~start_mask)) {
      Eigen::Vector3i id=cell;
      for (int axis=0;axis<3;++axis) if (mask & (1<<axis)) --id[axis];
      visit(id);
    }
    while (status==CLEAR_IN_INFLATED_MAP && !exhausted) {
      const double next=tmax.minCoeff();
      if (!std::isfinite(next) || next>1.+1e-10) break;
      int tied=0;
      for (int axis=0;axis<3;++axis) if (std::abs(tmax[axis]-next)<1e-10) tied|=1<<axis;
      for (int mask=1;mask<8;++mask) if (!(mask & ~tied)) {
        Eigen::Vector3i id=cell;
        for (int axis=0;axis<3;++axis) if (mask & (1<<axis)) id[axis]+=step[axis];
        visit(id);
      }
      for (int axis=0;axis<3;++axis) if (tied & (1<<axis)) {
        cell[axis]+=step[axis];tmax[axis]+=delta[axis];
      }
    }
    result.status[n]=static_cast<unsigned char>(status);
  }
  if (exhausted) {
    std::fill(result.status.begin(), result.status.end(), NOT_CHECKED);
    result.reason="query_budget_exhausted"; return result;
  }
  result.accepted=true;result.reason="inflated_cloud_geometry_only";
  return result;
}
}  // namespace fast_planner
#endif
