#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace fast_planner {
namespace recovery {
using Vec = Eigen::Vector3d;
using Controls = std::array<Vec, 4>;

// Independent candidate, not an alternate mission authority. No ROS, map
// mutation, FC command publisher or inferred health/reset event lives here.
struct Config {
  bool enabled = false;
  // Optional offline comparison with the Oct-6 evidence contract. Production
  // uses the existing localisation, as explicitly selected on Oct-8.
  bool require_reference_attestation = false;
  double radius = 0.8, max_speed = 0.3, max_acceleration = 0.5;
  double max_seconds = 8.0, max_input_age = 0.25, max_map_age = 0.5;
  double hard_min_z = 0.0, hard_max_z = 3.5;  // task-frame body reference
  double max_height_excess = 0.25, tracking_error = 0.12;
  double finish_position = 0.04, finish_speed = 0.05, settle_seconds = 0.2;
  double box_resolution = 0.01;
  unsigned max_attempts = 1;
  bool valid() const {
    const double values[] = {radius, max_speed, max_acceleration, max_seconds,
      max_input_age, max_map_age, hard_min_z, hard_max_z, max_height_excess,
      tracking_error, finish_position, finish_speed, settle_seconds, box_resolution};
    for (double x : values) if (!std::isfinite(x)) return false;
    return radius > 0 && radius <= 2 && max_speed > 0 && max_speed <= 0.5 &&
      max_acceleration > 0 && max_seconds > 0 && max_seconds <= 20 &&
      max_input_age > 0 && max_map_age > 0 && hard_min_z < hard_max_z &&
      max_height_excess >= 0 && tracking_error > 0 && finish_position > 0 &&
      finish_speed > 0 && settle_seconds > 0 && box_resolution > 0 &&
      box_resolution <= 0.05 && max_attempts > 0 && max_attempts <= 3;
  }
};

struct Evidence {
  double stamp = 0, map_stamp = 0;
  uint64_t generation = 0, map_generation = 0, map_revision = 0;
  std::string frame, map_frame;
  // All default false. Fresh odom, FC/LIO agreement and a static TF cannot
  // manufacture these attestations. Adapter must consume trusted producers.
  bool reference_verified = false, lio_healthy = false;
  bool fc_reset_stream_verified = false, independent_height_verified = false;
  bool coverage_verified = false;
};

// Recovery velocities must share the task frame with the certified curve.
// Keep ordinary-planner twist interpretation outside this opt-in adapter.
inline std::string parentVelocity(const Vec& raw, const Eigen::Quaterniond& rotation,
                                  const std::string& child_frame,
                                  const std::string& twist_frame, Vec& result) {
  result.setConstant(std::numeric_limits<double>::quiet_NaN());
  if (!raw.allFinite()) return "recovery_velocity_nonfinite";
  // Both modes publish the measured attitude, so reject unusable orientation
  // before consuming a recovery attempt even when twist is already in world.
  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1.)>.01)
    return "recovery_orientation_invalid";
  if (twist_frame == "world") { result=raw;return ""; }
  if (twist_frame != "child") return "recovery_twist_frame_invalid";
  if (child_frame.empty()) return "recovery_child_frame_missing";
  result=rotation.normalized()*raw;
  return "";
}

struct Context {
  double now = 0, action_deadline = 0, mission_deadline = 0, soft_max_z = 0;
  uint64_t goal = 0;  // immutable action identity, not a trajectory sequence
  Vec position = Vec::Zero(), velocity = Vec::Zero(), goal_position = Vec::Zero();
  std::string velocity_error;
  bool offboard = false, navigation_owner = false, takeover = false;
  bool release_transaction_active = false;
  Evidence evidence;
};

enum Source { PHYSICAL = 1, COLUMN = 2, BOUNDARY = 4, BUFFER = 8 };
struct MapQuery {
  // Conservative CLOSED boxes in the same task frame. physicalClear checks
  // measured geometry + full necessary body clearance, not raw point samples.
  // observedFree is separate: a fresh point cloud with no return is NOT proof.
  std::function<bool(const Vec&, const Vec&)> physicalClear, observedFree;
  // OR of Source bits across all intersecting voxels; -1 means unknown.
  std::function<int(const Vec&, const Vec&)> sources;
  // Ordinary planner start acceptance, with optional endpoint-position slack.
  // This is additional to recovery path clearance, never an exemption of it.
  std::function<bool(const Vec&, double)> canResume;
  bool valid() const { return physicalClear && observedFree && sources && canResume; }
};

inline bool normalPlannerStartClear(int occupied, double distance, double clearance,
                                    double position_slack, double resolution) {
  if (occupied!=0 || !std::isfinite(distance) || !std::isfinite(clearance) ||
      clearance<0 || !std::isfinite(position_slack) || position_slack<0 ||
      !std::isfinite(resolution) || resolution<=0) return false;
  // getDistance uses voxel indices. A nearby actual stopping position can
  // change the voxel-center distance by slack plus one voxel diagonal.
  const double margin=position_slack>0 ? position_slack+std::sqrt(3.)*resolution : 0.;
  return distance>=clearance+margin;
}

struct Curve {
  Controls controls;
  double duration = 0;
  Vec position(double t) const {
    const double u = std::max(0.0, std::min(1.0, t / duration)), w = 1-u;
    return w*w*w*controls[0] + 3*w*w*u*controls[1] +
           3*w*u*u*controls[2] + u*u*u*controls[3];
  }
  Vec velocity(double t) const {
    const double u = std::max(0.0, std::min(1.0, t / duration)), w = 1-u;
    return 3.0/duration * (w*w*(controls[1]-controls[0]) +
        2*w*u*(controls[2]-controls[1]) + u*u*(controls[3]-controls[2]));
  }
};

inline bool fresh(double now, double stamp, double age) {
  return std::isfinite(stamp) && stamp > 0 && now >= stamp && now-stamp <= age;
}

inline std::string admission(const Config& c, const Context& x) {
  if (!c.enabled) return "disabled";
  if (!x.velocity_error.empty()) return x.velocity_error;
  if (!c.valid() || !std::isfinite(x.now) || !std::isfinite(x.soft_max_z) ||
      !std::isfinite(x.action_deadline) || !std::isfinite(x.mission_deadline) ||
      !x.position.allFinite() || !x.velocity.allFinite() || !x.goal_position.allFinite())
    return "invalid_input";
  if (x.takeover || !x.offboard || !x.navigation_owner) return "ownership_lost";
  if (x.release_transaction_active) return "release_transaction_locked";
  if (x.now >= std::min(x.action_deadline, x.mission_deadline)) return "deadline";
  const auto& e = x.evidence;
  if ((c.require_reference_attestation &&
       (!e.reference_verified || !e.lio_healthy || !e.fc_reset_stream_verified)) ||
      e.frame.empty() || e.frame != e.map_frame || e.generation != e.map_generation)
    return "untrusted_reference";
  if (!fresh(x.now, e.stamp, c.max_input_age) ||
      !fresh(x.now, e.map_stamp, c.max_map_age)) return "stale_input";
  if (!e.coverage_verified || !e.map_revision) return "unverified_map_coverage";
  if (x.position.z() < c.hard_min_z || x.position.z() > c.hard_max_z)
    return "hard_height";
  if (c.require_reference_attestation && x.position.z() > x.soft_max_z && !e.independent_height_verified)
    return "untrusted_height";
  if (x.position.z() > x.soft_max_z+c.max_height_excess) return "height_excess";
  if (x.velocity.norm() > c.max_speed+1e-9) return "entry_speed";
  return "";
}

inline void split(const Controls& b, Controls& left, Controls& right) {
  const Vec a=(b[0]+b[1])/2, c=(b[1]+b[2])/2, d=(b[2]+b[3])/2;
  const Vec e=(a+c)/2, f=(c+d)/2, m=(e+f)/2;
  left={{b[0],a,e,m}}; right={{m,f,d,b[3]}};
}

// Convex-hull subdivision checks every point of the continuous curve,
// including t=0 and initial braking. It cannot step over thin voxels.
inline bool certify(const Controls& b, const Config& c, const Context& x,
                    const MapQuery& map, bool& exited, unsigned depth = 0) {
  Vec lo=b[0], hi=b[0];
  for (const auto& p:b) { if (!p.allFinite()) return false; lo=lo.cwiseMin(p); hi=hi.cwiseMax(p); }
  // Never exempt arena boundaries, necessary clearance, or unobserved space.
  const int mask=map.sources(lo,hi);
  // In addition to required body inflation, certify a tracking-error tube.
  // A nominally clear centerline alone cannot authorize a moving vehicle.
  const Vec tube=Vec::Constant(c.tracking_error);
  const int tube_mask=map.sources(lo-tube,hi+tube);
  const bool physical=mask>=0 && tube_mask>=0 && !(tube_mask & (PHYSICAL|BOUNDARY)) &&
      map.physicalClear(lo-tube,hi+tube) && map.observedFree(lo-tube,hi+tube);
  const bool hard=lo.z()-c.tracking_error>=c.hard_min_z &&
      hi.z()+c.tracking_error<=c.hard_max_z;
  const bool soft=hi.z()<=x.soft_max_z+1e-9;
  const bool legal=physical && hard && hi.z()+c.tracking_error<=x.soft_max_z && tube_mask==0;
  if (legal) { exited=true; return true; }
  if (depth >= 16) return false;  // no unresolved interval is accepted
  if ((hi-lo).norm() <= c.box_resolution) {
    if (!physical || !hard || exited) return false;
    // Only the extra column and a proven soft-height excess may be crossed.
    if (mask & ~(COLUMN|BUFFER)) return false;
    if (!soft && ((c.require_reference_attestation && !x.evidence.independent_height_verified) ||
        hi.z()>x.soft_max_z+c.max_height_excess)) return false;
    return true;
  }
  Controls left,right; split(b,left,right);
  return certify(left,c,x,map,exited,depth+1) && certify(right,c,x,map,exited,depth+1);
}

inline bool validate(const Curve& curve, const Config& c, const Context& start,
                     const MapQuery& map) {
  if (!map.valid() || !std::isfinite(curve.duration) || curve.duration<=0 ||
      curve.duration+c.settle_seconds>c.max_seconds) return false;
  for (unsigned i=0;i<3;++i)
    if ((3.0*(curve.controls[i+1]-curve.controls[i])/curve.duration).norm()>
        c.max_speed+1e-9) return false;
  for (unsigned i=0;i<2;++i)
    if ((6.0*(curve.controls[i+2]-2*curve.controls[i+1]+curve.controls[i])/
         (curve.duration*curve.duration)).norm()>c.max_acceleration+1e-9) return false;
  if ((curve.position(0)-start.position).norm()>1e-8 ||
      (curve.velocity(0)-start.velocity).norm()>1e-8 ||
      curve.velocity(curve.duration).norm()>1e-8) return false;
  for (const auto& p:curve.controls)
    if ((p-start.position).norm()>c.radius+1e-9) return false;
  const Vec end=curve.controls[3];
  if (end.z()>start.soft_max_z || map.sources(end,end)!=0 ||
      !map.physicalClear(end,end) || !map.observedFree(end,end) ||
      !map.canResume(end,c.finish_position)) return false;
  bool exited=map.sources(start.position,start.position)==0 &&
      start.position.z()<=start.soft_max_z;
  return certify(curve.controls,c,start,map,exited) && exited;
}

inline bool plan(const Config& c, const Context& x, const MapQuery& map,
                 std::vector<Vec> candidates, Curve& result, std::string& reason) {
  reason=admission(c,x);
  if (!reason.empty()) return false;
  if (!map.valid()) { reason="map_query_missing"; return false; }
  // Revalidate historical points against today's map, ordered by displacement.
  candidates.erase(std::remove_if(candidates.begin(),candidates.end(),
      [](const Vec& p){return !p.allFinite();}),candidates.end());
  std::stable_sort(candidates.begin(),candidates.end(),[&](const Vec& a,const Vec& b){
    return (a-x.position).squaredNorm()<(b-x.position).squaredNorm();
  });
  const double available=std::min({c.max_seconds,x.action_deadline-x.now,x.mission_deadline-x.now});
  for (const Vec& end:candidates) {
    if ((end-x.position).norm()>c.radius || (end-x.position).norm()<c.finish_position) continue;
    for (double duration=0.5;duration+c.settle_seconds<available;duration+=0.25) {
      Curve curve; curve.duration=duration;
      curve.controls={{x.position,x.position+x.velocity*duration/3.0,end,end}};
      if (validate(curve,c,x,map)) { result=curve;reason="validated";return true; }
    }
  }
  reason="no_certified_exit"; return false;
}

// Bounded transaction for an executor adapter. Resume is an event carrying the
// original goal; it neither claims goal reached nor renews a mission deadline.
class Transaction {
 public:
  enum State { IDLE, ACTIVE, RESUME, FAILED };
  explicit Transaction(Config config = Config()) : config_(config) {}
  bool begin(const Context& x, const MapQuery& map, const std::vector<Vec>& candidates) {
    if (state_==ACTIVE) { reason_="already_active";return false; }
    if (!has_goal_ || goal_!=x.goal) { attempts_=0;goal_=x.goal;has_goal_=true; }
    if (attempts_>=config_.max_attempts) { reason_="budget_exhausted";return false; }
    ++attempts_;
    if (!plan(config_,x,map,candidates,curve_,reason_)) { state_=FAILED;return false; }
    start_=x;last_time_=x.now;settle_since_=-1;
    deadline_=std::min({x.action_deadline,x.mission_deadline,x.now+config_.max_seconds});
    state_=ACTIVE; return true;
  }
  State step(const Context& x, const MapQuery& map, Vec& command, Vec& velocity) {
    // No command is valid unless this returns ACTIVE. In particular, a
    // localisation failure cannot justify publishing a fabricated hover pose.
    command.setConstant(std::numeric_limits<double>::quiet_NaN()); velocity=command;
    if (state_!=ACTIVE) return state_;
    reason_=admission(config_,x);
    if (config_.require_reference_attestation && reason_.empty() && start_.position.z()>start_.soft_max_z &&
        !x.evidence.independent_height_verified) reason_="height_evidence_lost";
    if (reason_.empty() && (x.goal!=start_.goal ||
        x.evidence.generation!=start_.evidence.generation ||
        x.evidence.frame!=start_.evidence.frame ||
        (x.goal_position-start_.goal_position).norm()>1e-9 ||
        x.soft_max_z!=start_.soft_max_z)) reason_="identity_or_constraint_changed";
    if (reason_.empty() && (x.now<last_time_ || x.now>=deadline_)) reason_="deadline_or_clock";
    // New map revision is allowed only after recertifying the complete curve.
    if (reason_.empty() && !validate(curve_,config_,start_,map)) reason_="path_invalidated";
    const double t=x.now-start_.now;
    if (reason_.empty() && (x.position-curve_.position(t)).norm()>config_.tracking_error)
      reason_="tracking_error";
    if (!reason_.empty()) { state_=FAILED;return state_; }
    last_time_=x.now;
    if (t>=curve_.duration && (x.position-curve_.controls[3]).norm()<=config_.finish_position &&
        x.velocity.norm()<=config_.finish_speed && map.canResume(x.position,0.)) {
      if (settle_since_<0) settle_since_=x.now;
      if (x.now-settle_since_>=config_.settle_seconds) {
        state_=RESUME;reason_="resume_original_goal";return state_;
      }
    } else settle_since_=-1;
    command=curve_.position(t);velocity=curve_.velocity(t);reason_="recovering";
    return state_;
  }
  const Curve& curve() const { return curve_; }
  const Context& original() const { return start_; }
  const std::string& reason() const { return reason_; }
  State state() const { return state_; }
 private:
  Config config_; Context start_; Curve curve_;
  State state_=IDLE; std::string reason_;
  bool has_goal_=false; uint64_t goal_=0; unsigned attempts_=0;
  double deadline_=0,last_time_=0,settle_since_=-1;
};
}  // namespace recovery
}  // namespace fast_planner
