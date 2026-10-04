#pragma once
#include <cmath>
#include <limits>
#include <string>
#include <Eigen/Core>
#include <ros/ros.h>

namespace fast_planner {
// FC-reference local Z, NOT occupancy or a wall with additional inflation.
// The same existing controller parameter is read by every planning layer.
struct ReferenceHeight {
  bool enabled = false, valid = true;
  double max_z = std::numeric_limits<double>::infinity();
  std::string frame;
  bool accepts(double z) const { return valid && std::isfinite(z) && (!enabled || z <= max_z + 1e-9); }
  bool controls(const Eigen::MatrixXd& p) const {
    if (!valid || !p.allFinite() || p.cols()!=3 || p.rows()==0) return false;
    // B-spline convex-hull bound guarantees the entire continuous curve.
    return !enabled || p.col(2).maxCoeff() <= max_z + 1e-9;
  }
  bool polynomial(double c0,double c1,double c2,double c3,double duration) const {
    if (!valid || !std::isfinite(duration) || duration < 0) return false;
    auto at=[&](double t){return accepts(((c3*t+c2)*t+c1)*t+c0);};
    if (!at(0) || !at(duration)) return false;
    auto root=[&](double t){return t<=0 || t>=duration || at(t);};
    if (std::abs(c3)<1e-12) return std::abs(c2)<1e-12 || root(-c1/(2*c2));
    double d=4*c2*c2-12*c3*c1;
    return d<0 || (root((-2*c2+std::sqrt(d))/(6*c3)) && root((-2*c2-std::sqrt(d))/(6*c3)));
  }
};
inline ReferenceHeight referenceHeight() {
  ReferenceHeight h;
  static const std::string ns=[](){std::string n;ros::param::param<std::string>("~height_constraint_namespace",n,"/navigation_height_constraint");return n;}();
  ros::param::getCached(ns+"/enabled", h.enabled);
  if (!h.enabled) return h;
  std::string source;
  source="/external_planner_max_command_z";ros::param::getCached(ns+"/limit_parameter",source);
  h.valid=ros::param::getCached(source,h.max_z) && std::isfinite(h.max_z) &&
      ros::param::getCached(ns+"/frame_id",h.frame) && !h.frame.empty();
  return h;
}
} // namespace fast_planner
