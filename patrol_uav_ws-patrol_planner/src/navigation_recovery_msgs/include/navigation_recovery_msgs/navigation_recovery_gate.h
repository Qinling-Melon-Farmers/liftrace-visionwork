#pragma once
#include <navigation_recovery_msgs/NavigationRecoveryCommand.h>
#include <navigation_recovery_msgs/NavigationRecoveryContext.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <stdexcept>
#include <cmath>

namespace navigation_recovery_msgs {
// Identical receiving fence in traj_server and final controller. The planner
// owns clearance; receivers fence identity, original deadline, local lead and
// height exemption. This does not create a second mission authority.
struct NavigationRecoveryGate {
  bool enabled=false;
  double max_age=.15, max_seconds=8., max_distance=.20, max_speed=.30;
  double hard_min_z=.05, hard_max_z=3.5, max_excess=.25;
  void configure(ros::NodeHandle& nh) {
    nh.param("navigation_recovery/enabled",enabled,false);
    nh.param("navigation_recovery/max_command_age",max_age,.15);
    nh.param("navigation_recovery/max_seconds",max_seconds,8.);
    nh.param("navigation_recovery/max_distance",max_distance,.20);
    nh.param("navigation_recovery/max_speed",max_speed,.30);
    nh.param("navigation_recovery/hard_min_z",hard_min_z,.05);
    nh.param("navigation_recovery/hard_max_z",hard_max_z,3.5);
    nh.param("navigation_recovery/max_height_excess",max_excess,.25);
    if (!(max_age>0 && max_age<=.5 && max_seconds>0 && max_seconds<=20 &&
          max_distance>0 && max_distance<=.3 && max_speed>0 && max_speed<=.5 &&
          std::isfinite(hard_min_z) && std::isfinite(hard_max_z) && hard_max_z>hard_min_z &&
          std::isfinite(max_excess) && max_excess>=0 && max_excess<=.5))
      throw std::invalid_argument("invalid navigation recovery receiver limits");
  }
  bool accepts(const NavigationRecoveryCommand& m,const NavigationRecoveryContext& context,
               const ros::Time& now,const Eigen::Vector3d& measured,bool owner) const {
    const double age=(now-m.header.stamp).toSec();
    const double elapsed=(now-m.started_at).toSec();
    const Eigen::Vector3d p(m.pose.position.x,m.pose.position.y,m.pose.position.z);
    const Eigen::Vector3d v(m.velocity.x,m.velocity.y,m.velocity.z);
    const auto& q=m.pose.orientation;
    const double qnorm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return enabled && owner && m.active && context.active && m.recovery_id>0 &&
      !m.goal_stamp.isZero() && m.goal_stamp==context.header.stamp &&
      m.header.frame_id==context.header.frame_id && !m.header.frame_id.empty() &&
      age>=0 && age<=max_age && elapsed>=0 && elapsed<max_seconds &&
      m.deadline>now && m.deadline<=context.deadline &&
      m.deadline<=m.started_at+ros::Duration(max_seconds) &&
      p.allFinite() && v.allFinite() && measured.allFinite() &&
      std::isfinite(qnorm) && std::abs(qnorm-1.)<.002 &&
      v.norm()<=max_speed+1e-6 && (p-measured).norm()<=max_distance &&
      std::isfinite(m.soft_max_z) && p.z()>=hard_min_z && p.z()<=hard_max_z &&
      p.z()<=m.soft_max_z+max_excess;
  }
};
}
