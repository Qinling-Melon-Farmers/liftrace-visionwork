/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/
/**
 *  @file kino_replan_fsm.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief 优化了原有planner的z轴策略以及yaw角规划策略等等，详见readme
 *  @version 0.2
 *  @date 5-16-2025
 */

#include <limits>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/goal_adjustment.h>
#include <tf/tf.h>

namespace fast_planner {

void KinoReplanFSM::updateEffectiveGoal() {
  geometry_msgs::PoseStamped effective_goal = goal_status_tracker_.effectiveGoal();
  effective_goal.pose.position.x = end_pt_(0);
  effective_goal.pose.position.y = end_pt_(1);
  effective_goal.pose.position.z = end_pt_(2);
  goal_status_tracker_.updateEffectiveGoal(effective_goal);
}

double KinoReplanFSM::currentGoalDistance() const {
  return have_odom_ ? (end_pt_ - odom_pos_).norm()
                    : std::numeric_limits<double>::quiet_NaN();
}

void KinoReplanFSM::publishGoalStatus(const plan_manage::PlannerStatus& msg) {
  goal_status_pub_.publish(msg);
}

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  trigger_     = false;
  have_target_ = false;
  have_odom_   = false;
  odom_pos_.setZero(); odom_vel_.setZero();
  goal_status_tracker_.reset();

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("fsm/tracking_replan_distance", tracking_replan_distance_, 0.45);
  nh.param("fsm/min_replan_interval", min_replan_interval_, 0.75);
  nh.param("fsm/allow_goal_adjustment", allow_goal_adjustment_, true);
  nh.param("fsm/goal_adjustment_radius", goal_adjustment_radius_, 1.0);
  nh.param<std::string>("goal_status_topic", goal_status_topic_, "/planning/goal_status");

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  nh.param("fsm/liveness_enabled", liveness_enabled_, false);
  nh.param("progress/enabled", progress_enabled_, false);
  nh.param("fsm/server_hold_replan_enabled", server_hold_replan_enabled_, false);
  nh.param("fsm/server_hold_seconds", server_hold_monitor_.window, 0.25);
  nh.param("fsm/server_progress_max_age", server_hold_monitor_.max_age, 0.50);
  nh.param("fsm/no_progress_seconds", motion_watchdog_.window, 4.0);
  nh.param("fsm/no_progress_m", motion_watchdog_.movement, 0.04);
  int recovery_budget = 2;
  nh.param("fsm/recovery_budget", recovery_budget, 2);
  nh.param("fsm/odom_max_age", odom_max_age_, 0.5);
  nh.param("fsm/map_max_age", map_max_age_, 2.0);
  if (!std::isfinite(motion_watchdog_.window) || motion_watchdog_.window < 2.0 ||
      !std::isfinite(motion_watchdog_.movement) || motion_watchdog_.movement < 0.01 ||
      recovery_budget < 1 || recovery_budget > 5 || odom_max_age_ <= 0 || map_max_age_ <= 0 ||
      !std::isfinite(server_hold_monitor_.window) || server_hold_monitor_.window < 0.05 ||
      server_hold_monitor_.window > 2.0 || !std::isfinite(server_hold_monitor_.max_age) ||
      server_hold_monitor_.max_age < server_hold_monitor_.window ||
      server_hold_monitor_.max_age > 2.0 || (server_hold_replan_enabled_ && !progress_enabled_))
    throw std::invalid_argument("invalid motion watchdog configuration");
  motion_watchdog_.budget = recovery_budget;
  std::string mode_topic, map_topic, progress_topic;
  nh.param<std::string>("fsm/controller_mode_topic", mode_topic, "/detect/point_class");
  nh.param<std::string>("fsm/map_freshness_topic", map_topic, "/freedom/static_pointcloud");
  nh.param<std::string>("progress/topic", progress_topic, "/planning/progress");
  controller_sub_ = nh.subscribe<std_msgs::Int8>(mode_topic, 1,
      [this](const std_msgs::Int8::ConstPtr& msg) {
        controller_mode_ = msg->data; controller_stamp_ = ros::Time::now();
      });
  map_age_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(map_topic, 1,
      [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (msg->width && msg->height && !msg->header.stamp.isZero()) map_stamp_ = msg->header.stamp;
      });
  if (progress_enabled_) progress_pub_ = nh.advertise<plan_manage::TrajectoryProgress>(progress_topic, 20);
  if (server_hold_replan_enabled_)
    server_progress_sub_ = nh.subscribe<plan_manage::TrajectoryProgress>(
        progress_topic, 20, &KinoReplanFSM::serverProgressCallback, this);

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/fastplanner/goal", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  goal_status_pub_ = nh.advertise<plan_manage::PlannerStatus>(goal_status_topic_, 10);
}

void KinoReplanFSM::waypointCallback(const geometry_msgs::PoseStamped msg) {
  double phase_radius = goal_adjustment_radius_;
  if (ros::param::getCached("~fsm/goal_adjustment_radius", phase_radius) &&
      std::isfinite(phase_radius) && phase_radius >= 0.0)
    goal_adjustment_radius_ = phase_radius;
  if (msg.pose.position.z <= -0.5) return;

  double cancelled_distance = std::numeric_limits<double>::quiet_NaN();
  if (goal_status_tracker_.active()) cancelled_distance = currentGoalDistance();

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  // PoseStamped carries vehicle attitude, not a translational velocity
  // contract.  A discrete mission goal must therefore end at rest; deriving
  // +x motion from the default identity quaternion bends narrow-door paths.
  end_vel_.setZero();
  requested_end_pt_ = end_pt_;
  motion_watchdog_.reset();
  server_hold_monitor_.reset();
  server_hold_replan_pending_ = false;
  goal_has_trajectory_ = false;
  next_planning_attempt_ = ros::Time(0);

  geometry_msgs::PoseStamped effective_goal = msg;
  effective_goal.pose.position.x = end_pt_(0);
  effective_goal.pose.position.y = end_pt_(1);
  effective_goal.pose.position.z = end_pt_(2);

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  have_target_ = true;
  const std::vector<plan_manage::PlannerStatus> status_events =
      goal_status_tracker_.replaceGoal(msg, effective_goal, ros::Time::now(), cancelled_distance,
                                       currentGoalDistance());
  for (const auto& status_event : status_events) publishGoalStatus(status_event);

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}


void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = odom_pos_.allFinite() && odom_vel_.allFinite();
  odom_stamp_ = msg->header.stamp;
}

void KinoReplanFSM::serverProgressCallback(
    const plan_manage::TrajectoryProgress::ConstPtr& msg) {
  if (!server_hold_replan_enabled_ || msg->source != msg->SERVER ||
      !have_target_ || !goal_has_trajectory_ || exec_state_ != EXEC_TRAJ ||
      controller_mode_ != 1 || !std::isfinite(msg->odom_age) ||
      msg->odom_age < 0.0 || msg->odom_age > odom_max_age_) return;

  const ros::Time now = ros::Time::now();
  const auto fresh = [&now](const ros::Time& stamp, double age) {
    return !stamp.isZero() && (now - stamp).toSec() >= 0.0 &&
        (now - stamp).toSec() <= age;
  };
  if (!fresh(controller_stamp_, 0.5) || !fresh(odom_stamp_, odom_max_age_) ||
      !fresh(map_stamp_, map_max_age_)) return;

  const LocalTrajData* info = &planner_manager_->local_data_;
  const ServerHoldMonitor::Result result = server_hold_monitor_.observe(
      now.toSec(), msg->header.stamp.toSec(), goal_status_tracker_.goalSeq(),
      info->traj_id_, info->start_time_.toNSec(), msg->goal_seq, msg->traj_id,
      msg->traj_start.toNSec(), msg->tracking_hold && !msg->interrupted);
  if (result == ServerHoldMonitor::REPLAN) server_hold_replan_pending_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  const ros::Time now = ros::Time::now();
  const auto fresh = [&now](const ros::Time& stamp, double age) {
    return !stamp.isZero() && (now-stamp).toSec() >= 0 && (now-stamp).toSec() <= age;
  };
  const bool motion = have_target_ && currentGoalDistance() > no_replan_thresh_ && goal_has_trajectory_ && controller_mode_ == 1 &&
      fresh(controller_stamp_, 0.5) && have_odom_ && fresh(odom_stamp_, odom_max_age_) &&
      fresh(map_stamp_, map_max_age_);
  std::string progress_reason = motion ? "tracking" : "motion_or_inputs_inactive";
  bool recovery_handled = false;
  if (server_hold_replan_pending_) {
    if (server_hold_replan_enabled_ && motion && exec_state_ == EXEC_TRAJ) {
      server_hold_replan_pending_ = false;
      const auto status = motion_watchdog_.requestRecovery(now.toSec());
      recovery_handled = true;
      if (status == MotionWatchdog::REPLAN) {
        progress_reason = "server_tracking_hold_replan";
        // traj_server is already holding the measured position. Keep that
        // safe generation alive until a validated replacement is published.
        changeFSMExecState(REPLAN_TRAJ, "SERVER_HOLD");
      } else if (status == MotionWatchdog::EXHAUSTED) {
        progress_reason = "server_hold_budget_exhausted";
        replan_pub_.publish(std_msgs::Empty());
        publishGoalStatus(goal_status_tracker_.record(
            plan_manage::PlannerStatus::FAILED_ATTEMPT, progress_reason, now,
            currentGoalDistance()));
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "SERVER_HOLD");
      }
    }
  }
  if (liveness_enabled_ && !recovery_handled) {
    const auto status = motion_watchdog_.observe(now.toSec(), odom_pos_, motion);
    if (status == MotionWatchdog::REPLAN) {
      progress_reason = "no_physical_progress_replan";
      // Retire the old curve; never unlatch a hold without a newly validated curve.
      replan_pub_.publish(std_msgs::Empty());
      if (exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ, "LIVENESS");
    } else if (status == MotionWatchdog::EXHAUSTED && have_target_) {
      progress_reason = "liveness_budget_exhausted";
      replan_pub_.publish(std_msgs::Empty());
      publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::FAILED_ATTEMPT,
          progress_reason, now, currentGoalDistance()));
      have_target_ = false;
      changeFSMExecState(WAIT_TARGET, "LIVENESS");
    }
  }
  publishProgress(progress_reason, motion);

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      if (ros::Time::now() < next_planning_attempt_) return;
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      const bool first_attempt = goal_status_tracker_.planningAttempt() == 0;
      publishGoalStatus(goal_status_tracker_.beginAttempt(
          plan_manage::PlannerStatus::PLANNING,
          first_attempt ? "new_trajectory_attempt" : "new_trajectory_retry", ros::Time::now(),
          currentGoalDistance()));
      bool success = callKinodynamicReplan();
      if (success) {
        publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::TRAJECTORY_READY,
                                                      "new_trajectory_ready", ros::Time::now(),
                                                      currentGoalDistance()));
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::FAILED_ATTEMPT,
                                                      "new_trajectory_attempt_failed",
                                                      ros::Time::now(), currentGoalDistance()));
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      const auto position = [info](double t) -> Eigen::Vector3d {
        return info->position_traj_.evaluateDeBoorT(t);
      };
      double following_lead = 0.4;
      ros::param::getCached("/traj_server/traj_server/target_dist", following_lead);
      if (!std::isfinite(following_lead) || following_lead <= 0.0) following_lead=0.4;
      info->execution_time_ = projectProgress(
          position, odom_pos_, info->execution_time_, info->duration_, std::min(0.4,following_lead));
      const double goal_distance = currentGoalDistance();
      if (goal_status_tracker_.canFinishWithin(goal_distance, no_replan_thresh_)) {
        publishGoalStatus(goal_status_tracker_.finish(
            "goal_reached_after_local_trajectory", ros::Time::now(), goal_distance));
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      }
      const double tracking_error = (position(info->execution_time_) - odom_pos_).norm();
      const bool partial = (position(info->duration_) - end_pt_).norm() > no_replan_thresh_;
      const bool exhausted = info->execution_time_ >= info->duration_ - 0.03;
      // A safe complete curve is worth following. Rebuilding it every 0.2 m
      // repeatedly restarts acceleration without adding useful map coverage.
      if (tracking_error > tracking_replan_distance_ || exhausted ||
          (partial && (odom_pos_ - info->start_pos_).norm() >= replan_thresh_ &&
           (time_now - info->start_time_).toSec() >= min_replan_interval_)) {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      if (ros::Time::now() < next_planning_attempt_) return;
      LocalTrajData* info     = &planner_manager_->local_data_;
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      if (info->duration_ > 0.0 && info->execution_time_ < info->duration_) {
        const Eigen::Vector3d reference_velocity =
            info->velocity_traj_.evaluateDeBoorT(info->execution_time_);
        if ((reference_velocity - odom_vel_).norm() < 0.25)
          start_acc_ = info->acceleration_traj_.evaluateDeBoorT(info->execution_time_);
      }

      // 提取yaw角
      double roll, pitch, yaw;
      tf::Quaternion q(odom_orient_.x(), odom_orient_.y(), odom_orient_.z(), odom_orient_.w());
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      start_yaw_(0) = yaw;
      start_yaw_(1) = 0.0;  // 如果没有估计可设为0
      start_yaw_(2) = 0.0;  // 同上

      publishGoalStatus(goal_status_tracker_.beginAttempt(
          plan_manage::PlannerStatus::REPLANNING, "trajectory_replan_attempt", ros::Time::now(),
          currentGoalDistance()));
      bool success = callKinodynamicReplan();
      if (success) {
        publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::TRAJECTORY_READY,
                                                      "replanned_trajectory_ready", ros::Time::now(),
                                                      currentGoalDistance()));
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::FAILED_ATTEMPT,
                                                      "trajectory_replan_attempt_failed",
                                                      ros::Time::now(), currentGoalDistance()));
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_ && allow_goal_adjustment_) {
    auto edt_env = planner_manager_->edt_environment_;

    const double minimum_clearance = planner_manager_->pp_.clearance_;
    auto clearance = [&](const Eigen::Vector3d& candidate) {
      if (!edt_env->sdf_map_->isInMap(candidate) ||
          edt_env->sdf_map_->getInflateOccupancy(candidate) != 0)
        return -std::numeric_limits<double>::infinity();
      Eigen::Vector3d point = candidate;
      return edt_env->evaluateCoarseEDT(
          point, planner_manager_->pp_.dynamic_ ? info->duration_ : -1.0);
    };

    // Use the same clearance as trajectory validation. The old hard-coded
    // extra 0.20 m beyond the already inflated map cannot fit a 0.80 m gap.
    // Retain a valid effective goal: do not chase every small map change.
    const double current_clearance = clearance(end_pt_);
    if (!std::isfinite(current_clearance) || current_clearance < minimum_clearance) {
      Eigen::Vector3d goal;
      const bool found = nearbyFreeGoal(requested_end_pt_, goal_adjustment_radius_,
          edt_env->sdf_map_->getResolution(), minimum_clearance, clearance, goal);
      if (found) {
        end_pt_ = goal;
        end_vel_.setZero();
        updateEffectiveGoal();
        ROS_INFO_STREAM("Goal adjusted within requested waypoint neighborhood: " << goal.transpose());
        if (exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        ROS_WARN_STREAM_THROTTLE(2.0,
            "No collision-free goal within requested waypoint neighborhood"
            << " requested=" << requested_end_pt_.transpose()
            << " effective=" << end_pt_.transpose()
            << " in_map=" << edt_env->sdf_map_->isInMap(requested_end_pt_)
            << " inflated=" << edt_env->sdf_map_->getInflateOccupancy(requested_end_pt_)
            << " clearance=" << current_clearance
            << " radius=" << goal_adjustment_radius_);
        if (exec_state_ == EXEC_TRAJ) {
          replan_pub_.publish(std_msgs::Empty());
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        // GEN_NEW_TRAJ already retries on the existing interval. No reset
        // or repeated cancellation is needed merely because the goal is blocked.
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      replan_pub_.publish(std_msgs::Empty());
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  next_planning_attempt_ = plan_success ? ros::Time(0) :
      ros::Time::now() + ros::Duration(min_replan_interval_);

  if (plan_success) {

    goal_has_trajectory_ = true;
    server_hold_replan_pending_ = false;
    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;
    bspline.goal_stamp = goal_status_tracker_.effectiveGoal().header.stamp;
    bspline.goal_frame = goal_status_tracker_.effectiveGoal().header.frame_id;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

void KinoReplanFSM::publishProgress(const std::string& reason, bool motion) {
  const ros::Time now = ros::Time::now();
  if (!progress_enabled_ || !goal_has_trajectory_) return;
  if (reason == progress_reason_ && (now-progress_stamp_).toSec() < 0.1) return;
  progress_stamp_ = now; progress_reason_ = reason;
  const auto info = &planner_manager_->local_data_;
  plan_manage::TrajectoryProgress m;
  m.header.stamp=now; m.header.frame_id=goal_status_tracker_.effectiveGoal().header.frame_id;
  m.source=m.FSM; m.goal_seq=goal_status_tracker_.goalSeq(); m.traj_id=info->traj_id_;
  m.traj_start=info->start_time_; m.projection_t=info->execution_time_; m.duration=info->duration_;
  auto point=[](const Eigen::Vector3d& v) { geometry_msgs::Point p; p.x=v.x();p.y=v.y();p.z=v.z();return p; };
  m.odom=point(odom_pos_);
  const Eigen::Vector3d projected=info->position_traj_.evaluateDeBoorT(info->execution_time_);
  m.projection=point(projected); m.tracking_error=(projected-odom_pos_).norm();
  m.motion_intent=motion; m.odom_age=(now-odom_stamp_).toSec(); m.map_age=(now-map_stamp_).toSec();
  m.stagnant_seconds=motion_watchdog_.stagnant(now.toSec()); m.recoveries=motion_watchdog_.attempts();
  m.reason=reason; progress_pub_.publish(m);
}

// KinoReplanFSM::
}  // namespace fast_planner
