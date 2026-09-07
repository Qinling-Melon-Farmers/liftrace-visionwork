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
  goal_status_tracker_.reset();

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("fsm/tracking_replan_distance", tracking_replan_distance_, 0.45);
  nh.param("fsm/min_replan_interval", min_replan_interval_, 0.75);
  nh.param("fsm/allow_goal_adjustment", allow_goal_adjustment_, true);
  nh.param<std::string>("goal_status_topic", goal_status_topic_, "/planning/goal_status");

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

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

  have_odom_ = true;
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
      info->execution_time_ = projectProgress(
          position, odom_pos_, info->execution_time_, info->duration_);
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

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.2) {
      bool new_goal = false;
      const double dr = 0.1, dtheta = 30, dz = 0.1;
      double min_dist_to_target = std::numeric_limits<double>::max();
      Eigen::Vector3d goal;

      for (double r = dr; r <= 1.0 + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
            double new_x = end_pt_(0) + r * cos(theta / 57.3);
            double new_y = end_pt_(1) + r * sin(theta / 57.3);
            double new_z = end_pt_(2);//新点z轴不变

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist >= 0.2) {
              double target_dist = (new_pt - end_pt_).norm();
              if (target_dist < min_dist_to_target) {
                min_dist_to_target = target_dist;
                goal = new_pt;
                new_goal = true;
              }
            }
        }
      }

      if (new_goal) {
        cout << "Goal adjusted, replan." << endl;
        end_pt_ = goal;
        end_vel_.setZero();
        have_target_ = true;
        updateEffectiveGoal();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        cout << "No valid goal found, keep retrying." << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
        std_msgs::Empty emt;
        replan_pub_.publish(emt);
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

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

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

// KinoReplanFSM::
}  // namespace fast_planner
