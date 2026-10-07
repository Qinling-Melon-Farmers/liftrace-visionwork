#include <plan_manage/kino_replan_fsm.h>
#include <plan_env/reference_height.h>

namespace fast_planner {
void KinoReplanFSM::initRecovery(ros::NodeHandle& nh) {
  auto& c=recovery_config_;
  nh.param("navigation_recovery/enabled",c.enabled,false);
  nh.param("navigation_recovery/radius",c.radius,.8);
  nh.param("navigation_recovery/max_speed",c.max_speed,.3);
  nh.param("navigation_recovery/max_acceleration",c.max_acceleration,.5);
  nh.param("navigation_recovery/max_seconds",c.max_seconds,8.);
  nh.param("navigation_recovery/max_input_age",c.max_input_age,.25);
  nh.param("navigation_recovery/max_map_age",c.max_map_age,.5);
  nh.param("navigation_recovery/hard_min_z",c.hard_min_z,.05);
  nh.param("navigation_recovery/hard_max_z",c.hard_max_z,3.5);
  nh.param("navigation_recovery/max_height_excess",c.max_height_excess,.25);
  nh.param("navigation_recovery/tracking_error",c.tracking_error,.08);
  // The Oct-8 integration deliberately uses existing localisation, no EV/reset
  // producer or predictor. Timestamp/frame checks remain mandatory.
  c.require_reference_attestation=false;
  if (!c.valid()) throw std::invalid_argument("invalid recovery configuration");
  recovery_transaction_=recovery::Transaction(c);
  if (!c.enabled) return;
  std::string context_topic,command_topic,state_topic;
  nh.param<std::string>("navigation_recovery/context_topic",context_topic,"/planning/recovery_context");
  nh.param<std::string>("navigation_recovery/command_topic",command_topic,"/planning/recovery_command");
  nh.param<std::string>("navigation_recovery/state_topic",state_topic,"/mavros/state");
  recovery_context_sub_=nh.subscribe<navigation_recovery_msgs::NavigationRecoveryContext>(context_topic,1,
    [this](const navigation_recovery_msgs::NavigationRecoveryContext::ConstPtr& m){
      if(m->header.stamp>=recovery_context_.header.stamp) recovery_context_=*m;
    });
  recovery_state_sub_=nh.subscribe<mavros_msgs::State>(state_topic,1,
    [this](const mavros_msgs::State::ConstPtr& m){recovery_fc_state_=*m;recovery_fc_stamp_=ros::Time::now();});
  recovery_command_pub_=nh.advertise<navigation_recovery_msgs::NavigationRecoveryCommand>(command_topic,1);
}

recovery::Context KinoReplanFSM::recoveryContext() const {
  recovery::Context x;
  const auto now=ros::Time::now();const auto height=referenceHeight();
  const auto map=planner_manager_->edt_environment_->sdf_map_;
  x.now=now.toSec();x.action_deadline=recovery_context_.deadline.toSec();
  // The manager's deadline is unchanged; its global timeout/cancellation still
  // revokes controller ownership. Never invent an additional action allowance.
  x.mission_deadline=x.action_deadline;
  x.soft_max_z=height.enabled && height.valid ? height.max_z : recovery_config_.hard_max_z;
  x.goal=goal_status_tracker_.effectiveGoal().header.stamp.toNSec();
  x.position=odom_pos_;x.velocity=odom_vel_;x.goal_position=end_pt_;
  const auto fresh=[&](const ros::Time& stamp,double age){
    return !stamp.isZero() && now>=stamp && (now-stamp).toSec()<=age;};
  x.offboard=recovery_fc_state_.connected && recovery_fc_state_.armed &&
      recovery_fc_state_.mode=="OFFBOARD" && fresh(recovery_fc_stamp_,2.5);
  x.navigation_owner=have_target_ && controller_mode_==1 && fresh(controller_stamp_,.5) &&
      recovery_context_.active && recovery_context_.header.stamp==goal_status_tracker_.effectiveGoal().header.stamp &&
      recovery_context_.header.frame_id==odom_frame_ && (!height.enabled || height.frame==odom_frame_);
  auto& e=x.evidence;e.stamp=odom_stamp_.toSec();e.map_stamp=map->recoveryStamp().toSec();
  e.frame=odom_frame_;e.map_frame=map->recoveryFrame();e.map_revision=map->recoveryRevision();
  // Use the existing local static-map coverage model. Query returns unknown
  // outside its freshly rebuilt interior, excluding source-inflation margins.
  e.coverage_verified=e.map_revision>0;
  return x;
}

recovery::MapQuery KinoReplanFSM::recoveryMap() {
  auto map=planner_manager_->edt_environment_->sdf_map_;
  recovery::MapQuery q;
  q.sources=[map](const Eigen::Vector3d& a,const Eigen::Vector3d& b){return map->recoverySources(a,b);};
  q.physicalClear=[map](const Eigen::Vector3d& a,const Eigen::Vector3d& b){
    const int m=map->recoverySources(a,b);return m>=0 && !(m&(recovery::PHYSICAL|recovery::BOUNDARY));};
  q.observedFree=[map](const Eigen::Vector3d& a,const Eigen::Vector3d& b){return map->recoverySources(a,b)>=0;};
  const double clearance=planner_manager_->pp_.clearance_;
  q.canResume=[map,clearance](const Eigen::Vector3d& p,double slack){
    return map->recoverySources(p,p)==0 && recovery::normalPlannerStartClear(
        map->getInflateOccupancy(p),map->getDistance(p),clearance,slack,map->getResolution());
  };
  return q;
}

bool KinoReplanFSM::tryRecovery() {
  const auto context=recoveryContext();
  const auto why=recovery::admission(recovery_config_,context);
  if (!why.empty()) { ROS_WARN_STREAM_THROTTLE(1.,"recovery admission: "<<why);return false; }
  auto query=recoveryMap();
  std::vector<Eigen::Vector3d> candidates;
  const double step=std::max(.1,planner_manager_->edt_environment_->sdf_map_->getResolution());
  const int n=std::ceil(recovery_config_.radius/step);
  for(int x=-n;x<=n;++x) for(int y=-n;y<=n;++y) for(int z=-n;z<=n;++z) {
    const Eigen::Vector3d d(x*step,y*step,z*step),p=context.position+d;
    if(d.norm()<step*.9 || d.norm()>recovery_config_.radius ||
       p.z()+recovery_config_.tracking_error>context.soft_max_z || query.sources(p,p)!=0 ||
       !query.canResume(p,recovery_config_.finish_position)) continue;
    candidates.push_back(p);
  }
  std::stable_sort(candidates.begin(),candidates.end(),[&](const Eigen::Vector3d& a,const Eigen::Vector3d& b){
    return (a-context.position).squaredNorm()<(b-context.position).squaredNorm();});
  if(candidates.size()>128) candidates.resize(128);
  if(!recovery_transaction_.begin(context,query,candidates)) {
    ROS_WARN_STREAM("recovery rejected: "<<recovery_transaction_.reason());return false;
  }
  recovery_active_=true;goal_has_trajectory_=true;server_hold_replan_pending_=false;
  replan_pub_.publish(std_msgs::Empty());
  // Retain an exact cubic for telemetry and the ordinary execution identity.
  // Recovery commands themselves are emitted after a fresh full-path check.
  const auto& curve=recovery_transaction_.curve();
  auto& info=planner_manager_->local_data_;
  Eigen::MatrixXd controls(4,3);for(int i=0;i<4;++i) controls.row(i)=curve.controls[i].transpose();
  Eigen::VectorXd knots(8);knots<<0,0,0,0,curve.duration,curve.duration,curve.duration,curve.duration;
  info.position_traj_=NonUniformBspline(controls,3,curve.duration);info.position_traj_.setKnot(knots);
  info.velocity_traj_=info.position_traj_.getDerivative();info.acceleration_traj_=info.velocity_traj_.getDerivative();
  info.start_time_=ros::Time::now();info.start_pos_=odom_pos_;info.duration_=curve.duration;info.execution_time_=0.;++info.traj_id_;
  ROS_WARN_STREAM("recovery started from "<<context.position.transpose()<<" to "<<curve.controls[3].transpose()
      <<" original_goal="<<end_pt_.transpose()<<" deadline="<<context.action_deadline);
  return true;
}

void KinoReplanFSM::publishRecovery(bool active,const Eigen::Vector3d& p,const Eigen::Vector3d& v) {
  navigation_recovery_msgs::NavigationRecoveryCommand m;
  m.header.stamp=ros::Time::now();m.header.frame_id=recovery_transaction_.original().evidence.frame;
  m.goal_stamp.fromNSec(recovery_transaction_.original().goal);
  m.started_at.fromSec(recovery_transaction_.original().now);
  m.deadline.fromSec(std::min(recovery_transaction_.original().action_deadline,
      recovery_transaction_.original().now+recovery_config_.max_seconds));
  m.recovery_id=m.started_at.toNSec();m.soft_max_z=recovery_transaction_.original().soft_max_z;m.active=active;
  m.pose.position.x=p.x();m.pose.position.y=p.y();m.pose.position.z=p.z();
  m.pose.orientation.w=odom_orient_.w();m.pose.orientation.x=odom_orient_.x();
  m.pose.orientation.y=odom_orient_.y();m.pose.orientation.z=odom_orient_.z();
  m.velocity.x=v.x();m.velocity.y=v.y();m.velocity.z=v.z();recovery_command_pub_.publish(m);
}

void KinoReplanFSM::recoveryTick() {
  Eigen::Vector3d p,v;
  const auto state=recovery_transaction_.step(recoveryContext(),recoveryMap(),p,v);
  if(state==recovery::Transaction::ACTIVE) { publishRecovery(true,p,v);return; }
  publishRecovery(false,odom_pos_,Eigen::Vector3d::Zero());recovery_active_=false;
  replan_pub_.publish(std_msgs::Empty());
  if(state==recovery::Transaction::RESUME) {
    next_planning_attempt_=ros::Time(0);
    ROS_WARN_STREAM("recovery complete; resuming original goal "<<end_pt_.transpose());
    changeFSMExecState(GEN_NEW_TRAJ,"RECOVERY_RESUME");
  } else {
    publishGoalStatus(goal_status_tracker_.record(plan_manage::PlannerStatus::FAILED_ATTEMPT,
        "recovery_"+recovery_transaction_.reason(),ros::Time::now(),currentGoalDistance()));
    have_target_=false;changeFSMExecState(WAIT_TARGET,"RECOVERY_FAILED");
  }
}
}
