"""Run production compensated-drop methods and async worker with offline transport.

No ROS nodes, master, simulation or hardware are required. Controller message/
time/transport boundaries use doubles. Geometry, settlement, FSM descent branches
and RPC admission/polling execute this checkout's code without a point-cloud map.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

# catkin/nose 按包名导入，unittest 按目录导入；两种入口都显式定位同目录测试工具。
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_async_servo import method

PACKAGE = Path(__file__).resolve().parents[1]

PROGRAM = r'''
#include "patrol_control/drop_slot_geometry.h"
#include "patrol_control/drop_action.h"
#include "patrol_control/landing_handoff_stability.h"
#include "patrol_control/drop_pre_descent_stability.h"
#include "patrol_control/drop_descent_profile.h"
#include "patrol_control/async_servo.h"
#include "patrol_control/servo_action_result.h"
#include "patrol_control/near_wall_align.h"
#include <array>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <limits>
#include <memory>
#include <map>
#include <string>
#include <vector>
#define ROS_INFO(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
#define ROS_DEBUG(...) ((void)0)
namespace ros {
static double clock_sec=100;
struct Duration { double seconds; double toSec() const { return seconds; } };
struct Time {
 double seconds; Time(double v=0):seconds(v){}
 double toSec() const { return seconds; }
 bool isZero() const { return seconds==0; }
 static Time now(){return Time(clock_sec);}
};
Duration operator-(Time a,Time b){return {a.seconds-b.seconds};}
bool operator==(Time a,Time b){return a.seconds==b.seconds;}
bool operator!=(Time a,Time b){return !(a==b);}
bool operator<=(Time a,Time b){return a.seconds<=b.seconds;}
bool operator>(Time a,Time b){return a.seconds>b.seconds;}
bool operator<(Time a,Time b){return a.seconds<b.seconds;}
struct Header { Time stamp; std::string frame_id="camera_init"; unsigned int seq=0; };
}
namespace geometry_msgs {
struct Point { double x=0,y=0,z=0; };
struct Quaternion { double x=0,y=0,z=0,w=1; };
struct Pose { Point position; Quaternion orientation; };
struct PoseStamped { ros::Header header; Pose pose; };
}
namespace nav_msgs {
struct Odometry {
 using ConstPtr=std::shared_ptr<const Odometry>;
 ros::Header header; std::string child_frame_id="base_link";
 struct { geometry_msgs::Pose pose; } pose;
 struct { struct { geometry_msgs::Point linear,angular; } twist; } twist;
};
}
namespace uav_vision {
struct AlignmentTargetContext {
 using ConstPtr=std::shared_ptr<const AlignmentTargetContext>;
 enum {SCHEMA_VERSION=1,ALIGN=2}; int schema_version=1,command=2;
 ros::Header header; bool active=true,has_target=true;
 std::string mission_id="mission",semantic_target_class="panzer",align_mode="drop_circle";
 unsigned int decision_seq=9,attempt=1,payload_slot=1,semantic_target_id=7;
 ros::Time semantic_target_first_seen=ros::Time(90),deadline=ros::Time(200);
};
struct DropOffset {ros::Header header; double dx_px=0,dy_px=0,radius_px=0;};
struct DropAlignmentFeedback {
 ros::Header header; ros::Time observation_stamp,odom_stamp,semantic_target_first_seen;
 std::string mission_id,semantic_target_class,align_mode,reason;
 unsigned int decision_seq=0,attempt=0,payload_slot=0,semantic_target_id=0;
 bool valid=false,aligned=false,frozen=false;
 geometry_msgs::Point target_fc; double horizontal_error_m=-1,horizontal_speed_mps=-1;
};
}
namespace tf {
double getYaw(const geometry_msgs::Quaternion&q){
 return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
}
namespace patrol_control {
struct Servo {struct {int req=0;}request;struct {bool res=false;}response;};
struct ServoAction {
 struct Request {
  std::uint64_t request_id=0,permission_revision=0;
  unsigned int payload_slot=0,decision_seq=0,attempt=0,target_id=0;
  std::string permission_epoch,mission_id,target_class,align_mode;
  ros::Time target_first_seen;
 } request;
 struct Response {
  enum {NOT_STARTED=1,RAW_CALL_STARTED=2,COMPLETED=3};
  std::uint64_t request_id=0; unsigned int payload_slot=0;
  bool res=true,terminal=true; int execution_state=COMPLETED;
 } response;
};
}
struct Transport {
 std::atomic<int> calls{0}; std::mutex mutex; std::condition_variable cv;
 bool released=false; patrol_control::ServoAction::Request request;
};
namespace ros {
struct ServiceClient {
 std::shared_ptr<Transport> impl;
 bool call(patrol_control::ServoAction&srv){
  impl->request=srv.request; ++impl->calls;
  std::unique_lock<std::mutex> lock(impl->mutex);
  impl->cv.wait(lock,[&]{return impl->released;});
  srv.response.request_id=srv.request.request_id;
  srv.response.payload_slot=srv.request.payload_slot; return true;
 }
 bool call(patrol_control::Servo&srv){++impl->calls;srv.response.res=true;return true;}
};
}
namespace patrol_control {
class LLController {
public:
 bool external_mission_mode_=true,compensated_alignment_enabled_=true;
 bool compensated_goal_valid_=false,compensated_goal_frozen_=false;
 bool drop_compensate_before_descent_=true,drop_descent_started_=false;
 bool drop_release_height_only_=true;
 bool drop_release_timeout_enabled_=true,drop_timeout_clock_invalid_=false;
 double drop_release_timeout_sec_=20.;
 ros::Time drop_alignment_started_,drop_descent_started_at_,drop_timeout_last_check_;
 int drop_horizontal_compensation_mode_=2;
 double drop_pre_descent_stable_xy_tolerance_m_=.04;
 DropDescentConfig drop_descent_config_;
 DropDescentProfile drop_descent_profile_;
 bool dropDescentControlActive() const;
 void loadDropSequenceConfig(){SERIAL_PARAMETERS}
 bool compensated_target_outside_boundary_=false,guard_proceeded=false;
 NearWallAlignFence near_wall_align_fence_;
 struct { uav_vision::DropAlignmentFeedback last; void publish(const uav_vision::DropAlignmentFeedback& m){last=m;} } compensated_alignment_pub_;
 geometry_msgs::PoseStamped patrol_cmd;
 geometry_msgs::PoseStamped mavros_point_cmd;
 double last_drop_vz=0.0; bool last_drop_velocity_lane=false;
 int composeDropDescentCommand(){
  mavros_point_cmd=compensated_fc_goal_;mavros_point_cmd.pose.position.z=drop_release_setpoint_height_;
  DROP_DESCENT_COMMANDS
  last_drop_vz=drop_vz_command;last_drop_velocity_lane=vertical_speed_drop;
  if(independent_drop&&drop_descent_config_.mode==4)return vertical_speed_drop?4:2;
  return independent_drop?(zero_velocity_drop?3:2):1;
 }
 geometry_msgs::PoseStamped uav_pose,compensated_fc_goal_,waypoint_mark_point,waypoint_temp;
 geometry_msgs::PoseStamped cross_mark_point,land_mark_point;
 geometry_msgs::Point compensated_target_center_;
 uav_vision::AlignmentTargetContext compensated_context_,servo_alignment_context_;
 ros::Time compensated_observation_stamp_,motion_odom_receipt_,latest_drop_ready_time_;
 nav_msgs::Odometry motion_odom_; bool motion_odom_valid_=false;
 std::string motion_twist_frame_="child",current_align_mode_="drop_circle";
 bool have_servo_alignment_context_=true,external_landing_active_=false;
 bool have_cross_mark=false,have_land_mark=false,have_waypoint_mark=false;
 unsigned int servo_alignment_decision_seq_=9,servo_alignment_target_id_=7;
 std::string servo_alignment_target_class_="panzer";
 double mission_release_permission_timeout_=.25,drop_offset_timeout_=.5;
 double external_alignment_capture_height_=1,drop_release_setpoint_height_=.40;
 double drop_release_min_height_=.35;
 struct { std::map<std::string,double> values;
  double param(const std::string& key,double fallback){auto it=values.find(key);return it==values.end()?fallback:it->second;}
 } nh_;
 void loadReleaseHeights(){RELEASE_HEIGHT_PARAMETERS}
 double drop_red_cross_xy_tolerance_m_=.04;
 void loadRedCrossXYTolerance(){RED_CROSS_PARAMETERS}
 double drop_height_threshold=.45,align_height=1;
 bool uav_drop_ready_=true,control_ready=true,permission_ready=true,should_drop=false;
 double drop_position_threshold_=.15; bool require_vision_release_permission_=true;
 DropReleaseGate currentDropReleaseGate() const {return {permission_ready,permission_ready};}
 static double distance3d(double x,double y,double z,double tx,double ty,double tz){
  return std::hypot(std::hypot(x-tx,y-ty),z-tz);
 }
 double dis_to_next_position=0;
 enum {Run_point,Aligning}; int Drone_mode=Aligning;
 std::array<std::array<double,2>,3> drop_slot_offsets_{{{{-.12,0}},{{0,-.12}},{{0,.12}}}};
 std::array<std::array<double,2>,3> dynamic_drop_slot_offsets_=drop_slot_offsets_;
 std::array<double,4> adjust_target_position{{0,0,0,0}};
 LandingHandoffStabilityConfig drop_settle_config_,landing_settle_config_;
 LandingHandoffStabilityWindow drop_capture_window_,drop_release_window_;
 DropPreDescentStabilityConfig drop_pre_descent_config_;
 double drop_pre_descent_red_cross_xy_tolerance_m_=.04;
 DropPreDescentStabilityWindow drop_pre_descent_window_;
 LandingHandoffStabilityWindow landing_capture_window_,landing_handoff_window_;
 LandingHandoffStabilityWindow landing_high_alignment_window_; // reset-only H dependency
 bool drop_metric_scale_enabled_=false; double drop_circle_radius_m_=.5,drop_cross_radius_m_=.175;
 double landing_pad_radius_m_=.5,pixel_to_meter_ratio_=.001,max_alignment_move_distance_=1;
 std::array<double,4> pixel_to_body_matrix_{{1,0,0,1}};
 double current_pixel_error=1000;
 bool dropPixelScales(ros::Time,double*,double*)const{return false;}
 bool externalLandingControlReady(ros::Time)const{return control_ready;}
 bool hasFreshMissionReleasePermission()const{return permission_ready;}
 ros::ServiceClient servo_client,servo_action_client_;
 AsyncServo async_servo_;
    std::array<AsyncServo,3> no_ack_servo_workers_;
    bool drop_recover_without_ack_=false, drop_recovery_started_=false;
    double drop_recovery_delay_sec_=0.0; // legacy fixtures test the zero-delay policy
    uav_vision::AlignmentTargetContext servo_invocation_context_;
    int invocation_events=0;
    void publishDropInvocation(){++invocation_events;}
    AsyncServo& servoWorkerForSlot(int);
    void beginDropRecoveryWithoutAck(); std::uint64_t servo_action_id_=1;
 int servo_action_slot_=0; bool servo_action_attempted_=false,servo_action_pending_=false;
 DropActionResult servo_action_result_=DropActionResult::kPending;
 double servo_call_timeout_sec_=10; struct {bool data=false;}servo_complete;
 bool drop_complete=false; int detect_point_counter=0; std::vector<bool>drop_completed{false,false,false};
 struct {std::string permission_epoch="epoch";std::uint64_t permission_revision=1;}release_authorization_;
 bool drop_condition_met=false,descent_completed=false; double final_target_height=0,last_target_height_=0;
 ros::Time last_check_time_;
 bool first_call=true,down_flag=true,align_ok=true,ignore_servo_complete=false;
 bool have_drop_offset_=false,mission_release_permission_active_=false;
 int times_detect=0,count_aligning=0;std::string latest_drop_ready_reason_;
 LLController(){
  DEFAULTS
  loadRedCrossXYTolerance();
  loadDropSequenceConfig();
  drop_capture_window_=LandingHandoffStabilityWindow(drop_settle_config_);
  drop_release_window_=LandingHandoffStabilityWindow(drop_settle_config_);
  servo_alignment_context_.header.stamp=ros::Time::now();
  latest_drop_ready_time_=ros::Time::now();
 }
 void motionOdomCallback(const nav_msgs::Odometry::ConstPtr&);
 const char* motionFeedbackStatus() const;
 bool motionTimePending() const;
 bool freshMotion(Eigen::Vector3d*,Eigen::Vector3d*,const char** =nullptr)const;
 bool setCompensatedDropTarget(geometry_msgs::PoseStamped*);
 double compensatedDropXYTolerance(bool release=true) const;
 bool compensatedDropTimeoutReady();
 bool compensatedDropSettled(bool,double* =nullptr,double* =nullptr,bool=true);
 double compensatedDropCommandHeight();
 bool freezeCompensatedDropTarget();
 void publishCompensatedAlignment();
 void advanceExternalGuard(){ EXTERNAL_GUARD guard_proceeded=true; }
 void applyDropSlotOffset(int,bool);
 void projectDropOffsetToTarget(const uav_vision::DropOffset&);
 void servoAlignmentContextCallback(const uav_vision::AlignmentTargetContext::ConstPtr&);
 DropActionResult executeDropAction(int); void pollDropAction();void cancelDropAction();
 double external_standard_recovery_setpoint_height_=1.4,external_cross_recovery_setpoint_height_=1.4,external_recovery_height_=.95;
 bool beginCircleRecovery(){ CIRCLE_RECOVERY_START return false; }
 bool beginCrossRecovery(){ CROSS_RECOVERY_START return false; }
 void resetDropState();void clearUavVisionAlignmentState();void resetDetectionState();
 void advanceCircle(){int servo_id=1; CIRCLE_DESCENT }
 void advanceCross(){int servo_id=1; CROSS_DESCENT }
 bool circleReleaseReady(){ CIRCLE_RELEASE return should_drop; }
 bool crossReleaseReady(){ CROSS_RELEASE return should_drop; }
};
METHODS
}
using namespace patrol_control;
void close(double a,double b){assert(std::abs(a-b)<1e-9);}
geometry_msgs::Quaternion attitude(double yaw,double pitch=0,double roll=0){
 const Eigen::Quaterniond q=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
  Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
 return {q.x(),q.y(),q.z(),q.w()};
}
void odom(LLController&c,double t,double x,double y,double z,double vx=0,double vy=0,
 geometry_msgs::Quaternion q=geometry_msgs::Quaternion(),double wz=0){
 ros::clock_sec=t;
 auto m=std::make_shared<nav_msgs::Odometry>();m->header.stamp=ros::Time(t);
 m->pose.pose.position={x,y,z};m->pose.pose.orientation=q;
 m->twist.twist.linear={vx,vy,0};m->twist.twist.angular={0,0,wz};
 c.uav_pose.header.stamp=ros::Time(t);c.uav_pose.pose=m->pose.pose;
 c.motionOdomCallback(m);
}
geometry_msgs::PoseStamped center(double t){
 geometry_msgs::PoseStamped p;p.header.stamp=ros::Time(t);p.pose.position={2,3,1};return p;
}
bool serial_test=false;
bool height_only_test=false;
void capture(LLController&c){
 if(!serial_test)c.drop_compensate_before_descent_=false; // Isolate the pre-existing release tests in explicit compatibility mode.
 if(!serial_test&&!height_only_test)c.drop_release_height_only_=false;
 odom(c,100,2,3,1);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
 const auto g=c.compensated_fc_goal_.pose.position;
 // Reliable visual capture is already ready; full 12cm correction may
 // remain while descent starts. It never bypasses final release settlement.
 c.latest_drop_ready_time_=ros::Time::now();
 assert(c.freezeCompensatedDropTarget());
 assert(c.compensated_goal_frozen_);
}
void finish(LLController&c,const std::shared_ptr<Transport>&t){
 {std::lock_guard<std::mutex>lock(t->mutex);t->released=true;}t->cv.notify_all();
 const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
 while(c.servo_action_pending_&&std::chrono::steady_clock::now()<end){
  c.pollDropAction();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 assert(!c.servo_action_pending_&&c.drop_complete);
 assert(c.servo_complete.data == !c.drop_recover_without_ack_);
}
void settle_before_descent(LLController&c,double start,double height=1){
 const auto g=c.compensated_fc_goal_.pose.position;
 odom(c,start,g.x,g.y,height);
 close(c.compensatedDropCommandHeight(),.40);
 assert(c.drop_descent_started_);
}
int main(int argc,char**argv){
 assert(argc==2);const std::string name=argv[1];ros::clock_sec=100;
  height_only_test=name.find("height_only_")==0;
  serial_test=name.find("serial_")==0||height_only_test;
  LLController c;if(!serial_test)c.drop_compensate_before_descent_=false;
  if(!serial_test&&!height_only_test)c.drop_release_height_only_=false;
 auto t=std::make_shared<Transport>();c.servo_client.impl=c.servo_action_client_.impl=t;
 if(name=="serial_sequence"){
  for(const std::string cls:{"pillbox","red_cross"}){
   ros::clock_sec=100;LLController d;auto transport=std::make_shared<Transport>();
   d.servo_client.impl=d.servo_action_client_.impl=transport;
   const bool cross=cls=="red_cross";d.current_align_mode_=cross?"drop_cross":"drop_circle";
   d.servo_alignment_context_.align_mode=d.current_align_mode_;
   d.servo_alignment_context_.semantic_target_class=cls;d.servo_alignment_target_class_=cls;
   odom(d,100,2,3,1,.5);auto p=center(100);assert(d.setCompensatedDropTarget(&p));
   d.waypoint_mark_point=d.cross_mark_point=p;d.have_waypoint_mark=!cross;d.have_cross_mark=cross;
   auto tick=[&]{if(cross)d.advanceCross();else d.advanceCircle();};tick();
   assert(d.compensated_goal_frozen_&&!d.drop_descent_started_);close(d.align_height,1);
   const auto g=d.compensated_fc_goal_.pose.position,original=g;
   // Freeze no longer needs continuing visual ready or a fresh source image.
   d.uav_drop_ready_=false;d.have_waypoint_mark=d.have_cross_mark=false;
   d.drop_settle_config_.stable_duration_sec=100;d.drop_settle_config_.min_samples=1000;
    odom(d,101,g.x+.061,g.y,1,.8);d.motion_odom_.twist.twist.linear.z=.5;tick();
    assert(!d.drop_descent_started_);close(d.align_height,1);
    for(int i=0;i<=10;++i){
     odom(d,101.1+i*.1,g.x+.02,g.y,1,.8);tick();
     assert(d.drop_descent_started_);close(d.align_height,.40);
    }
   assert(d.drop_descent_started_);close(d.align_height,.40);
   // Descent phase latches, but release readiness never latches.
    odom(d,102.2,g.x+.07,g.y,.40);tick();close(d.align_height,.40);
    assert(d.compensatedDropSettled(true)); // Height-only final gate permits XY rebound.
    d.permission_ready=false;
    assert(d.executeDropAction(1)==DropActionResult::kRejected&&!d.servo_action_attempted_);
    odom(d,102.3,g.x,g.y,.46);tick();close(d.align_height,.40);assert(!d.compensatedDropSettled(true));
    odom(d,102.4,g.x,g.y,.40);d.permission_ready=false;tick();
   assert(!(cross?d.crossReleaseReady():d.circleReleaseReady()));
   assert(d.executeDropAction(1)==DropActionResult::kRejected&&transport->calls==0);
   d.permission_ready=true;assert(d.executeDropAction(1)==DropActionResult::kPending&&d.servo_action_attempted_);
   finish(d,transport);assert(transport->calls==1);
   close(d.compensated_fc_goal_.pose.position.x,original.x);close(d.compensated_fc_goal_.pose.position.y,original.y);
  }
 }else if(name=="serial_admission"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  // Initial/current height already lies in the release window and permission is valid.
  odom(c,101,g.x,g.y,.40);assert(c.compensatedDropSettled(true));
  assert(!c.drop_descent_started_);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
   settle_before_descent(c,101,.40);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
  finish(c,t);assert(t->calls==1);
 }else if(name=="serial_invalid"){
  for(int reason=0;reason<6;++reason){
   ros::clock_sec=100;LLController d;capture(d);const auto g=d.compensated_fc_goal_.pose.position;
   odom(d,101,g.x,g.y,1);
   if(reason==0)d.servo_alignment_context_.mission_id="other";
   if(reason==1)d.servo_alignment_context_.deadline=ros::Time(100.5);
   if(reason==2)d.control_ready=false;
   if(reason==3){ros::clock_sec=101.3;d.uav_pose.header.stamp=ros::Time::now();}
   if(reason==4){auto cancelled=std::make_shared<uav_vision::AlignmentTargetContext>(d.servo_alignment_context_);
    cancelled->active=false;d.servoAlignmentContextCallback(cancelled);}
   if(reason==5)d.servo_alignment_context_.semantic_target_class="red_cross";
   close(d.compensatedDropCommandHeight(),1);assert(!d.drop_descent_started_);
   assert(!d.compensatedDropSettled(true));
  }
 }else if(name=="serial_cancel_after_descent"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   settle_before_descent(c,101);odom(c,102.1,g.x,g.y,.6);
  auto cancelled=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
  cancelled->active=false;c.servoAlignmentContextCallback(cancelled);
  c.advanceExternalGuard();assert(!c.guard_proceeded);close(c.align_height,.6);
  assert(c.drop_descent_started_&&!c.compensatedDropSettled(true));
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
 }else if(name=="serial_reset"){
  for(bool detection:{false,true}){
   ros::clock_sec=100;LLController d;capture(d);auto g=d.compensated_fc_goal_.pose.position;
    settle_before_descent(d,100.1);
   if(detection)d.resetDetectionState();else d.resetDropState();
   assert(!d.drop_descent_started_&&!d.compensated_goal_frozen_);
   d.servo_alignment_context_.mission_id="new_action";d.servo_alignment_context_.decision_seq=10;
   d.servo_alignment_decision_seq_=10;d.servo_alignment_context_.semantic_target_first_seen=ros::Time(95);
   d.servo_alignment_context_.header.stamp=ros::Time(102);
   odom(d,102,2,3,1);auto next=center(102);assert(d.setCompensatedDropTarget(&next));
   d.uav_drop_ready_=true;d.latest_drop_ready_time_=ros::Time(102);
   assert(d.freezeCompensatedDropTarget());close(d.compensatedDropCommandHeight(),1);
   assert(!d.drop_descent_started_);
  }
 }else if(name=="serial_config"){
  assert(c.drop_compensate_before_descent_);
  c.nh_.values["drop_system/compensate_before_descent"]=0;c.loadDropSequenceConfig();
  assert(!c.drop_compensate_before_descent_);capture(c);
  close(c.compensatedDropCommandHeight(),.40);assert(c.drop_descent_started_);
 }else if(name=="serial_outlet"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,1,0,0,attitude(std::acos(-1)/2));
  close(c.compensatedDropCommandHeight(),1);assert(!c.drop_descent_started_);
   settle_before_descent(c,101.1);
   assert(c.drop_descent_started_);ros::clock_sec=102.5;
   close(c.compensatedDropCommandHeight(),.40);assert(!c.compensatedDropSettled(true));
  }else if(name=="serial_speed_reset"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   c.drop_pre_descent_config_.max_horizontal_speed_mps=.001;
   c.drop_pre_descent_config_.stable_duration_sec=100;
   c.drop_pre_descent_config_.min_samples=1000;
   odom(c,101,g.x+.05,g.y,1,.8);
   close(c.compensatedDropCommandHeight(),.40);assert(c.drop_descent_started_);
  }else if(name=="serial_xy_reset"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   odom(c,101,g.x+.061,g.y,1);close(c.compensatedDropCommandHeight(),1);
   odom(c,101.01,g.x+.059,g.y,1,.8);
   close(c.compensatedDropCommandHeight(),.40);assert(c.drop_descent_started_);
  }else if(name=="serial_duplicate_gap"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   odom(c,101,g.x+.07,g.y,1);close(c.compensatedDropCommandHeight(),1);
   for(int i=0;i<50;++i)close(c.compensatedDropCommandHeight(),1);
   ros::clock_sec=101.3;assert(!c.compensatedDropSettled(true,nullptr,nullptr,false));
   // A fresh first sample after a gap can enter descent immediately.
   odom(c,102,g.x+.05,g.y,1,.8);close(c.compensatedDropCommandHeight(),.40);
  }else if(name=="serial_horizontal_only"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   for(int i=0;i<=10;++i){
    // No displacement, Z error or vertical-speed gate is imposed.
    odom(c,101+i*.1,g.x+(i<5?-.05:.05),g.y,1+i*.01,.8);
    c.motion_odom_.twist.twist.linear.z=.5;
    close(c.compensatedDropCommandHeight(),.40);
   }
  }else if(name=="serial_exact_limits"){
   for(const std::string cls:{"panzer","red_cross","tank"}){
    ros::clock_sec=100;LLController d;d.servo_alignment_context_.semantic_target_class=cls;
    d.servo_alignment_target_class_=cls;capture(d);
    // Put both FC and outlet at zero to test exact equality without subtraction rounding.
    d.drop_slot_offsets_[0]={{0,0}};d.compensated_fc_goal_.pose.position={0,0,1};
    d.compensated_target_center_={0,0,1};const double limit=cls=="panzer"?.06:.04;
    odom(d,101,limit,0,1,.8);
    assert(!d.compensatedDropSettled(true,nullptr,nullptr,false));
    close(d.compensatedDropCommandHeight(),1);
    odom(d,101.01,std::nextafter(limit,0.),0,1,.8);
    assert(d.compensatedDropSettled(true,nullptr,nullptr,false));
    close(d.compensatedDropCommandHeight(),.40);
   }
  }else if(name=="serial_outlet_speed"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   odom(c,101,g.x,g.y,1,0,0,attitude(0),10.0);
   // FC is stationary; outlet speed is 1.2m/s. It is diagnostic only.
   close(c.compensatedDropCommandHeight(),.40);
  }else if(name=="serial_invalid_window"){
   for(int reason=0;reason<3;++reason){
    ros::clock_sec=100;LLController d;capture(d);const auto g=d.compensated_fc_goal_.pose.position;
    odom(d,101,g.x,g.y,1);
    if(reason==0)d.control_ready=false;
    if(reason==1){ros::clock_sec=101.3;d.uav_pose.header.stamp=ros::Time::now();}
    if(reason==2)d.servo_alignment_context_.mission_id="other";
    close(d.compensatedDropCommandHeight(),1);
    d.control_ready=true;d.servo_alignment_context_.mission_id=d.compensated_context_.mission_id;
    odom(d,102.2,g.x,g.y,1,.8);close(d.compensatedDropCommandHeight(),.40);
   }
  }else if(name=="serial_partial_reset"){
   for(bool detection:{false,true}){
    ros::clock_sec=100;LLController d;capture(d);auto g=d.compensated_fc_goal_.pose.position;
    odom(d,101,g.x+.07,g.y,1);close(d.compensatedDropCommandHeight(),1);
    if(detection)d.resetDetectionState();else d.resetDropState();
    d.servo_alignment_context_.header.stamp=ros::Time(102);
    odom(d,102,2,3,1);auto p=center(102);assert(d.setCompensatedDropTarget(&p));
    d.uav_drop_ready_=true;d.latest_drop_ready_time_=ros::Time(102);
    assert(d.freezeCompensatedDropTarget());settle_before_descent(d,102.1);
   }
  }else if(name=="serial_split_thresholds"){
   for(const std::string cls:{"tent","pillbox","bridge","panzer","red_cross"})
    for(bool height_only:{false,true}){
     ros::clock_sec=100;LLController d;auto transport=std::make_shared<Transport>();
     d.servo_client.impl=d.servo_action_client_.impl=transport;
     const bool cross=cls=="red_cross";
     d.current_align_mode_=cross?"drop_cross":"drop_circle";
     d.servo_alignment_context_.align_mode=d.current_align_mode_;
     d.servo_alignment_context_.semantic_target_class=cls;d.servo_alignment_target_class_=cls;
     d.drop_settle_config_.xy_tolerance_m=.10;d.drop_red_cross_xy_tolerance_m_=.06;
     d.drop_release_height_only_=height_only;
     d.drop_alignment_started_=d.drop_timeout_last_check_=ros::Time(100);
     capture(d);const auto g=d.compensated_fc_goal_.pose.position;
     close(d.compensatedDropXYTolerance(),cross?.06:.10);
     close(d.compensatedDropXYTolerance(false),cross?.04:.06);
     const double middle=cross?.05:.08;
     // The final tolerance cannot enable high-level compensation, even after 20s.
     for(double t:{101.,121.}){
      odom(d,t,g.x+middle,g.y,1,.8);close(d.compensatedDropCommandHeight(),1);
      assert(!d.drop_descent_started_);
      assert(d.executeDropAction(1)==DropActionResult::kPending&&transport->calls==0);
     }
     // Disable fallback: ordinary entry and release must not depend on it.
     d.drop_release_timeout_enabled_=false;
     odom(d,121.01,g.x+(cross?.039:.059),g.y,1,.8);
     close(d.compensatedDropCommandHeight(),.40);assert(d.drop_descent_started_);
     odom(d,121.02,g.x+middle,g.y,.40,.8);
     assert(d.compensatedDropSettled(true));
     assert(cross?d.crossReleaseReady():d.circleReleaseReady());
     assert(d.executeDropAction(1)==DropActionResult::kPending&&d.servo_action_attempted_);
     for(int i=0;i<50;++i)d.executeDropAction(1);
     finish(d,transport);assert(transport->calls==1);
     close(d.compensated_fc_goal_.pose.position.x,g.x);close(d.compensated_fc_goal_.pose.position.y,g.y);
    }
 }else if(name=="serial_stability_modes"){
   // Mode 1 restores the original 4cm / 0.1m/s / 1s gate for every descent lane.
   for(int lane=1;lane<=3;++lane){
    ros::clock_sec=100;LLController d;
    d.nh_.values["drop_system/descent/mode"]=lane;
    d.loadDropSequenceConfig();
    d.nh_.values["drop_system/pre_descent/mode"]=1;
    d.loadRedCrossXYTolerance();capture(d);const auto g=d.compensated_fc_goal_.pose.position;
    odom(d,101,g.x,g.y,1,.101);close(d.compensatedDropCommandHeight(),1);
    odom(d,101.1,g.x+.041,g.y,1,0);close(d.compensatedDropCommandHeight(),1);
    for(int i=0;i<10;++i){odom(d,101.2+i*.1,g.x,g.y,1,.1);
     close(d.compensatedDropCommandHeight(),1);assert(!d.drop_descent_started_);}
    // A repeated sample cannot finish the window.
    close(d.compensatedDropCommandHeight(),1);
    odom(d,102.2,g.x,g.y,1,.1);close(d.compensatedDropCommandHeight(),.4);
    close(d.drop_descent_started_at_.toSec(),102.2);
    odom(d,105,g.x,g.y,.8,.8);close(d.compensatedDropCommandHeight(),.4);
    close(d.drop_descent_started_at_.toSec(),102.2);
   }
 }else if(name=="serial_pre_parameters"){
   close(c.drop_pre_descent_config_.xy_tolerance_m,.06);
   close(c.drop_pre_descent_red_cross_xy_tolerance_m_,.04);
   c.nh_.values["drop_system/pre_descent/xy_tolerance_m"]=.055;
   c.nh_.values["drop_system/pre_descent/red_cross_xy_tolerance_m"]=.035;
   for(const std::string key:{"max_horizontal_speed_mps","stable_duration_sec","max_sample_gap_sec","min_samples"})
    c.nh_.values["drop_system/pre_descent/"+key]=0;
   c.loadRedCrossXYTolerance();capture(c);
   close(c.compensatedDropXYTolerance(false),.055);
   c.compensated_context_.semantic_target_class="red_cross";
   close(c.compensatedDropXYTolerance(false),.035);
   c.compensated_context_.semantic_target_class="tank";
   close(c.compensatedDropXYTolerance(false),.035);
   for(const std::string key:{"xy_tolerance_m","red_cross_xy_tolerance_m","max_odom_age_sec"})
    for(double value:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}){
     LLController d;d.nh_.values["drop_system/pre_descent/"+key]=value;
     bool threw=false;try{d.loadRedCrossXYTolerance();}catch(const std::invalid_argument&){threw=true;}
     assert(threw);
    }
  }else if(name=="serial_pre_freshness"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   c.drop_pre_descent_config_.max_odom_age_sec=.05;
   odom(c,101,g.x,g.y,1);ros::clock_sec=101.06;
   assert(!c.compensatedDropSettled(true,nullptr,nullptr,false));
   odom(c,101.07,g.x,g.y,1,.8);close(c.compensatedDropCommandHeight(),.40);
  }else if(name=="serial_release_exact_limits"){
   for(const std::string cls:{"panzer","red_cross"}){
    ros::clock_sec=100;LLController d;
    d.servo_alignment_context_.semantic_target_class=cls;d.servo_alignment_target_class_=cls;
    d.drop_settle_config_.xy_tolerance_m=.10;d.drop_red_cross_xy_tolerance_m_=.06;
    d.drop_release_height_only_=false;capture(d);settle_before_descent(d,101);
    d.drop_slot_offsets_[0]={{0,0}};d.compensated_fc_goal_.pose.position={0,0,1};
    d.compensated_target_center_={0,0,1};const double limit=cls=="panzer"?.10:.06;
    odom(d,102.1,limit,0,.40,.8);assert(!d.compensatedDropSettled(true));
    odom(d,102.11,std::nextafter(limit,0.),0,.40,.8);assert(d.compensatedDropSettled(true));
   }
  }else if(name=="serial_descent_modes"){
   for(int mode=1;mode<=3;++mode){
    ros::clock_sec=100;LLController d;
    d.nh_.values["drop_system/descent/mode"]=mode;d.loadDropSequenceConfig();
    capture(d);settle_before_descent(d,101);
    assert(d.drop_descent_config_.mode==mode);
    assert(d.dropDescentControlActive()==(mode!=1));
    if(mode!=1){
     close(d.drop_descent_profile_.height(),1);
     const auto g=d.compensated_fc_goal_.pose.position;
     odom(d,101.1,g.x+.25,g.y-.20,1);
     assert(d.composeDropDescentCommand()==mode);close(d.mavros_point_cmd.pose.position.z,.98);
     close(d.mavros_point_cmd.pose.position.x,mode==2?g.x:g.x+.25);
     close(d.mavros_point_cmd.pose.position.y,mode==2?g.y:g.y-.20);
     // Horizontal motion cannot change the independent Z trajectory.
     odom(d,101.2,g.x+.35,g.y-.30,1);assert(d.composeDropDescentCommand()==mode);
     close(d.mavros_point_cmd.pose.position.z,.96);
     ros::clock_sec=101.5;d.uav_pose.header.stamp=ros::Time::now();
     assert(d.composeDropDescentCommand()==mode);close(d.mavros_point_cmd.pose.position.z,.96);
     d.servo_alignment_context_.active=false;assert(!d.dropDescentControlActive());
     assert(d.composeDropDescentCommand()==1);
     d.resetDropState();assert(!d.drop_descent_profile_.active());
    }
   }
  }else if(name=="serial_descent_profile"){
   DropDescentConfig config;config.mode=2;DropDescentProfile profile(config);
   assert(profile.start(1.4,.4,100));
   close(profile.update(100.1),1.38);close(profile.update(100.1),1.38);
   close(profile.update(101),1.38); // long missed interval holds, never catches up
   profile.pause(101.1);close(profile.update(101.2),1.36);
   for(int i=1;i<=60;++i)profile.update(101.2+i*.1);
   close(profile.height(),.4);profile.reset();assert(!profile.active());
   config.mode=5;assert(!config.valid());config.mode=3;config.speed_mps=0;assert(!config.valid());
  }else if(name=="no_ack_circle_cross_recovery"){
   serial_test=true;height_only_test=true;
   for(int mode=1;mode<=4;++mode){
    for(bool cross:{false,true}){
     ros::clock_sec=100;LLController d;d.drop_recover_without_ack_=true;
     d.nh_.values["drop_system/descent/mode"]=mode;d.loadDropSequenceConfig();
     d.servo_alignment_context_.align_mode=cross?"drop_cross":"drop_circle";
     d.servo_alignment_context_.semantic_target_class=cross?"red_cross":"panzer";
     d.current_align_mode_=d.servo_alignment_context_.align_mode;
     d.servo_alignment_target_class_=d.servo_alignment_context_.semantic_target_class;
     auto transport=std::make_shared<Transport>();
     d.servo_client.impl=d.servo_action_client_.impl=transport;
     capture(d);settle_before_descent(d,101);const auto g=d.compensated_fc_goal_.pose.position;
     odom(d,102,g.x,g.y,.40);
     assert(d.executeDropAction(1)==DropActionResult::kPending);
     const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
     while(transport->calls==0&&std::chrono::steady_clock::now()<until)std::this_thread::yield();
     assert(transport->calls==1);d.pollDropAction();
     assert(d.drop_complete&&d.drop_recovery_started_&&!d.servo_complete.data);
     assert(d.invocation_events==1&&!d.dropDescentControlActive());
     assert(cross?d.beginCrossRecovery():d.beginCircleRecovery());
     close(d.align_height,1.4);if(cross)assert(!d.down_flag);
     d.pollDropAction();assert(d.invocation_events==1&&transport->calls==1);
     finish(d,transport);
    }
   }
  }else if(name=="mode_two_three_mps"){
   DropDescentConfig config;config.mode=2;config.speed_mps=3.;assert(config.valid());
   DropDescentProfile profile(config);assert(profile.start(1.4,.4,100));
   close(profile.update(100.1),1.1);close(profile.update(100.2),.8);
   close(profile.update(100.3),.5);close(profile.update(100.4),.4);
   close(profile.update(100.5),.4); // terminal floor survives a fast ramp
   config.speed_mps=3.000001;assert(!config.valid());config.speed_mps=1.01;
   for(int mode:{1,3,4}){config.mode=mode;assert(!config.valid());}
  }else if(name=="velocity_descent_profile"){
   DropDescentConfig config;config.mode=4;DropDescentProfile profile(config);
   assert(config.valid()&&profile.start(1.4,.4,100));
   double z=1.4,vz=0,peak=0;bool braking=false;
   for(int i=1;i<=150&&!profile.holding();++i){
    const double previous=-vz;vz=profile.updateVelocity(100+i*.05,z,vz);
    const double speed=-vz;assert(speed>=0&&speed<=.6+1e-9);
    assert(speed<=previous+.6*.05+1e-9);
    peak=std::max(peak,speed);if(previous>.1&&speed<previous-1e-6)braking=true;
    z+=vz*.05;
   }
   assert(profile.holding()&&braking);close(peak,.6);close(profile.height(),.4);
   close(profile.updateVelocity(110,.6,-.1),0);close(profile.height(),.4);
   profile.reset();assert(!profile.active()&&!profile.holding());
  }else if(name=="velocity_descent_pause"){
   DropDescentConfig config;config.mode=4;DropDescentProfile profile(config);
   assert(profile.start(1.4,.4,100));close(profile.updateVelocity(100.1,1.4,0),-.06);
   close(profile.updateVelocity(100.5,1.4,-.06),0);
   close(profile.updateVelocity(100.6,1.4,0),-.06);
   profile.pause(100.7);close(profile.updateVelocity(100.8,1.4,0),-.06);
   close(profile.updateVelocity(100.9,NAN,0),0);
   close(profile.updateVelocity(101,1.4,0),-.06);
   close(profile.updateVelocity(101.1,.39,-.3),0);assert(profile.holding());
   config.velocity_speed_mps=NAN;assert(!config.valid());config.velocity_speed_mps=.6;
   config.braking_mps2=0;assert(!config.valid());
  }else if(name=="velocity_descent_command"){
   ros::clock_sec=100;LLController d;d.nh_.values["drop_system/descent/mode"]=4;
   d.loadDropSequenceConfig();capture(d);d.drop_release_height_only_=true;settle_before_descent(d,101);
   const auto g=d.compensated_fc_goal_.pose.position;
   odom(d,101.1,g.x+.2,g.y-.2,1);assert(d.composeDropDescentCommand()==4);
   close(d.last_drop_vz,-.06);close(d.mavros_point_cmd.pose.position.x,g.x);
   close(d.mavros_point_cmd.pose.position.y,g.y);close(d.mavros_point_cmd.pose.position.z,1);
   odom(d,101.2,g.x+.2,g.y,.44,-.1);assert(d.composeDropDescentCommand()==4);

   assert(d.compensatedDropSettled(true)); // release before terminal hold; no speed dwell
   odom(d,101.3,g.x,g.y,.414,-.1);assert(d.composeDropDescentCommand()==2);
   close(d.mavros_point_cmd.pose.position.z,.4);assert(!d.last_drop_velocity_lane);
   odom(d,101.4,g.x,g.y,.46);assert(d.composeDropDescentCommand()==2); // terminal latch
   d.servo_alignment_context_.active=false;assert(d.composeDropDescentCommand()==1);
   d.resetDropState();assert(!d.drop_descent_profile_.active());
  }else if(name=="height_only_rpc"){
   for(const std::string cls:{"pillbox","red_cross"}){
    ros::clock_sec=100;LLController d;auto transport=std::make_shared<Transport>();
    d.servo_client.impl=d.servo_action_client_.impl=transport;
    const bool cross=cls=="red_cross";d.current_align_mode_=cross?"drop_cross":"drop_circle";
    d.servo_alignment_context_.align_mode=d.current_align_mode_;
    d.servo_alignment_context_.semantic_target_class=cls;d.servo_alignment_target_class_=cls;
    capture(d);settle_before_descent(d,101);const auto g=d.compensated_fc_goal_.pose.position;
    odom(d,102.1,g.x+.30,g.y+.20,.40,.4);
    assert(d.compensatedDropSettled(true));
    assert(cross?d.crossReleaseReady():d.circleReleaseReady());
    assert(d.executeDropAction(1)==DropActionResult::kPending&&d.servo_action_attempted_);
    finish(d,transport);assert(transport->calls==1);
   }
  }else if(name=="height_only_guards"){
   for(int reason=0;reason<7;++reason){
    ros::clock_sec=100;LLController d;auto transport=std::make_shared<Transport>();
    d.servo_client.impl=d.servo_action_client_.impl=transport;
    capture(d);settle_before_descent(d,101);const auto g=d.compensated_fc_goal_.pose.position;
    odom(d,102.1,g.x+.30,g.y,.40);
    if(reason==0)odom(d,102.2,g.x+.30,g.y,.349);
    if(reason==1)odom(d,102.2,g.x+.30,g.y,.451);
    if(reason==2)d.control_ready=false;
    if(reason==3){ros::clock_sec=102.4;d.uav_pose.header.stamp=ros::Time::now();}
    if(reason==4)d.servo_alignment_context_.active=false;
    if(reason==5)d.servo_alignment_context_.mission_id="other";
    if(reason==6)d.permission_ready=false;
    const auto result=d.executeDropAction(1);
    assert(result==DropActionResult::kPending||result==DropActionResult::kRejected);
    assert(!d.servo_action_attempted_&&transport->calls==0);
   }
  }else if(name=="height_only_pre_descent"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   for(int i=0;i<=12;++i){odom(c,101+i*.1,g.x+.061,g.y,1);close(c.compensatedDropCommandHeight(),1);}
   odom(c,102.3,g.x+.059,g.y,1,.8);close(c.compensatedDropCommandHeight(),.40);
  }else if(name=="height_only_band"){
   capture(c);settle_before_descent(c,101);const auto g=c.compensated_fc_goal_.pose.position;
   int i=0;for(double z:{.349,.35,.40,.45,.451}){
    odom(c,102.1+(i++)*.01,g.x+.30,g.y,z);
    assert(c.compensatedDropSettled(true)==(z>=.35&&z<=.45));
   }
  }else if(name=="height_only_config"){
   assert(c.drop_release_height_only_);
   c.nh_.values["drop_system/release_height_only"]=0;c.loadDropSequenceConfig();
   assert(!c.drop_release_height_only_);capture(c);settle_before_descent(c,101);
   const auto g=c.compensated_fc_goal_.pose.position;
   odom(c,102.1,g.x+.07,g.y,.40);assert(!c.compensatedDropSettled(true));
  }else if(name=="class_thresholds"){
  for(const std::string cls:{"tent","pillbox","bridge","panzer","red_cross","","circle","tank","PILLBOX"}){
   ros::clock_sec=100;LLController d;d.servo_alignment_context_.semantic_target_class=cls;
   d.servo_alignment_target_class_=cls;capture(d);const auto g=d.compensated_fc_goal_.pose.position;
   const bool standard=cls=="tent"||cls=="pillbox"||cls=="bridge"||cls=="panzer";
   odom(d,101,g.x+.05,g.y,.40,.5);assert(d.compensatedDropSettled(true)==standard);
   odom(d,101.01,g.x+.039,g.y,.40);assert(d.compensatedDropSettled(true));
   odom(d,101.02,g.x+.061,g.y,.40);assert(!d.compensatedDropSettled(true));
  }
 }else if(name=="class_mode"){
  for(const std::string cls:{"panzer","red_cross"})for(const std::string mode:{"drop_circle","drop_cross"}){
   ros::clock_sec=100;LLController d;d.servo_alignment_context_.semantic_target_class=cls;
   d.servo_alignment_target_class_=cls;d.servo_alignment_context_.align_mode=mode;d.current_align_mode_=mode;
   capture(d);const auto g=d.compensated_fc_goal_.pose.position;odom(d,101,g.x+.05,g.y,.40);
   assert(d.compensatedDropSettled(true)==(cls=="panzer"));
  }
 }else if(name=="class_binding"){
  c.servo_alignment_context_.semantic_target_class="red_cross";c.servo_alignment_target_class_="red_cross";
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;odom(c,101,g.x+.05,g.y,.40);
  assert(!c.compensatedDropSettled(true));
  c.servo_alignment_context_.semantic_target_class="panzer";
  assert(!c.compensatedDropSettled(true)); // Changed live class cannot loosen frozen transaction.
 }else if(name=="class_parameters"){
  c.nh_.values["drop_system/settle/red_cross_xy_tolerance_m"]=.03;c.loadRedCrossXYTolerance();
  c.servo_alignment_context_.semantic_target_class="red_cross";c.servo_alignment_target_class_="red_cross";
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x+.035,g.y,.40);assert(!c.compensatedDropSettled(true));
  odom(c,101.01,g.x+.025,g.y,.40);assert(c.compensatedDropSettled(true));
  c.compensated_context_.semantic_target_class="tank";
  c.drop_red_cross_xy_tolerance_m_=.08;close(c.compensatedDropXYTolerance(),.04);
  c.drop_red_cross_xy_tolerance_m_=.03;close(c.compensatedDropXYTolerance(),.03);
 }else if(name=="red_parameters_invalid"){
  c.loadRedCrossXYTolerance();close(c.drop_red_cross_xy_tolerance_m_,.04);
  for(double value:{0.,-.01,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
   c.nh_.values["drop_system/settle/red_cross_xy_tolerance_m"]=value;
   bool threw=false;try{c.loadRedCrossXYTolerance();}catch(const std::invalid_argument&){threw=true;}assert(threw);
  }
 }else if(name=="class_boundary"){
  for(const std::string cls:{"panzer","red_cross"}){
   ros::clock_sec=100;LLController d;d.servo_alignment_context_.semantic_target_class=cls;
   d.servo_alignment_target_class_=cls;capture(d);const auto g=d.compensated_fc_goal_.pose.position;
   odom(d,101,g.x+(cls=="panzer"?.06:.04),g.y,.40);double error=0;d.compensatedDropSettled(true,&error);
   if(cls=="panzer")d.drop_settle_config_.xy_tolerance_m=error;else d.drop_red_cross_xy_tolerance_m_=error;
   assert(!d.compensatedDropSettled(true)); // Exact equality is always refused.
  }
 }else if(name=="class_rpc"){
  for(const std::string cls:{"pillbox","red_cross"}){
   ros::clock_sec=100;LLController d;auto transport=std::make_shared<Transport>();
   d.servo_client.impl=d.servo_action_client_.impl=transport;
   d.servo_alignment_context_.semantic_target_class=cls;d.servo_alignment_target_class_=cls;
   capture(d);const auto g=d.compensated_fc_goal_.pose.position;odom(d,101,g.x+.05,g.y,.40,.5);
   assert(d.executeDropAction(1)==DropActionResult::kPending);
   assert(d.servo_action_attempted_==(cls=="pillbox"));
   if(cls=="pillbox")finish(d,transport);else assert(transport->calls==0);
  }
 }else if(name=="arms"){
  const double pi=std::acos(-1),tilt=7.39*pi/180;
  for(const std::string mode:{"drop_circle","drop_cross"})for(int slot=1;slot<=3;++slot)
   for(double yaw:{0.,pi/2,-pi/2})for(double pitch:{0.,tilt,-tilt}){
    LLController d;d.current_align_mode_=mode;d.servo_alignment_context_.align_mode=mode;
    d.servo_alignment_context_.payload_slot=slot;
    const double roll=pitch;auto q=attitude(yaw,pitch,roll);odom(d,100,0,0,1,0,0,q);
    const double f=d.drop_slot_offsets_[slot-1][0],l=d.drop_slot_offsets_[slot-1][1];
    // Independent Rz*Ry*Rx expectation, not the production helper again.
    const double ax=std::cos(yaw)*std::cos(pitch)*f+
      (std::cos(yaw)*std::sin(pitch)*std::sin(roll)-std::sin(yaw)*std::cos(roll))*l;
    const double ay=std::sin(yaw)*std::cos(pitch)*f+
      (std::sin(yaw)*std::sin(pitch)*std::sin(roll)+std::cos(yaw)*std::cos(roll))*l;
    for(int n=0;n<5;++n){auto p=center(100);assert(d.setCompensatedDropTarget(&p));
     close(p.pose.position.x,2);close(p.pose.position.y,3);
     close(d.compensated_fc_goal_.pose.position.x,2-ax);close(d.compensated_fc_goal_.pose.position.y,3-ay);
     d.applyDropSlotOffset(slot,mode=="drop_cross");
     close(d.adjust_target_position[0],2-ax);close(d.adjust_target_position[1],3-ay);
     close(d.compensated_target_center_.x,2);close(d.compensated_target_center_.y,3);}
   }
 }else if(name=="boundary"){
  for(const std::string mode:{"drop_circle","drop_cross"})for(int slot=1;slot<=3;++slot)
   for(double yaw:{0.,std::acos(-1)/2}){
    LLController d;d.servo_client.impl=d.servo_action_client_.impl=t;
    d.current_align_mode_=mode;d.servo_alignment_context_.align_mode=mode;
    d.servo_alignment_context_.payload_slot=slot;
    auto& fence=d.near_wall_align_fence_;fence.enabled=true;fence.bounds={{0,4,0,4}};
    odom(d,100,2,2,1,0,0,attitude(yaw));
    auto p=center(100);p.pose.position={2,2,1};p.pose.orientation=attitude(yaw);
    assert(d.setCompensatedDropTarget(&p));
    // Move the observed center 4cm inside the legal FC edge, while the
    // physical outlet requires a further 12cm correction toward that edge.
    const double dx=d.compensated_fc_goal_.pose.position.x-2;
    const double dy=d.compensated_fc_goal_.pose.position.y-2;
    const double margin=fence.margin(yaw);
    if(std::abs(dx)>.1)p.pose.position.x=dx>0?4-margin-.04:margin+.04;
    else p.pose.position.y=dy>0?4-margin-.04:margin+.04;
    assert(fence.contains(p.pose.position.x,p.pose.position.y,yaw));
    assert(!d.setCompensatedDropTarget(&p));
    assert(d.compensated_target_outside_boundary_&&!d.compensated_goal_valid_&&!d.compensated_goal_frozen_);
    const auto rejected=d.compensated_fc_goal_.pose.position;
    const auto clamped=fence.clamp(rejected.x,rejected.y,yaw,yaw);
    close(std::hypot(rejected.x-clamped.first,rejected.y-clamped.second),.08);
    // No clipped endpoint is accepted as correctly compensated; no descent.
    d.advanceExternalGuard();assert(!d.guard_proceeded);
    close(d.patrol_cmd.pose.position.x,2);close(d.patrol_cmd.pose.position.y,2);close(d.align_height,1);
    d.uav_drop_ready_=true;assert(!d.freezeCompensatedDropTarget());
    assert(d.executeDropAction(slot)==DropActionResult::kPending&&t->calls==0);
    d.publishCompensatedAlignment();const auto feedback=d.compensated_alignment_pub_.last;
    assert(feedback.reason=="compensated_target_outside_boundary"&&!feedback.valid&&!feedback.aligned&&!feedback.frozen);
    assert(feedback.mission_id==d.servo_alignment_context_.mission_id&&feedback.payload_slot==slot&&feedback.decision_seq==9);
    close(feedback.target_fc.x,rejected.x);close(feedback.target_fc.y,rejected.y);
    // An unrelated new center cannot revive the failed attempt. A complete
    // existing action reset is required before accepting the next target.
    p.pose.position={2,2,1};assert(!d.setCompensatedDropTarget(&p));
    d.resetDetectionState();assert(!d.compensated_target_outside_boundary_);
    assert(d.setCompensatedDropTarget(&p));d.uav_drop_ready_=true;
    assert(d.freezeCompensatedDropTarget());
    p.pose.position={10,10,1};assert(!d.setCompensatedDropTarget(&p));
    assert(d.compensated_goal_frozen_&&!d.compensated_target_outside_boundary_);
   }
 }else if(name=="boundary_yaw"){
  auto& fence=c.near_wall_align_fence_;fence.enabled=true;fence.bounds={{0,4,0,4}};
  odom(c,100,2,2,1);auto p=center(100);p.pose.position={4-fence.margin(0)-.12-.001,2,1};
  // The current-yaw endpoint is legal, but the commanded 45deg body margin
  // would cause the same final-command clamp. Check both existing margins.
  p.pose.orientation=attitude(std::acos(-1)/4);
  assert(fence.contains(p.pose.position.x+.12,2,0));
  assert(!c.setCompensatedDropTarget(&p)&&c.compensated_target_outside_boundary_);
 }else if(name=="capture"){
  odom(c,100,2,3,1,.12);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
  c.uav_drop_ready_=false;assert(!c.freezeCompensatedDropTarget());
  c.uav_drop_ready_=true;assert(c.freezeCompensatedDropTarget());
  assert(c.compensated_goal_frozen_);
  // Full arm correction and high-altitude speed are not descent prerequisites.
  c.applyDropSlotOffset(1,false);close(c.adjust_target_position[0],2.12);
  close(c.compensated_target_center_.x,2);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
 }else if(name=="circle_descent" || name=="cross_descent"){
  c.drop_compensate_before_descent_=false; // Explicit compatibility mode keeps simultaneous descent.
  const bool cross=name=="cross_descent";c.current_align_mode_=cross?"drop_cross":"drop_circle";
  c.servo_alignment_context_.align_mode=c.current_align_mode_;
  odom(c,100,2,3,1,.12);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
  c.waypoint_mark_point=c.cross_mark_point=p;
  c.have_waypoint_mark=!cross;c.have_cross_mark=cross;
  c.uav_drop_ready_=false;
  if(cross)c.advanceCross();else c.advanceCircle();
  assert(!c.compensated_goal_frozen_&&c.count_aligning==0);close(c.align_height,1);
  c.uav_drop_ready_=true;
  if(cross)c.advanceCross();else c.advanceCircle();
  assert(c.compensated_goal_frozen_&&c.count_aligning==1);
  close(c.align_height,.40);close(c.adjust_target_position[0],2.12);close(c.adjust_target_position[1],3);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
  // Repeated ticks do not accumulate 12cm again, and preserve the target.
  for(int i=0;i<5;++i){if(cross)c.advanceCross();else c.advanceCircle();
   close(c.adjust_target_position[0],2.12);close(c.compensated_target_center_.x,2);}
 }else if(name=="no_map_circle" || name=="no_map_cross"){
  const bool cross=name=="no_map_cross";c.current_align_mode_=cross?"drop_cross":"drop_circle";
  c.servo_alignment_context_.align_mode=c.current_align_mode_;
  odom(c,100,2,3,1,.12);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
  c.waypoint_mark_point=c.cross_mark_point=p;
  c.have_waypoint_mark=!cross;c.have_cross_mark=cross;
  if(cross)c.advanceCross();else c.advanceCircle();
  assert(c.compensated_goal_frozen_&&c.count_aligning==1);
  close(c.align_height,.40);close(c.adjust_target_position[0],2.12);
  const auto g=c.compensated_fc_goal_.pose.position;
  const auto observation=c.compensated_observation_stamp_;
  // No map callback or further image is delivered. Existing mission permission
  // remains valid, while final physical position/speed must still settle.
  c.have_waypoint_mark=c.have_cross_mark=false;c.have_drop_offset_=false;c.uav_drop_ready_=false;
  odom(c,101,2,3,.35);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_);
  odom(c,101.125,g.x,g.y,.35,.051);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
  for(int i=0;i<3;++i){odom(c,101.25+i*.15625,g.x,g.y,.35);
   assert(c.executeDropAction(1)==DropActionResult::kPending);
   assert(c.servo_action_attempted_);
   assert(c.compensated_goal_frozen_&&c.compensated_observation_stamp_==observation);
   close(c.compensated_target_center_.x,2);close(c.compensated_fc_goal_.pose.position.x,g.x);}
  finish(c,t);
  assert(c.executeDropAction(1)==DropActionResult::kSuccess&&t->calls==1);
 }else if(name=="outer_circle"||name=="outer_cross"||
          name=="outer_reject_circle"||name=="outer_reject_cross"){
  const bool cross=name=="outer_cross"||name=="outer_reject_cross";
  const bool reject=name=="outer_reject_circle"||name=="outer_reject_cross";
  c.current_align_mode_=cross?"drop_cross":"drop_circle";
  c.servo_alignment_context_.align_mode=c.current_align_mode_;
  capture(c);c.waypoint_temp=center(100);
  const auto g=c.compensated_fc_goal_.pose.position;
  auto outer=[&]{return cross?c.crossReleaseReady():c.circleReleaseReady();};
  odom(c,101,g.x,g.y,.45);
  const auto p=c.uav_pose.pose.position;
  assert(LLController::distance3d(p.x,p.y,p.z,2,3,.35)>.15);
  assert(outer());  // Production outer gate no longer measures to the raw center.
  if(!reject){
   assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
   finish(c,t);assert(t->calls==1);
   assert(!outer()); // Original !drop_complete still prevents a second request.
  }else{
   odom(c,101.01,g.x+.061,g.y,.40);assert(!outer());
   odom(c,101.02,g.x,g.y,.451);assert(!outer());
   odom(c,101.03,g.x,g.y,.349);assert(!outer());
   odom(c,101.04,g.x,g.y,.45);assert(outer());
   ros::clock_sec=101.25;assert(!outer()); // Source and receipt are now stale.
   odom(c,101.26,g.x,g.y,.45);c.permission_ready=false;assert(!outer());
   assert(c.executeDropAction(1)==DropActionResult::kRejected);
   c.permission_ready=true;c.control_ready=false;assert(!outer());
   c.control_ready=true;c.drop_complete=true;assert(!outer());c.drop_complete=false;
   assert(outer()); // Final admission checks permission again after the outer gate.
   c.permission_ready=false;assert(c.executeDropAction(1)==DropActionResult::kRejected);
   assert(!c.servo_action_attempted_&&t->calls==0);
  }
 }else if(name=="outer_legacy"){
  c.drop_release_setpoint_height_=.35;c.drop_release_min_height_=.35;
  for(bool cross:{false,true}){
   auto outer=[&]{return cross?c.crossReleaseReady():c.circleReleaseReady();};
   c.waypoint_temp=center(100);
   for(bool external:{false,true}){
    c.external_mission_mode_=external;c.compensated_alignment_enabled_=!external;
    c.compensated_goal_valid_=false; // A legacy gate never demands a frozen target.
    odom(c,101,2.12,3,.45);assert(!outer());
    odom(c,101.01,2.12,3,.35);assert(outer());
    c.drop_complete=true;assert(!outer());c.drop_complete=false;
   }
  }
 }else if(name=="vision_frozen"){
  capture(c);auto saved=c.compensated_fc_goal_;auto stamp=c.compensated_observation_stamp_;
  for(int i=0;i<3;++i){uav_vision::DropOffset m;m.header.stamp=ros::Time(101+i*.1);
   m.dx_px=i==0?std::numeric_limits<double>::quiet_NaN():300;m.dy_px=-200;
   ros::clock_sec=m.header.stamp.toSec();c.projectDropOffsetToTarget(m);
   close(c.compensated_fc_goal_.pose.position.x,saved.pose.position.x);
   close(c.compensated_fc_goal_.pose.position.y,saved.pose.position.y);
   assert(c.compensated_observation_stamp_==stamp&&c.compensated_goal_frozen_);}
 }else if(name=="vision_replay"){
  odom(c,100,0,0,1);uav_vision::DropOffset m;m.header.stamp=ros::Time(100);m.dx_px=50;
  c.projectDropOffsetToTarget(m);assert(c.compensated_goal_valid_);
  const double x=c.compensated_fc_goal_.pose.position.x;
  m.dx_px=900;c.projectDropOffsetToTarget(m);close(c.compensated_fc_goal_.pose.position.x,x);
  m.header.stamp=ros::Time(99);c.projectDropOffsetToTarget(m);close(c.compensated_fc_goal_.pose.position.x,x);
  m.header.stamp=ros::Time(101);c.projectDropOffsetToTarget(m);close(c.compensated_fc_goal_.pose.position.x,x);
  }else if(name=="configured_settle"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   CONFIGURED_DROP
   c.drop_release_window_=LandingHandoffStabilityWindow(c.drop_settle_config_);
   close(c.drop_settle_config_.xy_tolerance_m,.10);
   for(int i=0;i<3;++i){odom(c,101+i*.15625,g.x+.101,g.y,.35,.07);
    assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);}
   // A single current sample is enough after the original visual capture.
   odom(c,103,g.x+.095,g.y,.35,.20);
   assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
   finish(c,t);assert(c.executeDropAction(1)==DropActionResult::kSuccess&&t->calls==1);
  }else if(name=="height_config_compat"){
   c.loadReleaseHeights();close(c.drop_release_setpoint_height_,.10);close(c.drop_release_min_height_,.10);
   c.nh_.values["drop_system/release_setpoint_height"]=.40;
   c.loadReleaseHeights();close(c.drop_release_setpoint_height_,.40);close(c.drop_release_min_height_,.40);
   c.nh_.values["drop_system/release_min_height"]=.35;
   c.loadReleaseHeights();close(c.drop_release_setpoint_height_,.40);close(c.drop_release_min_height_,.35);
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   odom(c,101,g.x,g.y,.375);assert(c.compensatedDropSettled(true));
  }else if(name=="height_band"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   CONFIGURED_DROP
   int i=0;
   close(c.drop_release_setpoint_height_,.40);
   for(double z:{.349,.35,.375,.40,.425,.45,.451}){
    odom(c,101+(i++)*.01,g.x,g.y,z);
    assert(c.compensatedDropSettled(true)==(z>=.35&&z<=.45));
   }
   // Omitted release_min_height follows the legacy setpoint at parameter load.
   c.drop_release_min_height_=c.drop_release_setpoint_height_;
   odom(c,102,g.x,g.y,.375);assert(!c.compensatedDropSettled(true));
   odom(c,102.01,g.x,g.y,.40);assert(c.compensatedDropSettled(true));
  }else if(name=="current_limits"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;
   CONFIGURED_DROP
   odom(c,101,g.x,g.y,.40,.08);
   assert(c.compensatedDropSettled(true));
   odom(c,101.01,g.x+.099,g.y,.40);
   assert(c.compensatedDropSettled(true));
   odom(c,101.02,g.x+.101,g.y,.40);
   assert(!c.compensatedDropSettled(true));
   // Equality must not pass; use the measured error as the exact threshold.
   double measured=0;
   c.compensatedDropSettled(true,&measured);
   c.drop_settle_config_.xy_tolerance_m=measured;
   assert(!c.compensatedDropSettled(true));
   c.drop_settle_config_.xy_tolerance_m=.10;
   // Earlier height-only and later position-only satisfaction cannot latch.
   odom(c,101.021,g.x+.11,g.y,.40);assert(!c.compensatedDropSettled(true));
   odom(c,101.022,g.x,g.y,.46);assert(!c.compensatedDropSettled(true));
   odom(c,101.03,g.x,g.y,.40,.081);
   assert(c.compensatedDropSettled(true));
   odom(c,101.04,g.x,g.y,.40);
   c.motion_odom_.twist.twist.linear.z=.10;assert(c.compensatedDropSettled(true));
   c.motion_odom_.twist.twist.linear.z=.20;assert(c.compensatedDropSettled(true));
   c.motion_odom_.twist.twist.linear.z=-.20;assert(c.compensatedDropSettled(true));
   odom(c,101.05,g.x,g.y,.40,0,0,geometry_msgs::Quaternion(),1);
   assert(c.compensatedDropSettled(true));
   // Speed alone is diagnostic; an attitude change that displaces the
   // actual outlet still fails the independent outlet position check.
   odom(c,101.06,g.x,g.y,.40,0,0,attitude(std::acos(-1)/2));
   assert(!c.compensatedDropSettled(true));
   odom(c,101.07,g.x,g.y,.40);
   c.motion_odom_.twist.twist.linear.z=0;c.control_ready=false;
   assert(!c.compensatedDropSettled(true));
  }else if(name=="slots_once"){
   for(int slot=1;slot<=3;++slot)for(double z:{.35,.40,.45}){
    ros::clock_sec=100;
    LLController d;auto transport=std::make_shared<Transport>();
    d.servo_client.impl=d.servo_action_client_.impl=transport;
    d.servo_alignment_context_.payload_slot=slot;capture(d);
    const auto goal=d.compensated_fc_goal_.pose.position;
    odom(d,101,goal.x,goal.y,z);
    assert(d.executeDropAction(slot)==DropActionResult::kPending&&d.servo_action_attempted_);
    assert(d.executeDropAction(slot==3?1:slot+1)==DropActionResult::kRejected);
    finish(d,transport);assert(transport->request.payload_slot==slot);
    for(int j=0;j<3;++j)assert(d.executeDropAction(slot)==DropActionResult::kSuccess);
    assert(transport->calls==1);
   }
  }else if(name=="cancel_permission"){
   capture(c);const auto g=c.compensated_fc_goal_.pose.position;odom(c,101,g.x,g.y,.40);
   c.permission_ready=false;assert(c.executeDropAction(1)==DropActionResult::kRejected);
   c.permission_ready=true;
   auto ctx=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
   ctx->active=false;c.servoAlignmentContextCallback(ctx);
   assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
  }else if(name=="xy"||name=="speed"||name=="angular"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  for(int i=0;i<5;++i){odom(c,101+i*.15625,g.x+(name=="xy"?.061:0),g.y,.35,
    name=="speed"?.051:0,0,geometry_msgs::Quaternion(),name=="angular"?.5:0);
   assert(c.executeDropAction(1)==DropActionResult::kPending);
   assert(c.servo_action_attempted_==(name!="xy"));}
  if(name!="xy")finish(c,t);
 }else if(name=="samples"||name=="async"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  close(c.drop_settle_config_.xy_tolerance_m,.06);close(c.drop_settle_config_.max_horizontal_speed_mps,.05);
  close(c.drop_settle_config_.stable_duration_sec,.3);assert(c.drop_settle_config_.min_samples==3);
  for(int i=0;i<3;++i){odom(c,101+i*.15625,g.x,g.y,.35);
   assert(c.executeDropAction(1)==DropActionResult::kPending);assert(c.servo_action_attempted_);}
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(!t->calls&&std::chrono::steady_clock::now()<end)std::this_thread::yield();assert(t->calls==1);
  if(name=="async"){
   auto ctx=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
   ctx->active=false;c.servoAlignmentContextCallback(ctx);c.permission_ready=false;c.control_ready=false;
   for(int i=0;i<5;++i){uav_vision::DropOffset m;m.header.stamp=ros::Time(102+i*.1);m.dx_px=300;
    ros::clock_sec=102+i*.1;c.projectDropOffsetToTarget(m);
    assert(c.executeDropAction(1)==DropActionResult::kPending);c.pollDropAction();
    assert(c.servo_action_pending_&&t->calls==1&&!c.drop_complete);}
  }
  finish(c,t);assert(c.executeDropAction(1)==DropActionResult::kSuccess&&t->calls==1);
 }else if(name=="odom_replay"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
  const auto old=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);
  for(int i=1;i<=3;++i){ros::clock_sec=101+i*.05;c.uav_pose.header.stamp=ros::Time::now();
   c.motionOdomCallback(old);assert(c.compensatedDropSettled(true));
   close(c.motion_odom_receipt_.toSec(),101);}
  ros::clock_sec=101.25;c.uav_pose.header.stamp=ros::Time::now();assert(!c.compensatedDropSettled(true));
  odom(c,101.3125,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
  odom(c,101.46875,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
  odom(c,101.625,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
 }else if(name=="context"){
  capture(c);auto ctx=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
  ++ctx->attempt;c.servoAlignmentContextCallback(ctx);
  assert(!c.compensatedDropSettled(false)&&!c.compensatedDropSettled(true));
  const auto old_goal=c.compensated_fc_goal_;auto p=center(ros::clock_sec);
  p.pose.position.x=8;assert(!c.setCompensatedDropTarget(&p));
  close(c.compensated_fc_goal_.pose.position.x,old_goal.pose.position.x);
  assert(c.executeDropAction(1)==DropActionResult::kPending);
  assert(!c.servo_action_attempted_&&t->calls==0&&c.compensated_goal_frozen_);
  c.resetDetectionState();
  assert(!c.compensated_goal_valid_&&!c.compensated_goal_frozen_);
  assert(c.compensated_observation_stamp_.isZero());
  // Full action reset permits a new observation, but restarts capture settling.
  ctx->header.stamp=ros::Time::now();c.servoAlignmentContextCallback(ctx);
  odom(c,100.5,2,3,1);ctx->header.stamp=ros::Time::now();c.servoAlignmentContextCallback(ctx);
  p=center(100.5);assert(c.setCompensatedDropTarget(&p));
  assert(c.compensatedDropSettled(false)&&!c.compensatedDropSettled(true));
 }else if(name=="pose_stale"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,.35);Eigen::Vector3d v,w;
  assert(c.freshMotion(&v,&w));
  c.uav_pose.header.stamp=ros::Time(100.75);assert(!c.freshMotion(&v,&w));
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
  c.uav_pose.header.stamp=ros::Time(101.01);assert(!c.freshMotion(&v,&w));
  c.uav_pose.header.stamp=ros::Time(0);assert(!c.freshMotion(&v,&w));
 }else if(name=="no_window"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  c.drop_settle_config_.stable_duration_sec=100;c.drop_settle_config_.min_samples=1000;
  c.drop_settle_config_.height_tolerance_m=.00001;c.drop_settle_config_.max_sample_gap_sec=.00001;
  c.drop_settle_config_.max_horizontal_speed_mps=.00001;c.drop_settle_config_.max_vertical_speed_mps=.00001;
  odom(c,101,g.x,g.y,.45,.2);c.motion_odom_.twist.twist.linear.z=-.2;
  assert(c.compensatedDropSettled(true));
  assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
  finish(c,t);assert(t->calls==1);
 }else if(name=="future_sequence" || name=="future_pose_sequence"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  for(int i=0;i<4;++i){
   const double now=101+i*.125;odom(c,now,g.x,g.y,.35);
   auto m=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);
   m->header.stamp=ros::Time(now+.003);
   c.uav_pose.header.stamp=ros::Time(now+.003);
   if(name=="future_sequence") c.motionOdomCallback(m);
   Eigen::Vector3d v,w;const char* reason=nullptr;
   assert(!c.freshMotion(&v,&w,&reason));
   assert(reason&&std::string(reason)=="motion_time_pending");
   assert(c.motionTimePending()&&!c.compensatedDropSettled(true));
   assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
   ros::clock_sec=now+.004;
   assert(c.freshMotion(&v,&w)&&!c.motionTimePending());
   // Same retained sample becomes usable; no callback or stamp rewriting.
   assert(c.compensatedDropSettled(true));
   assert(c.compensatedDropSettled(true));
  }
 }else if(name=="future_watermark"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,.35);const double accepted=c.motion_odom_.header.stamp.toSec();
  auto m=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);m->header.stamp=ros::Time(500);
  c.motionOdomCallback(m);assert(!c.motion_odom_valid_);
  close(c.motion_odom_.header.stamp.toSec(),accepted);
  Eigen::Vector3d v,w;assert(!c.freshMotion(&v,&w));
  odom(c,101.125,g.x,g.y,.35);assert(c.freshMotion(&v,&w));
  assert(c.compensatedDropSettled(true));
 }else if(name=="future_bad_numeric"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,.35);
  auto m=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);m->header.stamp=ros::Time(101.003);
  m->twist.twist.linear.x=std::numeric_limits<double>::quiet_NaN();
  c.motionOdomCallback(m);assert(!c.motion_odom_valid_&&!c.motionTimePending());
  ros::clock_sec=101.004;Eigen::Vector3d v,w;assert(!c.freshMotion(&v,&w));
  assert(!c.compensatedDropSettled(true));
 }else if(name=="invalid_motion"){
  odom(c,100,0,0,1);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
  auto m=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);m->header.stamp=ros::Time(100.1);
  m->twist.twist.linear.x=std::numeric_limits<double>::quiet_NaN();ros::clock_sec=100.1;
  c.motionOdomCallback(m);assert(!c.motion_odom_valid_&&!c.freezeCompensatedDropTarget());
 }else assert(false);
}
'''


class CompensatedDropGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        signatures = (
            'void LLController::motionOdomCallback(',
            'const char* LLController::motionFeedbackStatus(',
            'bool LLController::motionTimePending(',
            'bool LLController::freshMotion(',
            'bool LLController::setCompensatedDropTarget(',
            'double LLController::compensatedDropXYTolerance(',
            'bool LLController::compensatedDropTimeoutReady(',
            'bool LLController::compensatedDropSettled(',
            'bool LLController::dropDescentControlActive(',
            'double LLController::compensatedDropCommandHeight(',
            'bool LLController::freezeCompensatedDropTarget(',
            'void LLController::publishCompensatedAlignment(',
            'void LLController::applyDropSlotOffset(',
            'void LLController::projectDropOffsetToTarget(',
            'void LLController::servoAlignmentContextCallback(',
            'AsyncServo& LLController::servoWorkerForSlot(',
            'void LLController::beginDropRecoveryWithoutAck(',
            'DropActionResult LLController::executeDropAction(',
            'void LLController::pollDropAction(',
            'void LLController::cancelDropAction(',
            'void LLController::resetDropState(',
            'void LLController::clearUavVisionAlignmentState(',
            'void LLController::resetDetectionState(',
        )
        production = '\n'.join(method(source, signature) for signature in signatures)
        def recovery_start(signature):
            body=method(source, signature)
            start=body.index('if(servo_complete.data || (external_mission_mode_ && drop_recovery_started_)){')
            end=body.index('if(uav_pose.pose.position.z >= recovery_height){', start)
            return body[start:end]+'return true; }'
        # Run the actual default assignments, not a second hand-maintained policy.
        start = source.index('    drop_settle_config_.xy_tolerance_m =')
        end = source.index('    load_stability(', start)
        defaults = source[start:end]
        red_start = source.index('    drop_red_cross_xy_tolerance_m_ = nh_.param(')
        red_end = source.index('    landing_settle_config_.max_horizontal_speed_mps', red_start)
        red_parameters = source[red_start:red_end]
        serial_start = source.index('    drop_compensate_before_descent_ = nh_.param(')
        serial_parameters = source[serial_start:red_start]
        height_start = source.index('    drop_release_setpoint_height_ = nh_.param(')
        height_end = source.index('    drop_enabled =', height_start)
        height_parameters = source[height_start:height_end]
        root_config = PACKAGE.parent / 'uav_mission/config'
        configured = yaml.safe_load((root_config / 'competition/control_base.yaml').read_text())['drop_system']['settle']
        trial = yaml.safe_load((root_config / 'vcl06_horizontal_control.yaml').read_text())['drop_system']['settle']
        if configured != trial:
            raise AssertionError('formal and trial drop settlement must agree')
        configured_assignments = '\n'.join(
            (f'c.drop_red_cross_xy_tolerance_m_ = {value};' if key == 'red_cross_xy_tolerance_m'
             else f'c.drop_settle_config_.{key} = {value};') for key, value in configured.items())
        cls.folder = tempfile.TemporaryDirectory(prefix='compensated-drop-')
        cls.addClassCleanup(cls.folder.cleanup)
        root = Path(cls.folder.name)
        cpp = root / 'test.cpp'
        circle_start = source.index('        const int required_alignment_samples =', source.index('bool LLController::WayPointDetectDone()'))
        circle_end = source.index('            double ttt = distance3d(', circle_start)
        circle = source[circle_start:circle_end] + '}'
        cross_start = source.index('        if(have_cross_mark &&', source.index('bool LLController::CrossDetectionDone()'))
        cross_end = source.index('            double ttt = distance3d(', cross_start)
        cross = source[cross_start:cross_end] + '}'
        def outer_release(signature):
            body = method(source, signature)
            start = body.index('            double ttt = distance3d(')
            end = body.index('            if (external_mission_mode_ &&', start)
            return body[start:end]
        guard_start = source.index('    if (compensated_alignment_enabled_ &&', source.index('void LLController::externalMissionTick()'))
        guard_end = source.index('    std_msgs::Bool detect_enable_msg;', guard_start)
        descent_start = source.index('    const bool pending_independent_drop =')
        descent_end = source.index('    // H descent policy is independent', descent_start)
        cpp.write_text(PROGRAM.replace('METHODS', production).replace('DEFAULTS', defaults)
                       .replace('RELEASE_HEIGHT_PARAMETERS', height_parameters)
                       .replace('RED_CROSS_PARAMETERS', red_parameters)
                       .replace('SERIAL_PARAMETERS', serial_parameters)
                       .replace('CONFIGURED_DROP', configured_assignments)
                       .replace('EXTERNAL_GUARD', source[guard_start:guard_end])
                       .replace('DROP_DESCENT_COMMANDS', source[descent_start:descent_end])
                       .replace('CIRCLE_RECOVERY_START',recovery_start('bool LLController::WayPointDetectDone('))
                       .replace('CROSS_RECOVERY_START',recovery_start('bool LLController::CrossDetectionDone('))
                       .replace('CIRCLE_RELEASE', outer_release('bool LLController::WayPointDetectDone('))
                       .replace('CROSS_RELEASE', outer_release('bool LLController::CrossDetectionDone('))
                       .replace('CIRCLE_DESCENT', circle).replace('CROSS_DESCENT', cross), encoding='utf-8')
        cls.binary = root / 'test'
        subprocess.run([
            'g++', '-std=c++14', '-O1', '-pthread', '-fsanitize=undefined',
            '-fno-sanitize-recover=all', '-I', str(PACKAGE / 'include'),
            '-I', '/usr/include/eigen3', str(cpp), '-o', str(cls.binary),
        ], check=True, timeout=60)

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_serial_circle_and_cross_enter_descent_on_first_valid_xy_sample(self): self.run_case('serial_sequence')
    def test_serial_service_requires_descent_entry_even_if_initial_height_is_in_window(self): self.run_case('serial_admission')
    def test_invalid_context_cancel_expired_motion_and_control_block_descent_entry(self): self.run_case('serial_invalid')
    def test_cancel_after_descent_uses_existing_hold_guard_and_never_releases(self): self.run_case('serial_cancel_after_descent')
    def test_new_action_resets_descent_phase_via_both_existing_reset_paths(self): self.run_case('serial_reset')
    def test_serial_default_and_explicit_compatibility_parameter(self): self.run_case('serial_config')
    def test_serial_entry_checks_actual_outlet_and_later_stale_motion_does_not_climb(self): self.run_case('serial_outlet')
    def test_pre_descent_does_not_wait_for_speed_duration_or_sample_count(self): self.run_case('serial_speed_reset')
    def test_pre_descent_xy_violation_then_first_valid_sample_enters_descent(self): self.run_case('serial_xy_reset')
    def test_pre_descent_duplicate_invalid_samples_do_not_help_but_fresh_frame_enters(self): self.run_case('serial_duplicate_gap')
    def test_pre_descent_does_not_gate_displacement_height_or_vertical_speed(self): self.run_case('serial_horizontal_only')
    def test_pre_descent_standard_cross_and_unknown_strict_thresholds(self): self.run_case('serial_exact_limits')
    def test_pre_descent_rotating_outlet_speed_is_diagnostic_only(self): self.run_case('serial_outlet_speed')
    def test_pre_descent_invalid_control_stale_feedback_and_context_still_block(self): self.run_case('serial_invalid_window')
    def test_pre_descent_both_reset_paths_require_new_freeze(self): self.run_case('serial_partial_reset')
    def test_pre_descent_and_release_thresholds_separate_for_both_release_modes(self): self.run_case('serial_split_thresholds')
    def test_pre_descent_class_parameters_validate_xy_freshness_only(self): self.run_case('serial_pre_parameters')
    def test_pre_descent_keeps_its_independent_freshness_limit(self): self.run_case('serial_pre_freshness')
    def test_release_standard_ten_cm_cross_six_cm_are_strict(self): self.run_case('serial_release_exact_limits')
    def test_three_descent_modes_keep_entry_gate_and_reset_transaction(self): self.run_case('serial_descent_modes')
    def test_independent_descent_rate_pause_floor_and_invalid_config(self): self.run_case('serial_descent_profile')
    def test_mode_four_acceleration_braking_and_latched_terminal_hold(self): self.run_case('velocity_descent_profile')
    def test_mode_two_three_mps_slope_floor_and_other_mode_limits(self): self.run_case('mode_two_three_mps')
    def test_circle_cross_all_descent_modes_climb_before_ack_and_submit_once(self): self.run_case('no_ack_circle_cross_recovery')
    def test_mode_four_missed_feedback_pause_restart_and_invalid_config(self): self.run_case('velocity_descent_pause')
    def test_mode_four_frozen_xy_release_while_descending_and_position_hold(self): self.run_case('velocity_descent_command')
    def test_height_only_circle_and_cross_release_after_xy_rebound(self): self.run_case('height_only_rpc')
    def test_height_only_retains_height_freshness_context_control_and_permission_guards(self): self.run_case('height_only_guards')
    def test_height_only_does_not_bypass_independent_pre_descent_xy(self): self.run_case('height_only_pre_descent')
    def test_height_only_accepts_closed_height_band_despite_horizontal_error(self): self.run_case('height_only_band')
    def test_height_only_default_and_explicit_legacy_xy_mode(self): self.run_case('height_only_config')
    def test_bound_standard_red_cross_and_unknown_class_thresholds(self): self.run_case('class_thresholds')
    def test_semantic_class_not_geometry_mode_selects_threshold(self): self.run_case('class_mode')
    def test_live_class_change_cannot_loosen_frozen_transaction(self): self.run_case('class_binding')
    def test_stable_compensation_mode_requires_speed_and_one_second_for_all_descent_lanes(self): self.run_case('serial_stability_modes')
    def test_red_cross_parameter_is_independent_and_unknown_remains_capped(self): self.run_case('class_parameters')
    def test_red_cross_default_and_invalid_parameter_values(self): self.run_case('red_parameters_invalid')
    def test_standard_and_red_cross_exact_threshold_equality_rejected(self): self.run_case('class_boundary')
    def test_five_cm_standard_admits_rpc_but_red_cross_never_calls(self): self.run_case('class_rpc')
    def test_three_physical_arms_yaw_and_tilt_without_accumulation(self): self.run_case('arms')
    def test_outside_endpoint_holds_reports_and_cannot_freeze_or_release(self): self.run_case('boundary')
    def test_endpoint_uses_current_and_commanded_yaw_margin(self): self.run_case('boundary_yaw')
    def test_visual_capture_freezes_before_full_arm_correction_and_settlement(self): self.run_case('capture')
    def test_circle_fsm_starts_arm_correction_and_descent_after_visual_capture(self): self.run_case('circle_descent')
    def test_cross_fsm_starts_arm_correction_and_descent_after_visual_capture(self): self.run_case('cross_descent')
    def test_circle_without_map_or_new_images_settles_and_releases_once(self): self.run_case('no_map_circle')
    def test_cross_without_map_or_new_images_settles_and_releases_once(self): self.run_case('no_map_cross')
    def test_descent_invalid_visual_frames_preserve_frozen_target(self): self.run_case('vision_frozen')
    def test_circle_outer_gate_releases_compensated_outlet_beyond_raw_center_distance(self): self.run_case('outer_circle')
    def test_cross_outer_gate_releases_compensated_outlet_beyond_raw_center_distance(self): self.run_case('outer_cross')
    def test_circle_outer_gate_and_final_admission_keep_geometry_freshness_and_permission(self): self.run_case('outer_reject_circle')
    def test_cross_outer_gate_and_final_admission_keep_geometry_freshness_and_permission(self): self.run_case('outer_reject_cross')
    def test_both_legacy_outer_gates_keep_original_distance_and_completed_guard(self): self.run_case('outer_legacy')
    def test_duplicate_stale_future_visual_frames_cannot_retarget(self): self.run_case('vision_replay')
    def test_six_cm_error_blocks_real_rpc_submission(self): self.run_case('xy')
    def test_configured_ten_cm_releases_immediately_without_speed_gate(self): self.run_case('configured_settle')
    def test_closed_height_band_accepts_035_040_045_and_rejects_outside(self): self.run_case('height_band')
    def test_release_floor_parameter_defaults_to_setpoint_for_legacy_config(self): self.run_case('height_config_compat')
    def test_current_xy_and_control_gate_with_speed_diagnostics_only(self): self.run_case('current_limits')
    def test_all_three_slots_release_once_at_each_accepted_height(self): self.run_case('slots_once')
    def test_cancel_and_original_permission_still_block_release(self): self.run_case('cancel_permission')
    def test_horizontal_speed_does_not_block_real_rpc_submission(self): self.run_case('speed')
    def test_angular_speed_alone_does_not_block_real_rpc_submission(self): self.run_case('angular')
    def test_single_current_odom_admits_rpc(self): self.run_case('samples')
    def test_duplicate_odom_does_not_refresh_age_and_expired_odom_blocks(self): self.run_case('odom_replay')
    def test_new_context_cannot_reuse_previous_freeze(self): self.run_case('context')
    def test_stale_future_missing_pose_blocks_release_despite_fresh_odom(self): self.run_case('pose_stale')
    def test_release_ignores_drop_duration_frames_gap_and_symmetric_height_tolerance(self): self.run_case('no_window')
    def test_admitted_async_rpc_survives_new_frames_and_revocation(self): self.run_case('async')
    def test_invalid_new_motion_revokes_readiness(self): self.run_case('invalid_motion')
    def test_continuous_future_odom_waits_without_erasing_settlement(self): self.run_case('future_sequence')
    def test_continuous_future_pose_waits_without_erasing_settlement(self): self.run_case('future_pose_sequence')
    def test_far_future_stamp_does_not_poison_source_watermark(self): self.run_case('future_watermark')
    def test_future_numeric_invalid_sample_cannot_recover_by_waiting(self): self.run_case('future_bad_numeric')



if __name__ == '__main__':
    unittest.main(verbosity=2)
