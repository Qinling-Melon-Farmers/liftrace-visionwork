"""Run production compensated-drop methods and async worker with offline transport.

No ROS nodes, master, simulation or hardware are required. Controller message/
time/transport boundaries use doubles. Geometry, settlement, FSM descent branches
and RPC admission/polling execute this checkout's code without a point-cloud map.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

# catkin/nose 按包名导入，unittest 按目录导入；两种入口都显式定位同目录测试工具。
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_async_servo import method

PACKAGE = Path(__file__).resolve().parents[1]

PROGRAM = r'''
#include "patrol_control/drop_slot_geometry.h"
#include "patrol_control/landing_handoff_stability.h"
#include "patrol_control/async_servo.h"
#include "patrol_control/servo_action_result.h"
#include "patrol_control/near_wall_align.h"
#include <array>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <limits>
#include <memory>
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
 bool compensated_target_outside_boundary_=false,guard_proceeded=false;
 NearWallAlignFence near_wall_align_fence_;
 struct { uav_vision::DropAlignmentFeedback last; void publish(const uav_vision::DropAlignmentFeedback& m){last=m;} } compensated_alignment_pub_;
 geometry_msgs::PoseStamped patrol_cmd;
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
 double external_alignment_capture_height_=1,drop_release_setpoint_height_=.35;
 double drop_height_threshold=.4,align_height=1;
 bool uav_drop_ready_=true,control_ready=true,permission_ready=true,should_drop=false;
 double dis_to_next_position=0;
 enum {Run_point,Aligning}; int Drone_mode=Aligning;
 std::array<std::array<double,2>,3> drop_slot_offsets_{{{{-.12,0}},{{0,-.12}},{{0,.12}}}};
 std::array<std::array<double,2>,3> dynamic_drop_slot_offsets_=drop_slot_offsets_;
 std::array<double,4> adjust_target_position{{0,0,0,0}};
 LandingHandoffStabilityConfig drop_settle_config_,landing_settle_config_;
 LandingHandoffStabilityWindow drop_capture_window_,drop_release_window_;
 LandingHandoffStabilityWindow landing_capture_window_,landing_handoff_window_;
 bool drop_metric_scale_enabled_=false; double drop_circle_radius_m_=.5,drop_cross_radius_m_=.175;
 double landing_pad_radius_m_=.5,pixel_to_meter_ratio_=.001,max_alignment_move_distance_=1;
 std::array<double,4> pixel_to_body_matrix_{{1,0,0,1}};
 double current_pixel_error=1000;
 bool dropPixelScales(ros::Time,double*,double*)const{return false;}
 bool externalLandingControlReady(ros::Time)const{return control_ready;}
 bool hasFreshMissionReleasePermission()const{return permission_ready;}
 ros::ServiceClient servo_client,servo_action_client_;
 AsyncServo async_servo_; std::uint64_t servo_action_id_=1;
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
 bool compensatedDropSettled(bool,double* =nullptr,double* =nullptr);
 bool freezeCompensatedDropTarget();
 void publishCompensatedAlignment();
 void advanceExternalGuard(){ EXTERNAL_GUARD guard_proceeded=true; }
 void applyDropSlotOffset(int,bool);
 void projectDropOffsetToTarget(const uav_vision::DropOffset&);
 void servoAlignmentContextCallback(const uav_vision::AlignmentTargetContext::ConstPtr&);
 DropActionResult executeDropAction(int); void pollDropAction();void cancelDropAction();
 void resetDropState();void clearUavVisionAlignmentState();void resetDetectionState();
 void advanceCircle(){int servo_id=1; CIRCLE_DESCENT }
 void advanceCross(){int servo_id=1; CROSS_DESCENT }
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
void capture(LLController&c){
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
 assert(!c.servo_action_pending_&&c.drop_complete&&c.servo_complete.data);
}
int main(int argc,char**argv){
 assert(argc==2);const std::string name=argv[1];ros::clock_sec=100;
 LLController c;auto t=std::make_shared<Transport>();c.servo_client.impl=c.servo_action_client_.impl=t;
 if(name=="arms"){
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
  close(c.align_height,.35);close(c.adjust_target_position[0],2.12);close(c.adjust_target_position[1],3);
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
  close(c.align_height,.35);close(c.adjust_target_position[0],2.12);
  const auto g=c.compensated_fc_goal_.pose.position;
  const auto observation=c.compensated_observation_stamp_;
  // No map callback or further image is delivered. Existing mission permission
  // remains valid, while final physical position/speed must still settle.
  c.have_waypoint_mark=c.have_cross_mark=false;c.have_drop_offset_=false;c.uav_drop_ready_=false;
  odom(c,101,2,3,.35);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_);
  odom(c,101.125,g.x,g.y,.35,.051);
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_);
  for(int i=0;i<3;++i){odom(c,101.25+i*.15625,g.x,g.y,.35);
   assert(c.executeDropAction(1)==DropActionResult::kPending);
   assert(c.servo_action_attempted_==(i==2));
   assert(c.compensated_goal_frozen_&&c.compensated_observation_stamp_==observation);
   close(c.compensated_target_center_.x,2);close(c.compensated_fc_goal_.pose.position.x,g.x);}
  finish(c,t);
  assert(c.executeDropAction(1)==DropActionResult::kSuccess&&t->calls==1);
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
 }else if(name=="xy"||name=="speed"||name=="angular"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  for(int i=0;i<5;++i){odom(c,101+i*.15625,g.x+(name=="xy"?.06:0),g.y,.35,
    name=="speed"?.051:0,0,geometry_msgs::Quaternion(),name=="angular"?.5:0);
   assert(c.executeDropAction(1)==DropActionResult::kPending);assert(!c.servo_action_attempted_&&t->calls==0);}
 }else if(name=="samples"||name=="async"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  close(c.drop_settle_config_.xy_tolerance_m,.04);close(c.drop_settle_config_.max_horizontal_speed_mps,.05);
  close(c.drop_settle_config_.stable_duration_sec,.3);assert(c.drop_settle_config_.min_samples==3);
  for(int i=0;i<3;++i){odom(c,101+i*.15625,g.x,g.y,.35);
   assert(c.executeDropAction(1)==DropActionResult::kPending);assert(c.servo_action_attempted_==(i==2));}
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
  odom(c,101,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
  const auto old=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);
  for(int i=1;i<=3;++i){ros::clock_sec=101+i*.05;c.uav_pose.header.stamp=ros::Time::now();
   c.motionOdomCallback(old);assert(!c.compensatedDropSettled(true));}
  ros::clock_sec=101.25;c.uav_pose.header.stamp=ros::Time::now();assert(!c.compensatedDropSettled(true));
  odom(c,101.3125,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
  odom(c,101.46875,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
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
 }else if(name=="duration"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  for(int i=0;i<3;++i){odom(c,101+i*.05,g.x,g.y,.35);
   assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);}
  // Three frames alone do not satisfy the 0.30-second window.
  odom(c,101.25,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
  odom(c,101.3125,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
 }else if(name=="frame_count"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  // Widen only the gap limit to isolate >=3 samples from elapsed duration.
  auto cfg=c.drop_settle_config_;cfg.max_sample_gap_sec=.5;
  c.drop_release_window_=LandingHandoffStabilityWindow(cfg);
  odom(c,101,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
  odom(c,101.3125,g.x,g.y,.35);assert(!c.compensatedDropSettled(true));
  assert(c.executeDropAction(1)==DropActionResult::kPending&&!c.servo_action_attempted_&&t->calls==0);
  odom(c,101.375,g.x,g.y,.35);assert(c.compensatedDropSettled(true));
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
   assert(c.compensatedDropSettled(true)==(i==3));
   assert(c.compensatedDropSettled(true)==(i==3));
  }
 }else if(name=="future_watermark"){
  capture(c);const auto g=c.compensated_fc_goal_.pose.position;
  odom(c,101,g.x,g.y,.35);const double accepted=c.motion_odom_.header.stamp.toSec();
  auto m=std::make_shared<nav_msgs::Odometry>(c.motion_odom_);m->header.stamp=ros::Time(500);
  c.motionOdomCallback(m);assert(!c.motion_odom_valid_);
  close(c.motion_odom_.header.stamp.toSec(),accepted);
  Eigen::Vector3d v,w;assert(!c.freshMotion(&v,&w));
  odom(c,101.125,g.x,g.y,.35);assert(c.freshMotion(&v,&w));
  assert(!c.compensatedDropSettled(true));
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
            'bool LLController::compensatedDropSettled(',
            'bool LLController::freezeCompensatedDropTarget(',
            'void LLController::publishCompensatedAlignment(',
            'void LLController::applyDropSlotOffset(',
            'void LLController::projectDropOffsetToTarget(',
            'void LLController::servoAlignmentContextCallback(',
            'DropActionResult LLController::executeDropAction(',
            'void LLController::pollDropAction(',
            'void LLController::cancelDropAction(',
            'void LLController::resetDropState(',
            'void LLController::clearUavVisionAlignmentState(',
            'void LLController::resetDetectionState(',
        )
        production = '\n'.join(method(source, signature) for signature in signatures)
        # Run the actual default assignments, not a second hand-maintained policy.
        start = source.index('    drop_settle_config_.xy_tolerance_m =')
        end = source.index('    load_stability(', start)
        defaults = source[start:end]
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
        guard_start = source.index('    if (compensated_alignment_enabled_ &&', source.index('void LLController::externalMissionTick()'))
        guard_end = source.index('    std_msgs::Bool detect_enable_msg;', guard_start)
        cpp.write_text(PROGRAM.replace('METHODS', production).replace('DEFAULTS', defaults)
                       .replace('EXTERNAL_GUARD', source[guard_start:guard_end])
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

    def test_three_physical_arms_yaw_and_tilt_without_accumulation(self): self.run_case('arms')
    def test_outside_endpoint_holds_reports_and_cannot_freeze_or_release(self): self.run_case('boundary')
    def test_endpoint_uses_current_and_commanded_yaw_margin(self): self.run_case('boundary_yaw')
    def test_visual_capture_freezes_before_full_arm_correction_and_settlement(self): self.run_case('capture')
    def test_circle_fsm_starts_arm_correction_and_descent_after_visual_capture(self): self.run_case('circle_descent')
    def test_cross_fsm_starts_arm_correction_and_descent_after_visual_capture(self): self.run_case('cross_descent')
    def test_circle_without_map_or_new_images_settles_and_releases_once(self): self.run_case('no_map_circle')
    def test_cross_without_map_or_new_images_settles_and_releases_once(self): self.run_case('no_map_cross')
    def test_descent_invalid_visual_frames_preserve_frozen_target(self): self.run_case('vision_frozen')
    def test_duplicate_stale_future_visual_frames_cannot_retarget(self): self.run_case('vision_replay')
    def test_six_cm_error_blocks_real_rpc_submission(self): self.run_case('xy')
    def test_horizontal_speed_blocks_real_rpc_submission(self): self.run_case('speed')
    def test_angular_motion_of_outlet_blocks_real_rpc_submission(self): self.run_case('angular')
    def test_three_distinct_odom_and_duration_before_rpc(self): self.run_case('samples')
    def test_duplicate_and_expired_odom_cannot_accumulate_readiness(self): self.run_case('odom_replay')
    def test_new_context_cannot_reuse_previous_freeze(self): self.run_case('context')
    def test_stale_future_missing_pose_blocks_release_despite_fresh_odom(self): self.run_case('pose_stale')
    def test_three_new_frames_still_need_point_three_seconds(self): self.run_case('duration')
    def test_duration_alone_still_needs_three_distinct_frames(self): self.run_case('frame_count')
    def test_admitted_async_rpc_survives_new_frames_and_revocation(self): self.run_case('async')
    def test_invalid_new_motion_revokes_readiness(self): self.run_case('invalid_motion')
    def test_continuous_future_odom_waits_without_erasing_settlement(self): self.run_case('future_sequence')
    def test_continuous_future_pose_waits_without_erasing_settlement(self): self.run_case('future_pose_sequence')
    def test_far_future_stamp_does_not_poison_source_watermark(self): self.run_case('future_watermark')
    def test_future_numeric_invalid_sample_cannot_recover_by_waiting(self): self.run_case('future_bad_numeric')



if __name__ == '__main__':
    unittest.main(verbosity=2)
