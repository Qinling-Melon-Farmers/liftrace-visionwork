"""Compile and execute production callbacks with real generated ROS messages, no nodes."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from test_drop_height_scale import production_method

PACKAGE=Path(__file__).resolve().parents[1]
ROOT=PACKAGE.parents[2]

PROGRAM=r'''
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_vision/DropOffset.h>
#include <uav_vision/DropReady.h>
#include <uav_vision/AlignmentTargetContext.h>
#include <patrol_control/drop_geometry.h>
#include <patrol_control/drop_action.h>
#include <patrol_control/async_servo.h>
#include <atomic>
#include <future>
#include <patrol_control/ReleaseAuthorization.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <iostream>
using namespace patrol_control;
#undef ROS_INFO
#undef ROS_WARN_THROTTLE
#undef ROS_INFO_THROTTLE
#undef ROS_DEBUG_THROTTLE
#define ROS_INFO(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_DEBUG_THROTTLE(...) ((void)0)
bool nearWallAlignReleaseAllowed(int,const geometry_msgs::PoseStamped&){return true;}
double distance3d(double x,double y,double z,double a,double b,double d){return std::sqrt((x-a)*(x-a)+(y-b)*(y-b)+(z-d)*(z-d));}
#include <array>
#include <cassert>
#include <limits>

// Copyable fixture handle; submission, completion and slot fencing use the real worker.
struct TestWorker {
  std::shared_ptr<AsyncServo> worker=std::make_shared<AsyncServo>();
  bool poll(std::uint64_t action,int slot,AsyncServo::Completion* out){return worker->poll(action,slot,out);}
  bool releaseNotStarted(std::uint64_t action,int slot){return worker->releaseNotStarted(action,slot);}
};
struct TestTransport {
  std::atomic<int> calls{0};
  std::promise<void> ack;
  std::shared_future<void> ack_ready=ack.get_future().share();
};

class LLController {
public:
  bool drop_exact_projection_enabled_=true, have_drop_offset_=false, uav_drop_ready_=false;
  bool have_waypoint_mark=false,have_cross_mark=false,drop_condition_met=false;
  bool have_servo_alignment_context_=true;
  double drop_offset_timeout_=.5,drop_ground_z_=0,align_height=.38,max_alignment_move_distance_=5;
  double drop_exact_alignment_max_error_m_=0;
  std::string drop_map_frame_="camera_init",drop_camera_frame_="downward_camera_optical_frame";
  std::string current_align_mode_="drop_circle",latest_drop_ready_reason_;
  unsigned servo_alignment_decision_seq_=900,servo_alignment_target_id_=7;
  std::string servo_alignment_target_class_="panzer";
  int detect_point_counter=0,legacy_calls=0;
  ros::Time last_drop_projection_stamp_,drop_projection_cutoff_=ros::Time(99.1);
  ros::Time latest_drop_offset_time_,latest_drop_ready_time_;
  uav_vision::DropOffset latest_drop_offset_;
  uav_vision::AlignmentTargetContext servo_alignment_context_;
  geometry_msgs::PoseStamped uav_pose,waypoint_mark_point,cross_mark_point,waypoint_temp;
  std::array<double,4> adjust_target_position{{0,0,0,0}};
  std::array<std::array<double,2>,3>drop_slot_offsets_{{{{-.12,0}},{{0,-.12}},{{0,.12}}}};
  std::array<std::array<double,2>,3>dynamic_drop_slot_offsets_=drop_slot_offsets_;
  void projectDropOffsetToTarget(const uav_vision::DropOffset&){++legacy_calls;}
  double capture_tolerance_m_=0;
  uav_vision::AlignmentTargetContext exact_drop_goal_context_;
  bool beginExactDropDescent();
  bool exactDropCommitmentMatches()const;
  void clearExactDropCommitment();
  void clearUavVisionAlignmentState();
  bool external_mission_mode_=true, mission_release_permission_active_=false;
  ros::Time latest_mission_release_permission_time_;
  double mission_release_permission_timeout_=.25;
  ReleaseAuthorization release_authorization_;
  bool hasFreshMissionReleasePermission()const;
  void releaseAuthorizationCallback(const ReleaseAuthorization::ConstPtr&);
  void servoAlignmentContextCallback(const uav_vision::AlignmentTargetContext::ConstPtr&);
  DropReleaseGate currentDropReleaseGate()const;
  bool have_land_mark=false;
  int count_aligning=0,times_detect=0,waypoint_next=0,near_wall_align_fence_=0;
  float times_detect_threshould=70,waypoint_adjust_max_second_threshould=15;
  bool first_call=true,align_ok=true,drop_complete=false,down_flag=true;
  bool ignore_servo_complete=true,cross_drop_completed=false,cross_mission_completed=false;
  bool temp_storage=false,drop_time_flag=false,require_vision_release_permission_=true;
  double external_alignment_capture_height_=.38,drop_release_setpoint_height_=.1;
  double drop_height_threshold=.2,drop_position_threshold_=.15,external_recovery_height_=.95;
  double external_cross_recovery_setpoint_height_=1.15,external_standard_recovery_setpoint_height_=1.2;
  double time_temp=0;
  ros::Time detection_start_time;
  geometry_msgs::PoseStamped last_waypoint_mark,patrol_cmd,mavros_point_cmd,last_mavros_point_cmd;
  std_msgs::Bool servo_complete;
  std::vector<bool> drop_completed{false,false,false};
  struct Waypoint {double x=0,y=0,z=0,yaw=0;};
  std::vector<Waypoint> waypoint_list{Waypoint()};
  int release_submissions=0;
  bool exercise_async=false,servo_action_pending_=false,servo_action_attempted_=false;
  std::uint64_t servo_action_id_=1;
  int servo_action_slot_=0;
  DropActionResult servo_action_result_=DropActionResult::kPending;
  TestWorker async_servo_;
  std::shared_ptr<TestTransport> transport=std::make_shared<TestTransport>();
  DropActionResult executeDropAction(int slot){
    if(servo_action_attempted_) return servo_action_result_;
    ++release_submissions;
    if(exercise_async){
      auto t=transport;
      assert(async_servo_.worker->submit(servo_action_id_,slot,5,[t](int){
        ++t->calls;t->ack_ready.wait();return DropActionResult::kSuccess;
      }));
      servo_action_slot_=slot;servo_action_pending_=true;servo_action_attempted_=true;
    }
    return DropActionResult::kPending;
  }
  void pollDropAction();
  void stopDropAction(int){}
  void resetDropState(){}
  void cleanupAfterCrossDrop(){}
  bool CrossDetectionDone();
  bool WayPointDetectDone();
  enum {Land,Aligning,Run_point,MAIN_MISSION,CROSS_MISSION};
  int Drone_mode=Aligning,current_task_type=MAIN_MISSION,detect_control_pub_=0;
  bool external_waiting_for_motion_=false,have_planner_cmd=false;
  void publishLegacyVisionControl(int,const std_msgs::Bool&){}
  void externalLandingTick(){}
  void externalMissionTick();
  bool hasFreshDropOffset()const;
  void dropOffsetCallback(const uav_vision::DropOffset::ConstPtr&);
  bool projectExactDropOffsetToTarget(const uav_vision::DropOffset&);
  bool exactDropReleaseReady()const;
  void dropReadyCallback(const uav_vision::DropReady::ConstPtr&);
  void applyDropSlotOffset(int,bool);
};
PRODUCTION_METHODS

LLController controller(){
  LLController c;c.uav_pose.header.frame_id="camera_init";c.uav_pose.header.stamp=ros::Time(100);
  c.uav_pose.pose.orientation.w=1;c.uav_pose.pose.position.x=.22;c.uav_pose.pose.position.y=.0075;
  auto &ctx=c.servo_alignment_context_;ctx.active=true;ctx.has_target=true;ctx.header.stamp=ros::Time(100);
  ctx.mission_id="m";ctx.schema_version=ctx.SCHEMA_VERSION;ctx.command=ctx.ALIGN;
  ctx.align_mode="drop_circle";ctx.deadline=ros::Time(200);ctx.decision_seq=900;
  ctx.semantic_target_id=7;ctx.semantic_target_class="panzer";ctx.payload_slot=1;
  ctx.semantic_target_first_seen=ros::Time(1);
  return c;
}
uav_vision::DropOffset::Ptr observation(double stamp=99.9){
  auto m=boost::make_shared<uav_vision::DropOffset>();
  m->header.stamp=ros::Time(stamp);m->header.frame_id="downward_camera_optical_frame";
  m->map_valid=true;m->map_frame="camera_init";m->map_point.x=.22;m->map_point.y=.0075;
  m->alignment_error_m=0;m->alignment_tolerance_m=.006;m->quality=.9;
  m->target_id=99;m->target_first_seen=ros::Time(98);return m;
}
void authorize(LLController &c,bool permitted=true){
  auto p=boost::make_shared<ReleaseAuthorization>();const auto &x=c.servo_alignment_context_;
  p->header.stamp=ros::Time::now();p->valid_until=ros::Time::now()+ros::Duration(.2);
  p->permitted=permitted;p->permission_epoch="epoch";p->permission_revision=c.release_authorization_.permission_revision+1;
  p->mission_id=x.mission_id;p->decision_seq=x.decision_seq;p->attempt=x.attempt;
  p->payload_slot=x.payload_slot;p->target_id=x.semantic_target_id;p->target_class=x.semantic_target_class;
  p->target_first_seen=x.semantic_target_first_seen;p->align_mode=x.align_mode;c.releaseAuthorizationCallback(p);
}
void ready(LLController &c){auto r=boost::make_shared<uav_vision::DropReady>();r->header.stamp=ros::Time::now();r->ready=true;c.dropReadyCallback(r);}
int main(int argc,char **argv){
  assert(argc==2);ros::Time::init();ros::Time::setNow(ros::Time(100));
  std::string test=argv[1];auto c=controller();auto m=observation();
  if(test=="projection"){
    m->dx_px=1000;c.dropOffsetCallback(m);ready(c);assert(c.beginExactDropDescent());authorize(c);
    assert(c.have_drop_offset_ && c.have_waypoint_mark && c.exactDropReleaseReady());
    assert(c.waypoint_mark_point.pose.position.x==m->map_point.x && c.legacy_calls==0);
    c.adjust_target_position[0]=m->map_point.x;c.adjust_target_position[1]=m->map_point.y;
    c.applyDropSlotOffset(1,false);assert(c.adjust_target_position[0]==m->map_point.x);
    // Latest yaw/position never re-projects an already absolute target.
    c.uav_pose.pose.orientation=tf::createQuaternionMsgFromYaw(1.1);
    c.uav_pose.pose.position.x=-.5;m->header.stamp=ros::Time(99.95);c.dropOffsetCallback(m);
    assert(c.waypoint_mark_point.pose.position.x==m->map_point.x);assert(!c.exactDropReleaseReady());
  }else if(test=="reorder"){
    c.dropOffsetCallback(m);ready(c);const auto receipt=c.latest_drop_offset_time_;
    ros::Time::setNow(ros::Time(100.05));c.dropOffsetCallback(m);
    auto older=observation(99.8);older->map_valid=false;c.dropOffsetCallback(older);
    assert(c.have_drop_offset_ && c.uav_drop_ready_ && c.latest_drop_offset_time_==receipt);
    assert(c.latest_drop_offset_.header.stamp==m->header.stamp);
    ros::Time::setNow(ros::Time(100.41));assert(!c.hasFreshDropOffset());
  }else if(test=="invalid_current"){
    c.dropOffsetCallback(m);ready(c);auto invalid=observation(99.95);invalid->map_valid=false;
    c.dropOffsetCallback(invalid);assert(!c.have_drop_offset_ && !c.uav_drop_ready_ && c.legacy_calls==0);
    c.dropOffsetCallback(m);assert(!c.have_drop_offset_); // cannot resurrect older valid geometry
  }else if(test=="geometry_id"){
    c.dropOffsetCallback(m);m=observation(99.95);m->target_id=100;m->target_first_seen=ros::Time(99.5);
    c.dropOffsetCallback(m);assert(c.have_drop_offset_ && c.latest_drop_offset_.target_id==100);
  }else if(test=="time"){
    for(double stamp:{0.,99.,100.01}){c=controller();c.dropOffsetCallback(observation(stamp));assert(!c.have_drop_offset_);}
    c=controller();c.drop_projection_cutoff_=ros::Time(99.95);c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
  }else if(test=="invalid_geometry"){
    for(int i=0;i<7;++i){c=controller();m=observation();
      if(i==0)m->header.frame_id="wrong";if(i==1)m->map_frame="wrong";
      if(i==2)m->map_point.z=.1;if(i==3)m->map_point.x=std::numeric_limits<double>::quiet_NaN();
      if(i==4)m->alignment_error_m=-1;if(i==5)m->alignment_tolerance_m=0;if(i==6)m->quality=2;
      c.dropOffsetCallback(m);assert(!c.have_drop_offset_ && c.legacy_calls==0);
    }
  }else if(test=="context"){
    for(int i=0;i<5;++i){c=controller();auto &ctx=c.servo_alignment_context_;
      if(i==0)ctx.semantic_target_id=8;if(i==1)ctx.decision_seq=901;if(i==2)ctx.deadline=ros::Time(100);
      if(i==3)ctx.payload_slot=2;if(i==4)ctx.active=false;
      c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
    }
    c=controller();c.current_align_mode_="drop_cross";auto &ctx=c.servo_alignment_context_;
    ctx.align_mode="drop_cross";ctx.semantic_target_class="red_cross";c.servo_alignment_target_class_="red_cross";
    m->target_id=7;m->target_first_seen=ctx.semantic_target_first_seen;
    c.dropOffsetCallback(m);assert(c.have_cross_mark);
    m=observation(99.95);c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
  }else if(test=="release"){
    c.dropOffsetCallback(m);ready(c);assert(c.beginExactDropDescent());authorize(c);assert(c.exactDropReleaseReady());
    c.uav_pose.pose.position.x+=.02;assert(!c.exactDropReleaseReady());
    c.uav_pose.pose.position.x=m->map_point.x;c.uav_pose.header.stamp=ros::Time(99);assert(!c.exactDropReleaseReady());
    c.uav_pose.header.stamp=ros::Time(100);c.clearExactDropCommitment();c.drop_exact_alignment_max_error_m_=.001;
    assert(c.beginExactDropDescent());
    c.uav_pose.pose.position.x+=.003;assert(!c.exactDropReleaseReady());
  }else if(test.compare(0,6,"state_")==0){
    const bool cross=test.find("cross")!=std::string::npos;
    if(cross){
      c.current_task_type=c.CROSS_MISSION;c.current_align_mode_="drop_cross";
      c.servo_alignment_context_.align_mode="drop_cross";
      c.servo_alignment_target_class_="red_cross";c.servo_alignment_context_.semantic_target_class="red_cross";
      m->target_id=7;m->target_first_seen=c.servo_alignment_context_.semantic_target_first_seen;
    }
    if(test=="state_initial_stale"){
      c.dropOffsetCallback(m);ready(c);ros::Time::setNow(ros::Time(101));
      c.uav_pose.header.stamp=ros::Time(101);c.externalMissionTick();
      assert(c.capture_tolerance_m_==0 && c.count_aligning==0 && c.release_submissions==0);return 0;
    }
    if(test=="state_initial_unstable"){
      c.dropOffsetCallback(m);c.externalMissionTick();
      assert(c.capture_tolerance_m_==0 && c.count_aligning==0 && c.release_submissions==0);return 0;
    }
    if(test=="state_initial_future_ready" || test=="state_initial_pose_bad"){
      c.dropOffsetCallback(m);ready(c);
      if(test=="state_initial_future_ready"){
        auto r=boost::make_shared<uav_vision::DropReady>();r->ready=true;r->header.stamp=ros::Time(100.01);
        c.dropReadyCallback(r);
      }else c.uav_pose.pose.position.x+=.02;
      c.externalMissionTick();
      assert(c.capture_tolerance_m_==0 && c.count_aligning==0 && c.release_submissions==0);return 0;
    }
    c.dropOffsetCallback(m);ready(c);
    // Capture precedes permission; initial denial cannot consume the geometry lock.
    authorize(c,false);c.externalMissionTick();
    assert(c.count_aligning==1 && c.capture_tolerance_m_==m->alignment_tolerance_m);
    assert(c.release_submissions==0);
    if(test.find("recovery")!=std::string::npos){
      c.exercise_async=true;authorize(c);c.externalMissionTick();
      assert(c.servo_action_pending_ && c.release_submissions==1);
      auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
      while(c.transport->calls==0 && std::chrono::steady_clock::now()<end)std::this_thread::yield();
      assert(c.transport->calls==1);
      auto inactive=boost::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
      inactive->active=false;
      const bool before=test.find("before")!=std::string::npos;
      if(before){
        c.servoAlignmentContextCallback(inactive);
        assert(c.servo_action_pending_ && c.count_aligning==1 && c.capture_tolerance_m_==0);
        c.externalMissionTick();assert(c.release_submissions==1 && !c.drop_complete);
      }
      c.transport->ack.set_value();
      while(c.servo_action_pending_ && std::chrono::steady_clock::now()<end){
        c.pollDropAction();std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      assert(!c.servo_action_pending_ && c.drop_complete && c.servo_complete.data && c.drop_completed[0]);
      if(!before)c.servoAlignmentContextCallback(inactive);
      assert(c.count_aligning==1 && c.capture_tolerance_m_==0 && !c.hasFreshMissionReleasePermission());
      assert(!c.exactDropReleaseReady());
      c.externalMissionTick();
      const double recovery=cross?c.external_cross_recovery_setpoint_height_:c.external_standard_recovery_setpoint_height_;
      assert(c.patrol_cmd.pose.position.z==recovery && c.Drone_mode==c.Aligning);
      c.uav_pose.pose.position.z=c.external_recovery_height_+.01;
      c.externalMissionTick();
      assert(c.Drone_mode==c.Run_point && c.external_waiting_for_motion_ && c.detect_point_counter==1);
      assert(c.patrol_cmd.pose.position.z==recovery && c.mavros_point_cmd.pose.position.z==recovery);
      assert(c.release_submissions==1 && c.transport->calls==1);return 0;
    }
    const auto capture_limit=c.capture_tolerance_m_;
    const auto original_stamp=c.latest_drop_offset_.header.stamp;
    const bool update=test=="state_standard_update" || test=="state_cross_update";
    if(update || test=="state_command_clamp"){
      auto n=boost::make_shared<uav_vision::DropOffset>(*m);n->header.stamp=ros::Time(99.95);
      n->map_point.x+=.03;n->map_point.y+=.04;n->alignment_error_m=.05;n->alignment_tolerance_m=.001;
      if(test=="state_command_clamp"){
        c.max_alignment_move_distance_=.5;c.uav_pose.pose.position.x=-1;
      }
      c.dropOffsetCallback(n);c.externalMissionTick();
      if(update){
        assert(c.adjust_target_position[0]==n->map_point.x && c.adjust_target_position[1]==n->map_point.y);
        assert(c.patrol_cmd.pose.position.x==n->map_point.x && c.patrol_cmd.pose.position.y==n->map_point.y);
      }else{
        assert(std::abs(std::hypot(c.patrol_cmd.pose.position.x-c.uav_pose.pose.position.x,
          c.patrol_cmd.pose.position.y-c.uav_pose.pose.position.y)-.5)<1e-9);
        assert(c.latest_drop_offset_.map_point.x==n->map_point.x && !c.exactDropReleaseReady());return 0;
      }
      assert(c.capture_tolerance_m_==capture_limit && c.release_submissions==0);
      c.uav_pose.pose.position.x=n->map_point.x;c.uav_pose.pose.position.y=n->map_point.y;
    }
    if(test=="state_invalid" || test=="state_jump"){
      auto n=boost::make_shared<uav_vision::DropOffset>(*m);n->header.stamp=ros::Time(99.95);
      if(test=="state_invalid") n->map_valid=false;else n->map_point.x+=6;
      c.dropOffsetCallback(n);assert(c.capture_tolerance_m_==0);authorize(c);c.externalMissionTick();
      assert(c.release_submissions==0);return 0;
    }
    if(test=="state_cancel" || test=="state_new_action"){
      auto ctx=boost::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
      if(test=="state_cancel")ctx->active=false;else ctx->attempt++;
      c.servoAlignmentContextCallback(ctx);assert(c.capture_tolerance_m_==0);
      authorize(c);c.externalMissionTick();assert(c.release_submissions==0);return 0;
    }
    // Image timeout follows the timer behavior; actual source stamps stay unchanged.
    ros::Time::setNow(ros::Time(101));c.have_drop_offset_=false;
    auto lost=boost::make_shared<uav_vision::DropReady>();lost->header.stamp=ros::Time(101);
    lost->ready=false;lost->reason="stale_observation";c.dropReadyCallback(lost);
    c.uav_pose.header.stamp=ros::Time(101);c.uav_pose.pose.position.z=.1;
    authorize(c);
    if(test=="state_denied")authorize(c,false);
    if(test=="state_expired")c.release_authorization_.valid_until=ros::Time(101);
    if(test=="state_permission_stale")c.release_authorization_.header.stamp=ros::Time(100);
    if(test=="state_wrong_action")c.release_authorization_.attempt++;
    if(test=="state_pose_stale")c.uav_pose.header.stamp=ros::Time(100);
    if(test=="state_pose_bad")c.uav_pose.pose.position.x+=.02;
    if(test=="state_deadline"){
      c.exact_drop_goal_context_.deadline=ros::Time(101);
      c.servo_alignment_context_.deadline=ros::Time(200); // no extension of captured bound
    }
    c.externalMissionTick();
    const bool permitted=test=="state_image_loss" || update;
    assert(c.release_submissions==(permitted?1:0));
    assert(c.capture_tolerance_m_==capture_limit);
    assert(c.latest_drop_offset_.header.stamp==(update?ros::Time(99.95):original_stamp));
    if(test=="state_denied"){
      authorize(c);c.externalMissionTick();assert(c.release_submissions==1);
    }
  }else if(test=="legacy"){
    c.drop_exact_projection_enabled_=false;c.dropOffsetCallback(m);assert(c.legacy_calls==1);
    c.adjust_target_position[0]=1;c.applyDropSlotOffset(1,false);assert(c.adjust_target_position[0]==.88);
  }else assert(false);
}
'''


class ExactDropControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(PACKAGE/'src/patrol_control.cpp').read_text()
        signatures=('bool LLController::hasFreshDropOffset()',
                    'bool LLController::hasFreshMissionReleasePermission()',
                    'void LLController::releaseAuthorizationCallback(',
                    'void LLController::servoAlignmentContextCallback(',
                    'void LLController::clearUavVisionAlignmentState()',
                    'void LLController::clearExactDropCommitment()',
                    'bool LLController::exactDropCommitmentMatches()',
                    'bool LLController::beginExactDropDescent()',
                    'DropReleaseGate LLController::currentDropReleaseGate()',
                    'bool LLController::CrossDetectionDone()',
                    'bool LLController::WayPointDetectDone()',
                    'void LLController::externalMissionTick()',
                    'void LLController::pollDropAction()',
                    'void LLController::dropOffsetCallback(',
                    'bool LLController::projectExactDropOffsetToTarget(',
                    'bool LLController::exactDropReleaseReady()',
                    'void LLController::dropReadyCallback(',
                    'void LLController::applyDropSlotOffset(')
        methods='\n'.join(production_method(source,s) for s in signatures)
        cls.temp=tempfile.TemporaryDirectory(prefix='drop-exact-control-')
        cls.addClassCleanup(cls.temp.cleanup)
        folder=Path(cls.temp.name);program=folder/'test.cpp';program.write_text(PROGRAM.replace('PRODUCTION_METHODS',methods))
        cls.binary=folder/'test'
        flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp','tf'],text=True))
        subprocess.run(['g++','-std=c++14','-O0','-pthread','-fsanitize=undefined','-fno-sanitize-recover=all',
                        '-I',str(PACKAGE/'include'),'-I',str(ROOT/'vision_ws/devel/include'),
                        '-I',str(ROOT/'patrol_uav_ws-patrol_planner/devel/include'),
                        str(program),'-o',str(cls.binary),*flags],check=True)

    def run_case(self,case):subprocess.run([str(self.binary),case],check=True)
    def test_absolute_projection_and_single_slot_compensation(self):self.run_case('projection')
    def test_duplicate_and_reorder_preserve_original_lease(self):self.run_case('reorder')
    def test_invalid_current_observation_blocks_without_pixel_fallback(self):self.run_case('invalid_current')
    def test_geometry_tracking_id_can_change_within_semantic_transaction(self):self.run_case('geometry_id')
    def test_source_time_future_stale_and_cutoff(self):self.run_case('time')
    def test_invalid_frames_points_and_tolerance(self):self.run_case('invalid_geometry')
    def test_semantic_context_identity_and_cross_identity(self):self.run_case('context')
    def test_current_pose_movement_and_freshness_recheck(self):self.run_case('release')
    def test_default_legacy_path_and_offsets_preserved(self):self.run_case('legacy')
    def test_actual_standard_locked_goal_update(self):self.run_case('state_standard_update')
    def test_actual_cross_locked_goal_update(self):self.run_case('state_cross_update')
    def test_image_loss_after_stable_capture_can_release(self):self.run_case('state_image_loss')
    def test_initial_stale_observation_cannot_capture(self):self.run_case('state_initial_stale')
    def test_initial_unstable_observation_cannot_capture(self):self.run_case('state_initial_unstable')
    def test_initial_future_ready_cannot_capture(self):self.run_case('state_initial_future_ready')
    def test_initial_current_pose_outside_tolerance_cannot_capture(self):self.run_case('state_initial_pose_bad')
    def test_denied_permission_blocks_without_consuming_capture(self):self.run_case('state_denied')
    def test_expired_permission_blocks(self):self.run_case('state_expired')
    def test_stale_permission_blocks(self):self.run_case('state_permission_stale')
    def test_wrong_action_permission_blocks(self):self.run_case('state_wrong_action')
    def test_stale_fc_pose_blocks(self):self.run_case('state_pose_stale')
    def test_current_fc_pose_outside_captured_tolerance_blocks(self):self.run_case('state_pose_bad')
    def test_captured_deadline_cannot_extend_on_heartbeat(self):self.run_case('state_deadline')
    def test_malformed_new_geometry_invalidates_capture(self):self.run_case('state_invalid')
    def test_unbounded_new_goal_invalidates_capture(self):self.run_case('state_jump')
    def test_explicit_cancel_invalidates_capture(self):self.run_case('state_cancel')
    def test_new_action_invalidates_capture(self):self.run_case('state_new_action')
    def test_each_new_emitted_goal_keeps_current_body_clamp(self):self.run_case('state_command_clamp')
    def test_cross_inactive_before_ack_recovers_and_hands_off_once(self):self.run_case('state_cross_recovery_before')
    def test_cross_inactive_after_ack_recovers_and_hands_off_once(self):self.run_case('state_cross_recovery_after')
    def test_standard_inactive_before_ack_recovers_and_hands_off_once(self):self.run_case('state_standard_recovery_before')
    def test_standard_inactive_after_ack_recovers_and_hands_off_once(self):self.run_case('state_standard_recovery_after')


if __name__=='__main__':unittest.main()
