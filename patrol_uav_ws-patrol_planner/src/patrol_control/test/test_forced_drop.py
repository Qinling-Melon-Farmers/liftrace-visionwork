"""Continuous production release/ACK tests shared by B and trial descent.

Reuse the geometry fixture; compile this checkout's controller bodies with
UBSan. ROS time/transport boundaries are doubles. No nodes or actuators.
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_compensated_drop_geometry as geometry
from test_async_servo import method

EXTRA_MEMBERS = r'''
 struct AlignCommand {ros::Header header; geometry_msgs::PoseStamped goal;
  unsigned int target_id=7;std::string target_class="panzer";};
 enum Task {MAIN_MISSION,CROSS_MISSION};Task current_task_type=MAIN_MISSION;
 enum PointMode {Nothing_point,Detect_point};PointMode Point_mode=Nothing_point;
 bool external_landing_auto_land_requested_=false,external_planner_height_hold_active_=false;
 bool external_waiting_for_motion_=false,have_planner_cmd=true,temp_storage=false;
 bool cross_drop_completed=false,cross_mission_completed=false;
 bool cross_found_=false,cross_detection_active_=false,tank_mission_completed=false,tank_found_=false,mission_interrupted=false;
 double time_temp=0,external_standard_recovery_setpoint_height_=1.2;
 double external_cross_recovery_setpoint_height_=1.15,external_recovery_height_=1.;
 ros::Time servo_alignment_stamp_;
 geometry_msgs::PoseStamped last_mavros_point_cmd;
 std::vector<std::string> goal;
 int detect_control_pub_=0;
 void clearExternalLandingState(bool){}
 bool isQuaternionNormalized(const geometry_msgs::Quaternion&q){return std::abs(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z-1)<.01;}
 template<class T>void publishLegacyVisionControl(int,const T&){}
 void acceptAlign(const AlignCommand*msg){ALIGN_BODY}
 void cleanupAfterCrossDrop();
 bool recoverCircle(){CIRCLE_RECOVERY return false;}
 bool recoverCross(){CROSS_RECOVERY return false;}
 void finishAlignment(bool align_done){struct {bool data=false;}detect_enable_msg; EXTERNAL_DONE}
'''

EXTRA_FUNCTIONS = r'''
void narrowPrepare(LLController&c,bool cross=false){
 ros::clock_sec=100.;NORMAL_XY_MODE
 c.drop_compensate_before_descent_=false;
 c.current_align_mode_=cross?"drop_cross":"drop_circle";
 c.servo_alignment_context_.align_mode=c.current_align_mode_;
 c.servo_alignment_context_.semantic_target_class=cross?"red_cross":"panzer";
 c.servo_alignment_target_class_=c.servo_alignment_context_.semantic_target_class;
 c.current_task_type=cross?LLController::CROSS_MISSION:LLController::MAIN_MISSION;
 c.drop_alignment_started_=c.drop_timeout_last_check_=ros::Time(100);
 odom(c,100,2,3,1);auto p=center(100);assert(c.setCompensatedDropTarget(&p));
 c.waypoint_mark_point=c.cross_mark_point=p;c.have_waypoint_mark=!cross;c.have_cross_mark=cross;
 assert(c.freezeCompensatedDropTarget());
 close(c.compensatedDropCommandHeight(),.4);assert(c.drop_descent_started_);
}
void narrowSample(LLController&c,double t,double z=.4,double error=.08,double speed=.5){
 const auto g=c.compensated_fc_goal_.pose.position;odom(c,t,g.x+error,g.y,z,speed);
}
'''

EXTRA_CASES = r'''
 }else if(name=="narrow_circle" || name=="narrow_cross"){
  const bool cross=name=="narrow_cross";narrowPrepare(c,cross);
  narrowSample(c,119.999);assert(!c.compensatedDropSettled(true));
  narrowSample(c,120.,1.);assert(c.compensatedDropSettled(true));
  narrowSample(c,120.001,.4);assert(c.compensatedDropSettled(true));
  assert(c.executeDropAction(1)==DropActionResult::kPending&&c.servo_action_attempted_);
  for(int i=0;i<100;++i)c.executeDropAction(1);
  assert(!c.drop_complete&&!c.servo_complete.data);
  const bool before=cross?c.recoverCross():c.recoverCircle();assert(!before&&c.detect_point_counter==0);
  finish(c,t);assert(t->calls==1);assert(c.executeDropAction(1)==DropActionResult::kSuccess);
  assert(c.detect_point_counter==0);assert(!(cross?c.recoverCross():c.recoverCircle()));
  narrowSample(c,120.01,1.1,0,0);
  const bool recovered=cross?c.recoverCross():c.recoverCircle();assert(recovered);
  c.finishAlignment(recovered);assert(c.Drone_mode==LLController::Run_point);
  assert(c.detect_point_counter==1&&!c.have_planner_cmd&&c.external_waiting_for_motion_);
  assert(c.drop_alignment_started_.isZero()&&t->calls==1);
 }else if(name=="narrow_guards"){
  for(int n=0;n<18;++n){
   ros::clock_sec=100.;LLController d;narrowPrepare(d);narrowSample(d,120.);
   if(n==0)d.compensated_goal_frozen_=false;
   if(n==1)d.drop_descent_started_=false;
   if(n==2){d.permission_ready=false;assert(d.compensatedDropSettled(true));
    assert(d.executeDropAction(1)==DropActionResult::kRejected);continue;}
   if(n==3)d.servo_alignment_context_.active=false;
   if(n==4)d.servo_alignment_context_.payload_slot=2;
   if(n==5)d.servo_alignment_context_.semantic_target_id++;
   if(n==6)d.servo_alignment_context_.attempt++;
   if(n==7)d.servo_alignment_context_.mission_id="other";
   if(n==8)d.servo_alignment_context_.deadline=ros::Time(120);
   if(n==9)d.control_ready=false;
   if(n==10)d.Drone_mode=LLController::Run_point;
   if(n==11)d.have_servo_alignment_context_=false;
   if(n==12)d.motion_odom_valid_=false;
   if(n==13)d.uav_pose.header.stamp=ros::Time(100);
   if(n==14){d.compensated_target_outside_boundary_=true;d.advanceExternalGuard();
    assert(!d.guard_proceeded);continue;}
   if(n==15)d.servo_alignment_context_.semantic_target_first_seen=ros::Time(91);
   if(n==16)d.servo_alignment_context_.align_mode="drop_cross";
   if(n==17)d.servo_alignment_context_.semantic_target_class="tent";
   assert(!d.compensatedDropSettled(true));
  }
 }else if(name=="narrow_serial"){
  narrowPrepare(c);c.drop_compensate_before_descent_=true;c.drop_descent_started_=false;
  c.drop_descent_started_at_=ros::Time(0);
  for(int i=0;i<30;++i){narrowSample(c,120+i*.1,1.,.08,0);
   close(c.compensatedDropCommandHeight(),1.);assert(!c.drop_descent_started_);
   assert(!c.compensatedDropSettled(true,nullptr,nullptr,false));}
  narrowSample(c,123,1.,0,.8);close(c.compensatedDropCommandHeight(),.4);
  assert(c.drop_descent_started_);close(c.drop_descent_started_at_.toSec(),123);
  narrowSample(c,142.999,1.,.8);assert(!c.compensatedDropSettled(true));
  narrowSample(c,143,1.,.8);assert(c.compensatedDropSettled(true));
 }else if(name=="narrow_normal_disabled"){
  narrowPrepare(c);narrowSample(c,101,.4,0,.8);
  c.drop_settle_config_.stable_duration_sec=100;c.drop_settle_config_.min_samples=1000;
  assert(c.compensatedDropSettled(true));
  c.drop_release_timeout_enabled_=false;narrowSample(c,125);
  assert(!c.compensatedDropSettled(true));narrowSample(c,125.01,.4,0,.8);
  assert(c.compensatedDropSettled(true));
 }else if(name=="narrow_clock"){
  narrowPrepare(c);narrowSample(c,119);assert(!c.compensatedDropSettled(true));
  ros::clock_sec=118.;assert(!c.compensatedDropSettled(true));
  narrowSample(c,125);assert(!c.compensatedDropSettled(true)&&c.drop_timeout_clock_invalid_);
  narrowSample(c,125.01,.4,0,.8);assert(c.compensatedDropSettled(true));
  c.resetDetectionState();assert(c.drop_alignment_started_.isZero()&&!c.drop_timeout_clock_invalid_);
 }else if(name=="narrow_cancel"){
  narrowPrepare(c);narrowSample(c,120);assert(c.compensatedDropSettled(true));
  auto end=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
  end->active=false;c.servoAlignmentContextCallback(end);
  assert(!c.compensatedDropSettled(true)&&c.drop_alignment_started_.isZero());
  end->active=true;c.servoAlignmentContextCallback(end);
  narrowSample(c,150);assert(!c.compensatedDropSettled(true));
  c.resetDropState();assert(c.drop_alignment_started_.isZero()&&!c.drop_timeout_clock_invalid_);
 }else if(name=="narrow_timer"){
  LLController::AlignCommand cmd;cmd.goal=center(100);cmd.goal.header.seq=10;
  cmd.header.stamp=ros::Time(99);c.acceptAlign(&cmd);close(c.drop_alignment_started_.toSec(),100);
  ros::clock_sec=119.;cmd.header.stamp=ros::Time(119);c.acceptAlign(&cmd);
  close(c.drop_alignment_started_.toSec(),100);
  auto heartbeat=std::make_shared<uav_vision::AlignmentTargetContext>();
  heartbeat->header.stamp=ros::Time(119);heartbeat->decision_seq=10;
  c.servoAlignmentContextCallback(heartbeat);close(c.drop_alignment_started_.toSec(),100);
  cmd.goal.header.seq=11;cmd.target_id=8;c.acceptAlign(&cmd);
  close(c.drop_alignment_started_.toSec(),119);
 }else assert(false);
'''

def block(source, signature):
    return method(source, signature)[len(signature):].strip()

class NarrowDropTimeout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (geometry.PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        original = geometry.PROGRAM
        align = block(method(source, 'void LLController::missionCommandCallback('), 'case patrol_control::MissionCommand::ALIGN:')
        circle = block(method(source, 'bool LLController::WayPointDetectDone('), 'if(servo_complete.data)')
        cross = block(method(source, 'bool LLController::CrossDetectionDone('), 'if(servo_complete.data)')
        done = method(method(source, 'void LLController::externalMissionTick('), 'if (align_done)')
        members = (EXTRA_MEMBERS.replace('ALIGN_BODY', align.replace('break;', 'return;'))
                   .replace('CIRCLE_RECOVERY', 'if(servo_complete.data)' + circle)
                   .replace('CROSS_RECOVERY', 'if(servo_complete.data)' + cross)
                   .replace('EXTERNAL_DONE', done))
        xy = 'c.drop_release_height_only_=false;' if 'drop_release_height_only_' in source else ''
        program = original.replace('class LLController {\npublic:', 'class LLController {\npublic:\n' + members, 1)
        if 'geometry_msgs::PoseStamped mavros_point_cmd;' not in original:
            program = program.replace('geometry_msgs::PoseStamped last_mavros_point_cmd;',
                                      'geometry_msgs::PoseStamped last_mavros_point_cmd,mavros_point_cmd;', 1)
        program = program.replace('int main(int argc,char**argv){', EXTRA_FUNCTIONS.replace('NORMAL_XY_MODE', xy) + '\nint main(int argc,char**argv){', 1)
        program = program.replace(' }else assert(false);\n}', EXTRA_CASES + '\n}', 1)
        program = program.replace('namespace tf {', 'namespace tf {\ngeometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw){return {0,0,std::sin(yaw/2),std::cos(yaw/2)};}')
        program = program.replace('METHODS\n}', 'METHODS\n' + method(source, 'void LLController::cleanupAfterCrossDrop(') + '\n}', 1)
        geometry.PROGRAM = program
        try:
            geometry.CompensatedDropGeometryTests.setUpClass.__func__(cls)
        finally:
            geometry.PROGRAM = original

    run_case = geometry.CompensatedDropGeometryTests.run_case
    def test_standard_timeout_release_once_and_normal_ack_recovery(self): self.run_case('narrow_circle')
    def test_red_cross_timeout_release_once_and_normal_ack_recovery(self): self.run_case('narrow_cross')
    def test_freeze_descent_permission_identity_deadline_feedback_guards(self): self.run_case('narrow_guards')
    def test_timeout_never_skips_independent_pre_descent_xy(self): self.run_case('narrow_serial')
    def test_normal_precision_immediate_and_disabled_timeout_restores_xy(self): self.run_case('narrow_normal_disabled')
    def test_clock_rollback_latches_fallback_off_but_normal_precision_remains(self): self.run_case('narrow_clock')
    def test_cancellation_and_late_heartbeat_do_not_resurrect_timeout(self): self.run_case('narrow_cancel')
    def test_real_align_command_and_context_heartbeats_keep_fixed_start(self): self.run_case('narrow_timer')

if __name__ == '__main__':
    unittest.main()
